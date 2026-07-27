import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Twist

# Field order of /balance/state.  Recorded into the message layout label so a
# bag stays readable without this source file.
STATE_FIELDS = (
    'pitch', 'err', 'rate', 'trim', 'offset', 'vel', 'vel_raw', 'turn',
    'v_ref', 'turn_ref', 'vel_int', 'u', 'sync', 'engaged',
    'bias', 'turn_int',
)


class BalanceController(Node):
    """平衡控制器v2：角度内环(IMU pitch→PID) + 速度外环(编码器轮速→目标倾角修正)。

    符号约定（2026-07-19全链路实测定稿）：
      前倾 = pitch正 = 需要正向(前进)PWM去追重心
    内环在IMU回调里跑(~284Hz)；外环在编码器回调里算轮速，
    修正量叠加到pitch_offset_deg上，压制角度环兜不住的净漂移。
    参数可用 ros2 param set 在线调。

    话题分层（2026-07-26遥控接入时确立）：
      /cmd_vel     人的速度意图，SI单位(m/s, rad/s)，由teleop发布
      /motor_cmd   本节点输出给serial_bridge的PWM指令(PWM/100)
    遥控不新增回路：只是把速度环和偏航环的设定值从0换成指令值。
    """

    def __init__(self):
        super().__init__('balance_controller')
        self.declare_parameter('kp', 20.0)     # PWM每度
        self.declare_parameter('ki', 0.0)      # 最后再加
        self.declare_parameter('kd', 0.6)      # PWM每(度/秒)
        self.declare_parameter('pitch_offset_deg', 0.0)  # 机械平衡点不在0度时补偿
        self.declare_parameter('capture_offset_on_engage', False)
        self.declare_parameter('capture_offset_bias_deg', 0.0)  # Bias after capturing the upright pose
        self.declare_parameter('capture_settle_time_sec', 0.8)
        self.declare_parameter('capture_rate_below_dps', 3.0)
        self.declare_parameter('max_tilt_deg', 30.0)     # 超过判定摔倒，切断输出
        self.declare_parameter('max_pwm', 255.0)         # 输出上限；手扶调参时用较小值限制冲击
        self.declare_parameter('max_pwm_fwd', 0.0)       # >0 overrides the positive-output limit
        self.declare_parameter('max_pwm_bwd', 0.0)       # >0 overrides the negative-output limit
        self.declare_parameter('max_wheel_velocity', 0.0)  # <=0 disables wheel-runaway latching
        self.declare_parameter('motor_output_sign', 1.0)  # Set to -1.0 when physical wheel polarity is reversed
        self.declare_parameter('latch_on_fall', True)    # 保护触发后保持断开，避免自动再次接管
        self.declare_parameter('engage_below_deg', 3.0)  # 扶正到该角度内才上电
        self.declare_parameter('engage_rate_below_dps', 15.0)  # 角速度也需低于该值才接管，避免正摆动中途接管扛不住动量
        self.declare_parameter('deadband_pwm_fwd', 0.0)  # 正向(前进)死区前馈，两方向静摩擦不对称需分开设
        self.declare_parameter('deadband_pwm_bwd', 0.0)  # 反向(后退)死区前馈
        self.declare_parameter('deadband_err_deg', 1.0)  # 误差小于该角度不启动死区前馈，避免过零颤振
        self.declare_parameter('deadband_rate_dps', 0.0)  # Allow friction compensation for fast D-term braking
        self.declare_parameter('deadband_ramp_deg', 0.0)  # >0 enables smooth static-friction compensation
        self.declare_parameter('deadband_output_ramp_pwm', 0.0)  # >0 ramps static-friction assist from PID effort
        self.declare_parameter('deadband_dither_hz', 0.0)  # >0 enables sigma-delta low-effort static-friction pulses
        self.declare_parameter('deadband_dither_pulse_ms', 0.0)  # Minimum pulse width; 0 keeps one-control-frame pulses
        self.declare_parameter('deadband_velocity_threshold', 0.0)  # Compensation fades out above this wheel speed
        self.declare_parameter('recovery_pwm', 0.0)  # Extra authority after the tilt escapes the center band
        self.declare_parameter('recovery_pwm_fwd', -1.0)  # Negative uses recovery_pwm as a fallback
        self.declare_parameter('recovery_pwm_bwd', -1.0)  # Negative uses recovery_pwm as a fallback
        self.declare_parameter('recovery_err_deg', 0.7)
        self.declare_parameter('recovery_ramp_deg', 0.4)
        self.declare_parameter('kp_v', 0.0)      # 速度环比例：ticks/s → 目标倾角修正(度)
        self.declare_parameter('ki_v', 0.0)      # 速度环积分：消除长期位置漂移
        self.declare_parameter('kd_wheel', 0.0)  # Direct wheel-speed damping, PWM per (tick/s)
        self.declare_parameter('max_wheel_damping_pwm', 0.0)  # <=0 leaves direct damping uncapped
        self.declare_parameter('max_wheel_damping_return_pwm', 0.0)  # Higher cap while body is returning upright
        self.declare_parameter('wheel_sync_kp', 0.0)  # PWM per left/right wheel-speed difference
        # 纯P产生的差模修正可能整段都低于静摩擦突破电压，轮子挣不脱。
        # 泄放积分会一直爬到挣脱为止，误差消失后自行衰减。
        self.declare_parameter('wheel_sync_ki', 0.0)
        self.declare_parameter('turn_integral_leak_s', 6.0)
        self.declare_parameter('max_turn_integral', 2000.0)
        self.declare_parameter('max_wheel_sync_pwm', 0.0)  # <=0 leaves wheel synchronization uncapped
        self.declare_parameter('turn_velocity_filter_alpha', 0.15)
        # IMU零偏随通电时间漂移(实测两小时0.86度，冷却后完全归零)。
        # 站桩时速度环平均要顶住的trim，本身就等于零点误差，慢慢把它吸收掉。
        # 时间常数必须远大于位置环，否则两个积分器会互相打架。0=关闭。
        self.declare_parameter('offset_adapt_tau_s', 0.0)
        self.declare_parameter('max_offset_bias_deg', 2.0)
        self.declare_parameter('max_offset_trim_deg', 5.0)  # 速度环能修正offset的最大幅度，防止跑飞
        self.declare_parameter('max_velocity_integral', 500.0)  # 位置项抗饱和限幅(ticks)
        self.declare_parameter('velocity_filter_alpha', 0.3)  # 速度信号低通滤波系数(0~1)，越小越平滑但越滞后

        # ---- 遥控 ----
        # 输入是SI单位的速度意图，输出是给serial_bridge的PWM。两者必须分开，
        # 否则teleop和本节点会抢同一个/cmd_vel互相覆盖。
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_cmd_topic', '/motor_cmd')
        # m/s ↔ ticks/s 换算。与serial_bridge的ticks_per_rev和URDF轮径保持一致，
        # 改了机械参数这里也要跟着改。
        self.declare_parameter('ticks_per_rev', 277.5)
        self.declare_parameter('wheel_radius_m', 0.0325)
        self.declare_parameter('wheel_separation_m', 0.22)
        # 永远不要相信输入源：teleop_twist_keyboard默认0.5m/s，直接超过整车安全限。
        self.declare_parameter('max_cmd_linear_mps', 0.25)
        self.declare_parameter('max_cmd_angular_rps', 1.2)
        # 断连/停发即回零，指令超时是遥控失效保护的唯一可靠位置。
        self.declare_parameter('cmd_timeout_s', 0.5)
        # 阶跃指令会把车掀翻：设定值必须斜坡上升，让角度环有时间把重心送出去。
        self.declare_parameter('cmd_accel_ticks_s2', 250.0)
        self.declare_parameter('cmd_yaw_accel_ticks_s2', 600.0)
        # ki_v积分的是位置误差。行驶中继续累积会导致松手后倒车"补距离"，
        # 故有指令时改为泄放；回到零指令后恢复累积，站桩性能不受影响。
        self.declare_parameter('drive_integral_leak_s', 0.5)
        self.declare_parameter('drive_deadzone_ticks', 20.0)
        # 行驶结束回到站桩的瞬间清零位置积分：积分量代表相对"上一个保持点"的
        # 位移，而操作者刚刚是有意离开那个点的。不清零车会一路爬回出发地
        # （T8实测全程净位移+0.01圈——它精确回到了原点，这不是遥控想要的）。
        self.declare_parameter('reset_integral_on_stop', True)
        self.declare_parameter('angular_cmd_sign', 1.0)  # 实测方向不对时置-1
        self.declare_parameter('state_publish_decim', 3)  # /balance/state降频倍数
        self.declare_parameter('pitch_rate_filter_alpha', 0.3)  # pitch_rate低通滤波系数(0~1)，越小越平滑但越滞后
        self.declare_parameter('pitch_rate_deadband_dps', 0.0)  # Ignore small IMU-rate noise around upright
        self.declare_parameter('debug_control', False)

        self.integral = 0.0
        self.engaged = False
        self.safety_latched = False
        self.captured_offset = None
        self.capture_start_t = None
        self.capture_pitch_sum = 0.0
        self.capture_pitch_count = 0
        self.last_t = None
        self.last_debug_t = None
        self.last_pitch = None
        self.last_pitch_t = None
        self.pitch_rate_filtered = 0.0
        self._dither_accumulator = 0.0
        self._dither_sign = 0.0
        self._dither_hold_until = 0.0
        self._dither_pulse_output = 0.0

        self.wheel_velocity = 0.0  # ticks/s，均值，正负号需实测校准
        self.wheel_velocity_raw = 0.0
        self.left_wheel_velocity_raw = 0.0
        self.right_wheel_velocity_raw = 0.0
        self.turn_velocity_raw = 0.0
        self.turn_velocity = 0.0
        self.velocity_integral = 0.0
        self.turn_integral = 0.0
        self.offset_bias = 0.0
        self._last_enc = None
        self._last_enc_t = None

        # 遥控设定值：_cmd_* 是收到的原始目标，*_ref 是斜坡后实际送进环里的
        self._cmd_v_ticks = 0.0
        self._cmd_turn_ticks = 0.0
        self._last_cmd_t = None
        self._cmd_stale = True
        self.v_ref = 0.0
        self.turn_ref = 0.0
        self._ref_last_t = None
        self._station_keeping = True
        self._state_decim = 0

        motor_topic = self.get_parameter('motor_cmd_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        # Control commands must never queue behind stale PWM values.
        self.cmd_pub = self.create_publisher(Twist, motor_topic, 1)
        self.state_pub = self.create_publisher(Float32MultiArray, '/balance/state', 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(String, '/wheel/encoders', self._enc_cb, 10)
        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 1)
        self.get_logger().info(
            f'balance_controller started (disengaged, 扶正后自动接管) '
            f'cmd_in={cmd_topic} motor_out={motor_topic} '
            f'{self._ticks_per_meter():.1f} ticks/m')

    def _enc_cb(self, msg: String):
        try:
            l_str, r_str = msg.data.split(',')
            l, r = int(l_str), int(r_str)
        except ValueError:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._last_enc is not None:
            dt = t - self._last_enc_t
            if 0.005 < dt < 0.2:
                dl = l - self._last_enc[0]
                dr = r - self._last_enc[1]
                # 差速驱动左右编码器计数天生反向(左右镜像安装)，需相减而非相加才能叠加而非抵消
                left_v = dl / dt
                right_v = -dr / dt
                raw_v = (left_v + right_v) / 2.0
                self.wheel_velocity_raw = raw_v
                self.left_wheel_velocity_raw = left_v
                self.right_wheel_velocity_raw = right_v
                self.turn_velocity_raw = left_v - right_v
                alpha = self.get_parameter('velocity_filter_alpha').value
                turn_alpha = self.get_parameter('turn_velocity_filter_alpha').value
                # Use each encoder message so the brake has no timer/encoder phase error.
                self.wheel_velocity = alpha * raw_v + (1.0 - alpha) * self.wheel_velocity
                self.turn_velocity = (turn_alpha * self.turn_velocity_raw
                                      + (1.0 - turn_alpha) * self.turn_velocity)
        self._last_enc = (l, r)
        self._last_enc_t = t

    def _ticks_per_meter(self):
        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        radius = self.get_parameter('wheel_radius_m').value
        return ticks_per_rev / (2.0 * math.pi * radius)

    def _cmd_cb(self, msg: Twist):
        """人的速度意图 → 外环设定值(ticks/s)。"""
        max_lin = self.get_parameter('max_cmd_linear_mps').value
        max_ang = self.get_parameter('max_cmd_angular_rps').value
        lin = max(-max_lin, min(max_lin, msg.linear.x))
        ang = max(-max_ang, min(max_ang, msg.angular.z))

        ticks_per_meter = self._ticks_per_meter()
        separation = self.get_parameter('wheel_separation_m').value
        self._cmd_v_ticks = lin * ticks_per_meter
        # turn_velocity = left_v - right_v，而REP-103里angular.z为正是左转(逆时针)，
        # 左转时右轮更快，故差模设定值取负。
        self._cmd_turn_ticks = (self.get_parameter('angular_cmd_sign').value
                                * -ang * separation * ticks_per_meter)
        self._last_cmd_t = self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _slew(current, target, max_step):
        if max_step <= 0.0:
            return target
        delta = target - current
        return current + max(-max_step, min(max_step, delta))

    def _reset_command_refs(self):
        """接管/切断时把设定值清零，避免沿用上一段残留的目标速度。"""
        self.v_ref = 0.0
        self.turn_ref = 0.0
        self._cmd_v_ticks = 0.0
        self._cmd_turn_ticks = 0.0
        self._last_cmd_t = None

    def _update_command_refs(self):
        """把指令斜坡到设定值；超时则回零。

        用自己的墙钟dt而不是控制环的dt：斜坡是真实时间量，不该跟IMU帧率绑定。
        脱离接管时照常更新，这样车躺在地上也能验证整条指令链路
        （输出在未接管分支里是硬零，不受影响）。
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._ref_last_t is None else now - self._ref_last_t
        self._ref_last_t = now
        timeout = self.get_parameter('cmd_timeout_s').value
        fresh = (self._last_cmd_t is not None
                 and (now - self._last_cmd_t) <= timeout)
        if fresh:
            if self._cmd_stale:
                self.get_logger().info('遥控指令接入')
            self._cmd_stale = False
            target_v, target_turn = self._cmd_v_ticks, self._cmd_turn_ticks
        else:
            # 指令断流：目标归零，车停在原地继续平衡而不是保持最后的速度冲出去。
            if not self._cmd_stale:
                self.get_logger().warn('遥控指令超时，设定值回零')
            self._cmd_stale = True
            target_v, target_turn = 0.0, 0.0

        if not (0.0 < dt < 0.5):
            return
        self.v_ref = self._slew(
            self.v_ref, target_v,
            self.get_parameter('cmd_accel_ticks_s2').value * dt)
        self.turn_ref = self._slew(
            self.turn_ref, target_turn,
            self.get_parameter('cmd_yaw_accel_ticks_s2').value * dt)

    def _publish_state(self, pitch, err, trim, offset, u, sync):
        """把控制器内部状态录进包里，省得离线再去重放状态机猜。"""
        decim = max(1, int(self.get_parameter('state_publish_decim').value))
        self._state_decim += 1
        if self._state_decim % decim:
            return
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = ','.join(STATE_FIELDS)
        dim.size = len(STATE_FIELDS)
        dim.stride = len(STATE_FIELDS)
        msg.layout.dim = [dim]
        msg.data = [
            float(pitch), float(err), float(self.pitch_rate_filtered),
            float(trim), float(offset), float(self.wheel_velocity),
            float(self.wheel_velocity_raw), float(self.turn_velocity),
            float(self.v_ref), float(self.turn_ref),
            float(self.velocity_integral), float(u), float(sync),
            1.0 if self.engaged else 0.0,
            float(self.offset_bias), float(self.turn_integral),
        ]
        self.state_pub.publish(msg)

    def _imu_cb(self, msg: Imu):
        x, y, z, w = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.degrees(math.asin(sinp))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_pitch is None or self.last_pitch_t is None:
            pitch_rate = 0.0
        else:
            pitch_dt = t - self.last_pitch_t
            pitch_rate = ((pitch - self.last_pitch) / pitch_dt
                          if 0.001 < pitch_dt < 0.05 else 0.0)
        self.last_pitch = pitch
        self.last_pitch_t = t

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        offset = self.get_parameter('pitch_offset_deg').value
        capture_offset = self.get_parameter('capture_offset_on_engage').value
        capture_bias = self.get_parameter('capture_offset_bias_deg').value
        capture_settle_time = self.get_parameter('capture_settle_time_sec').value
        capture_rate_below = self.get_parameter('capture_rate_below_dps').value
        max_tilt = self.get_parameter('max_tilt_deg').value
        max_pwm = self.get_parameter('max_pwm').value
        max_pwm_fwd = self.get_parameter('max_pwm_fwd').value
        max_pwm_bwd = self.get_parameter('max_pwm_bwd').value
        max_wheel_velocity = self.get_parameter('max_wheel_velocity').value
        motor_sign = -1.0 if self.get_parameter('motor_output_sign').value < 0.0 else 1.0
        latch_on_fall = self.get_parameter('latch_on_fall').value
        engage_below = self.get_parameter('engage_below_deg').value
        engage_rate_below = self.get_parameter('engage_rate_below_dps').value
        deadband_fwd = self.get_parameter('deadband_pwm_fwd').value
        deadband_bwd = self.get_parameter('deadband_pwm_bwd').value
        deadband_err = self.get_parameter('deadband_err_deg').value
        deadband_rate = self.get_parameter('deadband_rate_dps').value
        deadband_ramp = self.get_parameter('deadband_ramp_deg').value
        deadband_output_ramp = self.get_parameter('deadband_output_ramp_pwm').value
        deadband_dither_hz = self.get_parameter('deadband_dither_hz').value
        deadband_dither_pulse_ms = self.get_parameter('deadband_dither_pulse_ms').value
        deadband_velocity = self.get_parameter('deadband_velocity_threshold').value
        recovery_pwm = self.get_parameter('recovery_pwm').value
        recovery_pwm_fwd = self.get_parameter('recovery_pwm_fwd').value
        recovery_pwm_bwd = self.get_parameter('recovery_pwm_bwd').value
        recovery_err = self.get_parameter('recovery_err_deg').value
        recovery_ramp = self.get_parameter('recovery_ramp_deg').value
        kp_v = self.get_parameter('kp_v').value
        ki_v = self.get_parameter('ki_v').value
        kd_wheel = self.get_parameter('kd_wheel').value
        max_wheel_damping = self.get_parameter('max_wheel_damping_pwm').value
        max_wheel_damping_return = self.get_parameter('max_wheel_damping_return_pwm').value
        wheel_sync_kp = self.get_parameter('wheel_sync_kp').value
        max_wheel_sync = self.get_parameter('max_wheel_sync_pwm').value
        max_offset_trim = self.get_parameter('max_offset_trim_deg').value
        rate_alpha = self.get_parameter('pitch_rate_filter_alpha').value
        rate_deadband = self.get_parameter('pitch_rate_deadband_dps').value

        # D项由同一姿态定义的pitch差分得到，避免IMU安装轴与Euler轴不一致。
        self.pitch_rate_filtered = rate_alpha * pitch_rate + (1.0 - rate_alpha) * self.pitch_rate_filtered

        # 安全判定用机械原始offset，不受速度环影响。
        # offset_bias是慢速零点自适应的输出，用于抵消IMU温漂，不是控制量。
        effective_offset = (self.captured_offset
                            if capture_offset and self.captured_offset is not None
                            else offset) + self.offset_bias
        raw_err = pitch - effective_offset
        # Wait for the serial bridge before capturing a balance point or engaging.
        cmd_link_ready = self.cmd_pub.get_subscription_count() > 0

        # 设定值斜坡与接管状态无关，脱离时也跑，便于地面验证指令链路。
        self._update_command_refs()

        # 状态机：摔倒切断 / 扶正接管
        # 跑飞判据是"相对设定值"的超速：遥控行驶时轮速本就该不为零，
        # 用绝对轮速判会在正常前进时误触发。
        #
        # 必须用滤波后的轮速，不能用raw：编码器277.5计数/圈、10ms窗口下
        # 一个计数就是约100 ticks/s，raw信号逐帧在0和±300之间跳。
        # 2026-07-26实测raw峰值687而滤波后仅356，三次"跑飞"保护里有两次
        # 触发时滤波轮速只有51和72 ticks/s——车几乎是静止的，纯属误触发。
        # 滤波时间常数 dt/alpha = 0.01/0.06 ≈ 0.17s，真跑飞仍能在0.3s内切断，
        # 何况还有倾角保护兜底。
        if (self.engaged and max_wheel_velocity > 0.0
                and abs(self.wheel_velocity) > max_wheel_velocity + abs(self.v_ref)):
            self.engaged = False
            self.safety_latched = latch_on_fall
            self.integral = 0.0
            self.velocity_integral = 0.0
            self._reset_command_refs()
            self.get_logger().warn(
                f'wheel speed raw={self.wheel_velocity_raw:.1f} '
                f'filtered={self.wheel_velocity:.1f} exceeds the safety limit; output disabled')
        elif self.engaged and abs(raw_err) > max_tilt:
            self.engaged = False
            self.safety_latched = latch_on_fall
            self.integral = 0.0
            self.velocity_integral = 0.0
            self._reset_command_refs()
            suffix = '，需重启控制器后才能再次接管' if self.safety_latched else ''
            self.get_logger().warn(f'倾角{pitch:.1f}°超限，输出切断{suffix}')
        elif not self.engaged and not self.safety_latched and cmd_link_ready:
            if capture_offset:
                if abs(self.pitch_rate_filtered) >= capture_rate_below:
                    self.capture_start_t = None
                    self.capture_pitch_sum = 0.0
                    self.capture_pitch_count = 0
                else:
                    if self.capture_start_t is None:
                        self.capture_start_t = t
                        self.capture_pitch_sum = 0.0
                        self.capture_pitch_count = 0
                    self.capture_pitch_sum += pitch
                    self.capture_pitch_count += 1
                    if t - self.capture_start_t >= capture_settle_time:
                        self.captured_offset = (
                            self.capture_pitch_sum / self.capture_pitch_count + capture_bias)
                        effective_offset = self.captured_offset
                        raw_err = pitch - effective_offset
                        self.engaged = True
                        self.integral = 0.0
                        self.velocity_integral = 0.0
                        self._reset_command_refs()
                        self.last_t = None
                        # A residual sample at the instant of capture must not start
                        # the wheels while the angle error is still zero.
                        self.pitch_rate_filtered = 0.0
                        self.get_logger().info(
                            f'engaged with stable pitch offset {self.captured_offset:.2f} deg')
            elif abs(raw_err) < engage_below and abs(pitch_rate) < engage_rate_below:
                self.engaged = True
                self.integral = 0.0
                self.velocity_integral = 0.0
                self._reset_command_refs()
                self.last_t = None
                self.pitch_rate_filtered = 0.0  # 避免沿用接管前残留角速度造成瞬间突变
                self.get_logger().info('已接管平衡控制')

        if not self.engaged:
            self._send_pwm(0.0)
            self._publish_state(pitch, raw_err, 0.0, effective_offset, 0.0, 0.0)
            return

        # dt由IMU消息时间戳求得
        dt = (t - self.last_t) if self.last_t is not None else 0.0
        self.last_t = t

        # 速度外环：车偏离设定速度时，修正目标倾角把它拉回来。
        # 设定值为0就是原地站桩(整定时的工况)，非0就是跟随遥控指令——
        # 同一套回路，只是设定值不同。
        vel_err = self.wheel_velocity - self.v_ref
        max_vel_integral = self.get_parameter('max_velocity_integral').value
        drive_deadzone = self.get_parameter('drive_deadzone_ticks').value
        station_keeping = abs(self.v_ref) < drive_deadzone
        if (station_keeping and not self._station_keeping
                and self.get_parameter('reset_integral_on_stop').value):
            # 刚刚结束行驶：把当前位置定义为新的保持点。
            # 此刻积分已被泄放到很小(T8实测约+19，折合trim跳变0.1度)，无冲击。
            self.velocity_integral = 0.0
        self._station_keeping = station_keeping
        if 0.0 < dt < 0.05:
            if station_keeping:
                # ∫v dt 就是位置，ki_v 实际是位置比例项，负责把车锁在原地。
                self.velocity_integral += vel_err * dt
            else:
                # 行驶中若继续累积，加速滞后欠下的"距离"会在松手后被讨回来，
                # 表现为倒车找位置。泄放掉，只留kp_v做速度比例(允许少量速度垂度)。
                leak = self.get_parameter('drive_integral_leak_s').value
                self.velocity_integral *= (max(0.0, 1.0 - dt / leak)
                                           if leak > 0.0 else 0.0)
            self.velocity_integral = max(-max_vel_integral,
                                         min(max_vel_integral, self.velocity_integral))
        offset_trim = kp_v * vel_err + ki_v * self.velocity_integral
        offset_trim = max(-max_offset_trim, min(max_offset_trim, offset_trim))

        # 零点慢速自适应：站桩且平均轮速为零时，速度环平均顶住的trim就是零点误差。
        # 只在站桩时更新——行驶时trim非零是正当的(维持速度)，那不是零点误差。
        # 闭环是 bias' = (真实平衡点 - 配置offset - bias)/tau，一阶稳定。
        adapt_tau = self.get_parameter('offset_adapt_tau_s').value
        if adapt_tau > 0.0 and station_keeping and 0.0 < dt < 0.05:
            max_bias = self.get_parameter('max_offset_bias_deg').value
            self.offset_bias += (offset_trim / adapt_tau) * dt
            self.offset_bias = max(-max_bias, min(max_bias, self.offset_bias))

        err = pitch - (effective_offset + offset_trim)

        if 0.0 < dt < 0.05:
            self.integral += err * dt
            self.integral = max(-100.0, min(100.0, self.integral))  # 抗饱和

        rate_for_control = math.copysign(
            max(0.0, abs(self.pitch_rate_filtered) - rate_deadband),
            self.pitch_rate_filtered)
        u = kp * err + ki * self.integral + kd * rate_for_control
        # Wheel speed is a separate state from body pitch.  Damping it directly
        # prevents a successful catch from turning into a long runaway roll.
        wheel_damping = -kd_wheel * self.wheel_velocity
        damping_limit = max_wheel_damping
        if err * rate_for_control < 0.0 and max_wheel_damping_return > 0.0:
            damping_limit = max_wheel_damping_return
        if damping_limit > 0.0:
            wheel_damping = max(-damping_limit, min(damping_limit, wheel_damping))
        u += wheel_damping

        # Equal and opposite side bias suppresses yaw without changing the
        # average forward/backward torque used by the balance loop.
        # 同理，设定值为0是走直线，非0就是按指令转向。
        turn_err = self.turn_velocity - self.turn_ref
        # 纯P时右转的修正量整段停在-34 PWM均值(峰值-59)，而负载突破电压是65~80,
        # 轮子根本挣不脱静摩擦，实测右转只到设定值的28%。泄放积分会一路爬到
        # 挣脱为止；误差消失后自行衰减，不会像纯积分那样锁死航向。
        turn_leak = self.get_parameter('turn_integral_leak_s').value
        max_turn_int = self.get_parameter('max_turn_integral').value
        if 0.0 < dt < 0.05:
            self.turn_integral += turn_err * dt
            if turn_leak > 0.0:
                self.turn_integral *= max(0.0, 1.0 - dt / turn_leak)
            self.turn_integral = max(-max_turn_int,
                                     min(max_turn_int, self.turn_integral))
        sync_correction = (wheel_sync_kp * turn_err
                           + self.get_parameter('wheel_sync_ki').value * self.turn_integral)
        if max_wheel_sync > 0.0:
            sync_correction = max(-max_wheel_sync,
                                  min(max_wheel_sync, sync_correction))

        # A hard deadband step can flip the motor direction at the PID zero crossing.
        # The optional ramp keeps the static-friction assist continuous and lets it
        # fade away once the wheels are already moving.  It must follow the actual
        # motor effort, not the angle error: a D-term can legitimately brake in the
        # opposite direction while the body is returning toward upright.
        compensation_allowed = (
            abs(err) >= deadband_err
            or (deadband_rate > 0.0 and abs(rate_for_control) >= deadband_rate))
        if deadband_dither_hz > 0.0:
            # Below the measured breakaway torque, use evenly distributed full-
            # amplitude pulses rather than a continuous kick.  The accumulator
            # gives the requested average torque without time-phase blind spots.
            # A configured hold time lets a pulse survive multiple IMU frames so
            # the motor driver can overcome static friction on the real floor.
            dither_deadband = deadband_fwd if u > 0.0 else deadband_bwd
            speed_ok = (deadband_velocity <= 0.0
                        or abs(self.wheel_velocity) < deadband_velocity)
            if (compensation_allowed and dither_deadband > 0.0 and speed_ok
                    and 0.0 < abs(u) < dither_deadband):
                duty = abs(u) / dither_deadband
                sign = 1.0 if u > 0.0 else -1.0
                if sign != self._dither_sign:
                    self._dither_accumulator = 0.0
                    self._dither_sign = sign
                    self._dither_hold_until = 0.0
                    self._dither_pulse_output = 0.0
                if self._dither_hold_until > t:
                    u = self._dither_pulse_output
                else:
                    self._dither_accumulator += duty
                    if self._dither_accumulator >= 1.0:
                        self._dither_accumulator -= 1.0
                        u = sign * dither_deadband
                        self._dither_pulse_output = u
                        self._dither_hold_until = t + max(0.0, deadband_dither_pulse_ms) / 1000.0
                    else:
                        u = 0.0
            else:
                self._dither_accumulator = 0.0
                self._dither_sign = 0.0
                self._dither_hold_until = 0.0
                self._dither_pulse_output = 0.0
        elif deadband_output_ramp > 0.0:
            # D-led corrections need static-friction assist before angle error grows.
            effort_scale = min(1.0, abs(u) / deadband_output_ramp)
            if deadband_velocity > 0.0:
                speed_scale = max(0.0, 1.0 - abs(self.wheel_velocity) / deadband_velocity)
            else:
                speed_scale = 1.0
            compensation_scale = (
                effort_scale * speed_scale if compensation_allowed else 0.0)
            if u > 0.0:
                u += deadband_fwd * compensation_scale
            elif u < 0.0:
                u -= deadband_bwd * compensation_scale
        elif deadband_ramp > 0.0:
            error_scale = min(1.0, abs(err) / deadband_ramp)
            if deadband_velocity > 0.0:
                speed_scale = max(0.0, 1.0 - abs(self.wheel_velocity) / deadband_velocity)
            else:
                speed_scale = 1.0
            compensation_scale = error_scale * speed_scale
            if err > 0.0:
                u += deadband_fwd * compensation_scale
            elif err < 0.0:
                u -= deadband_bwd * compensation_scale
        elif abs(err) > deadband_err:
            # Preserve the original thresholded behavior when no ramp is requested.
            if u > 0.0:
                u += deadband_fwd
            elif u < 0.0:
                u -= deadband_bwd

        # Keep small center corrections gentle, then add authority only when the
        # body is already escaping and the PID is pushing toward the same side.
        directional_recovery_pwm = (
            recovery_pwm_fwd if err > 0.0 else recovery_pwm_bwd)
        if directional_recovery_pwm < 0.0:
            directional_recovery_pwm = recovery_pwm
        if (directional_recovery_pwm > 0.0 and abs(err) > recovery_err
                and u * err > 0.0
                # Apply catch authority only while the body is escaping.  If it
                # is already rotating back toward upright, extra same-side PWM
                # would turn a recovery into the next oscillation.
                and err * rate_for_control >= 0.0):
            if recovery_ramp > 0.0:
                recovery_scale = min(1.0, (abs(err) - recovery_err) / recovery_ramp)
            else:
                recovery_scale = 1.0
            u += math.copysign(directional_recovery_pwm * recovery_scale, err)

        forward_limit = max_pwm_fwd if max_pwm_fwd > 0.0 else max_pwm
        backward_limit = max_pwm_bwd if max_pwm_bwd > 0.0 else max_pwm
        u = motor_sign * max(-backward_limit, min(forward_limit, u))
        if self.get_parameter('debug_control').value:
            if self.last_debug_t is None or t - self.last_debug_t >= 0.2:
                self.last_debug_t = t
                self.get_logger().info(
                    f'ctrl pitch={pitch:.2f} err={err:.2f} '
                    f'rate={self.pitch_rate_filtered:.2f}/{rate_for_control:.2f} '
                    f'vel={self.wheel_velocity:.1f}/{self.v_ref:.1f} '
                    f'wheel_d={wheel_damping:.1f} '
                    f'raw_vel={self.wheel_velocity_raw:.1f} '
                    f'L/R={self.left_wheel_velocity_raw:.1f}/{self.right_wheel_velocity_raw:.1f} '
                    f'turn_v={self.turn_velocity:.1f}/{self.turn_ref:.1f} '
                    f'trim={offset_trim:.2f} vint={self.velocity_integral:.0f} '
                    f'sync={sync_correction:.1f} pwm={u:.1f}')
        self._send_pwm(u, sync_correction)
        self._publish_state(pitch, err, offset_trim, effective_offset, u, sync_correction)

    def _send_pwm(self, u, sync_correction=0.0):
        # serial_bridge约定：left/right_pwm = (linear ∓ angular)*100
        cmd = Twist()
        cmd.linear.x = u / 100.0
        cmd.angular.z = sync_correction / 100.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(BalanceController())
    rclpy.shutdown()
