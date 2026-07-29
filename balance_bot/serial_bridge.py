import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        # 编码器一圈计数值：手转轮子整一圈看/wheel/encoders计数差来标定
        self.declare_parameter('ticks_per_rev', 1320.0)
        # 若手转轮子"向前滚"时计数变负，把对应invert设为True
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('cmd_timeout_s', 0.25)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.left_sign = -1.0 if self.get_parameter('invert_left').value else 1.0
        self.right_sign = -1.0 if self.get_parameter('invert_right').value else 1.0
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.last_cmd_t = time.monotonic()

        self.enc_pub = self.create_publisher(String, '/wheel/encoders', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        # by-id稳定路径：不受USB枚举顺序影响（ttyACM0/1漂移曾导致读到错误设备）
        self.port = ('/dev/serial/by-id/'
                     'usb-Arduino_Srl_Arduino_Mega_85431303636351613122-if00')
        self.ser = serial.Serial(port=self.port, baudrate=115200, timeout=1.0)
        # The Mega's USB serial interface needs DTR/RTS asserted after reconnect.
        self.ser.dtr = True
        self.ser.rts = True
        time.sleep(2.0)
        # Balance control publishes faster than the serial actuator needs.  Keep
        # only the latest setpoint so a scheduler hiccup cannot replay old PWM.
        self.create_subscription(Twist, '/cmd_vel', self._cb, 1)
        self.create_timer(0.05, self._watchdog_cb)
        self._send_stop()
        self.get_logger().info('serial_bridge started')
        self._read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._read_thread.start()

    def _cb(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        left_pwm = int((linear - angular) * 100)
        right_pwm = int((linear + angular) * 100)
        # 实测(2026-07-19)：电机线束与编码器同样整组左右对调，且第一路对物理右轮极性相反
        # 故协议映射为 field1=-右轮PWM, field2=+左轮PWM（i/j键实验鉴别，见docs/04）
        self._send_motor(left_pwm, right_pwm)
        self.last_cmd_t = time.monotonic()

    def _send_motor(self, left_pwm, right_pwm):
        if self.ser is None:
            return
        cmd = f'M,{-right_pwm},{left_pwm}\n'
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'write failed: {e}')
            self._drop()

    def _send_stop(self):
        if self.ser is None:
            return
        try:
            self.ser.write(b'M,0,0\n')
        except Exception:
            self._drop()

    def _drop(self):
        """Let go of a dead handle so the reader can reopen the device.

        A USB serial adapter that re-enumerates leaves this node writing to a
        closed descriptor.  The Arduino stops receiving commands and its own
        watchdog zeroes the motors, which is the right outcome, but the node
        never recovers on its own and the robot stays dead until someone
        restarts it.  Seen on the IMU's adapter, which came back as ttyUSB1
        after a reboot while the node still held ttyUSB0.
        """
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _watchdog_cb(self):
        # A stale command must never leave the Arduino driving the last PWM.
        if time.monotonic() - self.last_cmd_t >= self.cmd_timeout_s:
            self._send_stop()
            self.last_cmd_t = time.monotonic()

    def destroy_node(self):
        try:
            self._send_stop()
        finally:
            super().destroy_node()

    def _read_serial(self):
        while True:
            if self.ser is None:
                time.sleep(1.0)
                try:
                    self.ser = serial.Serial(port=self.port, baudrate=115200,
                                             timeout=1.0)
                    self.ser.dtr = True
                    self.ser.rts = True
                    time.sleep(2.0)
                    self.get_logger().info(f'reopened {self.port}')
                except Exception:
                    continue
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception as e:
                self.get_logger().error(f'read failed: {e}')
                self._drop()
                continue
            if not line.startswith('E,'):
                continue
            parts = line.split(',')
            if len(parts) != 3:
                continue
            # 实测(2026-07-19)：固件的"左/右"与URDF定义相反，在此对调归一
            # 对调后 /wheel/encoders 和 /joint_states 的左右均为真实左右
            parts[1], parts[2] = parts[2], parts[1]
            msg = String()
            msg.data = f'{parts[1]},{parts[2]}'
            self.enc_pub.publish(msg)

            # 编码器计数 → 轮子转角(rad)，发布/joint_states驱动RViz模型轮子
            # 降频到20Hz：100Hz的TF流会压垮WiFi热点链路，可靠传输排队导致RViz回放延迟
            self._js_decim = getattr(self, '_js_decim', 0) + 1
            if self._js_decim % 5:
                continue
            try:
                l_count = int(parts[1])
                r_count = int(parts[2])
            except ValueError:
                continue
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['left_wheel_joint', 'right_wheel_joint']
            js.position = [
                self.left_sign * 2.0 * math.pi * l_count / self.ticks_per_rev,
                self.right_sign * 2.0 * math.pi * r_count / self.ticks_per_rev,
            ]
            self.joint_pub.publish(js)


def main():
    rclpy.init()
    rclpy.spin(SerialBridge())
    rclpy.shutdown()
