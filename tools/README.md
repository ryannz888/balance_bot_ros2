# 调参分析工具

2026-07-25 调参过程中使用的离线分析脚本。用法与结论见
`docs/06_PID调参完全复盘_中文.md`。

依赖：`pip install rosbags numpy`

工作流：

```bash
# 1. Pi 上录包
ros2 bag record -o myrun /imu/data /cmd_vel /wheel/encoders

# 2. 拉回本地
scp -r pi:~/ros2_ws/bags/myrun results/rosbag/

# 3. 分析
cd tools
python analyze_run.py ../results/rosbag/myrun 7.53 3.0 12.0 600 5.0
```

## analyze_run.py

主力工具。离线复现控制器状态机，逐段输出指标。

```
参数：<bag路径> [offset] [engage_below] [max_tilt] [max_wheel_v] [最短报告时长]
```

输出两张表。第一张按段列出：时长、**退出原因**（`TILT` 倾角超限 / `WHEEL` 轮速超限 /
`END` 录制结束时仍在平衡）、误差范围、平均 PWM、饱和率、**带符号**平均轮速、
最大轮速、过零次数、平均差模轮速（偏航偏置）。

第二张是位置环诊断：位置峰峰值与标准差（单位：轮周）、摆动周期、
速度与误差的 RMS。摆动周期用 0.4 秒滑动平均后的轮速过零来数，
直接数原始轮速会数到噪声（会得到 6~9 Hz 的假结果）。

最后是各 PWM 区间的轮子移动率，用于粗看静摩擦。

## true_offset.py

从平衡数据反推真实机械平衡点，替代静态标定。

```
参数：<bag路径> <当前offset> <kp_v> <ki_v> [velocity_filter_alpha]
```

原理：车在平衡且长时间平均轮速为零时，平均 pitch 就是真实平衡点。
同时重建控制器内部的 `velocity_integral` 并报告其均值与限幅占比——
**零点精修到 0.2° 以下时应以积分均值为判据**（应接近 0 且不限幅），
平均 pitch 在这个尺度上已是 run-to-run 噪声。

## breakaway.py

测负载下的静摩擦突破电压，分正反方向。可同时传多个 bag 合并统计。

```
参数：<bag路径> [更多bag路径...]
```

**关键实现**：只统计"指令已稳定"的样本（与 100 ms 前相差 <15 PWM），
排除 PWM 正在爬升的瞬态。不加这个过滤会系统性低估阈值——
最初正是因此误判"静摩擦不是瓶颈"。

## jerk_trace.py

打印轮速极值时刻前后 2 秒的原始逐点轨迹（pitch / err / rate / pwm / vel / turn）。

```
参数：<bag路径> <offset>
```

聚合统计看不见的问题要靠它。stick-slip 极限环就是用它诊断出来的：
持续给 -45 PWM 而轮速恒为 0，车身自己前倾，PWM 追到 116 才挣脱，
随后暴冲到 341 ticks/s 并过冲到 -2.07°。
