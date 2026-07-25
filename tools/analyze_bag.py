"""从rosbag2直接算平衡PID的量化指标，替代手工数cmd_vel日志行数的做法。

用法：
    pip install rosbags numpy
    python tools/analyze_bag.py results/rosbag/kp20_kd18_run1 [results/rosbag/kp25_kd20_run1 ...]
    python tools/analyze_bag.py results/rosbag/*_run1 --offset -1.2 --csv out.csv

参数名（kp/kd/kp_v）从bag文件夹名里正则提取，提取不到就显示为bag路径本身。
--offset 对应balance_controller里的pitch_offset_deg（机械零点），不传默认按0算，
    这时mean pitch本身就能看出零点偏了多少，可以据此回填offset再重跑。
"""
import argparse
import re
import math
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def pitch_from_quat(x, y, z, w):
    # 与balance_controller.py中_imu_cb的公式保持一致
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(math.asin(sinp))


def _read_via_mcap_fallback(path):
    """metadata.yaml缺失/空(录制被强制中断)时，绕过rosbag2索引直接解析mcap文件本身。"""
    from mcap_ros2.reader import read_ros2_messages

    mcaps = sorted(Path(path).glob('*.mcap'))
    if not mcaps:
        raise RuntimeError(f'{path}: 没有metadata.yaml，目录下也找不到.mcap文件')

    imu_t, imu_pitch = [], []
    cmd_t, cmd_pwm = [], []
    for mcap_file in mcaps:
        for msg in read_ros2_messages(str(mcap_file)):
            topic = msg.channel.topic
            if topic == '/imu/data':
                q = msg.ros_msg.orientation
                t = msg.ros_msg.header.stamp.sec + msg.ros_msg.header.stamp.nanosec * 1e-9
                imu_t.append(t)
                imu_pitch.append(pitch_from_quat(q.x, q.y, q.z, q.w))
            elif topic == '/cmd_vel':
                cmd_t.append(msg.log_time.timestamp())  # mcap_ros2返回datetime，非纳秒int
                cmd_pwm.append(msg.ros_msg.linear.x * 100.0)
    return (np.array(imu_t), np.array(imu_pitch)), (np.array(cmd_t), np.array(cmd_pwm))


def read_bag(path):
    meta = Path(path) / 'metadata.yaml'
    if not meta.exists() or meta.stat().st_size == 0:
        print(f'[提示] {path}: metadata.yaml缺失/为空(录制未正常收尾)，改用mcap原始解析（较慢）')
        return _read_via_mcap_fallback(path)

    imu_t, imu_pitch = [], []
    cmd_t, cmd_pwm = [], []

    with AnyReader([Path(path)]) as reader:
        conns = [c for c in reader.connections if c.topic in ('/imu/data', '/cmd_vel')]
        if not conns:
            raise RuntimeError(f'{path}: 没找到/imu/data或/cmd_vel，检查bag是否录对了话题')
        for connection, bag_ts, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == '/imu/data':
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                q = msg.orientation
                imu_t.append(t)
                imu_pitch.append(pitch_from_quat(q.x, q.y, q.z, q.w))
            else:  # /cmd_vel, Twist无header，只能用录制时间戳
                cmd_t.append(bag_ts * 1e-9)
                cmd_pwm.append(msg.linear.x * 100.0)

    return (np.array(imu_t), np.array(imu_pitch)), (np.array(cmd_t), np.array(cmd_pwm))


def zero_cross_rate(t, err):
    if len(err) < 2:
        return 0.0
    sign = np.sign(err)
    crossings = np.where(np.diff(sign) != 0)[0]
    duration = t[-1] - t[0]
    return len(crossings) / duration if duration > 0 else 0.0


def find_engaged_segments(imu_t, pitch, offset, engage_below, max_tilt):
    """复刻balance_controller._imu_cb里的接管状态机，从原始pitch序列切出真正被控制的时间段。

    录制时开始/停止录制往往会带上摆弄车身、倒地后躺一会儿的死时间，
    这些时间pitch可能很小(摆弄时凑巧接近直立)或很大(倒地不会被误判成已接管)，
    不做这个切分的话，会把死时间混进duration/pitch统计里，数字完全失真。
    """
    err = np.abs(pitch - offset)
    segments = []
    engaged = False
    seg_start = None
    for i in range(len(err)):
        if engaged and err[i] > max_tilt:
            engaged = False
            segments.append((seg_start, i - 1))
        elif not engaged and err[i] < engage_below:
            engaged = True
            seg_start = i
    if engaged:
        segments.append((seg_start, len(err) - 1))
    return segments


def estimate_offset(pitch, cluster_thresh=30.0):
    """从pitch分布里估计机械零点：车摔倒/平放时pitch会顶到大角度(该硬件上约85-90°)，
    真正接近直立的样本会聚成一个远低于该值的簇，取该簇中位数作为offset估计。
    仅在没有手动传--offset时兜底用，不代表运行时实际配置的pitch_offset_deg。
    """
    cluster = pitch[np.abs(pitch) < cluster_thresh]
    if len(cluster) == 0:
        return 0.0
    return float(np.median(cluster))


def analyze_one(path, offset, engage_below, max_tilt, auto_offset=False):
    (imu_t, pitch), (cmd_t, pwm) = read_bag(path)
    if len(pitch) == 0:
        raise RuntimeError(f'{path}: /imu/data没有数据')

    if auto_offset:
        offset = estimate_offset(pitch)

    segments = find_engaged_segments(imu_t, pitch, offset, engage_below, max_tilt)
    if not segments:
        raise RuntimeError(f'{path}: 全程没有进入"已接管"状态(pitch从未接近offset±{engage_below}°，offset={offset:.2f})，检查offset是否传对')

    results = []
    for seg_idx, (i0, i1) in enumerate(segments):
        seg_t = imu_t[i0:i1 + 1]
        seg_pitch = pitch[i0:i1 + 1]
        err = seg_pitch - offset
        duration = seg_t[-1] - seg_t[0] if len(seg_t) > 1 else 0.0

        tail_mask = seg_t >= (seg_t[-1] - min(10.0, duration / 2 if duration > 0 else 0))
        steady_err = err[tail_mask]

        # 该时间窗内对应的cmd_vel/pwm样本(用bag录制时间戳对齐)
        pwm_mask = (cmd_t >= seg_t[0]) & (cmd_t <= seg_t[-1])
        seg_pwm = pwm[pwm_mask]

        result = {
            'path': path,
            'segment': seg_idx,
            'offset_used': offset,
            'duration_s': duration,
            'n_imu': len(seg_pitch),
            'pitch_mean_deg': float(np.mean(seg_pitch)),
            'pitch_std_deg': float(np.std(err)),
            'pitch_absmax_deg': float(np.max(np.abs(err))),
            'pitch_rms_deg': float(np.sqrt(np.mean(err ** 2))),
            'zero_cross_hz': zero_cross_rate(seg_t, err),
            'steady_mean_deg': float(np.mean(steady_err)) if len(steady_err) else float('nan'),
            'steady_std_deg': float(np.std(steady_err)) if len(steady_err) else float('nan'),
        }
        if len(seg_pwm):
            result['pwm_std'] = float(np.std(seg_pwm))
            result['pwm_sat_pct'] = float(np.mean(np.abs(seg_pwm) >= 254.0) * 100.0)
        else:
            result['pwm_std'] = float('nan')
            result['pwm_sat_pct'] = float('nan')
        results.append(result)
    return results


def extract_label(path):
    name = Path(path).name
    m = re.search(r'kp(\d+)_kd(\d+)', name)
    if m:
        label = f'kp={m.group(1)}/kd={float(m.group(2)) / 10:.1f}'
        mi = re.search(r'_ki(\d+)', name)
        if mi:
            label += f'/ki={float(mi.group(1)) / 10:.1f}'
        return label
    m = re.search(r'kpv0*(\d+)', name)
    if m:
        return f'kp_v={float(m.group(1)) / 1000:.3f}'
    return name


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', help='一个或多个rosbag2目录（含metadata.yaml）')
    ap.add_argument('--offset', type=float, default=0.0, help='pitch_offset_deg，机械零点，默认0')
    ap.add_argument('--auto-offset', action='store_true',
                     help='不用--offset手动值，改为从每条bag的pitch分布里自动估计机械零点(取abs(pitch)<30°样本簇的中位数)')
    ap.add_argument('--engage-below', type=float, default=3.0, help='engage_below_deg，需跟balance_controller当次设置一致')
    ap.add_argument('--max-tilt', type=float, default=30.0, help='max_tilt_deg，需跟balance_controller当次设置一致')
    ap.add_argument('--csv', default=None, help='把结果另存为csv')
    args = ap.parse_args()

    rows = []
    for b in args.bags:
        try:
            segs = analyze_one(b, args.offset, args.engage_below, args.max_tilt, auto_offset=args.auto_offset)
            label = extract_label(b)
            for r in segs:
                r['label'] = label if len(segs) == 1 else f'{label}#{r["segment"]}'
                rows.append(r)
        except Exception as e:
            print(f'[跳过] {b}: {e}')

    rows.sort(key=lambda r: r['pitch_std_deg'])

    header = ('label', 'offset', 'duration_s', 'pitch_mean', 'pitch_std', 'pitch_absmax',
               'pitch_rms', 'zc_Hz', 'steady_mean', 'steady_std', 'pwm_std', 'sat%')
    print(('{:<20}' * 1 + '{:>10}' * 11).format(*header))
    for r in rows:
        print('{:<20}{:>10.2f}{:>10.1f}{:>10.2f}{:>10.2f}{:>10.2f}{:>10.2f}{:>10.2f}{:>10.2f}{:>10.2f}{:>10.1f}{:>10.1f}'.format(
            r['label'], r['offset_used'], r['duration_s'], r['pitch_mean_deg'], r['pitch_std_deg'],
            r['pitch_absmax_deg'], r['pitch_rms_deg'], r['zero_cross_hz'],
            r['steady_mean_deg'], r['steady_std_deg'], r['pwm_std'], r['pwm_sat_pct']))

    if args.csv:
        import csv
        keys = ['label', 'path', 'segment', 'offset_used', 'duration_s', 'n_imu', 'pitch_mean_deg', 'pitch_std_deg',
                 'pitch_absmax_deg', 'pitch_rms_deg', 'zero_cross_hz', 'steady_mean_deg',
                 'steady_std_deg', 'pwm_std', 'pwm_sat_pct']
        with open(args.csv, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f'\n已写入 {args.csv}')


if __name__ == '__main__':
    main()
