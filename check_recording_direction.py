"""自动判定方向：自动找出静止段与转动段，再跑 verify_direction 的判据。
Automatic direction check: locate the still and motion intervals automatically, then apply the
same criterion as verify_direction.

用法 / usage
-----------
python check_recording_direction.py <live_recordings/时间戳_live 目录>

不需要手填时间；静止段由"陀螺残余小且比力模长稳定"自动识别。
No manual time marks: still intervals are found from small residual gyro and stable |a|.
"""
import argparse
import json
import math
import statistics as st
from pathlib import Path

SENSOR_BIAS = (-0.020495493656645894, -0.006844256493084482, -0.00866488206724287)


def load_packets(path):
    rows = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        packet = record.get('packet') or {}
        if packet.get('type') != 'imu':
            continue
        data = packet['data']
        rows.append(dict(t=record['arrival_monotonic_ns'] / 1e9,
                         a=(data['ax_mps2'], data['ay_mps2'], data['az_mps2']),
                         w=(data['gx_radps'], data['gy_radps'], data['gz_radps'])))
    rows.sort(key=lambda r: r['t'])
    start = rows[0]['t']
    for row in rows:
        row['t'] -= start
    return rows


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def unit(v):
    n = norm(v)
    return tuple(x / n for x in v)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def angle_between(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(unit(a), unit(b))))))


def mean(vectors):
    n = len(vectors)
    return tuple(sum(v[i] for v in vectors) / n for i in range(3))


def find_plateaus(rows, window_s=0.5, residual_limit=0.02, sd_limit=0.12, min_length=1.5):
    """返回静止平台 [(start, end, mean_a, mean_w)]。/ Return still plateaus."""
    plateaus = []
    current = []
    step = window_s
    t = 0.0
    while t < rows[-1]['t']:
        segment = [r for r in rows if t <= r['t'] < t + step]
        t += step
        if not segment:
            continue
        residual = norm([mean([r['w'] for r in segment])[i] - SENSOR_BIAS[i] for i in range(3)])
        sd = st.pstdev([norm(r['a']) for r in segment])
        if residual < residual_limit and sd < sd_limit:
            if current and abs(t - step - current[-1][1]) > 1e-9:
                current = []
            current.append((t - step, t))
        else:
            if current and current[-1][1] - current[0][0] >= min_length:
                plateaus.append((current[0][0], current[-1][1]))
            current = []
    if current and current[-1][1] - current[0][0] >= min_length:
        plateaus.append((current[0][0], current[-1][1]))
    result = []
    for start, end in plateaus:
        segment = [r for r in rows if start <= r['t'] < end]
        result.append((start, end, mean([r['a'] for r in segment]), mean([r['w'] for r in segment])))
    return result


def gyro_integral(rows, start, end):
    total = [0.0, 0.0, 0.0]
    previous = None
    used = 0
    for row in rows:
        if not (start <= row['t'] < end):
            continue
        if previous is not None:
            dt = row['t'] - previous
            if 0 < dt <= 0.2:
                for i in range(3):
                    total[i] += (row['w'][i] - SENSOR_BIAS[i]) * dt
                used += 1
        previous = row['t']
    return total, used


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('session', type=Path, help='live_recordings/..._live 目录或 packets.jsonl')
    args = parser.parse_args()

    path = args.session / 'packets.jsonl' if args.session.is_dir() else args.session
    rows = load_packets(path)
    print(f'文件 / file: {path}')
    print(f'包数 / packets: {len(rows)}, 跨度 / span: {rows[-1]["t"]:.2f} s')
    session_json = path.parent / 'session.json'
    if session_json.exists():
        notes = json.loads(session_json.read_text(encoding='utf-8')).get('notes')
        print(f'实验说明 / notes: {notes}')
    print()

    plateaus = find_plateaus(rows)
    print(f'=== 自动识别的静止平台 / detected still plateaus ({len(plateaus)}) ===')
    for index, (start, end, a, w) in enumerate(plateaus):
        print(f'[{index}] {start:6.2f}-{end:6.2f}s  |a|={norm(a):8.4f} '
              f'unit=({unit(a)[0]:+.4f}, {unit(a)[1]:+.4f}, {unit(a)[2]:+.4f}) '
              f'陀螺残余 / gyro residual={norm([w[i] - SENSOR_BIAS[i] for i in range(3)]):.5f}')
    if len(plateaus) < 2:
        raise SystemExit('少于两个静止平台，无法判定 / fewer than two plateaus; cannot judge')
    print()

    print('=== 相邻静止平台之间的转动判定 / verdict per motion interval ===')
    verdicts = []
    for index in range(len(plateaus) - 1):
        start = plateaus[index][1]
        end = plateaus[index + 1][0]
        a1 = plateaus[index][2]
        a2 = plateaus[index + 1][2]
        total, used = gyro_integral(rows, start, end)
        gyro_angle = norm(total)
        if gyro_angle < math.radians(2.0) or norm(cross(unit(a1), unit(a2))) < 1e-9:
            print(f'[{index}] {start:.2f}-{end:.2f}s: 无有效转动或转动轴平行于重力，跳过')
            continue
        gyro_axis = unit(total)
        change_axis = unit(cross(unit(a1), unit(a2)))
        change_angle = angle_between(a1, a2)
        cos_alpha = abs(dot(change_axis, unit(a1)))
        implied = None
        if 1.0 - cos_alpha ** 2 > 1e-9:
            cos_theta = 1.0 - (1.0 - math.cos(math.radians(change_angle))) / (1.0 - cos_alpha ** 2)
            if -1.0 <= cos_theta <= 1.0:
                implied = math.degrees(math.acos(cos_theta))
        negated = tuple(-x for x in gyro_axis)
        agree = angle_between(negated, change_axis)
        same = angle_between(gyro_axis, change_axis)
        print(f'[{index}] 转动区间 / motion {start:6.2f}-{end:6.2f}s, 样本 / samples={used}')
        print(f'    陀螺角 / gyro angle = {math.degrees(gyro_angle):8.3f} deg, '
              f'轴 / axis = ({gyro_axis[0]:+.4f}, {gyro_axis[1]:+.4f}, {gyro_axis[2]:+.4f})')
        print(f'    比力变化轴 / specific-force change axis = '
              f'({change_axis[0]:+.4f}, {change_axis[1]:+.4f}, {change_axis[2]:+.4f}), '
              f'变化角 / change angle = {change_angle:7.3f} deg')
        if implied:
            print(f'    反推转动角 / implied rotation angle = {implied:8.3f} deg, '
                  f'幅度比 / ratio = {math.degrees(gyro_angle) / implied:.3f}')
        print(f'    夹角：陀螺轴 vs 变化轴 = {same:7.2f} deg；取反后 = {agree:7.2f} deg')
        if agree <= 30.0:
            state = 'consistent'
            print('    判定 / verdict: 符号与手性一致 / signs and handedness agree')
        elif same <= 30.0:
            state = 'flipped'
            print('    判定 / verdict: 同向，说明世界系向量按 +θ 旋转，映射需整体取反 / needs a sign flip')
        else:
            state = 'unclear'
            print('    判定 / verdict: 两轴不平行也不反向，转动不干净 / the turn was not a single clean axis')
        verdicts.append((index, start, end, gyro_axis, same, agree, state, implied,
                         math.degrees(gyro_angle)))
    print()

    # 绕传感器 +X 的 180° 判定：需要一次转动轴接近 +X 的干净单轴转动。
    # The +X 180 degree question needs one clean single-axis motion whose axis is near sensor +X.
    candidates = [v for v in verdicts if abs(v[3][0]) > 0.9 and v[6] == 'consistent']
    print('=== 绕传感器 +X 的 180° 判定 / the sensor +X 180 degree question ===')
    if candidates:
        for index, start, end, axis, same, agree, state, implied, gyro_angle in candidates:
            print(f'[合格 / qualified] 转动 {start:.2f}-{end:.2f}s，轴接近 +X '
                  f'({axis[0]:+.3f})，陀螺角 {gyro_angle:.2f} deg')
            if implied:
                print(f'    陀螺角 {gyro_angle:.2f} vs 反推角 {implied:.2f} deg，'
                      f'误差 {gyro_angle - implied:+.2f} deg ({(gyro_angle / implied - 1) * 100:+.2f}%)')
            print('    该区间的符号一致即说明绕 +X 的 180° 取当前映射；若取反则相反。')
    else:
        print('当前录制没有轴接近传感器 +X 的干净单轴转动，因此本次无法判定该 180°。')
        print('需要按实验步骤做一次"绕传感器 +X 转约 90°"的录制（桌面或佩戴均可）。')
        print('No clean single-axis motion about sensor +X was found, so this recording cannot settle')
        print('the 180 degree question. Record one motion about sensor +X (about 90 degrees).')


if __name__ == '__main__':
    main()
