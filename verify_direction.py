"""方向一致性判定器：用一次"绕竖直轴转身"判定绕传感器 +X 轴的 180° 二义性。
Direction checker: one turnaround about the vertical axis settles the 180 degree ambiguity
about the sensor +X axis.

判据 / criterion
---------------
身体绕单位轴 n 转 θ 时，世界固定的向量（重力/比力）在传感器系中按 -θ 绕 n 变化。
把陀螺积分得到的旋转向量与比力变化反推出的旋转向量逐项比较：
  - 两者符号一致、大小接近  -> 当前传感器->身体映射成立；
  - 两者符号相反（夹角接近 180°）-> 绕传感器 +X 轴差了 180°，映射需要更正；
  - 大小差很多 -> 转动速度或时间区间标记不准，或陀螺刻度问题。
When the body rotates by theta about unit axis n, a world-fixed vector appears to rotate by
-theta about n in the sensor frame. Comparing the two rotation vectors separates the sign
question from the scale question.

用法 / usage
-----------
python verify_direction.py <packets.jsonl> --still1 a b --turn c d --still2 e f
时间用电脑接收相对秒（与终端 elapsed_s 一致）。
Times are host receive-relative seconds, matching the terminal output.
"""
import argparse
import json
import math
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


def window(rows, start, end):
    return [r for r in rows if start <= r['t'] < end]


def gyro_rotation_vector(rows, start, end, remove_bias=True):
    total = [0.0, 0.0, 0.0]
    previous = None
    used = 0
    for row in window(rows, start, end):
        rate = row['w']
        if remove_bias:
            rate = tuple(rate[i] - SENSOR_BIAS[i] for i in range(3))
        if previous is not None:
            dt = row['t'] - previous
            if 0 < dt <= 0.2:
                for i in range(3):
                    total[i] += rate[i] * dt
                used += 1
        previous = row['t']
    return total, used


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('packets', type=Path)
    parser.add_argument('--still1', nargs=2, type=float, required=True, metavar=('START', 'END'))
    parser.add_argument('--turn', nargs=2, type=float, required=True, metavar=('START', 'END'))
    parser.add_argument('--still2', nargs=2, type=float, required=True, metavar=('START', 'END'))
    args = parser.parse_args()

    rows = load_packets(args.packets)
    print(f'文件 / file: {args.packets}')
    print(f'包数 / packets: {len(rows)}, 跨度 / span: {rows[-1]["t"]:.2f} s')
    print()

    still1 = window(rows, *args.still1)
    still2 = window(rows, *args.still2)
    if not still1 or not still2:
        raise SystemExit('静止区间为空，检查时间标记 / a still interval is empty; check the marks')

    a1 = mean([r['a'] for r in still1])
    a2 = mean([r['a'] for r in still2])
    rate1 = mean([r['w'] for r in still1])
    rate2 = mean([r['w'] for r in still2])

    print('=== 静止段 / still intervals ===')
    for label, seg, a, rate in (('still1', still1, a1, rate1), ('still2', still2, a2, rate2)):
        residual = norm([rate[i] - SENSOR_BIAS[i] for i in range(3)])
        print(f'{label:7s} n={len(seg):4d} |a|={norm(a):8.4f} '
              f'unit=({unit(a)[0]:+.4f}, {unit(a)[1]:+.4f}, {unit(a)[2]:+.4f}) '
              f'陀螺残余 / gyro residual={residual:.5f} rad/s')
    print()

    # 比力变化反推转动：世界固定向量在传感器系中按 -theta 绕 n 变化，故取反得到等价转轴。
    # The gravity change implies a rotation; negating it yields the equivalent rotation vector.
    delta = tuple(a2[i] - a1[i] for i in range(3))
    implied_axis_raw = cross(unit(a1), unit(a2))
    change_angle = angle_between(a1, a2)
    print('=== 比力方向变化 / specific-force change ===')
    print(f'变化角 / change angle = {change_angle:.3f} deg')
    if norm(implied_axis_raw) < 1e-9:
        raise SystemExit('两次静止比力方向几乎相同，无法判定；转动轴可能平行于重力，请重做')
    implied_axis = unit(implied_axis_raw)
    # 反推转动角：cos(change) = 1 - (1-cos(theta)) * (1 - (n.u)^2)
    cos_alpha = abs(dot(implied_axis, unit(a1)))
    denominator = 1.0 - cos_alpha ** 2
    implied_angle = None
    if denominator > 1e-9:
        cos_theta = 1.0 - (1.0 - math.cos(math.radians(change_angle))) / denominator
        if -1.0 <= cos_theta <= 1.0:
            implied_angle = math.degrees(math.acos(cos_theta))
    print(f'变化轴 / change axis = ({implied_axis[0]:+.4f}, {implied_axis[1]:+.4f}, {implied_axis[2]:+.4f})')
    print(f'该轴与 still1 比力的夹角 / angle to still1 specific force = '
          f'{math.degrees(math.acos(max(-1.0, min(1.0, cos_alpha)))):.2f} deg')
    print(f'反推转动角 / implied rotation angle = '
          f'{implied_angle:.3f} deg' if implied_angle is not None else '反推转动角不可用')
    print()

    total, used = gyro_rotation_vector(rows, *args.turn)
    gyro_angle = norm(total)
    print('=== 陀螺积分 / gyro integral ===')
    print(f'积分区间 / interval = {args.turn[0]}-{args.turn[1]} s, 参与样本 / samples = {used}')
    print(f'积分旋转向量 / rotation vector = ({total[0]:+.5f}, {total[1]:+.5f}, {total[2]:+.5f}) rad')
    print(f'积分角 / angle = {math.degrees(gyro_angle):.3f} deg')
    if gyro_angle < 1e-9:
        raise SystemExit('陀螺积分为零，转动区间标记有误 / the turn interval looks wrong')
    gyro_axis = unit(total)
    print(f'积分轴 / gyro axis = ({gyro_axis[0]:+.4f}, {gyro_axis[1]:+.4f}, {gyro_axis[2]:+.4f})')
    print()

    print('=== 判定 / verdict ===')
    axis_angle = angle_between(gyro_axis, implied_axis)
    expected_axis = tuple(-x for x in gyro_axis)
    agreement = angle_between(expected_axis, implied_axis)
    print(f'陀螺轴与比力变化轴夹角 / angle between the two axes = {axis_angle:.2f} deg')
    print(f'若按"世界系向量按 -θ 旋转"约定，期望夹角 / expected under that convention = '
          f'{agreement:.2f} deg')
    if agreement <= 30.0:
        print('结论 / verdict: 符号与手性一致，当前 sensor -> body 映射成立。')
        print('             Signs and handedness agree; the current sensor-to-body map holds.')
    elif axis_angle <= 30.0:
        print('结论 / verdict: 两者同向，说明世界系向量按 +θ 旋转，映射需要整体取反。')
        print('             They point the same way, so the mapping needs an overall sign flip.')
    else:
        print('结论 / verdict: 两轴既不平行也不反向，转动区间或动作不干净，请重做并保持单轴。')
        print('             The axes are neither parallel nor opposed; redo the turn as a single axis.')
    scale = None
    if implied_angle and implied_angle > 5.0:
        scale = math.degrees(gyro_angle) / implied_angle
        print(f'幅度比 / gyro angle vs implied angle = {math.degrees(gyro_angle):.2f} / '
              f'{implied_angle:.2f} = {scale:.3f}  (期望接近 1.0 / expect about 1.0)')
    if norm(cross(gyro_axis, (1.0, 0.0, 0.0))) < 0.3:
        print('提示 / note: 本次转动的轴接近传感器 +X，因此这次数据可以直接判定绕 +X 的 180°。')
        print('             The turn axis is close to sensor +X, so this recording addresses the +X ambiguity.')
    else:
        print('提示 / note: 本次转动轴不接近传感器 +X，若要判定 +X 的 180°，'
              '需再做一次绕竖直轴（佩戴）或绕 +X 的转动。')
        print('             The axis is not close to sensor +X; another turn is needed for that ambiguity.')


if __name__ == '__main__':
    main()
