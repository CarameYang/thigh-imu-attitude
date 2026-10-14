"""只读分析：慢转体验证陀螺积分与重力方向的一致性，并给出安装倾角。
Read-only analysis: check gyro-integration vs gravity-direction consistency for the
slow body rotation, and report the approximate mounting tilt.

不修改任何原始录制；输出只写到本目录。
Does not modify any recording; outputs are written only into this folder.
"""
import json
import math
from pathlib import Path

SESSION = Path(__file__).resolve().parents[2] / 'live_recordings/20261012_003433_121271_live'
BIAS = (-0.020495493656645894, -0.006844256493084482, -0.00866488206724287)
OUT = Path(__file__).resolve().parent

# 用户口述的动作顺序与时长（秒）；起始时刻待数据核对。
# User-reported action order and durations (s); the start offset is checked against data.
NOMINAL = [(0, 20, 'stand_still'), (20, 30, 'slow_rotate_90deg'), (30, 40, 'still'),
           (40, 50, 'walk'), (50, 70, 'stand_again')]


def load(session):
    rows = []
    for line in (session / 'packets.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        packet = rec.get('packet') or {}
        if packet.get('type') != 'imu':
            continue
        data = packet['data']
        rows.append(dict(
            t=rec['arrival_monotonic_ns'] / 1e9,
            seq=packet.get('sequence_number'),
            node_us=packet.get('timestamp_us'),
            a=(data['ax_mps2'], data['ay_mps2'], data['az_mps2']),
            w=(data['gx_radps'], data['gy_radps'], data['gz_radps']),
        ))
    rows.sort(key=lambda r: r['t'])
    t0 = rows[0]['t']
    for r in rows:
        r['t'] -= t0
    return rows


def mean(vectors):
    n = len(vectors)
    return [sum(v[i] for v in vectors) / n for i in range(3)]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def unit(v):
    n = norm(v)
    return [x / n for x in v]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def angle(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(unit(a), unit(b))))))


def window(rows, start, end):
    return [r for r in rows if start <= r['t'] < end]


def segment_report(rows, start, end, label):
    seg = window(rows, start, end)
    if not seg:
        return None
    a = mean([r['a'] for r in seg])
    w = mean([r['w'] for r in seg])
    residual = [w[i] - BIAS[i] for i in range(3)]
    ua = unit(a)
    result = dict(
        label=label, start=start, end=end, n=len(seg),
        accel_mean=[round(x, 4) for x in a], accel_norm=round(norm(a), 4),
        accel_unit=[round(x, 4) for x in ua],
        angle_to_plus_x_deg=round(math.degrees(math.acos(max(-1, min(1, ua[0])))), 2),
        angle_to_plus_z_deg=round(math.degrees(math.acos(max(-1, min(1, ua[2])))), 2),
        gyro_mean=[round(x, 5) for x in w],
        gyro_bias_removed_norm=round(norm(residual), 5),
    )
    return result


def integrate(rows, start, end, step_max=0.2):
    """积分角速度得到总旋转向量（度）。Integrate angular rate into a total rotation vector (deg)."""
    total = [0.0, 0.0, 0.0]
    previous = None
    used = 0
    gaps = 0
    for r in window(rows, start, end):
        if previous is not None:
            dt = r['t'] - previous
            if 0 < dt <= step_max:
                for i in range(3):
                    total[i] += r['w'][i] * dt
                used += 1
            else:
                gaps += 1
        previous = r['t']
    return [math.degrees(x) for x in total], used, gaps


def main():
    rows = load(SESSION)
    lines = []
    def emit(text=''):
        print(text)
        lines.append(text)

    emit(f'session: {SESSION.name}')
    emit(f'packets: {len(rows)}  span: {rows[-1]["t"]:.3f} s  '
         f'median interval: {sorted(rows[i]["t"]-rows[i-1]["t"] for i in range(1, len(rows)))[len(rows)//2]*1000:.3f} ms')
    emit()

    emit('=== 1 s windows over the whole recording (找转体段 / locate the rotation) ===')
    emit(f'{"t":>6} {"|w|":>7} {"wX":>7} {"wY":>7} {"wZ":>7} {"|a|":>7} {"aX":>7} {"aY":>7} {"aZ":>7}')
    for k in range(int(rows[-1]['t'])):
        seg = window(rows, k, k + 1)
        if not seg:
            continue
        w = mean([r['w'] for r in seg])
        a = mean([r['a'] for r in seg])
        emit(f'{k:6d} {norm(w):7.4f} {w[0]:7.4f} {w[1]:7.4f} {w[2]:7.4f} '
             f'{norm(a):7.3f} {a[0]:7.3f} {a[1]:7.3f} {a[2]:7.3f}')
    emit()

    emit('=== user-reported phases on the nominal time base ===')
    reports = []
    for start, end, label in NOMINAL:
        rep = segment_report(rows, start, end, label)
        if rep:
            reports.append(rep)
            emit(f'{label:16s} {start:>3}-{end:<3}s n={rep["n"]:5d} |a|={rep["accel_norm"]:7.4f} '
                 f'unit={rep["accel_unit"]} vs+X={rep["angle_to_plus_x_deg"]:6.2f}deg vs+Z={rep["angle_to_plus_z_deg"]:6.2f}deg '
                 f'|w_bias_removed|={rep["gyro_bias_removed_norm"]:.5f} rad/s')
    emit()

    still_a = segment_report(rows, 0, 20, 'stand_still')
    still_b = segment_report(rows, 30, 40, 'still_after_rotation')
    still_c = segment_report(rows, 50, 70, 'stand_again')
    if still_a and still_b and still_c:
        emit('=== gravity direction changes between still segments ===')
        for x, y in ((still_a, still_b), (still_b, still_c), (still_a, still_c)):
            ax = cross(unit(x['accel_mean']), unit(y['accel_mean']))
            ax_u = unit(ax) if norm(ax) > 1e-9 else [0, 0, 0]
            emit(f'{x["label"]} -> {y["label"]}: '
                 f'delta={angle(x["accel_mean"], y["accel_mean"]):6.2f} deg, '
                 f'axis=({ax_u[0]:+.3f}, {ax_u[1]:+.3f}, {ax_u[2]:+.3f})')
        emit()

        emit('=== gyro integration vs gravity change (consistency check) ===')
        for x, y, label in ((still_a, still_b, '0-20s -> 30-40s'),
                            (still_b, still_c, '30-40s -> 50-70s')):
            theta, used, gaps = integrate(rows, x['end'], y['start'])
            emit(f'{label}: integrated over {x["end"]:.0f}-{y["start"]:.0f} s (n={used}, gaps={gaps})')
            emit(f'   gyro total rotation vector (deg) = [{theta[0]:+8.3f}, {theta[1]:+8.3f}, {theta[2]:+8.3f}] '
                 f'|total|={norm(theta):8.3f}')
            axis = cross(unit(x['accel_mean']), unit(y['accel_mean']))
            axis_u = unit(axis) if norm(axis) > 1e-9 else [0, 0, 0]
            emit(f'   gravity-change angle   = {angle(x["accel_mean"], y["accel_mean"]):8.3f} deg '
                 f'about ({axis_u[0]:+.3f}, {axis_u[1]:+.3f}, {axis_u[2]:+.3f})')
            emit(f'   projection of gyro integration onto that axis = {dot(theta, axis_u):+8.3f} deg')
        emit()

    emit('=== mounting tilt of the still segments relative to the reported +X-up direction ===')
    for rep in (still_a, still_b, still_c):
        if not rep:
            continue
        u = rep['accel_unit']
        tilt_y = math.degrees(math.atan2(u[2], u[0]))   # 绕 +Y 的倾角 / tilt about +Y
        tilt_z = math.degrees(math.atan2(u[1], u[0]))   # 绕 +Z 的倾角 / tilt about +Z
        emit(f'{rep["label"]:16s} unit=({u[0]:+.4f}, {u[1]:+.4f}, {u[2]:+.4f})  '
             f'total tilt from +X={rep["angle_to_plus_x_deg"]:6.2f} deg  '
             f'tilt about +Y={tilt_y:+7.2f} deg  tilt about +Z={tilt_z:+7.2f} deg')
    emit()

    emit('=== accel magnitude comparison (scale-error check) ===')
    for name, value in (('2026-09-23 flat +Z down', 9.8800), ('2026-09-23 upright +X up', 9.7508),
                        ('2026-10-11 233202 still', 9.7418), ('2026-10-12 003433 stand_still', still_a['accel_norm']),
                        ('2026-10-12 003433 stand_again', still_c['accel_norm'])):
        emit(f'   {name:32s} |a| = {value:.4f} m/s2')
    emit(f'   (reference local gravity 9.7803-9.8070 m/s2; deviation shown in the report)')

    (OUT / 'report.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    (OUT / 'report.json').write_text(json.dumps(dict(
        session=SESSION.name, packets=len(rows), span_s=round(rows[-1]['t'], 3),
        phases=reports, model_bias=BIAS), ensure_ascii=False, indent=2), encoding='utf-8')
    emit()
    emit(f'written: {OUT / "report.txt"} and report.json')


if __name__ == '__main__':
    main()
