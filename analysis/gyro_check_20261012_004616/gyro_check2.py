"""只读分析（第二轮）：用陀螺积分旋转预测终止静止姿态，与实测姿态逐分量比较。
Read-only analysis, second pass: apply the gyro-integrated rotation to the initial
static pose and compare the predicted final pose with the measured one.

结论用于判断 (a) 陀螺符号/尺度是否可信，(b) 静止段的重力模长是否存在比例误差。
Conclusion covers (a) gyro sign/scale credibility and (b) static gravity-magnitude scale error.

不修改原始录制；输出只写到本目录。
Does not modify the recording; outputs are written only into this folder.
"""
import json
import math
import statistics as st
from pathlib import Path

SESSION = Path(__file__).resolve().parents[2] / 'live_recordings/20261012_004616_059007_live'
OUT = Path(__file__).resolve().parent

# 依据逐秒表与用户口述确定的静止平台与转动区间。
# Static plateaus and motion intervals identified from the per-second table and the user's report.
STILL = {'A: 2-9 s (start)': (2.0, 9.5),
         'B: 22-29 s (after 1st turn)': (22.0, 29.5),
         'C: 45-59 s (after 2nd turn)': (45.0, 59.5)}
MOTION = {'1st: 10-21 s (user: about 10 s)': (9.5, 22.0),
          '2nd: 30-44 s (user: about 14 s)': (29.5, 45.0)}


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
        rows.append(dict(t=rec['arrival_monotonic_ns'] / 1e9,
                         a=(data['ax_mps2'], data['ay_mps2'], data['az_mps2']),
                         w=(data['gx_radps'], data['gy_radps'], data['gz_radps'])))
    rows.sort(key=lambda r: r['t'])
    t0 = rows[0]['t']
    for r in rows:
        r['t'] -= t0
    return rows


def mean(v):
    n = len(v)
    return [sum(x[i] for x in v) / n for i in range(3)]


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


def rotate(vector, axis, angle_rad):
    """Rodrigues 旋转 / Rodrigues rotation of `vector` about unit `axis`."""
    n = unit(axis)
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return [vector[i] * c + cross(n, vector)[i] * s + n[i] * dot(n, vector) * (1 - c) for i in range(3)]


def integrate(rows, start, end):
    # 变量名不可与调用处复用，否则积分结果会被覆盖。
    # Keep this local name distinct so the caller cannot overwrite the integral.
    accumulated = [0.0, 0.0, 0.0]
    prev = None
    used = gaps = 0
    for r in rows:
        if not (start <= r['t'] < end):
            continue
        if prev is not None:
            dt = r['t'] - prev
            if 0 < dt <= 0.2:
                for i in range(3):
                    accumulated[i] += r['w'][i] * dt
                used += 1
            else:
                gaps += 1
        prev = r['t']
    return accumulated, used, gaps


def main():
    rows = load(SESSION)
    lines = []
    def emit(text=''):
        print(text)
        lines.append(text)

    emit(f'session: {SESSION.name}')
    emit()

    poses = {}
    emit('=== static plateaus: mean accel, unit direction, magnitude ===')
    for label, (start, end) in STILL.items():
        seg = [r for r in rows if start <= r['t'] < end]
        a = mean([r['a'] for r in seg])
        w = mean([r['w'] for r in seg])
        poses[label] = a
        emit(f'{label:32s} n={len(seg):4d} |a|={norm(a):7.4f}  sd|a|={st.pstdev([norm(r["a"]) for r in seg]):.4f}  '
             f'unit=({unit(a)[0]:+.4f}, {unit(a)[1]:+.4f}, {unit(a)[2]:+.4f})  |w|={norm(w):.5f} rad/s')
    emit()

    emit('=== static magnitude scale check ===')
    for label, (start, end) in STILL.items():
        seg = [r for r in rows if start <= r['t'] < end]
        m = st.mean([norm(r['a']) for r in seg])
        emit(f'{label:32s} mean |a| = {m:.4f} m/s2   ratio to 9.80665 = {m/9.80665:.5f}')
    emit()

    pairs = [('A: 2-9 s (start)', 'B: 22-29 s (after 1st turn)', '1st: 10-21 s (user: about 10 s)'),
             ('B: 22-29 s (after 1st turn)', 'C: 45-59 s (after 2nd turn)', '2nd: 30-44 s (user: about 14 s)')]
    for first, second, motion in pairs:
        emit(f'=== {motion} ===')
        # integrate 返回的是弧度制角增量；角度显示与旋转都从这里换算。
        # integrate returns an angular increment in radians; convert once here.
        theta, used, gaps = integrate(rows, *MOTION[motion])
        total_rad = norm(theta)
        total_deg = math.degrees(total_rad)
        raw_axis = [x / total_rad for x in theta] if total_rad else [0, 0, 0]
        a0, a1 = poses[first], poses[second]
        # 世界系向量按陀螺积分的反号角旋转；对两种约定都比较，取残差小者。
        # A world-frame vector rotates by the negative of the body angular increment. Both
        # conventions are evaluated and the smaller residual is kept, so a handedness
        # problem cannot be mistaken for a scale error.
        best = None
        for label, angle_used in (('body_increment', total_rad), ('negative_of_body_increment', -total_rad)):
            predicted_candidate = rotate(a0, raw_axis, angle_used)
            residual_candidate = [predicted_candidate[i] - a1[i] for i in range(3)]
            candidate = dict(label=label, angle=angle_used, predicted=predicted_candidate,
                             residual=residual_candidate, error=norm(residual_candidate),
                             direction_error=angle(predicted_candidate, a1))
            if best is None or candidate['error'] < best['error']:
                best = candidate
        predicted = best['predicted']
        diff = best['residual']
        emit(f'   gyro integral: angle = {total_deg:8.3f} deg, raw axis = ({raw_axis[0]:+.4f}, {raw_axis[1]:+.4f}, {raw_axis[2]:+.4f}), '
             f'n={used}, gaps={gaps}')
        emit(f'   best convention = {best["label"]} (angle {math.degrees(best["angle"]):+.3f} deg)')
        emit(f'   measured start  a = ({a0[0]:+8.4f}, {a0[1]:+8.4f}, {a0[2]:+8.4f})  |a|={norm(a0):.4f}')
        emit(f'   predicted final a = ({predicted[0]:+8.4f}, {predicted[1]:+8.4f}, {predicted[2]:+8.4f})  |a|={norm(predicted):.4f}')
        emit(f'   measured  final a = ({a1[0]:+8.4f}, {a1[1]:+8.4f}, {a1[2]:+8.4f})  |a|={norm(a1):.4f}')
        diff = [predicted[i] - a1[i] for i in range(3)]
        emit(f'   residual = ({diff[0]:+7.4f}, {diff[1]:+7.4f}, {diff[2]:+7.4f})  |residual|={norm(diff):.4f} m/s2')
        emit(f'   direction error = {angle(predicted, a1):.3f} deg   magnitude error = {norm(predicted)-norm(a1):+.4f} m/s2')
        # 用实测姿态反推转动角，与陀螺角比较 / invert the measured poses for a rotation angle to compare
        n_meas = cross(unit(a0), unit(a1))
        if norm(n_meas) > 1e-9:
            n_meas_u = unit(n_meas)
            phi_meas = angle(a0, a1)
            cos_phi = math.cos(math.radians(phi_meas))
            cos_alpha = abs(dot(n_meas_u, unit(a0)))
            denom = 1 - cos_alpha ** 2
            implied = None
            if denom > 1e-9:
                cos_rot = 1 - (1 - cos_phi) / denom
                if -1 <= cos_rot <= 1:
                    implied = math.degrees(math.acos(cos_rot))
            emit(f'   measured axis = ({n_meas_u[0]:+.4f}, {n_meas_u[1]:+.4f}, {n_meas_u[2]:+.4f}), '
                 f'angle between start pose and axis = {math.degrees(math.acos(min(1, cos_alpha))):.2f} deg')
            emit(f'   angle between start and final pose = {phi_meas:.3f} deg; '
                 f'implied rotation angle = {implied:.3f} deg' if implied else
                 '   implied rotation angle unavailable (axis nearly parallel to gravity)')
            if implied:
                emit(f'   gyro angle - implied angle = {total_deg - implied:+.3f} deg  '
                     f'({(total_deg / implied - 1) * 100:+.2f} % if implied != 0)')
        emit()

    (OUT / 'report2.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    emit(f'written: {OUT / "report2.txt"}')


if __name__ == '__main__':
    main()
