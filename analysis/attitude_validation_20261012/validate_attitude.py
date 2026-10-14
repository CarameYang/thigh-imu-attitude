"""姿态解算验证：用已有录制检验互补滤波、安装变换和陀螺一致性。
Attitude validation: check the complementary filter, the mounting transform and gyro
consistency against the recordings already on disk.

不修改任何原始录制；结论写入本目录的 report.txt。
Does not modify recordings; the report is written into this folder.
"""
import json
import math
import statistics as st
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from attitude import (AttitudeEstimator, R_BS, body_from_sensor, euler_from_quat,  # noqa: E402
                      matrix_vector, quat_multiply, quat_normalize, specific_force_expected_body)
from stationary_stream import StationaryDetector  # noqa: E402

LIVE = ROOT / 'live_recordings'
OFFLINE = ROOT / 'recordings'
MODEL = ROOT / 'analysis/zupt_training_20261007_164450_519158/model.json'
OUT = Path(__file__).resolve().parent


def load(path):
    rows = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        packet = rec.get('packet') or {}
        if packet.get('type') != 'imu':
            continue
        data = packet['data']
        rows.append(dict(t=rec['arrival_monotonic_ns'] / 1e9,
                         seq=packet.get('sequence_number'),
                         node_us=packet.get('timestamp_us'),
                         a=(data['ax_mps2'], data['ay_mps2'], data['az_mps2']),
                         w=(data['gx_radps'], data['gy_radps'], data['gz_radps'])))
    rows.sort(key=lambda r: r['t'])
    t0 = rows[0]['t']
    for r in rows:
        r['t'] -= t0
    return rows


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def unit(v):
    n = norm(v)
    return tuple(x / n for x in v)


def mean(vectors):
    n = len(vectors)
    return tuple(sum(v[i] for v in vectors) / n for i in range(3))


def window(rows, start, end):
    return [r for r in rows if start <= r['t'] < end]


def rotate_vector(vector, axis, angle_rad):
    n = unit(axis)
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    cross = (n[1] * vector[2] - n[2] * vector[1],
             n[2] * vector[0] - n[0] * vector[2],
             n[0] * vector[1] - n[1] * vector[0])
    d = sum(n[i] * vector[i] for i in range(3))
    return tuple(vector[i] * c + cross[i] * s + n[i] * d * (1 - c) for i in range(3))


def angle(a, b):
    cos = sum(unit(a)[i] * unit(b)[i] for i in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def tilt_of_body_accel(acceleration):
    """身体系比力与 +z 的夹角，即该帧的安装倾角。/ Mounting tilt of one sample."""
    body = body_from_sensor(acceleration)
    return math.degrees(math.acos(max(-1.0, min(1.0, body[2] / norm(body)))))


def integrate_gyro(rows, start, end):
    """只按测量角速度积分，不做重力校正。/ Pure gyro integration for comparison."""
    total = [0.0, 0.0, 0.0]
    previous = None
    used = 0
    for r in window(rows, start, end):
        if previous is not None:
            dt = r['t'] - previous
            if 0 < dt <= 0.2:
                for i in range(3):
                    total[i] += r['w'][i] * dt
                used += 1
        previous = r['t']
    return total, used


def quat_angle(q1, q2):
    """两个姿态之间的旋转角（度）。Euler 角在 pitch 接近 ±90° 时有奇异，故用四元数。
    Rotation angle between two attitudes in degrees. Euler angles are singular near pitch +-90,
    so stability is measured with quaternions.
    """
    d = abs(sum(a * b for a, b in zip(unit(q1), unit(q2))))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, d))))


def attitude_stability(states, start, end):
    """返回该区间内的姿态路径长度与相对起始姿态的最大偏离。"""
    seg = [s for s in states if start <= s['t'] < end]
    if len(seg) < 2:
        return 0.0, 0.0
    reference = seg[0]['state']['quaternion']
    path = 0.0
    worst = 0.0
    for previous, current in zip(seg, seg[1:]):
        path += quat_angle(previous['state']['quaternion'], current['state']['quaternion'])
        worst = max(worst, quat_angle(reference, current['state']['quaternion']))
    return path, worst


def run_filter(rows, model, initial_window=2.0, calibration_window=None):
    estimator = AttitudeEstimator()
    detector = StationaryDetector(model)
    first = window(rows, 0.0, max(initial_window, 1.0))
    if first:
        estimator.align_to_gravity(mean([r['a'] for r in first]))
    if calibration_window:
        # 用一段确认静止的数据标定零偏（含竖直分量）。
        # Calibrate the bias, including its vertical component, from a confirmed still interval.
        estimator.calibrate_still([(r['t'], r['a'], r['w'])
                                   for r in window(rows, calibration_window[0], calibration_window[1])])
    states = []
    for r in rows:
        state = estimator.update(r['t'], r['a'], r['w'])
        # 检测器使用节点测量时间，与实时接收端一致。
        # The detector uses node timestamps, matching the live receiver.
        candidate = detector.push(r['node_us'], r['seq'], list(r['a']), list(r['w']))
        states.append(dict(t=r['t'], accel=r['a'], gyro=r['w'], state=state,
                           zupt_allowed=candidate['zupt_allowed'],
                           candidate_state=candidate['state'], reason=candidate['reason']))
    return states


def load_recorded_states(session_dir):
    """读取实时接收端保存的状态，用于核对离线重算。/ Read the receiver's saved states for cross-check."""
    path = Path(session_dir) / 'states.jsonl'
    if not path.exists():
        return None
    recorded = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        if rec.get('event') != 'measurement':
            continue
        recorded[rec.get('sequence_number')] = rec
    return recorded


def segment_summary(rows, start, end, label, lines, emit):
    seg = window(rows, start, end)
    if not seg:
        return
    tilts = [tilt_of_body_accel(r['a']) for r in seg]
    body = mean([body_from_sensor(r['a']) for r in seg])
    emit(f'   {label:34s} n={len(seg):4d}  tilt={st.mean(tilts):6.2f} deg (sd {st.pstdev(tilts):5.2f})  '
         f'|a|={norm(body):7.4f}  body=({body[0]:+7.3f}, {body[1]:+7.3f}, {body[2]:+7.3f})')


def main():
    lines = []
    def emit(text=''):
        print(text)
        lines.append(text)

    model = json.loads(MODEL.read_text(encoding='utf-8'))

    emit('安装旋转 R_bs (sensor -> body):')
    for row in R_BS:
        emit(f'   [{row[0]:+.0f} {row[1]:+.0f} {row[2]:+.0f}]')
    determinant = (R_BS[0][0] * (R_BS[1][1] * R_BS[2][2] - R_BS[1][2] * R_BS[2][1])
                   - R_BS[0][1] * (R_BS[1][0] * R_BS[2][2] - R_BS[1][2] * R_BS[2][0])
                   + R_BS[0][2] * (R_BS[1][0] * R_BS[2][1] - R_BS[1][1] * R_BS[2][0]))
    emit(f'   determinant = {determinant:+.1f}  (must be +1 for a proper rotation)')
    emit()

    sessions = {
        '20261012_004616_059007_live': dict(
            path=LIVE / '20261012_004616_059007_live' / 'packets.jsonl',
            calibration=(2.0, 7.0),
            still=[(2.0, 9.5, 'A start (table, +Z up)'),
                   (22.0, 29.5, 'B after 1st turn (+X up)'),
                   (45.0, 59.0, 'C after 2nd turn (+Z up)')],
            motions=[('1st turn', 9.5, 22.0), ('2nd turn', 29.5, 45.0)]),
        '20261012_003433_121271_live': dict(
            path=LIVE / '20261012_003433_121271_live' / 'packets.jsonl',
            calibration=(2.0, 6.0),
            still=[(2.0, 19.5, 'stand still (thigh)'), (31.0, 39.5, 'pause in the middle'),
                   (51.0, 70.0, 'stand again (thigh)')],
            motions=[]),
        '20261011_233919_343102_live': dict(
            path=LIVE / '20261011_233919_343102_live' / 'packets.jsonl',
            calibration=(3.0, 6.0),
            still=[(6.5, 10.0, 'early stand, after calibration'), (28.0, 40.0, 'late stand')],
            motions=[('walk + turn between the stands', 10.0, 28.0)]),
    }

    for name, spec in sessions.items():
        rows = load(spec['path'])
        emit(f'=== {name}  ({len(rows)} packets, {rows[-1]["t"]:.1f} s) ===')
        emit('   静止段的安装倾角（身体系比力与 +z 的夹角）/ mounting tilt in still segments:')
        for start, end, label in spec['still']:
            segment_summary(rows, start, end, label, lines, emit)

        states = run_filter(rows, model, calibration_window=spec.get('calibration'))
        recorded = load_recorded_states(spec['path'].parent)
        if recorded:
            mismatch = 0
            allowed = 0
            for row, computed in zip(rows, states):
                rec = recorded.get(row['seq'])
                if rec is None:
                    continue
                if bool(rec.get('zupt_allowed')) != bool(computed['zupt_allowed']):
                    mismatch += 1
                if computed['zupt_allowed']:
                    allowed += 1
            reasons = {}
            for computed in states:
                reasons[computed['reason']] = reasons.get(computed['reason'], 0) + 1
            emit(f'   离线重算与接收端状态核对 / offline recomputation vs recorded states: '
                 f'mismatch={mismatch}, zupt_allowed={allowed}/{len(states)}')
            emit(f'   判定原因分布 / reason counts: {reasons}')
        emit('   滤波器输出 / filter output:')
        for start, end, label in spec['still']:
            seg = [s for s in states if start <= s['t'] < end]
            if not seg:
                continue
            yaws = [math.degrees(euler_from_quat(s['state']['quaternion'])[2]) for s in seg]
            rolls = [math.degrees(euler_from_quat(s['state']['quaternion'])[0]) for s in seg]
            pitches = [math.degrees(euler_from_quat(s['state']['quaternion'])[1]) for s in seg]
            # 航向按角度展开以计算漂移。/ Unwrap yaw to measure drift.
            unwrapped = [yaws[0]]
            for y in yaws[1:]:
                delta = y - unwrapped[-1]
                while delta > 180:
                    delta -= 360
                while delta < -180:
                    delta += 360
                unwrapped.append(unwrapped[-1] + delta)
            drift = unwrapped[-1] - unwrapped[0]
            # roll/pitch 有重力观测，yaw 没有；三者分开报告。
            # Roll and pitch are gravity-observed; yaw is not. Report them separately.
            roll_span = max(rolls) - min(rolls)
            pitch_span = max(pitches) - min(pitches)
            path, worst = attitude_stability(states, start, end)
            allowed = sum(1 for s in seg if s['zupt_allowed'])
            tilts = [tilt_of_body_accel(s['accel']) for s in seg]
            emit(f'   {label:34s} attitude path={path:6.2f} deg  max deviation={worst:6.2f} deg  '
                 f'(roll span {roll_span:5.2f}, pitch span {pitch_span:5.2f} - Euler, singular near 90)  '
                 f'yaw drift={drift:+8.2f} deg (no heading observation)  '
                 f'zupt_allowed={allowed}/{len(seg)}  tilt={st.mean(tilts):6.2f} deg')
        bias = states[-1]['state']['bias']
        emit(f'   估计残余零偏 / estimated residual bias = '
             f'({bias[0]:+.5f}, {bias[1]:+.5f}, {bias[2]:+.5f}) rad/s (传感器系 / sensor frame)')

        for label, start, end in spec['motions']:
            total, used = integrate_gyro(rows, start, end)
            magnitude = norm(total)
            a0 = mean([r['a'] for r in window(rows, max(0.0, start - 2.0), start)] or [r['a'] for r in rows[:1]])
            a1 = mean([r['a'] for r in window(rows, end, end + 2.0)] or [r['a'] for r in rows[-1:]])
            # 用陀螺积分旋转起始比力，与结束比力比较：同一份数据内部的强一致性检查。
            # Rotate the initial specific force by the gyro integral and compare with the final
            # one: a strong internal consistency check that needs no external ground truth.
            predicted = rotate_vector(a0, total, -magnitude)
            predicted_body = rotate_vector(body_from_sensor(a0), body_from_sensor(total), -magnitude)
            measured_body = body_from_sensor(a1)
            emit(f'   {label:34s} gyro integral={math.degrees(magnitude):7.2f} deg over {end - start:4.1f} s '
                 f'(n={used})')
            emit(f'      accelerometer-frame prediction error = {angle(predicted, a1):6.2f} deg')
            emit(f'      body-frame prediction error          = {angle(predicted_body, measured_body):6.2f} deg')
        emit()

    emit('=== offline recordings: mounting tilt and filter stability (thigh-worn ones) ===')
    for folder in sorted(OFFLINE.glob('*/packets.jsonl')):
        rows = load(folder)
        states = run_filter(rows, model)
        tilts = [tilt_of_body_accel(r['a']) for r in rows]
        allowed = sum(1 for s in states if s['zupt_allowed'])
        emit(f'   {folder.parent.name:34s} n={len(rows):5d} tilt={st.mean(tilts):6.2f} deg '
             f'(sd {st.pstdev(tilts):5.2f})  zupt_allowed={allowed:5d}/{len(rows)}')

    (OUT / 'report.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    emit()
    emit(f'written: {OUT / "report.txt"}')


if __name__ == '__main__':
    main()
