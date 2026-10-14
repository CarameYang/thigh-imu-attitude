"""用四份已有录制检查姿态；只打印，不覆盖历史报告或原始数据。
Check attitude using four existing recordings; print only, preserving reports and raw data.

Run: python check_attitude_replay.py
"""
import json
import math
from pathlib import Path

from attitude import AttitudeEstimator
from gravity_removal import remove_gravity

ROOT = Path(__file__).resolve().parent
# 区间取自录制清单，为接收相对时间；平台不等于独立姿态真值。
# Windows come from the recording index in host-relative time; plateaus are not attitude truth.
SESSIONS = [
    ('20261012_003433_121271_live', (2, 6), [(7, 19.5), (31, 39.5), (51, 70)]),
    ('20261012_004616_059007_live', (2, 7), [(7, 9.5), (22, 29), (45, 59)]),
    ('20261012_024655_838538_live', (1, 5), [(5, 7), (17, 19), (28.5, 31.5), (44.5, 50), (56, 66)]),
    ('20261012_025851_252946_live', (4.5, 6.5), [(8.5, 10), (19, 21), (30.5, 32.5), (41.5, 44), (51.5, 56)]),
]


def mean(vectors):
    return tuple(sum(v[i] for v in vectors) / len(vectors) for i in range(3))


def replay(name, calibration):
    rows = []
    for line in (ROOT / 'live_recordings' / name / 'packets.jsonl').read_text(encoding='utf-8').splitlines():
        rec = json.loads(line)
        packet = rec.get('packet') or {}
        if packet.get('type') != 'imu':
            continue
        d = packet['data']
        rows.append(dict(host=rec['arrival_monotonic_ns'] / 1e9,
                         node=packet['timestamp_us'] / 1e6,
                         node_us=packet['timestamp_us'],
                         seq=packet['sequence_number'],
                         a=tuple(d[k] for k in ('ax_mps2', 'ay_mps2', 'az_mps2')),
                         w=tuple(d[k] for k in ('gx_radps', 'gy_radps', 'gz_radps'))))
    origin = rows[0]['host']
    for r in rows:
        r['t'] = r['host'] - origin
    cal = [r for r in rows if calibration[0] <= r['t'] < calibration[1]]
    est = AttitudeEstimator()
    est.align_to_gravity(mean([r['a'] for r in cal]))
    bias = est.calibrate_still([(r['node'], r['a'], r['w']) for r in cal])
    # 标定完成后才传播，不用未来标定值给之前时段打分。
    # Propagate only after calibration; do not score earlier data using future calibration.
    out = []
    previous = cal[-1]
    est.previous_time = previous['node']
    gaps = 0
    yaw_unwrapped = None
    for r in rows:
        if r['t'] < calibration[1]:
            continue
        if r['seq'] != previous['seq'] + 1 or not 0 < r['node'] - previous['node'] <= 0.2:
            gaps += 1
            raise ValueError(f'{name}: discontinuity at {r["t"]:.3f}; split replay before scoring')
        state = est.update(r['node'], r['a'], r['w'])
        yaw = state['euler'][2]
        if yaw_unwrapped is None:
            yaw_unwrapped = yaw
        else:
            yaw_unwrapped += (yaw - yaw_unwrapped + math.pi) % (2 * math.pi) - math.pi
        # 已检查连续性；姿态与比力来自同一节点测量时刻，无速度/位置积分。
        # Continuity checked above; attitude and force share node time, with no navigation integration.
        sample = remove_gravity(r['a'], state['quaternion'], timestamp_us=r['node_us'],
                                attitude_timestamp_us=r['node_us'], attitude_valid=True)
        world = sample.specific_force_world_mps2
        residual = sample.linear_acceleration_world_mps2
        tilt_error = math.degrees(math.atan2(math.hypot(world[0], world[1]), world[2]))
        out.append(dict(t=r['t'], yaw=math.degrees(yaw_unwrapped), residual=residual, tilt=tilt_error,
                        raw_accel=r['a'], timestamp_us=r['node_us'],
                        euler_deg=tuple(math.degrees(v) for v in state['euler'])))
        previous = r
    return out, bias, gaps


def main():
    print('已有录制回放 / Existing recording replay; node timestamps drive integration.')
    print('区间是电脑接收相对秒数 / Windows use host-relative seconds.')
    print('Residual uses the same accelerometer as attitude: internal consistency, not independent accuracy.')
    for name, calibration, windows in SESSIONS:
        out, bias, gaps = replay(name, calibration)
        print(f'\n{name}: calibration={calibration}; body gyro bias={tuple(round(x, 6) for x in bias)}; gaps={gaps}')
        for lo, hi in windows:
            seg = [r for r in out if lo <= r['t'] < hi]
            residuals = [r['residual'] for r in seg]
            avg = mean(residuals)
            rms = tuple(math.sqrt(sum(v[i] ** 2 for v in residuals) / len(seg)) for i in range(3))
            print(f'  {lo:g}-{hi:g}s n={len(seg)}: residual mean={tuple(round(x, 4) for x in avg)}, '
                  f'RMS={tuple(round(x, 4) for x in rms)} m/s^2; '
                  f'mean tilt mismatch={sum(r["tilt"] for r in seg)/len(seg):.3f}deg; '
                  f'yaw change={seg[-1]["yaw"]-seg[0]["yaw"]:.3f}deg' if name == SESSIONS[0][0] else
                  f'  {lo:g}-{hi:g}s n={len(seg)}: residual mean={tuple(round(x, 4) for x in avg)}, '
                  f'RMS={tuple(round(x, 4) for x in rms)} m/s^2; '
                  f'mean tilt mismatch={sum(r["tilt"] for r in seg)/len(seg):.3f}deg')
        if name == SESSIONS[0][0]:
            before = [r['yaw'] for r in out if 18 <= r['t'] < 19]
            after = [r['yaw'] for r in out if 31 <= r['t'] < 32]
            delta = sum(after)/len(after) - sum(before)/len(before)
            print(f'  转体前后航向变化 / Turn yaw change = {delta:.3f}deg (approximate user action, not precision truth)')


if __name__ == '__main__':
    main()
