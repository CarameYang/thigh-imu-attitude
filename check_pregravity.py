"""去重力前的只读检查：数据完整性、姿态与静止标定；不计算去重力加速度。
Read-only pre-gravity check: data integrity, attitude and still calibration; no gravity subtraction.

Run: python -X utf8 check_pregravity.py
"""
import base64
import json
import math
import statistics as st
from pathlib import Path

from attitude import AttitudeEstimator, body_from_sensor, specific_force_expected_body, quat_angle
from check_attitude_replay import SESSIONS

ROOT = Path(__file__).resolve().parent
KEYS = ('ax_mps2', 'ay_mps2', 'az_mps2', 'gx_radps', 'gy_radps', 'gz_radps')


def angle(a, b):
    scale = math.sqrt(sum(x*x for x in a) * sum(x*x for x in b))
    return math.degrees(math.acos(max(-1, min(1, sum(x*y for x, y in zip(a, b))/scale))))


def main():
    print('去重力前检查 / Pre-gravity checks. No raw data or history is modified.')
    print('平台来自录制清单，不是精密姿态真值。/ Index plateaus are not precision attitude truth.')
    for name, cal_window, windows in SESSIONS:
        records = [json.loads(line) for line in (ROOT/'live_recordings'/name/'packets.jsonl').read_text(encoding='utf-8').splitlines()]
        records = [r for r in records if (r.get('packet') or {}).get('type') == 'imu']
        packets = [r['packet'] for r in records]
        dt = [(b['timestamp_us']-a['timestamp_us'])/1e6 for a, b in zip(packets, packets[1:])]
        bad_seq = sum(b['sequence_number'] != a['sequence_number']+1 for a, b in zip(packets, packets[1:]))
        # 发布用的脱敏录制不含 raw_base64；此时跳过原始字节核对并明确说明。
        # Redacted published recordings have no raw_base64; skip that check and say so.
        raw_records = [r for r in records if 'raw_base64' in r]
        mismatch = sum(json.loads(base64.b64decode(r['raw_base64'])) != r['packet']
                       for r in raw_records)
        if len(raw_records) != len(records):
            print(f'{name}: raw bytes not published for {len(records) - len(raw_records)} '
                  f'of {len(records)} records; raw-vs-parsed check skipped')
        repeats = sum(a['data'] == b['data'] for a, b in zip(packets, packets[1:]))
        finite = all(math.isfinite(p['data'][k]) for p in packets for k in KEYS)
        if bad_seq or mismatch or not finite or any(not 0 < d <= 0.2 for d in dt):
            raise ValueError(f'{name}: data continuity or integrity failed')
        rows = []
        origin = records[0]['arrival_monotonic_ns']
        for rec, p in zip(records, packets):
            v = tuple(p['data'][k] for k in KEYS)
            rows.append(((rec['arrival_monotonic_ns']-origin)/1e9, p['timestamp_us']/1e6, v[:3], v[3:]))
        cal = [r for r in rows if cal_window[0] <= r[0] < cal_window[1]]
        est = AttitudeEstimator()
        est.align_to_gravity(tuple(st.mean(r[2][i] for r in cal) for i in range(3)))
        bias = est.calibrate_still([(node, a, w) for _, node, a, w in cal])
        est.previous_time = cal[-1][1]
        out = []
        for t, node, a, w in rows:
            if t < cal_window[1]:
                continue
            state = est.update(node, a, w)
            q = state['quaternion']
            if not all(math.isfinite(x) for x in q) or abs(sum(x*x for x in q)-1) > 1e-10:
                raise ValueError(f'{name}: invalid quaternion')
            out.append((t, q, angle(body_from_sensor(a), specific_force_expected_body(q))))
        print(f'\n{name}: n={len(rows)}; dt(ms) min/median/max={min(dt)*1000:.3f}/{st.median(dt)*1000:.3f}/{max(dt)*1000:.3f}')
        print(f'  seq errors={bad_seq}; raw mismatch={mismatch}; identical adjacent six-axis samples={repeats}')
        print(f'  cal={cal_window}; body bias(rad/s)={tuple(round(x,6) for x in bias)}')
        for lo, hi in windows:
            segment = [r for r in out if lo <= r[0] < hi]
            errors = sorted(r[2] for r in segment)
            p95 = errors[math.ceil(0.95*len(errors))-1]
            # 四元数角度避免桌面近竖直姿态的 Euler 奇异；不把它称为航向误差。
            # Quaternion angle avoids Euler singularities in table poses; it is not heading error.
            excursion = max(quat_angle(segment[0][1], r[1]) for r in segment)
            print(f'  {lo:g}-{hi:g}s: tilt consistency median/p95={st.median(errors):.3f}/{p95:.3f}deg; max attitude change={excursion:.3f}deg')
    print('\n倾角一致性使用相同加速度信号，只是内部检查，不是独立精度验收。')
    print('Tilt consistency reuses the accelerometer: internal check, not independent accuracy acceptance.')
    print('No gravity subtraction, velocity propagation or zero-velocity update was performed.')


if __name__ == '__main__':
    main()
