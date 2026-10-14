"""零偏敏感性诊断：只读已有录制，不修改标定、模型或历史报告。
Bias sensitivity diagnostic: read existing recordings without changing calibration, models or reports.

Run: python -X utf8 diagnose_gyro_bias.py
"""
import json
import math
import statistics as st
from pathlib import Path

from check_attitude_replay import replay, SESSIONS

ROOT = Path(__file__).resolve().parent
GYRO = ('gx_radps', 'gy_radps', 'gz_radps')


def load(name):
    rows = []
    for line in (ROOT / 'live_recordings' / name / 'packets.jsonl').read_text(encoding='utf-8').splitlines():
        r = json.loads(line)
        p = r.get('packet') or {}
        if p.get('type') == 'imu':
            rows.append((r['arrival_monotonic_ns'] / 1e9, tuple(p['data'][k] for k in GYRO)))
    start = rows[0][0]
    return [(t - start, w) for t, w in rows]


def summarize(rows, start, end):
    seg = [w for t, w in rows if start <= t < end]
    if not seg:
        raise ValueError(f'Empty interval: {start}-{end}')
    avg = tuple(st.mean(w[i] for w in seg) for i in range(3))
    sd = tuple(st.pstdev(w[i] for w in seg) for i in range(3))
    return len(seg), avg, sd


def fmt(v):
    return '(' + ', '.join(f'{x:+.5f}' for x in v) + ')'


def yaw_mean(out, start, end):
    return st.mean(r['yaw'] for r in out if start <= r['t'] < end)


def main():
    print('零偏检查 / Gyro bias check; raw sensor X,Y,Z; rad/s.')
    print('时间为电脑接收相对秒数；姿态传播使用节点时间。')
    print('Times are host-relative seconds; attitude propagation uses node time.')
    print('均值差不能单独区分真实微动与零偏变化；不按转体是否接近90度选参数。')
    print('Mean differences alone cannot separate physical motion from bias changes; no tuning to 90 degrees.')
    specs = [
        (SESSIONS[0][0], [(2, 6), (6, 10), (10, 15), (15, 19.5), (31, 39.5), (51, 70)]),
        ('20261011_232142_279415_live', [(2, 7), (10, 15), (20, 25)]),
        (SESSIONS[2][0], [(1, 5), (5, 7), (28.5, 31.5), (56, 60), (60, 65)]),
    ]
    for name, windows in specs:
        rows = load(name)
        print('\n' + name)
        for lo, hi in windows:
            n, avg, sd = summarize(rows, lo, hi)
            print(f'  {lo:g}-{hi:g}s n={n}: mean={fmt(avg)}, SD={fmt(sd)}')

    print('\n标定窗口敏感性 / Calibration-window sensitivity: same later evaluation windows.')
    print('以下均为同一数据的探索性对比，不是独立验收；当前默认标定未更改。')
    print('Exploratory comparisons on the same recording, not independent acceptance; default unchanged.')
    for cal in ((2, 6), (2, 10), (2, 15), (10, 15)):
        out, bias, _ = replay(SESSIONS[0][0], cal)
        turn = yaw_mean(out, 31, 32) - yaw_mean(out, 18, 19)
        late = yaw_mean(out, 69, 70) - yaw_mean(out, 51, 52)
        print(f'  cal={cal}: body bias={fmt(bias)}; turn={turn:+.2f}deg; late heading change={late:+.2f}deg')

    # 桌面对照按时间分割：只用较早数据标定，在后续静止段检查原始残差。
    # Chronological table control: calibrate on earlier data, check raw residuals on later still data.
    rows = load('20261011_232142_279415_live')
    _, bias, _ = summarize(rows, 2, 12)
    _, later, _ = summarize(rows, 12, 28)
    residual = tuple(later[i] - bias[i] for i in range(3))
    print('\n桌面对照：2-12s标定，12-28s留出验证 / Table control: calibrate 2-12s, hold out 12-28s.')
    print(f'  mean residual={fmt(residual)} rad/s')
    print(f'  rate equivalent={fmt(tuple(math.degrees(x) for x in residual))} deg/s')
    print('这是传感器各轴残差，不是佩戴航向精度；其它会话零偏不能直接沿用。')
    print('These are sensor-axis residuals, not worn heading accuracy; do not transfer bias across sessions.')


if __name__ == '__main__':
    main()
