"""只读分析：桌面 ±90° 绕模块 +Y 轴转动，检验陀螺积分与重力方向变化是否一致。
Read-only analysis: table test rotating about the module +Y axis by about ±90 degrees.
Checks whether gyro integration agrees with the change of the gravity direction.

不修改原始录制；输出只写到本目录。
Does not modify the recording; outputs are written only into this folder.
"""
import json
import math
import statistics as st
from pathlib import Path

SESSION = Path(__file__).resolve().parents[2] / 'live_recordings/20261012_004616_059007_live'
BIAS = (-0.020495493656645894, -0.006844256493084482, -0.00866488206724287)
OUT = Path(__file__).resolve().parent


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
            node_us=packet.get('timestamp_us'),
            a=(data['ax_mps2'], data['ay_mps2'], data['az_mps2']),
            w=(data['gx_radps'], data['gy_radps'], data['gz_radps']),
        ))
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


def window(rows, start, end):
    return [r for r in rows if start <= r['t'] < end]


def rows_of(v):
    return [round(x, 4) for x in v]


def main():
    rows = load(SESSION)
    lines = []
    def emit(text=''):
        print(text)
        lines.append(text)

    intervals = sorted(rows[i]['t'] - rows[i - 1]['t'] for i in range(1, len(rows)))
    emit(f'session: {SESSION.name}')
    emit(f'packets={len(rows)}  span={rows[-1]["t"]:.3f} s  '
         f'median interval={intervals[len(intervals)//2]*1000:.3f} ms  max={intervals[-1]*1000:.3f} ms  '
         f'gaps>200ms={sum(1 for x in intervals if x > 0.2)}')
    emit()

    emit('=== per-second summary ===')
    emit(f'{"t":>4} {"n":>3} {"|w|":>7} {"wX":>8} {"wY":>8} {"wZ":>8} {"|a|":>7} {"aX":>7} {"aY":>7} {"aZ":>7} '
         f'{"aXY":>6} {"aXZ":>6}')
    seconds = int(rows[-1]['t']) + 1
    for k in range(seconds):
        seg = window(rows, k, k + 1)
        if not seg:
            continue
        w = mean([r['w'] for r in seg])
        a = mean([r['a'] for r in seg])
        axy = math.degrees(math.atan2(a[1], a[0]))
        axz = math.degrees(math.atan2(a[2], a[0]))
        emit(f'{k:4d} {len(seg):3d} {norm(w):7.4f} {w[0]:8.4f} {w[1]:8.4f} {w[2]:8.4f} '
             f'{norm(a):7.3f} {a[0]:7.3f} {a[1]:7.3f} {a[2]:7.3f} {axy:6.2f} {axz:6.2f}')
    emit()

    # 自动找静止平台：1 秒窗 |w| 小且 |a| 稳定，连续 >=2 秒。
    # Automatic still-plateau detection: 1 s windows with low |w| and stable |a|, at least 2 s long.
    per_s = []
    for k in range(seconds):
        seg = window(rows, k, k + 1)
        if not seg:
            continue
        w = mean([r['w'] for r in seg])
        a = mean([r['a'] for r in seg])
        sd = st.pstdev([norm(r['a']) for r in seg])
        per_s.append((k, norm(w), sd, a))
    still = [p for p in per_s if p[1] < 0.02 and p[2] < 0.1]
    plateaus = []
    for k, wnorm, sd, a in still:
        if plateaus and k == plateaus[-1][-1] + 1:
            plateaus[-1].append(k)
        else:
            plateaus.append([k])
    plateaus = [p for p in plateaus if len(p) >= 2]

    emit('=== still plateaus (auto) and their gravity direction ===')
    for p in plateaus:
        seg = [r for r in rows if p[0] <= r['t'] < p[-1] + 1]
        a = mean([r['a'] for r in seg])
        emit(f'{p[0]:>3}-{p[-1]+1:<3}s n={len(seg):4d} |a|={norm(a):7.4f} '
             f'unit=({unit(a)[0]:+.4f}, {unit(a)[1]:+.4f}, {unit(a)[2]:+.4f})')
    emit()

    emit('=== consistency: gyro integration between consecutive plateaus vs gravity change ===')
    gyro_ok = True
    for i in range(len(plateaus) - 1):
        seg_a = [r for r in rows if plateaus[i][0] <= r['t'] < plateaus[i][-1] + 1]
        seg_b = [r for r in rows if plateaus[i + 1][0] <= r['t'] < plateaus[i + 1][-1] + 1]
        a_bar = mean([r['a'] for r in seg_a])
        b_bar = mean([r['a'] for r in seg_b])
        start = plateaus[i][-1] + 1
        end = plateaus[i + 1][0]
        theta = [0.0, 0.0, 0.0]
        prev = None
        used = 0
        for r in window(rows, start, end):
            if prev is not None:
                dt = r['t'] - prev
                if 0 < dt <= 0.2:
                    for j in range(3):
                        theta[j] += r['w'][j] * dt
                    used += 1
            prev = r['t']
        theta_deg = [math.degrees(x) for x in theta]
        g_change = angle(a_bar, b_bar)
        axis = cross(unit(a_bar), unit(b_bar))
        axis_u = unit(axis) if norm(axis) > 1e-9 else [0.0, 0.0, 0.0]
        projection = dot(theta_deg, axis_u)
        emit(f'plateau {plateaus[i][0]}-{plateaus[i][-1]+1}s -> {plateaus[i+1][0]}-{plateaus[i+1][-1]+1}s  '
             f'(motion {start}-{end}s, n={used})')
        emit(f'   gyro integral vector (deg) = {rows_of(theta_deg)}  |total|={norm(theta_deg):8.3f}')
        emit(f'   gravity change             = {g_change:8.3f} deg about ({axis_u[0]:+.3f}, {axis_u[1]:+.3f}, {axis_u[2]:+.3f})')
        emit(f'   gyro projection on that axis = {projection:+8.3f} deg   '
             f'difference = {projection - g_change:+8.3f} deg')
        emit(f'   |a| ratio between the two plateaus = {norm(b_bar)/norm(a_bar):.5f}')
        emit()

    emit('=== accel magnitude over the whole session (scale check) ===')
    all_norm = [norm(r['a']) for r in rows]
    emit(f'mean |a| = {st.mean(all_norm):.4f}  sd = {st.pstdev(all_norm):.4f}  '
         f'min = {min(all_norm):.4f}  max = {max(all_norm):.4f}')
    emit()

    (OUT / 'report.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    (OUT / 'report.json').write_text(json.dumps(dict(
        session=SESSION.name, packets=len(rows), span_s=round(rows[-1]['t'], 3),
        plateaus=[[p[0], p[-1] + 1] for p in plateaus], bias=BIAS),
        ensure_ascii=False, indent=2), encoding='utf-8')
    emit(f'written: {OUT / "report.txt"}')


if __name__ == '__main__':
    main()
