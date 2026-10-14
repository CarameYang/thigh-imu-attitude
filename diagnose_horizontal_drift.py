"""分段检查水平残差及估计姿态，只打印，不调参或修改原始数据。
Inspect segmented horizontal residuals and estimated attitude; print only, without tuning or raw edits.
"""
import math

from check_attitude_replay import SESSIONS, replay
from compare_initial_residual import integrate_window


def summarize(edges):
    """按节点时间积分所有相邻区间；均值为梯形时间加权均值。
    Integrate all adjacent intervals using node time; means are trapezoidal time-weighted means.
    """
    duration = 0.0
    delta = [0.0, 0.0]
    absolute = [0.0, 0.0]
    for left, right in edges:
        dt = (right['timestamp_us']-left['timestamp_us'])/1e6
        if not 0 < dt <= 0.2:
            raise ValueError('invalid interval: stop diagnosis')
        duration += dt
        for axis in range(2):
            contribution = 0.5*(left['residual'][axis]+right['residual'][axis])*dt
            delta[axis] += contribution
            absolute[axis] += abs(contribution)
    if duration <= 0:
        raise ValueError('empty segment')
    samples = [edges[0][0]] + [right for _, right in edges]
    angles = [r['euler_deg'] for r in samples]
    return dict(duration=duration, delta=tuple(delta),
                mean=tuple(v/duration for v in delta),
                consistency=tuple(abs(v)/a if a else 0.0 for v, a in zip(delta, absolute)),
                peak=tuple(max(abs(r['residual'][i]) for r in samples) for i in range(2)),
                roll_span=max(v[0] for v in angles)-min(v[0] for v in angles),
                pitch_span=max(v[1] for v in angles)-min(v[1] for v in angles),
                yaw_delta=samples[-1]['yaw']-samples[0]['yaw'])


def pair(values):
    return '(' + ', '.join(f'{v:+.4f}' for v in values) + ')'


def main():
    name, calibration, windows = SESSIONS[0]
    rows, _, _ = replay(name, calibration)
    print(f'{name}; calibration={calibration}')
    print('每个原窗口按电脑接收相对时间分三段；积分和均值使用节点时间。')
    print('Split each original window into host-relative thirds; integrate and average using node time.')
    print('跨分界的小区间归右端点所在段，保留所有积分区间，不漏点、不归零。')
    print('Assign boundary-crossing intervals to their right endpoint bin; retain every interval without resets.')
    print('mean_a: 时间加权水平加速度 m/s^2; delta_v: 本段速度增量 m/s; cumulative_v: 窗口内累计速度。')
    print('mean_a: time-weighted horizontal acceleration; delta_v: segment increment; cumulative_v: window velocity.')
    print('consistency=|净增量|/各区间增量绝对值之和；接近1表示同向累积，接近0表示抵消，不是置信度。')
    print('consistency=absolute net increment/sum of absolute increments; near 1 means aligned accumulation, not confidence.')
    for lo, hi in windows:
        selected = [r for r in rows if lo <= r['t'] < hi]
        if len(selected) < 4:
            raise ValueError('insufficient samples')
        bins = [[], [], []]
        for left, right in zip(selected, selected[1:]):
            index = min(2, int(3*(right['t']-lo)/(hi-lo)))
            bins[index].append((left, right))
        cumulative = [0.0, 0.0]
        print(f'\n窗口 / Window {lo:g}-{hi:g}s, samples={len(selected)}')
        for i, edges in enumerate(bins):
            m = summarize(edges)
            for axis in range(2):
                cumulative[axis] += m['delta'][axis]
            start, end = lo+(hi-lo)*i/3, lo+(hi-lo)*(i+1)/3
            print(f'  {start:.3f}-{end:.3f}s node_dt={m["duration"]:.3f}s mean_a={pair(m["mean"])}')
            print(f'    delta_v={pair(m["delta"])} cumulative_v={pair(cumulative)}')
            print(f'    consistency={pair(m["consistency"])} abs_peak_a={pair(m["peak"])}')
            print(f'    estimated roll_span={m["roll_span"]:.3f} pitch_span={m["pitch_span"]:.3f} '
                  f'yaw_delta={m["yaw_delta"]:+.3f} deg')
        expected, _ = integrate_window(selected, (0, 0, 0))
        # 对照已有积分器，确保分段边界没有丢失或重复积分。
        # Compare with the existing integrator to detect missing or duplicate boundary intervals.
        if not all(math.isclose(a, b, abs_tol=1e-10, rel_tol=1e-10)
                   for a, b in zip(cumulative, expected[:2])):
            raise AssertionError('segmented increments disagree with whole-window propagation')
        print('  分段积分与整段一致 / Segmented and whole-window integration agree.')
    print('\n姿态跨度是估计变化，不是姿态误差；同源加速度无法独立区分倾角误差、偏置与实际运动。')
    print('Attitude spans are estimated changes, not errors; shared acceleration cannot isolate tilt error, bias and motion.')
    print('每个完整窗口只在起点假设零速，三段之间不重置；不改变用户动作标签。')
    print('Assume zero velocity only at each full-window start, never between thirds; preserve user action labels.')


if __name__ == '__main__':
    main()
