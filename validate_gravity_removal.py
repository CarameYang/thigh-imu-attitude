"""使用固定窗口回放去重力，输出全部窗口的残差；不调参、不改原始数据。
Replay gravity removal on fixed windows, reporting all residuals without tuning or raw edits.

Run: python -X utf8 validate_gravity_removal.py
"""
import math
import statistics as st

from check_attitude_replay import SESSIONS, replay

REFERENCE_RMS = 0.2


def metrics(rows):
    """分别统计每轴平均、RMS、绝对95百分位和绝对峰值。
    Compute each axis mean, RMS, absolute 95th percentile and absolute peak separately.
    """
    if not rows:
        raise ValueError('empty validation window')
    axes = [[r['residual'][i] for r in rows] for i in range(3)]
    return dict(mean=tuple(st.mean(a) for a in axes),
                rms=tuple(math.sqrt(st.mean(x*x for x in a)) for a in axes),
                abs_p95=tuple(sorted(abs(x) for x in a)[math.ceil(0.95*len(a))-1] for a in axes),
                abs_peak=tuple(max(abs(x) for x in a) for a in axes))


def fmt(vector):
    return '(' + ', '.join(f'{v:.4f}' for v in vector) + ')'


def main():
    print('去重力回放 / Gravity-removal replay; world X,Y,Z, m/s^2.')
    print('参考线：各轴RMS<0.2，仅诊断，不是用户确认的正式验收。')
    print('Per-axis RMS<0.2 is a diagnostic reference, not agreed formal acceptance.')
    print('所有窗口保留，不筛选静止检测颜色；标定与评估区间分开。')
    print('All windows retained without detector gating; calibration and evaluation are separate.')
    below, count = 0, 0
    for name, calibration, windows in SESSIONS:
        rows, _, _ = replay(name, calibration)
        print(f'\n{name}; calibration={calibration} host-relative seconds')
        for lo, hi in windows:
            selected = [r for r in rows if lo <= r['t'] < hi]
            m = metrics(selected)
            within = all(v < REFERENCE_RMS for v in m['rms'])
            count += 1
            below += within
            print(f'  {lo:g}-{hi:g}s n={len(selected)}: mean={fmt(m["mean"])} RMS={fmt(m["rms"])}')
            print(f'    abs_p95={fmt(m["abs_p95"])} abs_peak={fmt(m["abs_peak"])} below_reference={within}')
            if not within:
                # 拆分诊断只解释差异，不替换整段结果或删除超限样本。
                # Subwindows explain variation without replacing the full result or dropping samples.
                for start in (lo, (lo+hi)/2):
                    end = (lo+hi)/2 if start == lo else hi
                    sub = metrics([r for r in selected if start <= r['t'] < end])
                    print(f'    diagnostic half {start:g}-{end:g}s RMS={fmt(sub["rms"])}')
                mag = st.mean(math.sqrt(sum(x*x for x in r['raw_accel'])) for r in selected)
                print(f'    raw force mean magnitude={mag:.4f}; mean tilt mismatch={st.mean(r["tilt"] for r in selected):.3f}deg')
    print(f'\n参考线以下 / Below reference: {below}/{count} windows.')
    print('姿态使用同一加速度计：静止残差是内部检查，不证明运动加速度或轨迹精度。')
    print('Attitude reuses the accelerometer; static residuals do not prove dynamic acceleration or trajectory accuracy.')
    print('No velocity integration, ZUPT update, detector tuning or firmware change.')


if __name__ == '__main__':
    main()
