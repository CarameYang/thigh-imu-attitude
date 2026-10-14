"""已有静止平台的独立速度漂移诊断，只打印结果。
Independent velocity-drift diagnostics on existing still plateaus; print results only.
"""
import math

from check_attitude_replay import SESSIONS, replay
from velocity_propagation import VelocityState, propagate_velocity


def main():
    print('每窗口起点假设速度为零，随后自由积分；不是连续轨迹或ZUPT。')
    print('Each window starts at assumed zero velocity, then integrates freely; no trajectory or ZUPT.')
    print('窗口为电脑接收相对时间；积分使用节点时间。单位 m/s。')
    print('Windows use host receive-relative time; integration uses node time. Units: m/s.')
    print('max_dt=0.2s为显式诊断保护设置，不是硬件保证或精度验收。')
    print('max_dt=0.2s is an explicit diagnostic guard, not a hardware guarantee or accuracy criterion.')
    for name, calibration, windows in SESSIONS:
        rows, _, _ = replay(name, calibration)
        print(f'\n{name}')
        for lo, hi in windows:
            selected = [r for r in rows if lo <= r['t'] < hi]
            if len(selected) < 2:
                raise ValueError('window needs at least two samples')
            first = selected[0]
            state = VelocityState(first['timestamp_us'], (0, 0, 0), first['residual'])
            peak = 0.0
            for row in selected[1:]:
                state = propagate_velocity(state, timestamp_us=row['timestamp_us'],
                                           acceleration_world_mps2=row['residual'], max_dt_s=0.2)
                peak = max(peak, math.hypot(*state.velocity_world_mps))
            duration = (state.timestamp_us-first['timestamp_us'])/1e6
            vector = ', '.join(f'{v:.4f}' for v in state.velocity_world_mps)
            print(f'  {lo:g}-{hi:g}s n={len(selected)} node_duration={duration:.3f}s '
                  f'end_xyz=({vector}) end_norm={math.hypot(*state.velocity_world_mps):.4f} peak_norm={peak:.4f}')
    print('\n起点零速度是假设；姿态复用加速度计，结果不是独立动态精度验收。')
    print('Initial zero velocity is assumed; attitude reuses accelerometer data. This is not independent dynamic acceptance.')


if __name__ == '__main__':
    main()
