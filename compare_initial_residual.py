"""仅用初始静止段拟合固定世界系残差，独立检查后续速度漂移。
Fit a fixed world-frame residual using initial still data only; evaluate later velocity drift.

只打印诊断，不修改原始数据、姿态算法或正式传播模块。
Print diagnostics only; do not modify raw data, attitude algorithms or the propagation module.
"""
import json
import math

from attitude import AttitudeEstimator
from check_attitude_replay import ROOT, SESSIONS, mean, replay
from gravity_removal import remove_gravity
from velocity_propagation import VelocityState, propagate_velocity


def initial_residual(name, calibration):
    """使用初始对齐姿态估计标定段平均残差，不回放未来姿态。
    Estimate calibration mean residual using initial alignment, without replaying future attitude.
    """
    records = []
    path = ROOT / 'live_recordings' / name / 'packets.jsonl'
    for line in path.read_text(encoding='utf-8').splitlines():
        record = json.loads(line)
        packet = record.get('packet') or {}
        if packet.get('type') == 'imu':
            records.append(record)
    if not records:
        raise ValueError('no IMU records')
    origin = records[0]['arrival_monotonic_ns'] / 1e9
    selected = [r for r in records if calibration[0] <=
                r['arrival_monotonic_ns']/1e9-origin < calibration[1]]
    if len(selected) < 2:
        raise ValueError('calibration requires at least two samples')
    for left, right in zip(selected, selected[1:]):
        a, b = left['packet'], right['packet']
        if (a['node_id'] != b['node_id'] or b['sequence_number'] != a['sequence_number']+1
                or not 0 < b['timestamp_us']-a['timestamp_us'] <= 200000):
            raise ValueError('calibration discontinuity')
    forces = [tuple(r['packet']['data'][k] for k in ('ax_mps2', 'ay_mps2', 'az_mps2'))
              for r in selected]
    estimator = AttitudeEstimator()
    estimator.align_to_gravity(mean(forces))
    # 这是静止段固定姿态假设，不将逐帧真实姿态声称为已知。
    # This assumes a fixed calibration attitude, not known true per-sample attitude.
    residuals = [remove_gravity(force, estimator.q,
                               timestamp_us=r['packet']['timestamp_us'],
                               attitude_timestamp_us=r['packet']['timestamp_us'],
                               attitude_valid=True).linear_acceleration_world_mps2
                 for force, r in zip(forces, selected)]
    return mean(residuals), len(selected)


def integrate_window(rows, offset):
    """每段仅在起点设零速，后续不归零，不删除样本。
    Set zero velocity only at each window start; do not reset later or remove samples.
    """
    if len(rows) < 2:
        raise ValueError('evaluation requires at least two samples')
    corrected = [tuple(a-b for a, b in zip(r['residual'], offset)) for r in rows]
    state = VelocityState(rows[0]['timestamp_us'], (0, 0, 0), corrected[0])
    peak = 0.0
    for row, acceleration in zip(rows[1:], corrected[1:]):
        state = propagate_velocity(state, timestamp_us=row['timestamp_us'],
                                   acceleration_world_mps2=acceleration, max_dt_s=0.2)
        peak = max(peak, math.hypot(*state.velocity_world_mps))
    return state.velocity_world_mps, peak


def main():
    name, calibration, windows = SESSIONS[0]
    offset, n = initial_residual(name, calibration)
    rows, _, _ = replay(name, calibration)
    print(f'{name}\n标定 / Calibration: {calibration}, n={n}')
    print('所有窗口为电脑接收相对秒数；积分使用节点时间。')
    print('All windows use host receive-relative seconds; integration uses node time.')
    print('固定世界系补偿 / Fixed world offset XYZ (m/s^2): '
          + str(tuple(round(x, 6) for x in offset)))
    print('初始倾角复用标定段加速度，水平均值约零是对齐结果，不证明水平零偏为零。')
    print('Initial tilt reuses calibration acceleration; near-zero horizontal mean does not prove zero horizontal bias.')
    print('固定姿态下估计；仅诊断，不是三轴传感器零偏或完整标定。')
    print('Estimated at fixed attitude; diagnostic only, not sensor-axis bias or full calibration.')
    for lo, hi in windows:
        if lo < calibration[1]:
            raise ValueError('evaluation overlaps calibration')
        selected = [r for r in rows if lo <= r['t'] < hi]
        before, peak_before = integrate_window(selected, (0, 0, 0))
        after, peak_after = integrate_window(selected, offset)
        duration = (selected[-1]['timestamp_us']-selected[0]['timestamp_us'])/1e6
        # 固定补偿的积分差应等于补偿乘时长，独立检查对照实现。
        # The integral difference must equal offset times duration; independently check the comparison.
        for v0, v1, b in zip(before, after, offset):
            if not math.isclose(v0-v1, b*duration, rel_tol=1e-9, abs_tol=1e-10):
                raise AssertionError('fixed-offset integral identity failed')
        print(f'\n{lo:g}-{hi:g}s n={len(selected)} node_duration={duration:.3f}s')
        for label, vector, peak in (('before', before, peak_before), ('after', after, peak_after)):
            xyz = ', '.join(f'{v:.4f}' for v in vector)
            print(f'  {label}: end_xyz=({xyz}) end_norm={math.hypot(*vector):.4f} peak_norm={peak:.4f} m/s')
    print('\n每窗口独立从假设零速开始；全部样本保留，验证窗口不参与拟合。')
    print('Each window starts independently at assumed zero velocity; retain all samples and exclude evaluation from fitting.')
    print('未执行ZUPT、位置积分或动态精度验收；不自动采用补偿。')
    print('No ZUPT, position integration or dynamic acceptance; compensation is not automatically adopted.')


if __name__ == '__main__':
    main()
