"""世界系速度的单步梯形积分；不执行ZUPT或位置传播。
One-step world-frame trapezoidal velocity integration; no ZUPT or position propagation.

每次调用须来自同一节点、同一连续数据段；调用者负责序号与姿态有效性。
Each call must use the same node and continuous segment; callers check sequence and attitude validity.
遇到错误应停止该段；恢复前须明确初始化，不能跳过坏点继续跨段积分。
Stop the segment on error and explicitly initialize before resuming; do not skip bad samples.
"""
from dataclasses import dataclass
import math


def _time(value):
    if type(value) is not int or value < 0:
        raise ValueError('timestamp must be nonnegative integer microseconds')
    return value


def _vector(value):
    try:
        result = tuple(float(x) for x in value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('expected three finite numbers') from exc
    if len(result) != 3 or not all(math.isfinite(x) for x in result):
        raise ValueError('expected three finite numbers')
    return result


@dataclass(frozen=True)
class VelocityState:
    timestamp_us: int
    velocity_world_mps: tuple[float, float, float]
    acceleration_world_mps2: tuple[float, float, float]

    def __post_init__(self):
        # 固化输入副本，防止外部列表修改状态。
        # Freeze input copies so external lists cannot mutate the state.
        _time(self.timestamp_us)
        object.__setattr__(self, 'velocity_world_mps', _vector(self.velocity_world_mps))
        object.__setattr__(self, 'acceleration_world_mps2', _vector(self.acceleration_world_mps2))


def propagate_velocity(previous, *, timestamp_us, acceleration_world_mps2, max_dt_s):
    """使用已去重力的加速度；初值或外部修正通过显式VelocityState传入。
    Use gravity-removed acceleration; pass initialization or external correction as VelocityState.

    外部修正必须在状态时刻、同一世界系给出；保留该时刻加速度。
    External corrections must share the state time/world frame and retain its acceleration.
    延迟修正需要调用者回放；本函数不实现延迟融合或协方差传播。
    Delayed corrections require caller replay; this function has no delayed fusion or covariance propagation.
    """
    if not isinstance(previous, VelocityState):
        raise ValueError('previous must be a VelocityState')
    _time(timestamp_us)
    accel = _vector(acceleration_world_mps2)
    if isinstance(max_dt_s, bool) or not isinstance(max_dt_s, (int, float)) or not math.isfinite(max_dt_s) or max_dt_s <= 0:
        raise ValueError('max_dt_s must be finite and positive')
    delta_us = timestamp_us - previous.timestamp_us
    if delta_us <= 0:
        raise ValueError('duplicate or reversed node timestamp')
    if delta_us > max_dt_s * 1_000_000:
        raise ValueError('time gap: stop and explicitly reinitialize the segment')
    dt = delta_us / 1_000_000
    # 两端各取一半再相加，降低中间求和溢出的风险。
    # Halve endpoints before adding to reduce intermediate overflow risk.
    velocity = tuple(v + (0.5*a0 + 0.5*a1)*dt for v, a0, a1 in
                     zip(previous.velocity_world_mps, previous.acceleration_world_mps2, accel))
    return VelocityState(timestamp_us, velocity, accel)
