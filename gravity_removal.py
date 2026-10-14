"""去重力的纯计算模块；无网络、滤波、速度积分或原始数据修改。
Pure gravity removal: no network, filtering, velocity integration or raw-data mutation.

q_bw 使用 (w,x,y,z)，把身体向量旋转到世界；R_BS 采用已确认的安装方向。
q_bw uses (w,x,y,z) and rotates body vectors into world; R_BS uses confirmed mounting.
a_world = R_bw R_BS f_sensor + (0,0,-g), where f is accelerometer specific force.
调用者必须提供同一节点时刻的有效姿态；断流后须先恢复可信姿态再调用。
The caller must supply a valid attitude at the same node time and recover it after gaps.
本模块不能检测时间标签正确但物理上错误的姿态，也不估计加速度计偏置。
This module cannot detect physically wrong attitudes with correct time labels or estimate accel bias.
"""
from dataclasses import dataclass
import math

from attitude import body_from_sensor, matrix_vector, quat_to_matrix


@dataclass(frozen=True)
class GravityRemovedSample:
    timestamp_us: int
    specific_force_sensor_mps2: tuple[float, float, float]
    specific_force_world_mps2: tuple[float, float, float]
    linear_acceleration_world_mps2: tuple[float, float, float]


def remove_gravity(specific_force_sensor_mps2, q_bw, *, timestamp_us,
                   attitude_timestamp_us, attitude_valid, gravity_mps2=9.80665):
    """返回世界系线加速度，输入错误则抛出 ValueError；不把异常输出伪装为零。
    Return world linear acceleration; invalid inputs raise ValueError rather than fake zero output.
    """
    if any(type(t) is not int or t < 0 for t in (timestamp_us, attitude_timestamp_us)):
        raise ValueError('timestamps must be nonnegative integer microseconds')
    if timestamp_us != attitude_timestamp_us:
        raise ValueError('measurement and attitude timestamps must match')
    if attitude_valid is not True:
        raise ValueError('attitude is not valid for this sample')
    force = tuple(specific_force_sensor_mps2)
    q = tuple(q_bw)
    if len(force) != 3 or not all(math.isfinite(v) for v in force):
        raise ValueError('specific force must contain three finite values')
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('attitude must contain four finite values')
    # 允许数值舍入，拒绝明显非单位四元数，避免掩盖上游姿态错误。
    # Allow roundoff, but reject clearly nonunit quaternions to expose upstream errors.
    if abs(math.hypot(*q) - 1.0) > 1e-6:
        raise ValueError('attitude quaternion must be unit length')
    if not math.isfinite(gravity_mps2) or gravity_mps2 <= 0:
        raise ValueError('gravity magnitude must be finite and positive')
    world = matrix_vector(quat_to_matrix(q), body_from_sensor(force))
    linear = (world[0], world[1], world[2] - gravity_mps2)
    if not all(math.isfinite(v) for v in (*world, *linear)):
        raise ValueError('gravity-removal output overflow')
    return GravityRemovedSample(timestamp_us, force, world, linear)
