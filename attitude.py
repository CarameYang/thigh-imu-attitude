"""姿态解算：四元数互补滤波（Mahony），只依赖标准库。
Attitude estimation: quaternion complementary (Mahony) filter using only the standard library.

坐标系约定 / Frame conventions
-----------------------------
传感器系 sensor：IMU 原始轴，绝不取反或交换（原始数据保持原样）。
  Raw IMU axes, never negated or swapped.
板面标识 board labels（用户 2026-10-12 确认 / user-confirmed）：
  +X 向上 / up,  +Y 向左 / left,  +Z 向后 / back
身体系 body（与仓库 docs/coordinate_frames.md 一致）：
  +x 前 / forward,  +y 左 / left,  +z 上 / up
世界系 world：+z 向上，水平朝向由初始化选定。/ +z up; horizontal heading is chosen at initialization.

安装旋转 R_bs（传感器系 -> 身体系）/ mounting rotation (sensor -> body)：
  按用户确认的名义安装方向，R_bs 是一个轴置换加符号的纯旋转：
    sensor +X -> body +z (up)
    sensor +Y -> body +y (left)
    sensor +Z -> body -x (back)
  该矩阵行列式为 +1，是正常旋转，转速向量可用同一矩阵变换。
  The user-confirmed nominal mounting defines an axis permutation with signs and determinant
  +1; the same matrix applies to angular rates. This is not a precision mounting calibration.

  实测站立的重力残差约为 6.7°（绕 +Y 约 -3°，绕 +Z 约 +6°），那属于大腿实际安装倾角，
  不是坐标约定的一部分。它由滤波器用重力观测持续修正，不作为 R_bs 的固定偏置。
  The measured 6.7 degree offset from body +z is the real thigh mounting tilt; it is tracked
  by the gravity correction, not baked into R_bs.

已知限制 / Known limitations
---------------------------
- 六轴没有绝对航向观测，yaw 只在"世界系 +x 与初始身体朝向对齐"的意义下有意义；
  绝对航向需要磁力计或 UWB 等外部观测。
  There is no absolute heading observation from six axes; yaw is relative to the initial
  body heading and needs an external reference.
- 加速度计在明显运动时会包含运动加速度，此时重力校正被削弱，不应把姿态当作精确值。
  During significant motion the accelerometer is not a pure gravity reference.
"""
import math

# 安装旋转：传感器系 -> 身体系；行是身体轴，列是传感器轴。
# Mounting rotation: sensor -> body; rows are body axes, columns are sensor axes.
# 用户确认 +X 上、+Y 左、+Z 后；这是名义轴映射，实际佩戴倾角由姿态表示。
# User confirms +X up, +Y left, +Z back; this nominal mapping leaves mounting tilt to attitude.
# sensor +X -> body +z, sensor +Y -> body +y, sensor +Z -> body -x.
# 原始六轴数据保持不变；手持录制不用于重新定义佩戴方向。
# Preserve raw six-axis data; handheld recordings do not redefine the worn orientation.
R_BS = ((0.0, 0.0, -1.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0))


def quat_normalize(q):
    if len(q) != 4 or not all(math.isfinite(x) for x in q):
        raise ValueError('quaternion must contain four finite values')
    n = math.sqrt(sum(x * x for x in q))
    if n == 0.0:
        raise ValueError('zero quaternion')
    return tuple(x / n for x in q)


def quat_multiply(a, b):
    """四元数乘法 (w, x, y, z)，与 Hamilton 约定一致。/ Quaternion product, Hamilton convention."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_to_matrix(q):
    """四元数转旋转矩阵，把身体系向量映射到世界系。/ Body-to-world rotation matrix."""
    w, x, y, z = quat_normalize(q)
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def matrix_vector(matrix, vector):
    return tuple(sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3))


def gravity_expected_body(q):
    """世界系的重力单位向量 -z 在身体系中的表示（比力方向为 +z）。/ Gravity direction in the body frame."""
    r = quat_to_matrix(q)
    return tuple(-x for x in r[2])


def specific_force_expected_body(q):
    """世界系比力 +z 在身体系中的表示，与加速度计静止读数同向。/ Expected specific force in body axes."""
    r = quat_to_matrix(q)
    return tuple(r[2])


def quat_angle(q1, q2):
    """两个姿态之间的旋转角（度）。Euler 角在 pitch 接近 ±90° 时奇异，故稳定性用它衡量。
    Rotation angle between two attitudes in degrees. Euler angles are singular near pitch +-90,
    so stability is measured with quaternions.
    """
    a = quat_normalize(q1)
    b = quat_normalize(q2)
    d = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, d))))


def euler_from_quat(q):
    """返回 (roll, pitch, yaw)，单位弧度；yaw 相对初始朝向。/ Returns roll, pitch, yaw in radians."""
    w, x, y, z = quat_normalize(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class AttitudeEstimator:
    """互补滤波器：陀螺预测 + 重力慢校正。/ Complementary filter with slow gravity correction."""

    def __init__(self, proportional_gain=0.5, integral_gain=0.01, accel_trust_window=0.35,
                 tilt_only=True, bias_limit=0.05):
        # proportional_gain 决定倾角收敛快慢；integral_gain 估计残余零偏。
        # proportional_gain sets the tilt correction rate; integral_gain estimates residual bias.
        self.kp = float(proportional_gain)
        self.ki = float(integral_gain)
        # bias_limit 限制相对静止标定值的在线修正范围。
        # bias_limit bounds online adaptation relative to the still calibration.
        self.bias_limit = float(bias_limit)
        # 加速度偏离 1 g 超过该比例时削弱重力校正，避免运动加速度污染。
        # Attenuate the gravity correction when the measured magnitude departs from 1 g.
        self.accel_trust_window = float(accel_trust_window)
        # tilt_only：只校正倾角，不把加速度计用于航向（六轴没有航向观测）。
        # tilt_only: correct tilt only; never use the accelerometer for heading.
        self.tilt_only = bool(tilt_only)
        self.reset()

    def reset(self, quaternion=(1.0, 0.0, 0.0, 0.0)):
        self.q = quat_normalize(quaternion)
        self.bias = [0.0, 0.0, 0.0]
        self.calibrated_bias = [0.0, 0.0, 0.0]
        self.previous_time = None
        self.samples = 0

    def align_to_gravity(self, acceleration):
        """用第一帧静止加速度把初始姿态对齐到重力，只影响倾角。/ Align initial tilt to gravity."""
        _finite_vector(acceleration)
        body = matrix_vector(R_BS, acceleration)
        n = math.sqrt(sum(x * x for x in body))
        if n == 0.0:
            return
        # 先转到世界系，再左乘校正；重复对齐不会叠加同一倾角。
        # Rotate into world axes first, then left-multiply; repeated alignment is idempotent.
        measured = matrix_vector(quat_to_matrix(self.q), tuple(x / n for x in body))
        axis = (measured[1], -measured[0], 0.0)
        axis_norm = math.sqrt(sum(x * x for x in axis))
        if axis_norm < 1e-12:
            if measured[2] < 0.0:
                self.q = quat_multiply((0.0, 1.0, 0.0, 0.0), self.q)
            return
        angle = math.acos(max(-1.0, min(1.0, measured[2])))
        scale = math.sin(angle / 2.0) / axis_norm
        rotation = (math.cos(angle / 2.0), *(x * scale for x in axis))
        self.q = quat_normalize(quat_multiply(rotation, self.q))

    def calibrate_still(self, samples):
        """用一段确认静止的测量标定零偏，含重力无法观测的竖直分量。
        Calibrate the bias from a confirmed still interval, including the vertical component that
        gravity cannot observe. `samples` is a sequence of (timestamp_s, acceleration, angular_rate).

        重力校正只能约束垂直于重力的两个方向；沿重力的陀螺零偏会直接变成航向漂移，
        必须像这样从静止段估计，否则 yaw 会稳定漂移。
        The gravity correction only constrains the two directions perpendicular to gravity; the
        bias along gravity becomes pure yaw drift and must be estimated from still data.
        """
        usable = [(s[1], s[2]) for s in samples]
        if not usable:
            return None
        for acceleration, angular_rate in usable:
            _finite_vector(acceleration)
            _finite_vector(angular_rate)
        # 已确认静止时，三轴陀螺均值均可作为零偏；不只保留竖直分量。
        # During confirmed stillness, all three gyro means estimate bias, not just the vertical part.
        angular_rate = tuple(sum(s[1][i] for s in usable) / len(usable) for i in range(3))
        self.bias = list(matrix_vector(R_BS, angular_rate))
        self.calibrated_bias = list(self.bias)
        return tuple(self.bias)

    def update(self, timestamp_s, acceleration, angular_rate):
        """输入 SI 单位的一帧测量，返回状态字典。/ Feed one SI-unit sample and return a status dict."""
        # 坏值必须在修改状态前拒绝，否则一个 NaN 会污染后续所有姿态。
        # Reject invalid values before mutation; one NaN would otherwise poison later attitudes.
        if not math.isfinite(timestamp_s):
            raise ValueError('timestamp must be finite')
        _finite_vector(acceleration)
        _finite_vector(angular_rate)
        body_accel = matrix_vector(R_BS, acceleration)
        body_gyro = matrix_vector(R_BS, angular_rate)
        accel_norm = math.sqrt(sum(x * x for x in body_accel))
        gap = None if self.previous_time is None else timestamp_s - self.previous_time
        if gap is not None and (gap <= 0.0 or gap > 0.2):
            # 时间倒退或长间断：不积分，只更新参考时间。/ Do not integrate across gaps or time reversals.
            self.previous_time = timestamp_s
            return dict(quaternion=self.q, euler=euler_from_quat(self.q), bias=tuple(self.bias),
                        correction=None, trust=0.0, gap_s=gap, samples=self.samples)
        dt = 0.0 if gap is None else gap
        self.previous_time = timestamp_s

        correction = None
        trust = 0.0
        if accel_norm > 1e-6:
            trust = max(0.0, min(1.0, 1.0 - abs(accel_norm - 9.80665) / (self.accel_trust_window * 9.80665)))
        if trust > 0.0:
            measured = tuple(x / accel_norm for x in body_accel)
            expected = specific_force_expected_body(self.q)
            # q 表示 body -> world，世界 +z 在身体中是 R 的第三行。
            # q maps body -> world; world +z expressed in body axes is the third row of R.
            # measured x expected 给出正确的身体角速度反馈方向。
            # measured x expected gives the correct body angular-rate feedback direction.
            error = (measured[1] * expected[2] - measured[2] * expected[1],
                     measured[2] * expected[0] - measured[0] * expected[2],
                     measured[0] * expected[1] - measured[1] * expected[0])
            if self.tilt_only:
                # 只投影校正项，绝不清除陀螺记录到的真实转身。
                # Project only the correction, never the actual turn measured by the gyro.
                projection = sum(error[i] * expected[i] for i in range(3))
                error = tuple(error[i] - projection * expected[i] for i in range(3))
            error = tuple(trust * x for x in error)
            correction = error
            # 仅增量来自可观测倾角；保留已标定的完整零偏，包括竖直分量。
            # Only the increment comes from observable tilt; retain the entire calibrated bias.
            for i in range(3):
                self.bias[i] -= self.ki * error[i] * dt
                # 限制在线修正相对标定值的幅度，不裁掉静止标定结果。
                # Bound adaptation relative to calibration without clipping the calibrated result.
                base = self.calibrated_bias[i]
                self.bias[i] = max(base - self.bias_limit, min(base + self.bias_limit, self.bias[i]))

        # 即使加速度不可信也要减零偏，并继续积分全部真实角速度。
        # Subtract bias even when acceleration is untrusted, and integrate all actual angular rates.
        body_gyro = tuple(body_gyro[i] - self.bias[i] +
                          self.kp * (correction[i] if correction is not None else 0.0)
                          for i in range(3))
        # 身体角速度右乘 body -> world 四元数：q_dot = 0.5 q ⊗ (0, omega_body)。
        # Right-multiply body rate: q_dot = 0.5 q ⊗ (0, omega_body) for body -> world attitude.
        q = self.q
        derivative = quat_multiply(q, (0.0, *body_gyro))
        dq = tuple(0.5 * x * dt for x in derivative)
        self.q = quat_normalize(tuple(q[i] + dq[i] for i in range(4)))
        self.samples += 1
        return dict(quaternion=self.q, euler=euler_from_quat(self.q), bias=tuple(self.bias),
                    correction=correction, trust=trust, gap_s=gap, samples=self.samples,
                    accel_norm=accel_norm)


def body_from_sensor(vector):
    """把传感器系向量转到身体系，供其它模块复用。/ Convert a sensor-frame vector into the body frame."""
    return matrix_vector(R_BS, vector)


def _finite_vector(vector):
    """检查三轴输入，不修改原始测量。/ Validate three-axis input without altering raw measurements."""
    if len(vector) != 3 or not all(math.isfinite(x) for x in vector):
        raise ValueError('measurement must contain three finite values')
