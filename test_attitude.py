"""姿态模块单元测试，覆盖已经出现过的三类错误。
Unit tests for the attitude module, covering the three defects found during validation.

运行 / Run: python test_attitude.py
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from attitude import (AttitudeEstimator, R_BS, body_from_sensor, euler_from_quat,  # noqa: E402
                      matrix_vector, quat_angle, quat_multiply, quat_normalize,
                      specific_force_expected_body, quat_to_matrix)

GRAVITY = 9.80665
# 真实佩戴站立时实测的比力方向（传感器系，单位向量）/ measured standing specific-force direction
# in the sensor frame, as a unit vector.
STANDING = (0.993, 0.102, -0.057)


def rotation_matrix_determinant(matrix):
    return (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]))


class MountingTransformTest(unittest.TestCase):
    def test_is_proper_rotation(self):
        # 必须是正常旋转（行列式 +1），否则转速向量不能用同一矩阵变换。
        # Must be a proper rotation, otherwise angular rates cannot share the transform.
        self.assertAlmostEqual(rotation_matrix_determinant(R_BS), 1.0, places=12)

    def test_standing_specific_force_is_mostly_up(self):
        # 已确认佩戴时 +X 向上、+Z 向后；站立比力朝上，应主要落在身体 +z，且水平分量很小。
        # Confirmed worn orientation: +X up, +Z back. The standing specific force is up, so it must
        # land mostly on body +z with small horizontal components.
        body = body_from_sensor(STANDING)
        self.assertGreater(body[2], 0.95)
        self.assertLess(abs(body[0]), 0.2)

    def test_alternative_mapping_would_put_standing_force_down(self):
        # 若把传感器 x 轴理解为朝下（silk x 指向膝盖的读法），站立比力会落到身体 −z，与站立冲突。
        # Reading sensor x as pointing down (the "silk x points at the knee" reading) would put the
        # standing specific force on body -z, contradicting the standing condition.
        alternative = ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
        body = matrix_vector(alternative, STANDING)
        self.assertLess(body[2], -0.9)

    def test_seated_handheld_is_not_mounting_evidence(self):
        # 坐姿手持录制的"大腿水平"一组按本映射竖直分量接近零，说明那次手持朝向与佩戴不同，
        # 只作为算法交叉检查，不作为安装依据。
        # The seated hand-held "thigh horizontal" pose maps to a near-zero vertical component, so
        # that hand-held orientation differs from the worn one and is a cross-check only.
        body = body_from_sensor((-0.018, 0.027, 9.88))
        self.assertLess(abs(body[2]), 0.1)


class IntegrationSignTest(unittest.TestCase):
    def test_yaw_follows_gyro_sign(self):
        # 竖直轴方向的角速度必须让 yaw 增大；这一条抓出过四元数乘法顺序错误。
        # A rate along the vertical axis must increase yaw; this caught a reversed quaternion order.
        estimator = AttitudeEstimator(integral_gain=0.0, tilt_only=False)
        # 传感器 +X 朝上且世界 +z 朝上时，竖直方向约等于 sensor +X。
        # With sensor +X up and world +z up, vertical is approximately sensor +X.
        estimator.align_to_gravity((GRAVITY, 0.0, 0.0))
        rate = 0.2
        for step in range(1, 251):
            estimator.update(step * 0.02, (GRAVITY, 0.0, 0.0), (rate, 0.0, 0.0))
        yaw = euler_from_quat(estimator.q)[2]
        self.assertGreater(yaw, 0.0)
        self.assertAlmostEqual(yaw, rate * 5.0, delta=0.2)

    def test_yaw_does_not_drift_with_vertical_bias_declared(self):
        # 竖直方向零偏已知时，静止段 yaw 不应漂移。
        # With the vertical bias declared, yaw must not drift during a still interval.
        estimator = AttitudeEstimator(integral_gain=0.0)
        estimator.align_to_gravity((GRAVITY, 0.0, 0.0))
        estimator.calibrate_still([(i * 0.02, (GRAVITY, 0.0, 0.0), (0.03, 0.0, 0.0)) for i in range(50)])
        for step in range(1, 501):
            estimator.update(step * 0.02, (GRAVITY, 0.0, 0.0), (0.03, 0.0, 0.0))
        yaw = euler_from_quat(estimator.q)[2]
        self.assertAlmostEqual(yaw, 0.0, delta=0.02)

    def test_tilt_is_bounded_under_constant_bias(self):
        # 常值零偏下，20 秒内倾角不应被推飞；这一条抓出过校正符号反向的正反馈。
        # A constant bias must not drive the tilt away; this caught the reversed correction sign.
        estimator = AttitudeEstimator(integral_gain=0.0)
        estimator.align_to_gravity((GRAVITY, 0.0, 0.0))
        for step in range(1, 1001):
            estimator.update(step * 0.02, (GRAVITY, 0.0, 0.0), (0.02, -0.01, 0.005))
        roll, pitch, _ = euler_from_quat(estimator.q)
        self.assertLess(abs(math.degrees(roll)), 10.0)
        self.assertLess(abs(math.degrees(pitch)), 10.0)

    def test_error_direction_carries_estimate_to_measurement(self):
        # 误差叉积方向必须把估计推向实测方向；这一条抓出过取反后发散的正反馈。
        # The cross-product error must drive the estimate toward the measurement; the reversed order
        # used to diverge and this test pins the direction.
        estimator = AttitudeEstimator(integral_gain=0.0, tilt_only=False)
        estimator.q = (1.0, 0.0, 0.0, 0.0)
        # 静止实测比力仍然是身体 +z，但当前估计已经绕 +y 倾斜。
        # The measurement stays at body +z while the current estimate is tilted about +y.
        estimator.q = quat_normalize((math.cos(0.05), 0.0, math.sin(0.05), 0.0))
        state = estimator.update(0.0, (GRAVITY, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertIsNotNone(state['correction'])
        self.assertLess(state['correction'][1], 0.0)  # 需要绕 -y 回转 / must rotate back about -y
        for i in range(1, 1001):
            estimator.update(i * 0.02, (GRAVITY, 0.0, 0.0), (0.0, 0.0, 0.0))
        roll, pitch, _ = euler_from_quat(estimator.q)
        self.assertLess(abs(math.degrees(pitch)), 0.1)  # 朝 0 靠近 / moves toward zero

    def test_vertical_bias_calibration(self):
        # 静止标定必须把不可观测的重直分量估出来。/ Still calibration must estimate the vertical part.
        estimator = AttitudeEstimator(integral_gain=0.0)
        # 理想站立：传感器系比力为 (+g, 0, 0)，即 sensor +X 朝上。
        # Ideal standing: sensor specific force is (+g, 0, 0), i.e. sensor +X up.
        samples = [(i * 0.02, (GRAVITY, 0.0, 0.0), (0.03, 0.0, 0.0)) for i in range(100)]
        bias = estimator.calibrate_still(samples)
        # 传感器 +X 对应身体 +z，因此绕传感器 x 的转速落在身体 z 方向。
        # Sensor +X maps to body +z, so a rate about sensor x lies along body z.
        self.assertAlmostEqual(bias[2], 0.03, delta=1e-6)
        self.assertAlmostEqual(bias[0], 0.0, delta=1e-9)
        self.assertAlmostEqual(bias[1], 0.0, delta=1e-9)

    def test_gap_is_not_bridged(self):
        # 长间断不能跨越积分。/ A long gap must not be integrated across.
        estimator = AttitudeEstimator(integral_gain=0.0)
        estimator.align_to_gravity((GRAVITY, 0.0, 0.0))
        estimator.update(0.0, (GRAVITY, 0.0, 0.0), (0.0, 0.0, 1.0))
        before = estimator.q
        state = estimator.update(5.0, (GRAVITY, 0.0, 0.0), (0.0, 0.0, 1.0))
        self.assertIsNotNone(state['gap_s'])
        self.assertGreater(state['gap_s'], 0.2)
        for a, b in zip(before, estimator.q):
            self.assertAlmostEqual(a, b, places=9)


class QuaternionTest(unittest.TestCase):
    def test_multiply_matches_matrix_product(self):
        a = quat_normalize((0.9, 0.1, -0.2, 0.3))
        b = quat_normalize((0.5, -0.4, 0.2, 0.7))
        product = quat_multiply(a, b)
        self.assertAlmostEqual(sum(x * x for x in product), 1.0, places=9)

    def test_quat_angle_helper(self):
        self.assertAlmostEqual(quat_angle((1.0, 0.0, 0.0, 0.0),
                                          (math.cos(0.1), math.sin(0.1), 0.0, 0.0)), 11.459, delta=0.01)

    def test_specific_force_matches_gravity_convention(self):
        # 理想朝向时，期望比力应为世界 +z 方向。/ At the identity attitude the expected specific
        # force is world +z.
        expected = specific_force_expected_body((1.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(expected[0], 0.0, places=9)
        self.assertAlmostEqual(expected[1], 0.0, places=9)
        self.assertAlmostEqual(expected[2], 1.0, places=9)


def sensor_from_body(vector):
    # 固定逆映射，独立于被测矩阵实现。/ Fixed inverse mapping independent of the matrix under test.
    x, y, z = vector
    return (z, y, -x)


class PhysicalRegressionTest(unittest.TestCase):
    def test_default_mode_preserves_turn_after_bias_calibration(self):
        # 已标定零偏叠加真实转身：默认模式必须保留正反两个方向的 90 度转动。
        # With calibrated bias plus real rotation, default mode must preserve both 90-degree turns.
        for sign in (-1, 1):
            est = AttitudeEstimator()
            est.calibrate_still([(0, (GRAVITY, 0, 0), (0.03, 0, 0))] * 100)
            for i in range(1001):
                est.update(i * 0.01, (GRAVITY, 0, 0), (0.03 + sign * math.pi / 20, 0, 0))
            self.assertAlmostEqual(math.degrees(euler_from_quat(est.q)[2]), sign * 90, delta=0.01)
            self.assertAlmostEqual(est.bias[2], 0.03, places=10)

    def test_calibrated_bias_applies_without_accel_trust(self):
        # 自由落体类输入没有重力校正，但仍必须扣除三轴零偏。
        # Free-fall-like input provides no gravity correction, but all bias must still be removed.
        est = AttitudeEstimator()
        w = (0.03, -0.01, 0.02)
        est.calibrate_still([(0, (GRAVITY, 0, 0), w)])
        for i in range(1001):
            est.update(i * 0.01, (0, 0, 0), w)
        self.assertLess(quat_angle(est.q, (1, 0, 0, 0)), 1e-6)

    def test_body_rate_at_nonidentity_attitude(self):
        # 初始绕世界 z 轴 90 度，再绕身体 x 转 60 度；解析终点区分左右乘法。
        # Start at world yaw 90, then rotate body x by 60; analytic endpoint distinguishes products.
        c, h = math.cos(math.pi / 4), math.sin(math.pi / 4)
        est = AttitudeEstimator(proportional_gain=0, integral_gain=0)
        est.reset((c, 0, 0, h))
        for i in range(1001):
            est.update(i * 0.01, (0, 0, 0), sensor_from_body((math.pi / 30, 0, 0)))
        u, v = math.cos(math.pi / 6), math.sin(math.pi / 6)
        self.assertLess(quat_angle(est.q, (c*u, c*v, h*v, h*u)), 0.001)

    def test_gravity_inverse_and_tilt_convergence(self):
        # 解析的 yaw 60、roll 30 姿态，其身体比力为 (0, g/2, sqrt(3)g/2)。
        # Analytic yaw 60, roll 30 pose has body force (0, g/2, sqrt(3)g/2).
        cy, sy = math.cos(math.pi/6), math.sin(math.pi/6)
        cr, sr = math.cos(math.pi/12), math.sin(math.pi/12)
        q = (cy*cr, cy*sr, sy*sr, sy*cr)
        expected = (0, 0.5, math.sqrt(3)/2)
        for a, b in zip(specific_force_expected_body(q), expected):
            self.assertAlmostEqual(a, b, places=12)
        est = AttitudeEstimator(integral_gain=0)
        est.reset(q)
        est.align_to_gravity((GRAVITY, 0, 0))
        world = matrix_vector(quat_to_matrix(est.q), (0, 0, GRAVITY))
        self.assertAlmostEqual(world[2], GRAVITY, places=10)
        est.reset(q)
        for i in range(2001):
            est.update(i * 0.01, (GRAVITY, 0, 0), (0, 0, 0))
        self.assertLess(abs(euler_from_quat(est.q)[0]), 0.0001)

    def test_alignment_is_idempotent_and_handles_inversion(self):
        # 倾斜和倒置均应正确对齐；再次对齐不改变姿态。
        # Tilt and inversion must align correctly; repeating alignment must not change attitude.
        for a in ((-GRAVITY, 0, 0), (8, 3, 4)):
            est = AttitudeEstimator()
            est.align_to_gravity(a)
            before = est.q
            est.align_to_gravity(a)
            self.assertLess(quat_angle(before, est.q), 1e-5)
            world = matrix_vector(quat_to_matrix(est.q), body_from_sensor(a))
            self.assertAlmostEqual(world[0], 0, places=10)
            self.assertAlmostEqual(world[1], 0, places=10)
            self.assertGreater(world[2], 0)


class StageReadinessTest(unittest.TestCase):
    def test_combined_yaw_pitch_trajectory(self):
        # 独立解析轨迹：同时转航向和倾角，生成身体角速度与比力。
        # Independent analytic trajectory: simultaneous yaw and pitch, with body rates and force.
        est = AttitudeEstimator()
        dt = 0.002
        for i in range(5001):
            t = i * dt
            yaw, pitch = 0.12*t, 0.07*t
            rate = (-0.12*math.sin(pitch), 0.07, 0.12*math.cos(pitch))
            force = (-GRAVITY*math.sin(pitch), 0, GRAVITY*math.cos(pitch))
            est.update(t, sensor_from_body(force), sensor_from_body(rate))
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        truth = (cy*cp, -sy*sp, cy*sp, sy*cp)
        self.assertLess(quat_angle(est.q, truth), 0.05)

    def test_invalid_sample_preserves_state(self):
        # 无效输入不破坏此前姿态、零偏与时间。/ Invalid input preserves attitude, bias and time.
        est = AttitudeEstimator()
        est.update(0, (GRAVITY, 0, 0), (0, 0, 0))
        before = (est.q, tuple(est.bias), est.previous_time)
        for t,a,w in ((float('nan'), (1,2,3), (0,0,0)),
                      (1, (float('inf'),0,0), (0,0,0)),
                      (1, (1,2,3), (0,float('nan'),0))):
            with self.assertRaises(ValueError):
                est.update(t,a,w)
            self.assertEqual(before, (est.q, tuple(est.bias), est.previous_time))

    def test_large_calibrated_bias_is_not_clipped(self):
        # 标定值可超出在线修正限幅；不得把它裁掉重新造成漂移。
        # Calibration can exceed adaptation limits; clipping it would reintroduce drift.
        est = AttitudeEstimator()
        est.calibrate_still([(0, (GRAVITY,0,0), (0.1,0,0))])
        for i in range(501):
            est.update(i*0.02, (GRAVITY,0,0), (0.1,0,0))
        self.assertLess(quat_angle(est.q, (1,0,0,0)), 1e-6)
        self.assertAlmostEqual(est.bias[2], 0.1)

    def test_constant_tilt_bias_converges_without_heading_lock(self):
        # 不锁航向；重力反馈仍能估计水平陀螺偏差并收敛倾角。
        # Without locking heading, gravity feedback still estimates horizontal bias and settles tilt.
        est = AttitudeEstimator()
        for i in range(10001):
            est.update(i*0.02, (GRAVITY,0,0), (0,0.02,0))
        self.assertAlmostEqual(est.bias[1], 0.02, delta=0.001)
        self.assertLess(abs(euler_from_quat(est.q)[1]), 0.001)


if __name__ == '__main__':
    unittest.main(verbosity=2)
