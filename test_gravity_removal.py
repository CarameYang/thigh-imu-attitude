"""去重力物理与接口测试。/ Physical and interface tests for gravity removal."""
import math
import unittest
from gravity_removal import remove_gravity

G = 9.80665


def run(force, q=(1, 0, 0, 0), **overrides):
    args = dict(timestamp_us=1000, attitude_timestamp_us=1000, attitude_valid=True)
    args.update(overrides)
    return remove_gravity(force, q, **args).linear_acceleration_world_mps2


class GravityRemovalTest(unittest.TestCase):
    def assert_vector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=10)

    def test_still_upright_and_input_preserved(self):
        force = [G, 0, 0]
        self.assert_vector(run(force), (0, 0, 0))
        self.assertEqual(force, [G, 0, 0])

    def test_still_tilted_and_inverted(self):
        # 独立解析绕身体 y 的姿态，包括90度与倒置。
        # Independent analytic body-y attitudes, including 90 degrees and inversion.
        for theta in (0.3, -0.7, math.pi/2, math.pi):
            q = (math.cos(theta/2), 0, math.sin(theta/2), 0)
            force = (G*math.cos(theta), 0, G*math.sin(theta))
            self.assert_vector(run(force, q), (0, 0, 0))

    def test_real_linear_acceleration_is_preserved(self):
        # 身体与世界重合时，前向2、左向3、向上4不能被当重力删掉。
        # With body aligned to world, forward 2, left 3 and up 4 must remain.
        self.assert_vector(run((G+4, 3, -2)), (2, 3, 4))

    def test_yaw_rotates_horizontal_acceleration(self):
        q = (math.sqrt(0.5), 0, 0, math.sqrt(0.5))
        self.assert_vector(run((G, 0, -2), q), (0, 2, 0))

    def test_tilted_linear_acceleration_and_quaternion_sign(self):
        # 身体绕y转90度，独立构造世界线加速度(2,3,4)；q与-q等价。
        # At body pitch 90, independently construct world acceleration (2,3,4); q and -q agree.
        q = (math.sqrt(0.5), 0, math.sqrt(0.5), 0)
        force = (2, 3, G+4)
        self.assert_vector(run(force, q), (2, 3, 4))
        self.assert_vector(run(force, tuple(-v for v in q)), (2, 3, 4))

    def test_free_fall_is_downward_not_zero(self):
        self.assert_vector(run((0, 0, 0)), (0, 0, -G))

    def test_reject_stale_invalid_and_mismatched_attitudes(self):
        for overrides in (dict(attitude_valid=False), dict(attitude_timestamp_us=999),
                          dict(timestamp_us=-1), dict(timestamp_us=1.5)):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                run((G, 0, 0), **overrides)

    def test_reject_invalid_vectors_and_gravity(self):
        for force, q in (((float('nan'),0,0),(1,0,0,0)), ((G,0),(1,0,0,0)),
                         ((G,0,0),(0,0,0,0)), ((G,0,0),(2,0,0,0)),
                         ((G,0,0),(float('inf'),0,0,0))):
            with self.assertRaises(ValueError):
                run(force,q)
        for g in (0,-1,float('nan')):
            with self.assertRaises(ValueError):
                run((G,0,0),gravity_mps2=g)


if __name__ == '__main__':
    unittest.main()
