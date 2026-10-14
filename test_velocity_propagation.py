"""速度积分解析解与边界测试。/ Analytic and boundary tests for velocity integration."""
import unittest

from gravity_removal import remove_gravity
from velocity_propagation import VelocityState, propagate_velocity


class VelocityTest(unittest.TestCase):
    def step(self, state, time, accel, limit=1.0):
        return propagate_velocity(state, timestamp_us=time,
                                  acceleration_world_mps2=accel, max_dt_s=limit)

    def test_zero_acceleration_preserves_nonzero_velocity(self):
        state = VelocityState(0, (1, -2, 3), (0, 0, 0))
        self.assertEqual(self.step(state, 20000, (0, 0, 0)).velocity_world_mps, (1, -2, 3))

    def test_constant_acceleration_with_irregular_times(self):
        state = VelocityState(0, (1, 2, 3), (2, -4, 1))
        for time in (10000, 35000, 100000, 1000000):
            state = self.step(state, time, (2, -4, 1))
        for actual, expected in zip(state.velocity_world_mps, (3, -2, 4)):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_linear_ramp_matches_analytic_integral(self):
        state = VelocityState(0, (0, 0, 0), (0, 0, 0))
        for time in (100000, 400000, 1000000):
            state = self.step(state, time, (2*time/1e6, 0, 0))
        self.assertAlmostEqual(state.velocity_world_mps[0], 1)

    def test_gravity_removal_connection(self):
        sample = remove_gravity((9.80665, 0, -2), (1, 0, 0, 0),
                                timestamp_us=0, attitude_timestamp_us=0, attitude_valid=True)
        state = VelocityState(0, (0, 0, 0), sample.linear_acceleration_world_mps2)
        self.assertEqual(self.step(state, 1000000, sample.linear_acceleration_world_mps2).velocity_world_mps,
                         (2, 0, 0))

    def test_gap_duplicate_and_reversed_time_leave_input_unchanged(self):
        state = VelocityState(20000, (1, 2, 3), (0, 0, 0))
        for time in (20000, 19999, 220001):
            with self.subTest(time=time), self.assertRaises(ValueError):
                self.step(state, time, (0, 0, 0), limit=0.2)
        self.assertEqual(state.velocity_world_mps, (1, 2, 3))
        self.step(state, 220000, (0, 0, 0), limit=0.2)

    def test_external_correction_and_input_copy(self):
        velocity = [3, 2, 1]
        state = VelocityState(0, velocity, (1, 0, 0))
        velocity[0] = 99
        corrected = VelocityState(state.timestamp_us, (0, 0, 0), state.acceleration_world_mps2)
        self.assertEqual(self.step(corrected, 1000000, (1, 0, 0)).velocity_world_mps, (1, 0, 0))
        self.assertEqual(state.velocity_world_mps, (3, 2, 1))

    def test_invalid_inputs(self):
        state = VelocityState(0, (0, 0, 0), (0, 0, 0))
        for time in (True, -1, 2.5):
            with self.assertRaises(ValueError):
                self.step(state, time, (0, 0, 0))
        for accel in ((0, 0), (float('nan'), 0, 0), (float('inf'), 0, 0), None):
            with self.assertRaises(ValueError):
                self.step(state, 20000, accel)
        for limit in (0, -1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                self.step(state, 20000, (0, 0, 0), limit)

    def test_overflow_rejected(self):
        state = VelocityState(0, (1.7e308, 0, 0), (1.7e308, 0, 0))
        with self.assertRaises(ValueError):
            self.step(state, 1000000, (1.7e308, 0, 0))


if __name__ == '__main__':
    unittest.main()
