import math
import unittest

from module_robot_localization.heading_initializer import HeadingInitializer


class TestHeadingMath(unittest.TestCase):
    def test_enu_delta_uses_rep103_east_north_axes(self):
        east, north = HeadingInitializer._enu_delta((0.0, 0.0), (0.001, 0.001))

        self.assertAlmostEqual(east, 111.319, delta=0.001)
        self.assertAlmostEqual(north, 111.319, delta=0.001)

    def test_enu_delta_wraps_across_antimeridian(self):
        east, north = HeadingInitializer._enu_delta(
            (0.0, 179.999),
            (0.0, -179.999),
        )

        self.assertAlmostEqual(east, 222.639, delta=0.001)
        self.assertAlmostEqual(north, 0.0, delta=1.0e-9)

    def test_circular_statistics_handle_pi_wrap(self):
        values = (math.radians(179.0), math.radians(-179.0))

        mean = HeadingInitializer._circular_mean(values)
        self.assertAlmostEqual(abs(mean), math.pi)
        self.assertLess(
            HeadingInitializer._circular_std(values),
            math.radians(1.1),
        )

    def test_normalize_angle(self):
        cases = (
            (0.0, 0.0),
            (3.0 * math.pi, math.pi),
            (-3.0 * math.pi, -math.pi),
        )

        for value, expected in cases:
            with self.subTest(value=value):
                self.assertAlmostEqual(
                    HeadingInitializer._normalize_angle(value),
                    expected,
                )
