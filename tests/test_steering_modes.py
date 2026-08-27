from __future__ import annotations

import math
import unittest

from easytowing.geometry import Point2D
from easytowing.demo_server import _ideal_steering_request_payload
from easytowing.model import Axle, VehicleLayout
from easytowing.steering import solve_ideal_steering_from_radius


class SteeringModeTests(unittest.TestCase):
    def _vehicle(self, *axles: Axle) -> VehicleLayout:
        return VehicleLayout(id="mode_case", name="Steering mode case", axles=axles)

    def test_fixed_axle_does_not_follow_icr(self) -> None:
        vehicle = self._vehicle(
            Axle(
                id="fixed_axle",
                center=Point2D(2500.0, 0.0),
                track_mm=2400.0,
                steerable=False,
                steering_mode="FIXED",
            )
        )
        solution = solve_ideal_steering_from_radius(vehicle, 8000.0)
        axle = solution.axles[0]

        self.assertAlmostEqual(axle.center_steering_angle_rad, 0.0)
        self.assertAlmostEqual(axle.left_wheel.steering_angle_rad, 0.0)
        self.assertAlmostEqual(axle.right_wheel.steering_angle_rad, 0.0)

    def test_user_defined_axle_uses_explicit_center_angle(self) -> None:
        vehicle = self._vehicle(
            Axle(
                id="user_axle",
                center=Point2D(2500.0, 0.0),
                track_mm=2400.0,
                steering_mode="USER_DEFINED",
                user_defined_steering_angle_rad=math.radians(7.5),
            )
        )
        solution = solve_ideal_steering_from_radius(vehicle, 8000.0)
        axle = solution.axles[0]

        self.assertAlmostEqual(axle.center_steering_angle_rad, math.radians(7.5))
        self.assertAlmostEqual(axle.left_wheel.steering_angle_rad, math.radians(7.5))
        self.assertAlmostEqual(axle.right_wheel.steering_angle_rad, math.radians(7.5))

    def test_api_defaults_to_forced_steer_when_mode_is_omitted(self) -> None:
        payload = _ideal_steering_request_payload(
            {
                "turn_radius_mm": 8000.0,
                "axles": [{"id": "front", "x_mm": 2500.0, "track_mm": 2400.0}],
            }
        )

        self.assertEqual(payload["axles"][0]["steering_mode"], "FORCED_STEER")
        self.assertNotAlmostEqual(payload["axles"][0]["center_steering_angle_deg"], 0.0)


if __name__ == "__main__":
    unittest.main()
