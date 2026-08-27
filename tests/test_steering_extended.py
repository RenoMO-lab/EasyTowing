from __future__ import annotations

import math
import unittest

from easytowing.errors import SteeringLimitExceededError
from easytowing.model import Axle, Point2D, VehicleLayout
from easytowing.steering import solve_ideal_steering_from_radius


class ExtendedSteeringTests(unittest.TestCase):
    def test_steering_angles_are_relative_to_articulated_axle_heading(self) -> None:
        vehicle = VehicleLayout(
            id="articulated_vehicle",
            name="Articulated Vehicle",
            axles=(
                Axle(
                    id="front_axle",
                    center=Point2D(1000.0, 0.0),
                    track_mm=2500.0,
                    heading_rad=math.radians(20.0),
                ),
            ),
        )

        axle = solve_ideal_steering_from_radius(vehicle, None).axles[0]
        self.assertTrue(math.isclose(axle.reference_heading_rad, math.radians(20.0), abs_tol=1e-12))
        self.assertTrue(math.isclose(axle.center_steering_angle_rad, 0.0, abs_tol=1e-12))
        self.assertTrue(math.isclose(axle.left_wheel.steering_angle_rad, 0.0, abs_tol=1e-12))

    def test_ideal_solver_reports_steering_stop_exceeded(self) -> None:
        vehicle = VehicleLayout(
            id="limited_vehicle",
            name="Limited Vehicle",
            axles=(
                Axle(
                    id="front_axle",
                    center=Point2D(1000.0, 0.0),
                    track_mm=2500.0,
                    steering_stop_deg=5.0,
                ),
            ),
        )

        with self.assertRaises(SteeringLimitExceededError):
            solve_ideal_steering_from_radius(vehicle, 5000.0)


if __name__ == "__main__":
    unittest.main()
