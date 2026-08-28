from __future__ import annotations

import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.model import Axle, Point2D, VehicleLayout  # type: ignore  # noqa: E402
from easytowing.steering import (  # type: ignore  # noqa: E402
    ackermann_expected_angles,
    solve_ideal_steering_from_radius,
)


class AckermannGeometryTests(unittest.TestCase):
    def test_single_axle_ackermann_matches_closed_form(self) -> None:
        wheelbase_mm = 4360.0
        track_mm = 2500.0
        turn_radius_mm = 14000.0
        vehicle = VehicleLayout(
            id="test_vehicle",
            name="Test Vehicle",
            axles=(
                Axle(
                    id="front_axle",
                    center=Point2D(wheelbase_mm, 0.0),
                    track_mm=track_mm,
                ),
            ),
            body_length_mm=wheelbase_mm + 1200.0,
            body_width_mm=track_mm + 300.0,
        )

        solution = solve_ideal_steering_from_radius(vehicle, turn_radius_mm)
        axle_solution = solution.axles[0]
        expected_inner, expected_outer = ackermann_expected_angles(
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
            turn_radius_mm=turn_radius_mm,
        )

        self.assertTrue(math.isclose(axle_solution.left_wheel.heading_rad, expected_inner, rel_tol=1e-9, abs_tol=1e-9))
        self.assertTrue(math.isclose(axle_solution.right_wheel.heading_rad, expected_outer, rel_tol=1e-9, abs_tol=1e-9))

    def test_right_turn_mirrors_sign(self) -> None:
        wheelbase_mm = 4360.0
        track_mm = 2500.0
        turn_radius_mm = -14000.0
        vehicle = VehicleLayout(
            id="test_vehicle",
            name="Test Vehicle",
            axles=(
                Axle(
                    id="front_axle",
                    center=Point2D(wheelbase_mm, 0.0),
                    track_mm=track_mm,
                ),
            ),
            body_length_mm=wheelbase_mm + 1200.0,
            body_width_mm=track_mm + 300.0,
        )

        solution = solve_ideal_steering_from_radius(vehicle, turn_radius_mm)
        axle_solution = solution.axles[0]
        expected_inner, expected_outer = ackermann_expected_angles(
            wheelbase_mm=wheelbase_mm,
            track_mm=track_mm,
            turn_radius_mm=turn_radius_mm,
        )

        self.assertTrue(math.isclose(axle_solution.left_wheel.heading_rad, expected_outer, rel_tol=1e-9, abs_tol=1e-9))
        self.assertTrue(math.isclose(axle_solution.right_wheel.heading_rad, expected_inner, rel_tol=1e-9, abs_tol=1e-9))

    def test_straight_motion_returns_zero_angles(self) -> None:
        vehicle = VehicleLayout(
            id="straight_vehicle",
            name="Straight Vehicle",
            axles=(
                Axle(id="front_axle", center=Point2D(1000.0, 0.0), track_mm=2500.0),
                Axle(id="rear_axle", center=Point2D(-1000.0, 0.0), track_mm=2500.0),
            ),
            body_length_mm=3000.0,
            body_width_mm=3200.0,
        )

        solution = solve_ideal_steering_from_radius(vehicle, None)
        for angle in solution.wheel_angles_rad.values():
            self.assertTrue(math.isclose(angle, 0.0, abs_tol=1e-12))

    def test_multi_wheel_axle_solves_and_reports_every_wheel(self) -> None:
        vehicle = VehicleLayout(
            id="dual_wheel_vehicle",
            name="Dual wheel steering test",
            axles=(
                Axle(
                    id="front_axle",
                    center=Point2D(4360.0, 0.0),
                    track_mm=2800.0,
                    wheel_count=4,
                    wheel_lateral_offsets_mm=(1400.0, 1180.0, -1180.0, -1400.0),
                ),
            ),
            body_length_mm=5600.0,
            body_width_mm=3400.0,
        )

        solution = solve_ideal_steering_from_radius(vehicle, 14000.0)

        self.assertEqual(len(solution.axles[0].wheel_solutions), 4)
        self.assertEqual(set(solution.wheel_angles_rad), {
            "front_axle_left_1",
            "front_axle_left_2",
            "front_axle_right_1",
            "front_axle_right_2",
        })
        self.assertNotEqual(
            solution.wheel_angles_rad["front_axle_left_1"],
            solution.wheel_angles_rad["front_axle_left_2"],
        )


if __name__ == "__main__":
    unittest.main()
