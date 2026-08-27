from __future__ import annotations

import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.geometry import Point2D  # type: ignore  # noqa: E402
from easytowing.linkage import (  # type: ignore  # noqa: E402
    PlanarLinkageBranchHint,
    PlanarLinkageSpec,
    LinkageNoSolutionError,
    driver_point_arc,
    solve_planar_linkage,
    solve_planar_linkage_sweep,
)


class PlanarLinkageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = PlanarLinkageSpec(
            id="demo_linkage",
            steering_pivot=Point2D(560.0, 180.0),
            steering_arm_length_mm=180.0,
            steering_arm_neutral_angle_rad=0.0,
            bell_crank_pivot=Point2D(0.0, 0.0),
            bell_crank_input_arm_length_mm=200.0,
            bell_crank_input_neutral_angle_rad=0.0,
            bell_crank_output_arm_length_mm=180.0,
            bell_crank_output_neutral_angle_rad=math.pi / 2.0,
            input_rod_length_mm=120.0,
            tie_rod_length_mm=560.0,
        )
        self.branch_hint = PlanarLinkageBranchHint(
            input_endpoint=Point2D(200.0, 0.0),
            steering_endpoint=Point2D(740.0, 180.0),
        )

    def test_single_solution_preserves_lengths(self) -> None:
        driver_point = Point2D(200.0, 120.0)
        state = solve_planar_linkage(
            self.spec,
            driver_point,
            branch_hint=self.branch_hint,
        )

        self.assertTrue(math.isclose((state.input_endpoint - self.spec.bell_crank_pivot).length(), self.spec.bell_crank_input_arm_length_mm, abs_tol=1e-9))
        self.assertTrue(math.isclose((state.driver_point - state.input_endpoint).length(), self.spec.input_rod_length_mm, abs_tol=1e-9))
        self.assertTrue(math.isclose((state.output_endpoint - self.spec.bell_crank_pivot).length(), self.spec.bell_crank_output_arm_length_mm, abs_tol=1e-9))
        self.assertTrue(math.isclose((state.steering_endpoint - self.spec.steering_pivot).length(), self.spec.steering_arm_length_mm, abs_tol=1e-9))
        self.assertTrue(math.isclose((state.steering_endpoint - state.output_endpoint).length(), self.spec.tie_rod_length_mm, abs_tol=1e-9))
        self.assertTrue(abs(state.input_stage_error_mm) < 1e-9)
        self.assertTrue(abs(state.tie_rod_error_mm) < 1e-9)

    def test_continuous_sweep_keeps_branch(self) -> None:
        driver_points = (
            Point2D(200.0, 120.0),
            Point2D(210.0, 125.0),
            Point2D(220.0, 130.0),
            Point2D(230.0, 135.0),
        )
        sweep = solve_planar_linkage_sweep(
            self.spec,
            driver_points,
            branch_hint=self.branch_hint,
        )

        self.assertTrue(sweep.succeeded)
        self.assertEqual(len(sweep.states), len(driver_points))
        self.assertEqual(len({state.input_branch_index for state in sweep.states}), 1)
        self.assertEqual(len({state.steering_branch_index for state in sweep.states}), 1)
        for state in sweep.states:
            self.assertTrue(abs(state.input_stage_error_mm) < 1e-9)
            self.assertTrue(abs(state.tie_rod_error_mm) < 1e-9)

        angle_changes = [
            abs(sweep.states[index + 1].steering_angle_rad - sweep.states[index].steering_angle_rad)
            for index in range(len(sweep.states) - 1)
        ]
        self.assertTrue(all(change < math.radians(20.0) for change in angle_changes))

    def test_impossible_mechanism_reports_failure(self) -> None:
        with self.assertRaises(LinkageNoSolutionError):
            solve_planar_linkage(
                self.spec,
                Point2D(900.0, 900.0),
                branch_hint=self.branch_hint,
            )

    def test_driver_point_arc_helper(self) -> None:
        point = driver_point_arc(Point2D(0.0, 0.0), 100.0, math.pi / 2.0)
        self.assertTrue(math.isclose(point.x_mm, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(point.y_mm, 100.0, abs_tol=1e-9))


if __name__ == "__main__":
    unittest.main()
