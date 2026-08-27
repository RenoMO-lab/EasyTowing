from __future__ import annotations

import math
import unittest

from easytowing.linkage import (
    build_reference_linkage_demo,
    driver_point_arc,
    solve_planar_linkage,
    solve_reference_linkage_demo,
)
from easytowing.optimization import build_reference_optimization_problem, optimize_linkage_problem
from easytowing.reporting import build_steering_sweep_bundle


class CompanionLinkageTests(unittest.TestCase):
    def test_reference_linkage_solves_both_steering_arms(self) -> None:
        rig = build_reference_linkage_demo()
        state = solve_reference_linkage_demo(30.0)

        self.assertIsNotNone(state.companion_steering_endpoint)
        self.assertIsNotNone(state.companion_steering_angle_rad)
        self.assertAlmostEqual(state.companion_tie_rod_error_mm, 0.0, places=6)
        assert rig.spec.companion_steering_pivot is not None
        assert rig.spec.companion_steering_arm_length_mm is not None
        self.assertTrue(
            math.isclose(
                (state.companion_steering_endpoint - rig.spec.companion_steering_pivot).length(),
                rig.spec.companion_steering_arm_length_mm,
                abs_tol=0.01,
            )
        )

    def test_companion_branch_is_continuous(self) -> None:
        rig = build_reference_linkage_demo()
        state = None
        for beta_deg in range(-45, 46, 5):
            driver = driver_point_arc(
                rig.driver_arc_center,
                rig.driver_arc_radius_mm,
                math.radians(beta_deg),
            )
            state = solve_planar_linkage(
                rig.spec,
                driver,
                previous_state=state,
                branch_hint=rig.branch_hint if state is None else None,
            )
        self.assertIsNotNone(state)
        self.assertIsNotNone(state.companion_branch_index)

    def test_optimization_reports_inner_and_outer_wheel_errors(self) -> None:
        result = optimize_linkage_problem(build_reference_optimization_problem(mode="quick"))
        self.assertIsNotNone(result.baseline_metrics.max_abs_inner_error_deg)
        self.assertIsNotNone(result.optimized_metrics.max_abs_outer_error_deg)

    def test_sweep_exposes_companion_actual_wheel(self) -> None:
        sweep = build_steering_sweep_bundle(step_deg=15.0)
        sample = sweep["samples"][0]
        self.assertIn("baseline_front_right_deg", sample)
        self.assertIn("optimized_front_right_error_deg", sample)


if __name__ == "__main__":
    unittest.main()
