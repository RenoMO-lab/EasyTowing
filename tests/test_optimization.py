from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.linkage import build_reference_linkage_demo  # type: ignore  # noqa: E402
from easytowing.demo_server import _parse_linkage_rig  # type: ignore  # noqa: E402
from easytowing.optimization import (  # type: ignore  # noqa: E402
    LinkageOptimizationProblem,
    OptimizationVariable,
    OptimizationWeights,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)
from easytowing.steering import build_demo_solution  # type: ignore  # noqa: E402


class OptimizationEngineTests(unittest.TestCase):
    def test_reference_problem_runs(self) -> None:
        problem = build_reference_optimization_problem(mode="quick")
        result = optimize_linkage_problem(problem)

        self.assertEqual(len(result.baseline_variables), len(problem.variables))
        self.assertEqual(len(result.optimized_variables), len(problem.variables))
        self.assertGreater(result.evaluations, 0)
        self.assertGreaterEqual(result.baseline_metrics.score, 0.0)
        self.assertGreaterEqual(result.optimized_metrics.score, 0.0)

    def test_single_variable_problem_improves_score(self) -> None:
        problem = LinkageOptimizationProblem(
            base_rig=build_reference_linkage_demo(),
            variables=(
                OptimizationVariable(
                    id="steering_arm_length_mm",
                    current=120.0,
                    minimum=100.0,
                    maximum=240.0,
                    enabled=True,
                    preferred=180.0,
                ),
            ),
            beta_samples_deg=(-30.0, -15.0, 0.0, 15.0, 30.0),
            clearance_target_mm=20.0,
            weights=OptimizationWeights(
                steering_error=1.0,
                clearance=12.0,
                clearance_violation=250.0,
                failure=100000.0,
                preferred=0.0,
                complexity=0.0,
            ),
            mode="quick",
            seed=11,
        )

        result = optimize_linkage_problem(problem)

        self.assertLess(result.optimized_metrics.score, result.baseline_metrics.score)
        self.assertNotEqual(result.optimized_variables[0].optimized, result.optimized_variables[0].current)

    def test_custom_rig_and_vehicle_are_optimized(self) -> None:
        rig = _parse_linkage_rig({"steering_arm_length_mm": 150.0})
        vehicle, _solution, _radius = build_demo_solution(0.0)
        problem = build_reference_optimization_problem(
            base_rig=rig,
            vehicle=vehicle,
            enabled_ids={"steering_arm_length_mm"},
        )

        result = optimize_linkage_problem(problem)
        variable = next(item for item in result.optimized_variables if item.id == "steering_arm_length_mm")
        self.assertEqual(variable.current, 150.0)
        self.assertLess(result.optimized_metrics.score, result.baseline_metrics.score)


if __name__ == "__main__":
    unittest.main()
