from __future__ import annotations

import unittest

from easytowing.optimization import (
    OptimizationWeights,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)


class OptimizationObjectiveTests(unittest.TestCase):
    def test_custom_objective_is_carried_into_result(self) -> None:
        weights = OptimizationWeights(
            steering_error=2.0,
            clearance=3.0,
            clearance_violation=4.0,
            failure=5.0,
            preferred=0.6,
            complexity=0.7,
            synchronization_error=0.9,
        )
        problem = build_reference_optimization_problem(
            mode="quick",
            clearance_target_mm=15.0,
            weights=weights,
        )

        self.assertEqual(problem.clearance_target_mm, 15.0)
        self.assertIs(problem.weights, weights)
        result = optimize_linkage_problem(problem)
        self.assertEqual(result.clearance_target_mm, 15.0)
        self.assertIs(result.weights, weights)
        self.assertEqual(result.weights.to_dict()["synchronization_error"], 0.9)

    def test_negative_weight_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            OptimizationWeights(clearance=-0.1)

    def test_non_finite_clearance_target_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            build_reference_optimization_problem(clearance_target_mm=float("nan"))


if __name__ == "__main__":
    unittest.main()
