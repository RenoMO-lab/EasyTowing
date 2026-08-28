from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.linkage import build_reference_linkage_demo  # type: ignore  # noqa: E402
from easytowing.demo_server import _parse_linkage_rig  # type: ignore  # noqa: E402
from easytowing.design_cases import DesignCase  # type: ignore  # noqa: E402
from easytowing.errors import OptimizationNoFeasibleSolutionError, SweepSampleLimitError  # type: ignore  # noqa: E402
from easytowing.graph_optimization import (  # type: ignore  # noqa: E402
    build_mechanism_graph_optimization_problem,
    optimize_mechanism_graph_problem,
)
from easytowing.mechanism_graph import (  # type: ignore  # noqa: E402
    MechanismAngleOutput,
    MechanismPoint,
    MechanismSteeringAssignment,
    PlanarMechanismGraph,
)
from easytowing.optimization import (  # type: ignore  # noqa: E402
    LinkageOptimizationProblem,
    OptimizationVariable,
    OptimizationWeights,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)
from easytowing.steering import build_demo_solution  # type: ignore  # noqa: E402
from easytowing.geometry import Point2D  # type: ignore  # noqa: E402
from easytowing.model import ArticulationJoint, Axle, MountedAxle, Pose2D, RigidBody, SteeringSynchronization, VehicleCombination  # type: ignore  # noqa: E402


class OptimizationEngineTests(unittest.TestCase):
    def test_reference_problem_runs(self) -> None:
        problem = build_reference_optimization_problem(mode="quick")
        result = optimize_linkage_problem(problem)

        self.assertEqual(len(result.baseline_variables), len(problem.variables))
        self.assertEqual(len(result.optimized_variables), len(problem.variables))
        self.assertGreater(result.evaluations, 0)
        self.assertGreaterEqual(result.baseline_metrics.score, 0.0)
        self.assertGreaterEqual(result.optimized_metrics.score, 0.0)
        self.assertFalse(result.baseline_metrics.feasible)
        self.assertTrue(result.optimized_metrics.feasible)
        self.assertEqual(result.optimized_metrics.violations, ())

    def test_single_variable_problem_rejects_infeasible_result(self) -> None:
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

        with self.assertRaises(OptimizationNoFeasibleSolutionError) as raised:
            optimize_linkage_problem(problem)

        self.assertIn("COLLISION_DETECTED", raised.exception.violations)
        self.assertIn("MIN_CLEARANCE_VIOLATED", raised.exception.violations)

    def test_custom_rig_and_vehicle_are_optimized(self) -> None:
        rig = _parse_linkage_rig({"steering_arm_length_mm": 150.0})
        vehicle, _solution, _radius = build_demo_solution(0.0)
        problem = build_reference_optimization_problem(
            base_rig=rig,
            vehicle=vehicle,
        )

        result = optimize_linkage_problem(problem)
        variable = next(item for item in result.optimized_variables if item.id == "steering_arm_length_mm")
        self.assertEqual(variable.current, 150.0)
        self.assertTrue(result.optimized_metrics.feasible)
        self.assertGreaterEqual(result.optimized_metrics.minimum_clearance_mm or 0.0, 20.0)

    def test_multi_body_graph_optimizer_validates_the_full_articulation_range(self) -> None:
        combination = VehicleCombination(
            id="graph_train",
            name="Graph train",
            bodies=(
                RigidBody("rear", "Rear", Pose2D(), 600.0, 300.0),
                RigidBody("front", "Front", body_length_mm=600.0, body_width_mm=300.0),
            ),
            joints=(
                ArticulationJoint(
                    "hitch",
                    "rear",
                    "front",
                    Point2D(500.0, 0.0),
                    Point2D(-500.0, 0.0),
                ),
            ),
            mounted_axles=(
                MountedAxle(Axle("rear_axle", Point2D(0.0, 0.0), 100.0), "rear", Point2D(0.0, 0.0)),
                MountedAxle(Axle("front_axle", Point2D(0.0, 0.0), 100.0), "front", Point2D(0.0, 0.0)),
            ),
            root_body_id="rear",
            steering_synchronizations=(
                SteeringSynchronization(
                    "front_phase",
                    target_axle_id="front_axle",
                    source_axle_id="rear_axle",
                    mode="SAME_PHASE",
                ),
            ),
        )
        points = []
        outputs = []
        assignments = []
        for body_id, axle_id in (("rear", "rear_axle"), ("front", "front_axle")):
            for side, y_mm in (("left", 50.0), ("right", -50.0)):
                pivot_id = f"{body_id}_{side}_pivot"
                endpoint_id = f"{body_id}_{side}_endpoint"
                output_id = f"{body_id}_{side}_output"
                points.extend(
                    (
                        MechanismPoint(pivot_id, Point2D(0.0, y_mm), "fixed", body_id=body_id),
                        MechanismPoint(endpoint_id, Point2D(100.0, y_mm), "fixed", body_id=body_id),
                    )
                )
                outputs.append(MechanismAngleOutput(output_id, pivot_id, endpoint_id, 0.0))
                assignments.append(MechanismSteeringAssignment(output_id, f"{axle_id}_{side}"))
        graph = PlanarMechanismGraph("fixed_graph", tuple(points), (), tuple(outputs))

        problem = build_mechanism_graph_optimization_problem(
            combination=combination,
            graph=graph,
            drivers=(),
            assignments=tuple(assignments),
            beta_min_deg=-20.0,
            beta_max_deg=20.0,
            root_turn_radius_mm=9000.0,
            enabled_ids=(),
        )
        graph_result = optimize_mechanism_graph_problem(problem)

        self.assertEqual(graph_result.result.optimized_metrics.violations, ())
        self.assertTrue(graph_result.result.optimized_metrics.feasible)
        self.assertEqual(graph_result.result.optimized_metrics.solved_samples, 5)
        self.assertEqual(len(graph_result.optimized_assignments), 4)
        self.assertGreater(
            graph_result.result.optimized_metrics.max_abs_synchronization_error_deg or 0.0,
            0.0,
        )

    def test_multi_joint_graph_optimizer_uses_the_cartesian_joint_grid(self) -> None:
        combination = VehicleCombination(
            id="three_body_graph_train",
            name="Three body graph train",
            bodies=tuple(
                RigidBody(body_id, body_id.title(), Pose2D(), 500.0, 100.0)
                for body_id in ("tractor", "dolly", "trailer")
            ),
            joints=(
                ArticulationJoint("tractor_dolly", "tractor", "dolly", Point2D(1000.0, 0.0), Point2D(-1000.0, 0.0)),
                ArticulationJoint("dolly_trailer", "dolly", "trailer", Point2D(1000.0, 0.0), Point2D(-1000.0, 0.0)),
            ),
            mounted_axles=tuple(
                MountedAxle(Axle(f"{body_id}_axle", Point2D(0.0, 0.0), 100.0), body_id, Point2D(0.0, 0.0))
                for body_id in ("tractor", "dolly", "trailer")
            ),
            root_body_id="tractor",
        )
        points = []
        outputs = []
        assignments = []
        for body_id in ("tractor", "dolly", "trailer"):
            axle_id = f"{body_id}_axle"
            for side, y_mm in (("left", 30.0), ("right", -30.0)):
                pivot_id = f"{body_id}_{side}_pivot"
                endpoint_id = f"{body_id}_{side}_endpoint"
                output_id = f"{body_id}_{side}_output"
                points.extend(
                    (
                        MechanismPoint(pivot_id, Point2D(0.0, y_mm), "fixed", body_id=body_id),
                        MechanismPoint(endpoint_id, Point2D(40.0, y_mm), "fixed", body_id=body_id),
                    )
                )
                outputs.append(MechanismAngleOutput(output_id, pivot_id, endpoint_id, 0.0))
                assignments.append(MechanismSteeringAssignment(output_id, f"{axle_id}_{side}"))
        graph = PlanarMechanismGraph("three_body_fixed_graph", tuple(points), (), tuple(outputs))

        problem = build_mechanism_graph_optimization_problem(
            combination=combination,
            graph=graph,
            drivers=(),
            assignments=tuple(assignments),
            beta_min_deg=-10.0,
            beta_max_deg=10.0,
            root_turn_radius_mm=9000.0,
            enabled_ids=(),
            joint_ranges={
                "tractor_dolly": {"min_deg": -10.0, "max_deg": 10.0, "step_deg": 10.0},
                "dolly_trailer": {"min_deg": -10.0, "max_deg": 10.0, "step_deg": 10.0},
            },
        )
        graph_result = optimize_mechanism_graph_problem(problem)

        self.assertEqual(len(problem.joint_sample_values()), 9)
        self.assertEqual(graph_result.result.optimized_metrics.solved_samples, 9)
        self.assertEqual(graph_result.result.optimized_metrics.sample_count, 9)

    def test_graph_optimizer_rejects_design_cases_that_exceed_sample_budget(self) -> None:
        combination = VehicleCombination(
            id="bounded_graph_train",
            name="Bounded graph train",
            bodies=(
                RigidBody("tractor", "Tractor", Pose2D(), 500.0, 100.0),
                RigidBody("trailer", "Trailer", Pose2D(), 500.0, 100.0),
            ),
            joints=(
                ArticulationJoint(
                    "hitch",
                    "tractor",
                    "trailer",
                    Point2D(1000.0, 0.0),
                    Point2D(-1000.0, 0.0),
                ),
            ),
            root_body_id="tractor",
        )

        with self.assertRaises(SweepSampleLimitError):
            build_mechanism_graph_optimization_problem(
                combination=combination,
                graph=PlanarMechanismGraph(
                    "bounded_graph",
                    (MechanismPoint("origin", Point2D(0.0, 0.0), "fixed"),),
                    (),
                ),
                drivers=(),
                assignments=(),
                beta_min_deg=-10.0,
                beta_max_deg=10.0,
                maximum_samples=3,
                design_cases=(
                    DesignCase("outside_grid", "Outside grid", beta_deg=15.0),
                ),
            )


if __name__ == "__main__":
    unittest.main()
