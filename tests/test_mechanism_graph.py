from __future__ import annotations

from dataclasses import replace
import math
import unittest

from easytowing.errors import InvalidGeometryError, LinkageNoSolutionError
from easytowing.actual_steering import solve_actual_steering_from_graph
from easytowing.clearance_model import build_mechanism_graph_clearance_items
from easytowing.collision import analyze_clearance
from easytowing.geometry import Point2D, heading_vector
from easytowing.linkage import (
    build_reference_linkage_demo,
    driver_point_arc,
    solve_reference_linkage_demo,
)
from easytowing.mechanism_graph import (
    MechanismAngleOutput,
    MechanismGraphState,
    MechanismPoint,
    MechanismDriverArc,
    MechanismSteeringAssignment,
    PlanarMechanismGraph,
    RigidMember,
    planar_linkage_to_mechanism_graph,
    solve_mechanism_graph,
    resolve_driver_arc_positions,
)
from easytowing.model import Pose2D, VehicleLayout
from easytowing.steering import build_demo_solution


class MechanismGraphTests(unittest.TestCase):
    def test_graph_adapter_matches_analytical_linkage_sweep(self) -> None:
        rig = build_reference_linkage_demo()
        neutral_driver = driver_point_arc(
            rig.driver_arc_center,
            rig.driver_arc_radius_mm,
            0.0,
        )
        graph = planar_linkage_to_mechanism_graph(
            rig.spec,
            driver_neutral_position=neutral_driver,
            branch_hint=rig.branch_hint,
        )
        previous = None

        for beta_deg in (-30.0, -15.0, 0.0, 15.0, 30.0):
            driver = driver_point_arc(
                rig.driver_arc_center,
                rig.driver_arc_radius_mm,
                math.radians(beta_deg),
            )
            state = solve_mechanism_graph(
                graph,
                {"driver": driver},
                previous_state=previous,
            )
            expected = solve_reference_linkage_demo(beta_deg)

            self.assertLessEqual(state.maximum_residual_mm, 0.01)
            self.assertAlmostEqual(
                state.output_angle_deg("steering"),
                expected.steering_angle_deg,
                places=3,
            )
            self.assertAlmostEqual(
                state.output_angle_deg("companion_steering"),
                expected.companion_steering_angle_deg or 0.0,
                places=3,
            )
            previous = state

    def test_graph_solves_a_third_chained_steering_arm(self) -> None:
        rig = build_reference_linkage_demo()
        neutral_driver = driver_point_arc(
            rig.driver_arc_center,
            rig.driver_arc_radius_mm,
            0.0,
        )
        graph = planar_linkage_to_mechanism_graph(
            rig.spec,
            driver_neutral_position=neutral_driver,
            branch_hint=rig.branch_hint,
        )
        third_pivot = Point2D(560.0, -540.0)
        third_neutral_angle = 0.0
        third_endpoint = third_pivot + heading_vector(third_neutral_angle).scale(180.0)
        companion_endpoint = next(
            point.neutral_position
            for point in graph.points
            if point.id == "companion_steering_endpoint"
        )
        graph = replace(
            graph,
            points=graph.points
            + (
                MechanismPoint("third_steering_pivot", third_pivot, "fixed"),
                MechanismPoint("third_steering_endpoint", third_endpoint),
            ),
            members=graph.members
            + (
                RigidMember(
                    "third_tie_rod",
                    "companion_steering_endpoint",
                    "third_steering_endpoint",
                    (third_endpoint - companion_endpoint).length(),
                    "rod",
                    14.0,
                ),
                RigidMember(
                    "third_steering_arm",
                    "third_steering_pivot",
                    "third_steering_endpoint",
                    180.0,
                    "arm",
                    14.0,
                ),
            ),
            angle_outputs=graph.angle_outputs
            + (
                MechanismAngleOutput(
                    "third_steering",
                    "third_steering_pivot",
                    "third_steering_endpoint",
                    third_neutral_angle,
                ),
            ),
        )
        driver = driver_point_arc(
            rig.driver_arc_center,
            rig.driver_arc_radius_mm,
            math.radians(30.0),
        )

        state = solve_mechanism_graph(graph, {"driver": driver})

        self.assertIn("third_steering", state.output_angles_rad)
        self.assertLessEqual(state.maximum_residual_mm, 0.01)
        self.assertNotAlmostEqual(state.output_angle_deg("third_steering"), 0.0, places=4)

    def test_underconstrained_graph_is_rejected(self) -> None:
        graph = PlanarMechanismGraph(
            id="underconstrained",
            points=(
                MechanismPoint("fixed", Point2D(0.0, 0.0), "fixed"),
                MechanismPoint("free", Point2D(100.0, 0.0)),
            ),
            members=(RigidMember("arm", "fixed", "free", 100.0, "arm"),),
        )

        with self.assertRaises(LinkageNoSolutionError):
            solve_mechanism_graph(graph)

    def test_driven_point_requires_an_explicit_driver_position(self) -> None:
        graph = PlanarMechanismGraph(
            id="missing_driver",
            points=(
                MechanismPoint("anchor", Point2D(0.0, 0.0), "fixed"),
                MechanismPoint("driver", Point2D(100.0, 0.0), "driven"),
            ),
            members=(RigidMember("rod", "anchor", "driver", 100.0),),
        )

        with self.assertRaisesRegex(InvalidGeometryError, "no resolved driver position"):
            solve_mechanism_graph(graph)

    def test_driver_position_cannot_be_assigned_to_a_free_point(self) -> None:
        graph = PlanarMechanismGraph(
            id="invalid_driver_target",
            points=(
                MechanismPoint("anchor", Point2D(0.0, 0.0), "fixed"),
                MechanismPoint("free", Point2D(100.0, 0.0)),
            ),
            members=(RigidMember("rod", "anchor", "free", 100.0),),
        )

        with self.assertRaisesRegex(ValueError, "Only driven mechanism points"):
            solve_mechanism_graph(graph, {"free": Point2D(100.0, 0.0)})

    def test_member_connectivity_is_derived_from_shared_points(self) -> None:
        graph = PlanarMechanismGraph(
            id="connectivity",
            points=(
                MechanismPoint("a", Point2D(0.0, 0.0), "fixed"),
                MechanismPoint("b", Point2D(100.0, 0.0)),
                MechanismPoint("c", Point2D(200.0, 0.0), "fixed"),
                MechanismPoint("d", Point2D(0.0, 100.0), "fixed"),
            ),
            members=(
                RigidMember("ab", "a", "b", 100.0),
                RigidMember("bc", "b", "c", 100.0),
                RigidMember("ad", "a", "d", 100.0),
            ),
        )

        connected = graph.connected_member_pairs()

        self.assertIn(frozenset(("ab", "bc")), connected)
        self.assertIn(frozenset(("ab", "ad")), connected)
        self.assertNotIn(frozenset(("bc", "ad")), connected)

    def test_graph_clearance_skips_shared_joints_but_detects_real_crossing(self) -> None:
        points = (
            MechanismPoint("a", Point2D(-10.0, 0.0), "fixed"),
            MechanismPoint("b", Point2D(0.0, 0.0), "fixed"),
            MechanismPoint("c", Point2D(10.0, 0.0), "fixed"),
            MechanismPoint("e", Point2D(-5.0, -10.0), "fixed"),
            MechanismPoint("f", Point2D(-5.0, 10.0), "fixed"),
        )
        graph = PlanarMechanismGraph(
            id="collision_connectivity",
            points=points,
            members=(
                RigidMember("ab", "a", "b", 10.0, envelope_radius_mm=1.0),
                RigidMember("bc", "b", "c", 10.0, envelope_radius_mm=1.0),
                RigidMember("ef", "e", "f", 20.0, envelope_radius_mm=1.0),
            ),
        )
        state = MechanismGraphState(
            point_positions={point.id: point.neutral_position for point in points},
            member_residuals_mm={member.id: 0.0 for member in graph.members},
            output_angles_rad={},
            iterations=0,
        )

        report = analyze_clearance(build_mechanism_graph_clearance_items(graph, state))
        pair_ids = {
            frozenset((pair.item_a_id, pair.item_b_id))
            for pair in report.pairs
        }

        self.assertNotIn(frozenset(("ab", "bc")), pair_ids)
        self.assertIn(frozenset(("ab", "ef")), pair_ids)
        self.assertTrue(report.collision_detected)

    def test_graph_clearance_does_not_hide_overlapping_connected_members(self) -> None:
        points = (
            MechanismPoint("a", Point2D(0.0, 0.0), "fixed"),
            MechanismPoint("b", Point2D(100.0, 0.0), "fixed"),
            MechanismPoint("c", Point2D(50.0, 0.0), "fixed"),
        )
        graph = PlanarMechanismGraph(
            id="overlapping_connected_members",
            points=points,
            members=(
                RigidMember("ab", "a", "b", 100.0, envelope_radius_mm=1.0),
                RigidMember("ac", "a", "c", 50.0, envelope_radius_mm=1.0),
            ),
        )
        state = MechanismGraphState(
            point_positions={point.id: point.neutral_position for point in points},
            member_residuals_mm={member.id: 0.0 for member in graph.members},
            output_angles_rad={},
            iterations=0,
        )

        report = analyze_clearance(build_mechanism_graph_clearance_items(graph, state))
        pair_ids = {
            frozenset((pair.item_a_id, pair.item_b_id))
            for pair in report.pairs
        }

        self.assertIn(frozenset(("ab", "ac")), pair_ids)
        self.assertTrue(report.collision_detected)

    def test_body_mounted_driver_arc_resolves_in_world_coordinates(self) -> None:
        graph = PlanarMechanismGraph(
            id="mounted_driver",
            points=(
                MechanismPoint(
                    "driver",
                    Point2D(0.0, 0.0),
                    "driven",
                    body_id="trailer",
                ),
            ),
            members=(),
        )
        driver = MechanismDriverArc("driver", Point2D(10.0, 0.0), 5.0)

        positions = resolve_driver_arc_positions(
            graph,
            (driver,),
            0.0,
            body_poses={"trailer": Pose2D(100.0, 50.0, math.pi / 2.0)},
        )

        self.assertAlmostEqual(positions["driver"].x_mm, 100.0, places=9)
        self.assertAlmostEqual(positions["driver"].y_mm, 65.0, places=9)

    def test_body_mounted_output_angle_excludes_body_yaw(self) -> None:
        graph = PlanarMechanismGraph(
            id="body_local_output",
            points=(
                MechanismPoint("pivot", Point2D(0.0, 0.0), "fixed", body_id="trailer"),
                MechanismPoint("endpoint", Point2D(100.0, 0.0), "fixed", body_id="trailer"),
            ),
            members=(),
            angle_outputs=(
                MechanismAngleOutput("steering", "pivot", "endpoint", 0.0),
            ),
        )

        state = solve_mechanism_graph(
            graph,
            {},
            body_poses={"trailer": Pose2D(100.0, 50.0, math.pi / 2.0)},
        )

        self.assertAlmostEqual(state.output_angle_deg("steering"), 0.0, places=9)

    def test_body_mounted_branch_continuity_is_measured_in_body_coordinates(self) -> None:
        graph = PlanarMechanismGraph(
            id="mounted_branch_continuity",
            points=(
                MechanismPoint("pivot_a", Point2D(0.0, 0.0), "fixed", body_id="trailer"),
                MechanismPoint("pivot_b", Point2D(100.0, 100.0), "fixed", body_id="trailer"),
                MechanismPoint("joint", Point2D(100.0, 0.0), body_id="trailer"),
            ),
            members=(
                RigidMember("arm_a", "pivot_a", "joint", 100.0),
                RigidMember("arm_b", "pivot_b", "joint", 100.0),
            ),
        )

        first_pose = Pose2D()
        first = solve_mechanism_graph(
            graph,
            body_poses={"trailer": first_pose},
            branch_tolerance_mm=1.0,
        )
        second = solve_mechanism_graph(
            graph,
            previous_state=first,
            body_poses={"trailer": Pose2D(yaw_rad=math.pi / 2.0)},
            branch_tolerance_mm=1.0,
        )

        self.assertLessEqual(second.maximum_residual_mm, 0.01)
        second_local = second.body_poses["trailer"].inverse_transform_point(
            second.point_positions["joint"]
        )
        self.assertAlmostEqual(second_local.x_mm, 100.0, places=6)
        self.assertAlmostEqual(second_local.y_mm, 0.0, places=6)

    def test_cross_body_output_is_measured_in_endpoint_body_frame(self) -> None:
        graph = PlanarMechanismGraph(
            id="cross_body_output",
            points=(
                MechanismPoint("tractor_pivot", Point2D(0.0, 0.0), "fixed", body_id="tractor"),
                MechanismPoint("trailer_endpoint", Point2D(100.0, 0.0), "fixed", body_id="trailer"),
            ),
            members=(),
            angle_outputs=(
                MechanismAngleOutput("steering", "tractor_pivot", "trailer_endpoint", 0.0),
            ),
        )

        state = solve_mechanism_graph(
            graph,
            {},
            body_poses={
                "tractor": Pose2D(0.0, 0.0, 0.0),
                "trailer": Pose2D(0.0, 100.0, math.pi / 2.0),
            },
        )

        self.assertAlmostEqual(state.output_angle_deg("steering"), 0.0, places=9)

    def test_named_graph_outputs_drive_named_wheels_on_multiple_axles(self) -> None:
        vehicle, _ideal, _radius = build_demo_solution(0.0)
        output_angles = {
            "front_left_output": math.radians(12.0),
            "front_right_output": math.radians(10.0),
            "rear_left_output": math.radians(-8.0),
            "rear_right_output": math.radians(-7.0),
        }
        graph_state = MechanismGraphState(
            point_positions={},
            member_residuals_mm={},
            output_angles_rad=output_angles,
            iterations=0,
        )
        assignments = tuple(
            MechanismSteeringAssignment(output_id, wheel_id)
            for output_id, wheel_id in (
                ("front_left_output", "front_axle_left"),
                ("front_right_output", "front_axle_right"),
                ("rear_left_output", "rear_axle_left"),
                ("rear_right_output", "rear_axle_right"),
            )
        )

        actual = solve_actual_steering_from_graph(vehicle, graph_state, assignments)

        self.assertAlmostEqual(actual.wheel_steering_angles_deg()["front_axle_left"], 12.0)
        self.assertAlmostEqual(actual.wheel_steering_angles_deg()["rear_axle_right"], -7.0)
        self.assertEqual(actual.axles[0].source, "mechanism_graph")

    def test_graph_study_rejects_unassigned_user_defined_steerable_wheel(self) -> None:
        vehicle, _ideal, _radius = build_demo_solution(0.0)
        user_defined_vehicle = VehicleLayout(
            id=vehicle.id,
            name=vehicle.name,
            axles=tuple(
                replace(
                    axle,
                    steering_mode="USER_DEFINED",
                    user_defined_steering_angle_rad=math.radians(2.0),
                )
                for axle in vehicle.axles
            ),
        )
        graph_state = MechanismGraphState(
            point_positions={},
            member_residuals_mm={},
            output_angles_rad={"front_output": math.radians(2.0)},
            iterations=0,
        )

        with self.assertRaisesRegex(InvalidGeometryError, "has no mechanism output assignment"):
            solve_actual_steering_from_graph(
                user_defined_vehicle,
                graph_state,
                (MechanismSteeringAssignment("front_output", "front_axle_left"),),
            )


if __name__ == "__main__":
    unittest.main()
