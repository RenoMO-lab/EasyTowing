from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.collision import (  # type: ignore  # noqa: E402
    CapsuleEnvelope,
    CircleEnvelope,
    CollisionItem,
    PolygonEnvelope,
    analyze_clearance,
    axis_aligned_rectangle,
    clearance_between_items,
)
from easytowing.clearance_model import (  # type: ignore  # noqa: E402
    build_linkage_clearance_items,
    build_mechanism_graph_clearance_items,
)
from easytowing.geometry import Point2D  # type: ignore  # noqa: E402
from easytowing.errors import InvalidGeometryError  # type: ignore  # noqa: E402
from easytowing.linkage import (  # type: ignore  # noqa: E402
    build_reference_linkage_demo,
    solve_reference_linkage_demo,
)
from easytowing.mechanism_graph import (  # type: ignore  # noqa: E402
    MechanismGraphState,
    MechanismPoint,
    PlanarMechanismGraph,
    RigidMember,
)
from easytowing.model import (  # type: ignore  # noqa: E402
    ArticulationJoint,
    Pose2D,
    RigidBody,
    VehicleCombination,
)


class ClearanceEngineTests(unittest.TestCase):
    def test_collision_primitives_reject_invalid_geometry(self) -> None:
        with self.assertRaises(InvalidGeometryError):
            CircleEnvelope(Point2D(0.0, 0.0), -1.0)
        with self.assertRaises(InvalidGeometryError):
            CapsuleEnvelope(Point2D(0.0, 0.0), Point2D(0.0, 0.0), 1.0)
        with self.assertRaises(InvalidGeometryError):
            axis_aligned_rectangle(Point2D(0.0, 0.0), 0.0, 100.0)
        with self.assertRaises(InvalidGeometryError):
            PolygonEnvelope((
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
                Point2D(0.0, 100.0),
                Point2D(100.0, 100.0),
            ))

    def test_clearance_report_rejects_duplicate_item_ids(self) -> None:
        items = (
            CollisionItem("same", CircleEnvelope(Point2D(0.0, 0.0), 1.0)),
            CollisionItem("same", CircleEnvelope(Point2D(100.0, 0.0), 1.0)),
        )

        with self.assertRaises(InvalidGeometryError):
            analyze_clearance(items)

    def test_circle_circle_positive_clearance_and_margin_violation(self) -> None:
        item_a = CollisionItem("a", CircleEnvelope(Point2D(0.0, 0.0), 10.0), margin_mm=8.0)
        item_b = CollisionItem("b", CircleEnvelope(Point2D(40.0, 0.0), 10.0), margin_mm=15.0)

        pair = clearance_between_items(item_a, item_b)

        self.assertFalse(pair.overlaps)
        self.assertAlmostEqual(pair.raw_clearance_mm, 20.0, places=9)
        self.assertAlmostEqual(pair.clearance_mm, -3.0, places=9)
        self.assertTrue(pair.violates_margin)

    def test_capsule_capsule_overlap_is_negative(self) -> None:
        item_a = CollisionItem("a", CapsuleEnvelope(Point2D(-10.0, 0.0), Point2D(10.0, 0.0), 5.0))
        item_b = CollisionItem("b", CapsuleEnvelope(Point2D(0.0, -10.0), Point2D(0.0, 10.0), 5.0))

        pair = clearance_between_items(item_a, item_b)

        self.assertTrue(pair.overlaps)
        self.assertLess(pair.raw_clearance_mm, 0.0)
        self.assertLess(pair.clearance_mm, 0.0)
        self.assertEqual(pair.description, "capsule-capsule")

    def test_polygon_overlap_detected_and_minimum_pair_selected(self) -> None:
        rectangle = axis_aligned_rectangle(Point2D(0.0, 0.0), 100.0, 100.0)
        circle = CircleEnvelope(Point2D(0.0, 0.0), 15.0)
        far_circle = CircleEnvelope(Point2D(200.0, 0.0), 10.0)

        report = analyze_clearance(
            (
                CollisionItem("rectangle", rectangle),
                CollisionItem("circle", circle),
                CollisionItem("far_circle", far_circle),
            )
        )

        self.assertTrue(report.collision_detected)
        self.assertIsNotNone(report.minimum_pair)
        self.assertEqual(report.minimum_pair.item_a_id, "rectangle")
        self.assertEqual(report.minimum_pair.item_b_id, "circle")
        self.assertLess(report.minimum_clearance_mm or 0.0, 0.0)

    def test_excluded_pairs_are_not_reported(self) -> None:
        beam = CollisionItem(
            "axle_beam",
            CapsuleEnvelope(Point2D(0.0, -100.0), Point2D(0.0, 100.0), 10.0),
        )
        tire = CollisionItem(
            "wheel_tire",
            CircleEnvelope(Point2D(0.0, 0.0), 40.0),
            excluded_pair_ids=("axle_beam",),
        )

        report = analyze_clearance((beam, tire))

        self.assertEqual(report.pairs, ())
        self.assertIsNone(report.minimum_pair)

    def test_linkage_builder_excludes_only_connected_component_pairs(self) -> None:
        rig = build_reference_linkage_demo()
        state = solve_reference_linkage_demo(30.0)

        report = analyze_clearance(build_linkage_clearance_items(rig.spec, state))
        pair_ids = {
            frozenset((pair.item_a_id, pair.item_b_id))
            for pair in report.pairs
        }

        self.assertNotIn(frozenset(("steering_arm", "steering_pivot")), pair_ids)
        self.assertNotIn(frozenset(("tie_rod", "steering_arm")), pair_ids)
        self.assertNotIn(frozenset(("companion_tie_rod", "companion_steering_arm")), pair_ids)
        self.assertIn(frozenset(("steering_pivot", "companion_tie_rod")), pair_ids)
        self.assertTrue(report.collision_detected)

    def test_articulated_body_collision_is_not_hidden_by_joint_connectivity(self) -> None:
        graph = PlanarMechanismGraph(
            id="body_collision",
            points=(MechanismPoint("anchor", Point2D(0.0, 0.0), "fixed"),),
            members=(),
        )
        state = MechanismGraphState(
            point_positions={"anchor": Point2D(0.0, 0.0)},
            member_residuals_mm={},
            output_angles_rad={},
            iterations=0,
        )
        combination = VehicleCombination(
            id="overlapping_train",
            name="Overlapping articulated train",
            bodies=(
                RigidBody(
                    id="tractor",
                    name="Tractor",
                    pose=Pose2D(),
                    body_length_mm=100.0,
                    body_width_mm=100.0,
                ),
                RigidBody(
                    id="trailer",
                    name="Trailer",
                    body_length_mm=100.0,
                    body_width_mm=100.0,
                ),
            ),
            joints=(
                ArticulationJoint(
                    id="hitch",
                    parent_body_id="tractor",
                    child_body_id="trailer",
                    parent_anchor=Point2D(0.0, 0.0),
                    child_anchor=Point2D(0.0, 0.0),
                ),
            ),
            root_body_id="tractor",
        )

        report = analyze_clearance(
            build_mechanism_graph_clearance_items(
                graph,
                state,
                combination=combination,
            )
        )

        pair_ids = {
            frozenset((pair.item_a_id, pair.item_b_id))
            for pair in report.pairs
        }
        self.assertIn(frozenset(("body:tractor", "body:trailer")), pair_ids)
        self.assertTrue(report.collision_detected)

    def test_cross_body_component_collision_is_not_hidden_by_mounting(self) -> None:
        graph = PlanarMechanismGraph(
            id="cross_body_component",
            points=(
                MechanismPoint("a", Point2D(-50.0, 0.0), "fixed", body_id="tractor"),
                MechanismPoint("b", Point2D(50.0, 0.0), "fixed", body_id="tractor"),
            ),
            members=(RigidMember("cross_body_link", "a", "b", 100.0, envelope_radius_mm=10.0),),
        )
        state = MechanismGraphState(
            point_positions={"a": Point2D(-50.0, 0.0), "b": Point2D(50.0, 0.0)},
            member_residuals_mm={"cross_body_link": 0.0},
            output_angles_rad={},
            iterations=0,
        )
        combination = VehicleCombination(
            id="cross_body_train",
            name="Cross-body component train",
            bodies=(
                RigidBody(
                    id="tractor",
                    name="Tractor",
                    pose=Pose2D(),
                    body_length_mm=200.0,
                    body_width_mm=200.0,
                ),
                RigidBody(
                    id="trailer",
                    name="Trailer",
                    pose=Pose2D(),
                    body_length_mm=200.0,
                    body_width_mm=200.0,
                ),
            ),
            joints=(
                ArticulationJoint(
                    id="hitch",
                    parent_body_id="tractor",
                    child_body_id="trailer",
                    parent_anchor=Point2D(0.0, 0.0),
                    child_anchor=Point2D(0.0, 0.0),
                ),
            ),
            root_body_id="tractor",
        )

        report = analyze_clearance(
            build_mechanism_graph_clearance_items(
                graph,
                state,
                combination=combination,
            )
        )

        pair_ids = {
            frozenset((pair.item_a_id, pair.item_b_id))
            for pair in report.pairs
        }
        self.assertNotIn(frozenset(("body:tractor", "cross_body_link")), pair_ids)
        self.assertIn(frozenset(("body:trailer", "cross_body_link")), pair_ids)
        self.assertTrue(report.collision_detected)


if __name__ == "__main__":
    unittest.main()
