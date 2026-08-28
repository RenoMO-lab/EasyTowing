from __future__ import annotations

import sitecustomize
import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.errors import InvalidGeometryError  # type: ignore  # noqa: E402
from easytowing.model import (  # type: ignore  # noqa: E402
    ArticulationJoint,
    Axle,
    MountedAxle,
    Point2D,
    Pose2D,
    RigidBody,
    SteeringSynchronization,
    VehicleCombination,
    VehicleLayout,
    build_reference_demo_combination,
    combination_to_vehicle_layout,
    serialize_vehicle_combination,
)


class VehicleCombinationTests(unittest.TestCase):
    def test_vehicle_layout_validates_and_preserves_articulation_metadata(self) -> None:
        layout = VehicleLayout(
            id="metadata_case",
            name="Metadata case",
            axles=(
                Axle(
                    id="axle_1",
                    center=Point2D(0.0, 0.0),
                    track_mm=2400.0,
                    tire_width_mm=385.0,
                    outside_diameter_mm=1100.0,
                ),
            ),
            body_length_mm=5000.0,
            body_width_mm=3000.0,
            origin=Point2D(100.0, -50.0),
            body_polygon=(
                Point2D(-2500.0, -1500.0),
                Point2D(2500.0, -1500.0),
                Point2D(2500.0, 1500.0),
                Point2D(-2500.0, 1500.0),
            ),
            front_articulation_point=Point2D(2300.0, 0.0),
            rear_articulation_point=Point2D(-2300.0, 0.0),
            kingpin_point=Point2D(0.0, 0.0),
            maximum_articulation_deg=52.0,
        )

        self.assertEqual(len(layout.body_polygon), 4)
        self.assertEqual(layout.front_articulation_point, Point2D(2300.0, 0.0))
        self.assertEqual(layout.kingpin_point, Point2D(0.0, 0.0))
        self.assertEqual(layout.maximum_articulation_deg, 52.0)
        self.assertEqual(layout.wheels()[0].tire_width_mm, 385.0)
        self.assertEqual(layout.wheels()[0].outside_diameter_mm, 1100.0)

        with self.assertRaises(InvalidGeometryError):
            VehicleLayout(
                id="invalid_metadata",
                name="Invalid metadata",
                axles=(),
                origin=Point2D(float("nan"), 0.0),
            )

    def test_pose_transform_and_chain_resolution(self) -> None:
        root_body = RigidBody(
            id="tractor",
            name="Tractor",
            pose=Pose2D(0.0, 0.0, 0.0),
            body_length_mm=6500.0,
            body_width_mm=2500.0,
        )
        trailer_body = RigidBody(
            id="trailer",
            name="Trailer",
            body_length_mm=7200.0,
            body_width_mm=2550.0,
        )
        combo = VehicleCombination(
            id="train",
            name="Tractor + Trailer",
            bodies=(root_body, trailer_body),
            joints=(
                ArticulationJoint(
                    id="drawbar_joint",
                    parent_body_id="tractor",
                    child_body_id="trailer",
                    parent_anchor=Point2D(-1600.0, 0.0),
                    child_anchor=Point2D(0.0, 0.0),
                    articulation_rad=math.radians(15.0),
                ),
            ),
            mounted_axles=(
                MountedAxle(
                    axle=Axle(id="tractor_axle", center=Point2D(0.0, 0.0), track_mm=2500.0),
                    body_id="tractor",
                    local_center=Point2D(1200.0, 0.0),
                ),
                MountedAxle(
                    axle=Axle(id="trailer_axle", center=Point2D(0.0, 0.0), track_mm=2500.0),
                    body_id="trailer",
                    local_center=Point2D(-1800.0, 0.0),
                ),
            ),
            root_body_id="tractor",
        )

        parent_pose = Pose2D(100.0, 50.0, math.radians(90.0))
        transformed = parent_pose.transform_point(Point2D(25.0, 0.0))
        self.assertTrue(math.isclose(transformed.x_mm, 100.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(transformed.y_mm, 75.0, abs_tol=1e-9))
        restored = parent_pose.inverse_transform_point(transformed)
        self.assertTrue(math.isclose(restored.x_mm, 25.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(restored.y_mm, 0.0, abs_tol=1e-9))

        body_poses = combo.resolve_body_poses()
        trailer_pose = body_poses["trailer"]
        self.assertTrue(math.isclose(trailer_pose.x_mm, -1600.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(trailer_pose.y_mm, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(trailer_pose.yaw_rad, math.radians(15.0), abs_tol=1e-9))

        layout = combo.to_vehicle_layout()
        self.assertEqual(layout.id, "train")
        self.assertEqual(layout.name, "Tractor + Trailer")
        self.assertEqual(len(layout.axles), 2)
        self.assertTrue(math.isclose(layout.body_length_mm, 7200.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(layout.body_width_mm, 2550.0, abs_tol=1e-9))

        trailer_axle = next(axle for axle in layout.axles if axle.id == "trailer_axle")
        expected_x = -1600.0 - 1800.0 * math.cos(math.radians(15.0))
        expected_y = -1800.0 * math.sin(math.radians(15.0))
        self.assertTrue(math.isclose(trailer_axle.center.x_mm, expected_x, abs_tol=1e-9))
        self.assertTrue(math.isclose(trailer_axle.center.y_mm, expected_y, abs_tol=1e-9))

    def test_combination_requires_a_connected_chain(self) -> None:
        with self.assertRaisesRegex(InvalidGeometryError, "missing a parent"):
            VehicleCombination(
                id="invalid",
                name="Invalid Combination",
                bodies=(
                    RigidBody(id="tractor", name="Tractor"),
                    RigidBody(id="trailer", name="Trailer"),
                ),
                mounted_axles=(),
                root_body_id="tractor",
            )

    def test_combination_rejects_invalid_joint_and_mount_references_early(self) -> None:
        with self.assertRaisesRegex(InvalidGeometryError, "cannot connect"):
            ArticulationJoint(
                id="self_joint",
                parent_body_id="tractor",
                child_body_id="tractor",
                parent_anchor=Point2D(0.0, 0.0),
                child_anchor=Point2D(0.0, 0.0),
            )
        with self.assertRaisesRegex(InvalidGeometryError, "existing body"):
            VehicleCombination(
                id="invalid_mount",
                name="Invalid mount",
                bodies=(RigidBody(id="tractor", name="Tractor"),),
                mounted_axles=(
                    MountedAxle(
                        axle=Axle("axle", Point2D(0.0, 0.0), 2400.0),
                        body_id="missing",
                        local_center=Point2D(0.0, 0.0),
                    ),
                ),
                root_body_id="tractor",
            )

    def test_rigid_body_polygon_is_preserved_and_used_for_combination_bounds(self) -> None:
        polygon = (
            Point2D(-200.0, -100.0),
            Point2D(200.0, -100.0),
            Point2D(200.0, 100.0),
            Point2D(-200.0, 100.0),
        )
        combination = VehicleCombination(
            id="polygon_body",
            name="Polygon body",
            bodies=(RigidBody("body", "Body", body_polygon=polygon),),
            root_body_id="body",
        )

        layout = combination_to_vehicle_layout(combination)
        serialized = serialize_vehicle_combination(combination)

        self.assertEqual(layout.body_length_mm, 400.0)
        self.assertEqual(layout.body_width_mm, 200.0)
        self.assertEqual(serialized["bodies"][0]["body_polygon"][0], {"x_mm": -200.0, "y_mm": -100.0})

    def test_body_polygons_reject_degenerate_and_self_intersecting_outlines(self) -> None:
        invalid_polygons = (
            (
                Point2D(0.0, 0.0),
                Point2D(100.0, 100.0),
                Point2D(0.0, 100.0),
                Point2D(100.0, 0.0),
            ),
            (
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
                Point2D(200.0, 0.0),
            ),
            (
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
                Point2D(100.0, 100.0),
                Point2D(100.0, 100.0),
            ),
        )

        for polygon in invalid_polygons:
            with self.subTest(polygon=polygon):
                with self.assertRaises(InvalidGeometryError):
                    RigidBody(id="invalid_body", name="Invalid body", body_polygon=polygon)
                with self.assertRaises(InvalidGeometryError):
                    VehicleLayout(
                        id="invalid_layout",
                        name="Invalid layout",
                        body_polygon=polygon,
                    )

    def test_articulated_layout_carries_body_yaw_into_axle_heading(self) -> None:
        combination = build_reference_demo_combination(articulation_rad=math.radians(20.0))
        layout = combination_to_vehicle_layout(combination)
        front_axle = next(axle for axle in layout.axles if axle.id == "front_axle")

        self.assertTrue(math.isclose(front_axle.heading_rad, math.radians(20.0), abs_tol=1e-9))
        left_wheel, right_wheel = front_axle.wheels()
        self.assertTrue(math.isclose((left_wheel.center - right_wheel.center).length(), 2500.0, abs_tol=1e-9))

    def test_combination_preserves_steering_coordination_channels(self) -> None:
        combination = build_reference_demo_combination()
        layout = combination_to_vehicle_layout(combination)
        serialized = serialize_vehicle_combination(combination)

        self.assertEqual(len(combination.steering_synchronizations), 1)
        self.assertEqual(layout.steering_synchronizations[0].target_axle_id, "rear_axle")
        self.assertEqual(serialized["steering_synchronizations"][0]["mode"], "OPPOSITE_PHASE")

    def test_steering_coordination_rejects_ambiguous_targets(self) -> None:
        with self.assertRaisesRegex(InvalidGeometryError, "cannot target its source"):
            SteeringSynchronization(
                id="self_sync",
                target_axle_id="axle_a",
                source_axle_id="axle_a",
            )

        axles = (
            Axle("axle_a", Point2D(0.0, 0.0), 2400.0),
            Axle("axle_b", Point2D(1000.0, 0.0), 2400.0),
        )
        with self.assertRaisesRegex(InvalidGeometryError, "targets must be unique"):
            VehicleLayout(
                id="duplicate_sync_targets",
                name="Duplicate sync targets",
                axles=axles,
                steering_synchronizations=(
                    SteeringSynchronization("sync_1", "axle_a", source_axle_id="axle_b"),
                    SteeringSynchronization("sync_2", "axle_a", source_axle_id="axle_b"),
                ),
            )

        with self.assertRaisesRegex(InvalidGeometryError, "targets must be unique"):
            VehicleCombination(
                id="duplicate_combo_sync_targets",
                name="Duplicate combination sync targets",
                bodies=(RigidBody("body", "Body"),),
                mounted_axles=tuple(
                    MountedAxle(axle, "body", Point2D(0.0, float(index) * 1000.0))
                    for index, axle in enumerate(axles)
                ),
                root_body_id="body",
                steering_synchronizations=(
                    SteeringSynchronization("sync_1", "axle_a", source_axle_id="axle_b"),
                    SteeringSynchronization("sync_2", "axle_a", source_axle_id="axle_b"),
                ),
            )

    def test_multi_wheel_axle_requires_explicit_positions(self) -> None:
        with self.assertRaisesRegex(InvalidGeometryError, "explicit wheel_lateral_offsets_mm"):
            Axle(
                id="dual_wheel_axle",
                center=Point2D(0.0, 0.0),
                track_mm=2500.0,
                wheel_count=4,
            )

    def test_multi_wheel_axle_preserves_all_positions_and_outer_wheels(self) -> None:
        axle = Axle(
            id="dual_wheel_axle",
            center=Point2D(0.0, 0.0),
            track_mm=2800.0,
            wheel_count=4,
            wheel_lateral_offsets_mm=(1400.0, 1180.0, -1180.0, -1400.0),
        )

        wheels = axle.wheels()
        self.assertEqual([wheel.id for wheel in wheels], [
            "dual_wheel_axle_left_1",
            "dual_wheel_axle_left_2",
            "dual_wheel_axle_right_1",
            "dual_wheel_axle_right_2",
        ])
        self.assertEqual(len(wheels), 4)
        outer_left, outer_right = axle.outer_wheels()
        self.assertEqual(outer_left.lateral_offset_mm, 1400.0)
        self.assertEqual(outer_right.lateral_offset_mm, -1400.0)


if __name__ == "__main__":
    unittest.main()

