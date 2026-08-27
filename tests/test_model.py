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
    VehicleCombination,
    VehicleLayout,
    build_reference_demo_combination,
    combination_to_vehicle_layout,
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
        combo = VehicleCombination(
            id="invalid",
            name="Invalid Combination",
            bodies=(
                RigidBody(id="tractor", name="Tractor"),
                RigidBody(id="trailer", name="Trailer"),
            ),
            mounted_axles=(),
            root_body_id="tractor",
        )

        with self.assertRaises(InvalidGeometryError):
            combo.resolve_body_poses()

    def test_articulated_layout_carries_body_yaw_into_axle_heading(self) -> None:
        combination = build_reference_demo_combination(articulation_rad=math.radians(20.0))
        layout = combination_to_vehicle_layout(combination)
        front_axle = next(axle for axle in layout.axles if axle.id == "front_axle")

        self.assertTrue(math.isclose(front_axle.heading_rad, math.radians(20.0), abs_tol=1e-9))
        left_wheel, right_wheel = front_axle.wheels()
        self.assertTrue(math.isclose((left_wheel.center - right_wheel.center).length(), 2500.0, abs_tol=1e-9))


if __name__ == "__main__":
    unittest.main()

