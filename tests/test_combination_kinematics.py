from __future__ import annotations

import math
import unittest
from dataclasses import replace

from easytowing.combination_kinematics import solve_combination_kinematics
from easytowing.errors import ArticulationLimitExceededError, InvalidGeometryError, MultiBodyKinematicConstraintError
from easytowing.geometry import Point2D
from easytowing.model import ArticulationJoint, Axle, MountedAxle, Pose2D, RigidBody, VehicleCombination


def two_body_combination(
    *,
    articulation_deg: float,
    steerable: bool,
    articulation_limit_deg: float = 45.0,
) -> VehicleCombination:
    return VehicleCombination(
        id="tractor_trailer",
        name="Tractor and trailer",
        bodies=(
            RigidBody(id="tractor", name="Tractor", pose=Pose2D()),
            RigidBody(id="trailer", name="Trailer"),
        ),
        joints=(
            ArticulationJoint(
                id="hitch",
                parent_body_id="tractor",
                child_body_id="trailer",
                parent_anchor=Point2D(-1500.0, 0.0),
                child_anchor=Point2D(2500.0, 0.0),
                articulation_rad=math.radians(articulation_deg),
                maximum_articulation_deg=articulation_limit_deg,
            ),
        ),
        mounted_axles=(
            MountedAxle(
                axle=Axle(
                    id="tractor_axle",
                    center=Point2D(0.0, 0.0),
                    track_mm=2400.0,
                    steerable=steerable,
                    steering_mode="FORCED_STEER" if steerable else "FIXED",
                ),
                body_id="tractor",
                local_center=Point2D(0.0, 0.0),
            ),
            MountedAxle(
                axle=Axle(
                    id="trailer_axle",
                    center=Point2D(0.0, 0.0),
                    track_mm=2500.0,
                    steerable=steerable,
                    steering_mode="FORCED_STEER" if steerable else "FIXED",
                ),
                body_id="trailer",
                local_center=Point2D(-1500.0, 0.0),
            ),
        ),
        root_body_id="tractor",
    )


def three_body_combination(
    *,
    first_articulation_deg: float,
    second_articulation_deg: float,
    second_articulation_limit_deg: float = 45.0,
) -> VehicleCombination:
    return VehicleCombination(
        id="three_body_train",
        name="Tractor, dolly, and trailer",
        bodies=(
            RigidBody("tractor", "Tractor", Pose2D(), 6000.0, 1400.0),
            RigidBody("dolly", "Dolly", Pose2D(), 4000.0, 1400.0),
            RigidBody("trailer", "Trailer", Pose2D(), 6000.0, 1400.0),
        ),
        joints=(
            ArticulationJoint(
                "tractor_dolly",
                "tractor",
                "dolly",
                Point2D(3000.0, 0.0),
                Point2D(-2000.0, 0.0),
                articulation_rad=math.radians(first_articulation_deg),
                maximum_articulation_deg=45.0,
            ),
            ArticulationJoint(
                "dolly_trailer",
                "dolly",
                "trailer",
                Point2D(2000.0, 0.0),
                Point2D(-3000.0, 0.0),
                articulation_rad=math.radians(second_articulation_deg),
                maximum_articulation_deg=second_articulation_limit_deg,
            ),
        ),
        mounted_axles=tuple(
            MountedAxle(
                axle=Axle(
                    f"{body_id}_axle",
                    Point2D(0.0, 0.0),
                    1200.0,
                    steerable=True,
                    steering_mode="FORCED_STEER",
                ),
                body_id=body_id,
                local_center=Point2D(0.0, 0.0),
            )
            for body_id in ("tractor", "dolly", "trailer")
        ),
        root_body_id="tractor",
    )


class CombinationKinematicsTests(unittest.TestCase):
    def test_fixed_axles_derive_common_icr_from_articulation(self) -> None:
        solution = solve_combination_kinematics(
            two_body_combination(articulation_deg=20.0, steerable=False)
        )

        self.assertIsNotNone(solution.icr)
        self.assertAlmostEqual(solution.maximum_constraint_residual_mm, 0.0, places=7)
        self.assertAlmostEqual(solution.maximum_joint_closure_error_mm, 0.0, places=7)
        self.assertAlmostEqual(solution.root_icr_longitudinal_offset_mm or 0.0, 0.0, places=7)
        self.assertEqual(len(solution.axle_constraints), 2)

    def test_explicit_radius_solves_all_steerable_axles_on_articulated_bodies(self) -> None:
        solution = solve_combination_kinematics(
            two_body_combination(articulation_deg=15.0, steerable=True),
            root_turn_radius_mm=9000.0,
        )

        self.assertIsNotNone(solution.icr)
        self.assertAlmostEqual(solution.root_turn_radius_mm or 0.0, 9000.0, places=7)
        self.assertEqual(len(solution.ideal_steering.axles), 2)
        trailer = next(axle for axle in solution.ideal_steering.axles if axle.axle_id == "trailer_axle")
        self.assertFalse(math.isclose(trailer.reference_heading_rad, 0.0, abs_tol=1e-9))

    def test_steerable_combination_requires_an_explicit_maneuver_radius(self) -> None:
        with self.assertRaisesRegex(InvalidGeometryError, "explicit root turn radius"):
            solve_combination_kinematics(
                two_body_combination(articulation_deg=15.0, steerable=True)
            )

    def test_single_fixed_axle_does_not_infer_a_straight_maneuver(self) -> None:
        combination = two_body_combination(articulation_deg=0.0, steerable=False)
        under_constrained = replace(
            combination,
            mounted_axles=(combination.mounted_axles[0],),
        )

        with self.assertRaisesRegex(InvalidGeometryError, "explicit root turn radius"):
            solve_combination_kinematics(under_constrained)

    def test_coincident_parallel_fixed_constraints_require_a_radius(self) -> None:
        combination = two_body_combination(articulation_deg=0.0, steerable=False)
        same_constraint_line = replace(
            combination,
            mounted_axles=(
                combination.mounted_axles[0],
                replace(combination.mounted_axles[1], local_center=Point2D(4000.0, 0.0)),
            ),
        )

        with self.assertRaisesRegex(InvalidGeometryError, "explicit root turn radius"):
            solve_combination_kinematics(same_constraint_line)

    def test_explicit_radius_rejects_fixed_axle_constraint_conflict(self) -> None:
        with self.assertRaises(MultiBodyKinematicConstraintError):
            solve_combination_kinematics(
                two_body_combination(articulation_deg=20.0, steerable=False),
                root_turn_radius_mm=9000.0,
            )

    def test_parallel_fixed_axles_resolve_as_straight_motion(self) -> None:
        solution = solve_combination_kinematics(
            two_body_combination(articulation_deg=0.0, steerable=False)
        )

        self.assertIsNone(solution.icr)
        self.assertTrue(all(abs(value) <= 1e-12 for value in solution.ideal_steering.wheel_angles_rad.values()))

    def test_three_body_chain_resolves_every_body_and_axle(self) -> None:
        solution = solve_combination_kinematics(
            three_body_combination(first_articulation_deg=12.0, second_articulation_deg=-7.0),
            root_turn_radius_mm=9000.0,
        )

        self.assertEqual(set(solution.body_poses), {"tractor", "dolly", "trailer"})
        self.assertEqual(len(solution.joint_states), 2)
        self.assertEqual(len(solution.ideal_steering.axles), 3)
        self.assertAlmostEqual(solution.body_poses["dolly"].yaw_rad, math.radians(12.0), places=7)
        self.assertAlmostEqual(solution.body_poses["trailer"].yaw_rad, math.radians(5.0), places=7)
        self.assertAlmostEqual(solution.maximum_joint_closure_error_mm, 0.0, places=7)

    def test_secondary_joint_physical_limit_is_enforced(self) -> None:
        with self.assertRaisesRegex(ArticulationLimitExceededError, "dolly_trailer"):
            solve_combination_kinematics(
                three_body_combination(
                    first_articulation_deg=12.0,
                    second_articulation_deg=-7.0,
                    second_articulation_limit_deg=5.0,
                ),
                root_turn_radius_mm=9000.0,
            )

    def test_primary_joint_physical_limit_is_enforced(self) -> None:
        with self.assertRaisesRegex(ArticulationLimitExceededError, "hitch"):
            solve_combination_kinematics(
                two_body_combination(
                    articulation_deg=31.0,
                    steerable=True,
                    articulation_limit_deg=30.0,
                ),
                root_turn_radius_mm=9000.0,
            )


if __name__ == "__main__":
    unittest.main()
