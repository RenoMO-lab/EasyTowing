from __future__ import annotations

import math
import unittest

from easytowing.actual_steering import (
    compare_actual_to_ideal,
    solve_actual_steering,
    solve_actual_steering_from_graph,
)
from easytowing.errors import SteeringLimitExceededError
from easytowing.geometry import Point2D
from easytowing.linkage import PlanarLinkageState
from easytowing.mechanism_graph import MechanismGraphState, MechanismSteeringAssignment
from easytowing.model import Axle, SteeringSynchronization, SteeringTargetPoint, VehicleLayout
from easytowing.steering import solve_ideal_steering_from_radius


def make_linkage_state(angle_deg: float, companion_angle_deg: float | None = None) -> PlanarLinkageState:
    return PlanarLinkageState(
        driver_point=Point2D(0.0, 0.0),
        input_endpoint=Point2D(0.0, 0.0),
        bell_crank_angle_rad=0.0,
        output_endpoint=Point2D(0.0, 0.0),
        steering_endpoint=Point2D(0.0, 0.0),
        steering_angle_rad=math.radians(angle_deg),
        input_stage_error_mm=0.0,
        tie_rod_error_mm=0.0,
        input_branch_index=0,
        steering_branch_index=0,
        companion_steering_angle_rad=(
            None if companion_angle_deg is None else math.radians(companion_angle_deg)
        ),
    )


class ActualSteeringTests(unittest.TestCase):
    def make_vehicle(self, synchronizations: tuple[SteeringSynchronization, ...]) -> VehicleLayout:
        return VehicleLayout(
            id="three_axle_actual",
            name="Three axle actual steering test",
            axles=(
                Axle(id="rear", center=Point2D(-1000.0, 0.0), track_mm=2400.0),
                Axle(id="middle", center=Point2D(0.0, 0.0), track_mm=2500.0),
                Axle(id="front", center=Point2D(1000.0, 0.0), track_mm=2600.0),
            ),
            body_length_mm=3000.0,
            body_width_mm=3300.0,
            steering_synchronizations=synchronizations,
        )

    def test_sync_modes_drive_every_axle_from_primary_linkage(self) -> None:
        vehicle = self.make_vehicle(
            (
                SteeringSynchronization(
                    id="middle_ratio",
                    target_axle_id="middle",
                    mode="RATIO",
                    ratio=0.5,
                ),
                SteeringSynchronization(
                    id="rear_opposite",
                    target_axle_id="rear",
                    mode="OPPOSITE_PHASE",
                ),
            )
        )
        actual = solve_actual_steering(vehicle, make_linkage_state(10.0, 12.0), math.radians(10.0))

        centers = actual.axle_center_steering_angles_deg()
        self.assertAlmostEqual(centers["front"], 11.0, places=7)
        self.assertAlmostEqual(centers["middle"], 5.5, places=7)
        self.assertAlmostEqual(centers["rear"], -11.0, places=7)
        self.assertEqual(actual.axles[0].source, "opposite_phase:front")
        self.assertEqual(actual.axles[1].source, "ratio:front")
        ideal = solve_ideal_steering_from_radius(vehicle, 12000.0)
        comparison = compare_actual_to_ideal(
            actual,
            ideal,
            vehicle=vehicle,
            beta_rad=math.radians(10.0),
        )
        self.assertEqual(set(comparison["synchronization_errors_deg"]), {"middle_ratio", "rear_opposite"})
        self.assertGreater(comparison["max_abs_synchronization_error_deg"], 0.0)

    def test_multi_wheel_axle_applies_wheel_end_commands_to_every_tire(self) -> None:
        vehicle = VehicleLayout(
            id="dual_wheel_actual",
            name="Dual wheel actual steering test",
            axles=(
                Axle(
                    id="front",
                    center=Point2D(1000.0, 0.0),
                    track_mm=2800.0,
                    wheel_count=4,
                    wheel_lateral_offsets_mm=(1400.0, 1180.0, -1180.0, -1400.0),
                ),
            ),
            body_length_mm=3000.0,
            body_width_mm=3400.0,
        )

        actual = solve_actual_steering(
            vehicle,
            make_linkage_state(10.0, 12.0),
            math.radians(10.0),
        )

        self.assertEqual(len(actual.axles[0].wheel_solutions), 4)
        angles = actual.wheel_steering_angles_deg()
        self.assertEqual(set(angles), {
            "front_left_1",
            "front_left_2",
            "front_right_1",
            "front_right_2",
        })
        self.assertTrue(all(math.isclose(angles[wheel_id], 10.0, abs_tol=1e-9) for wheel_id in ("front_left_1", "front_left_2")))
        self.assertTrue(all(math.isclose(angles[wheel_id], 12.0, abs_tol=1e-9) for wheel_id in ("front_right_1", "front_right_2")))

    def test_graph_steering_applies_one_output_to_each_multi_wheel_end(self) -> None:
        vehicle = VehicleLayout(
            id="dual_wheel_graph",
            name="Dual wheel graph steering test",
            axles=(
                Axle(
                    id="front",
                    center=Point2D(1000.0, 0.0),
                    track_mm=2800.0,
                    wheel_count=4,
                    wheel_lateral_offsets_mm=(1400.0, 1180.0, -1180.0, -1400.0),
                ),
            ),
            body_length_mm=3000.0,
            body_width_mm=3400.0,
        )
        graph_state = MechanismGraphState(
            point_positions={},
            member_residuals_mm={},
            output_angles_rad={
                "left_output": math.radians(10.0),
                "right_output": math.radians(12.0),
            },
            iterations=0,
        )
        assignments = tuple(
            MechanismSteeringAssignment(output_id, wheel_id)
            for output_id, wheel_id in (
                ("left_output", "front_left_1"),
                ("left_output", "front_left_2"),
                ("right_output", "front_right_1"),
                ("right_output", "front_right_2"),
            )
        )

        actual = solve_actual_steering_from_graph(vehicle, graph_state, assignments)

        self.assertEqual(len(actual.axles[0].wheel_solutions), 4)
        self.assertAlmostEqual(actual.axle_center_steering_angles_deg()["front"], 11.0)
        self.assertTrue(all(math.isclose(angle, 10.0, abs_tol=1e-9) for wheel_id, angle in actual.wheel_steering_angles_deg().items() if "left" in wheel_id))
        self.assertTrue(all(math.isclose(angle, 12.0, abs_tol=1e-9) for wheel_id, angle in actual.wheel_steering_angles_deg().items() if "right" in wheel_id))

    def test_independent_target_is_interpolated(self) -> None:
        vehicle = self.make_vehicle(
            (
                SteeringSynchronization(
                    id="middle_target",
                    target_axle_id="middle",
                    mode="INDEPENDENT_TARGET",
                    target_curve=(
                        SteeringTargetPoint(-1.0, math.radians(-4.0)),
                        SteeringTargetPoint(1.0, math.radians(14.0)),
                    ),
                ),
            )
        )
        actual = solve_actual_steering(vehicle, make_linkage_state(10.0), 0.0)

        self.assertAlmostEqual(actual.axle_center_steering_angles_deg()["middle"], 5.0, places=7)
        self.assertEqual(actual.axles[1].source, "target_curve:middle_target")

    def test_comparison_reports_all_wheels_and_axles(self) -> None:
        vehicle = self.make_vehicle(())
        ideal = solve_ideal_steering_from_radius(vehicle, 12000.0)
        actual = solve_actual_steering(vehicle, make_linkage_state(8.0, 9.0), math.radians(8.0), ideal)
        comparison = compare_actual_to_ideal(actual, ideal)

        self.assertEqual(set(comparison["wheel_errors_deg"]), {
            "rear_left", "rear_right", "middle_left", "middle_right", "front_left", "front_right",
        })
        self.assertEqual(set(comparison["axle_center_errors_deg"]), {"rear", "middle", "front"})
        self.assertIsNotNone(comparison["front_rear_synchronization_error_deg"])

    def test_actual_synchronized_axle_respects_steering_stop(self) -> None:
        vehicle = VehicleLayout(
            id="limited_actual",
            name="Limited actual steering",
            axles=(
                Axle(id="rear", center=Point2D(-1000.0, 0.0), track_mm=2400.0),
                Axle(
                    id="front",
                    center=Point2D(1000.0, 0.0),
                    track_mm=2600.0,
                    steering_stop_deg=5.0,
                ),
            ),
        )

        with self.assertRaises(SteeringLimitExceededError):
            solve_actual_steering(
                vehicle,
                make_linkage_state(10.0),
                math.radians(10.0),
            )


if __name__ == "__main__":
    unittest.main()
