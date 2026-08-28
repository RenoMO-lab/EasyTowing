from __future__ import annotations

import math
import unittest
from dataclasses import replace

from easytowing.errors import ArticulationLimitExceededError
from easytowing.combination_kinematics import solve_combination_kinematics
from easytowing.demo_server import (
    _combination_kinematic_payload,
    _mechanism_graph_payload,
    _parse_driven_positions,
    _parse_linkage_rig,
    _parse_mechanism_graph,
    _parse_vehicle_layout,
    _parse_vehicle_combination,
    _parse_vehicle_config,
    _validated_cad_source,
    build_combination_sweep_payload,
    build_demo_payload,
)
from easytowing.demo_server import _ideal_steering_request_payload
from easytowing.mechanism_graph import MechanismSteeringAssignment, solve_mechanism_graph
from easytowing.geometry import Point2D
from easytowing.model import build_reference_demo_combination


class DemoPayloadTests(unittest.TestCase):
    def test_multi_wheel_axle_round_trips_through_api_payload(self) -> None:
        vehicle = _parse_vehicle_layout(
            {
                "id": "dual_wheel_vehicle",
                "name": "Dual wheel vehicle",
                "body_length_mm": 5600,
                "body_width_mm": 3400,
                "axles": [
                    {
                        "id": "front_axle",
                        "x_mm": 4360,
                        "track_mm": 2800,
                        "wheel_count": 4,
                        "wheel_lateral_offsets_mm": [1400, 1180, -1180, -1400],
                    }
                ],
            }
        )
        self.assertEqual(len(vehicle.wheels()), 4)
        payload = _ideal_steering_request_payload(
            {
                "id": "dual_wheel_vehicle",
                "body_length_mm": 5600,
                "body_width_mm": 3400,
                "turn_radius_mm": 14000,
                "axles": [
                    {
                        "id": "front_axle",
                        "x_mm": 4360,
                        "track_mm": 2800,
                        "wheel_count": 4,
                        "wheel_lateral_offsets_mm": [1400, 1180, -1180, -1400],
                    }
                ],
            }
        )
        axle = payload["axles"][0]
        self.assertEqual(len(axle["wheels"]), 4)
        self.assertEqual(axle["wheel_lateral_offsets_mm"], [1400.0, 1180.0, -1180.0, -1400.0])

    def test_cad_source_metadata_rejects_path_names_and_requires_confirmation(self) -> None:
        with self.assertRaisesRegex(ValueError, "safe filename"):
            _validated_cad_source(
                {
                    "source_name": "..\\layout.dxf",
                    "source_sha256": "0" * 64,
                    "source_units": "mm",
                    "unit_scale_to_mm": 1.0,
                    "coordinate_system": "x_forward_y_left",
                    "metadata_confirmed": True,
                }
            )

        with self.assertRaisesRegex(ValueError, "metadata_confirmed"):
            _validated_cad_source(
                {
                    "source_name": "layout.dxf",
                    "source_sha256": "0" * 64,
                    "source_units": "mm",
                    "unit_scale_to_mm": 1.0,
                    "coordinate_system": "x_forward_y_left",
                    "metadata_confirmed": False,
                }
            )

    def test_demo_payload_uses_resolved_combination_axles_and_explicit_radius(self) -> None:
        articulation_rad = math.radians(12.0)
        combination = build_reference_demo_combination(
            articulation_rad=articulation_rad,
        )

        payload = build_demo_payload(
            12.0,
            combination=combination,
            root_turn_radius_mm=9000.0,
        )

        front_axle = next(axle for axle in payload["axles"] if axle["axle_id"] == "front_axle")
        self.assertAlmostEqual(payload["turn_radius_mm"], 9000.0)
        self.assertGreater(abs(front_axle["center"]["y_mm"]), 1.0)
        self.assertIsNotNone(front_axle["left_wheel"])
        self.assertIsNotNone(front_axle["right_wheel"])
        self.assertEqual(payload["vehicle_combination"]["joint_count"], 1)
        self.assertEqual(
            payload["vehicle_config"]["steering_synchronizations"][0]["target_axle_id"],
            "rear_axle",
        )
        self.assertLessEqual(
            payload["combination_kinematics"]["maximum_constraint_residual_mm"],
            0.01,
        )

    def test_pre_graph_combination_payload_reports_body_collision(self) -> None:
        reference = build_reference_demo_combination()
        overlapping = replace(
            reference,
            joints=(
                replace(
                    reference.joints[0],
                    parent_anchor=Point2D(0.0, 0.0),
                    child_anchor=Point2D(0.0, 0.0),
                ),
            ),
        )

        payload = build_demo_payload(
            0.0,
            combination=overlapping,
            root_turn_radius_mm=9000.0,
        )

        self.assertTrue(payload["clearance"]["collision_detected"])
        pair_ids = {
            frozenset((pair["item_a_id"], pair["item_b_id"]))
            for pair in payload["clearance"]["pairs"]
        }
        self.assertIn(frozenset(("body:rear_body", "body:front_body")), pair_ids)

    def test_combination_parser_accepts_flat_revision_axle_records_and_sync(self) -> None:
        combination = _parse_vehicle_combination(
            {
                "id": "flat_revision",
                "root_body_id": "body",
                "bodies": [{"id": "body", "name": "Body"}],
                "mounted_axles": [
                    {
                        "body_id": "body",
                        "axle_id": "axle",
                        "track_mm": 2500,
                        "steerable": True,
                    }
                ],
                "steering_synchronizations": [],
            }
        )
        self.assertEqual(combination.mounted_axles[0].axle.id, "axle")

    def test_combination_payload_uses_named_graph_wheel_assignments(self) -> None:
        combination = _parse_vehicle_combination(
            {
                "id": "single_body",
                "root_body_id": "body",
                "bodies": [{"id": "body", "name": "Body"}],
                "mounted_axles": [
                    {
                        "body_id": "body",
                        "local_center": {"x_mm": 0, "y_mm": 0},
                        "axle": {"id": "axle", "track_mm": 2500},
                    }
                ],
            }
        )
        graph = _parse_mechanism_graph(
            {
                "id": "wheel_outputs",
                "points": [
                    {"id": "left_pivot", "mode": "fixed", "x_mm": 0, "y_mm": 0, "body_id": "body"},
                    {
                        "id": "left_endpoint",
                        "mode": "fixed",
                        "x_mm": 100 * math.cos(math.radians(10)),
                        "y_mm": 100 * math.sin(math.radians(10)),
                        "body_id": "body",
                    },
                    {"id": "right_pivot", "mode": "fixed", "x_mm": 0, "y_mm": 100, "body_id": "body"},
                    {
                        "id": "right_endpoint",
                        "mode": "fixed",
                        "x_mm": 100 * math.cos(math.radians(8)),
                        "y_mm": 100 + 100 * math.sin(math.radians(8)),
                        "body_id": "body",
                    },
                ],
                "members": [],
                "angle_outputs": [
                    {"id": "left_output", "pivot_point_id": "left_pivot", "endpoint_point_id": "left_endpoint"},
                    {"id": "right_output", "pivot_point_id": "right_pivot", "endpoint_point_id": "right_endpoint"},
                ],
            }
        )

        payload = build_demo_payload(
            0.0,
            combination=combination,
            root_turn_radius_mm=9000.0,
            mechanism_graph=graph,
            steering_assignments=(
                MechanismSteeringAssignment("left_output", "axle_left"),
                MechanismSteeringAssignment("right_output", "axle_right"),
            ),
            clearance_target_mm=47.5,
        )

        self.assertIsNone(payload["linkage"])
        self.assertEqual(payload["mechanism_graph"]["mechanism"]["id"], "wheel_outputs")
        self.assertEqual(
            payload["mechanism_mapping"]["steering_assignments"],
            [
                {
                    "output_id": "left_output",
                    "wheel_id": "axle_left",
                    "ratio": 1.0,
                    "phase_offset_rad": 0.0,
                    "phase_offset_deg": 0.0,
                },
                {
                    "output_id": "right_output",
                    "wheel_id": "axle_right",
                    "ratio": 1.0,
                    "phase_offset_rad": 0.0,
                    "phase_offset_deg": 0.0,
                },
            ],
        )
        self.assertAlmostEqual(payload["actual_steering"]["wheel_angles_deg"]["axle_left"], 10.0)
        self.assertAlmostEqual(payload["actual_steering"]["wheel_angles_deg"]["axle_right"], 8.0)
        self.assertEqual(payload["engineering_evaluation"]["clearance_target_mm"], 47.5)

    def test_mechanism_graph_request_is_solved_and_serialized(self) -> None:
        raw = {
            "id": "three_point",
            "points": [
                {"id": "a", "mode": "fixed", "x_mm": 0, "y_mm": 0},
                {"id": "driver", "mode": "driven", "x_mm": 6, "y_mm": 0},
                {"id": "joint", "mode": "free", "x_mm": 3, "y_mm": 4},
            ],
            "members": [
                {
                    "id": "left_arm",
                    "point_a_id": "a",
                    "point_b_id": "joint",
                    "length_mm": 5,
                    "kind": "arm",
                },
                {
                    "id": "input_rod",
                    "point_a_id": "driver",
                    "point_b_id": "joint",
                    "length_mm": 5,
                    "kind": "rod",
                },
            ],
            "angle_outputs": [
                {
                    "id": "arm_angle",
                    "pivot_point_id": "a",
                    "endpoint_point_id": "joint",
                    "neutral_angle_deg": 53.130102354,
                }
            ],
        }
        graph = _parse_mechanism_graph(raw)
        driven = _parse_driven_positions({"driver": {"x_mm": 8, "y_mm": 0}})

        state = solve_mechanism_graph(graph, driven, geometric_tolerance_mm=1e-7)
        payload = _mechanism_graph_payload(graph, state)

        self.assertEqual(payload["mechanism"]["id"], "three_point")
        self.assertLessEqual(payload["state"]["maximum_residual_mm"], 0.01)
        self.assertAlmostEqual(payload["state"]["point_positions"]["joint"]["x_mm"], 4.0, places=3)
        self.assertIn("arm_angle", payload["state"]["output_angles_deg"])

    def test_combination_request_parses_and_serializes_real_maneuver(self) -> None:
        raw = {
            "id": "road_train",
            "name": "Road train",
            "root_body_id": "tractor",
            "bodies": [
                {"id": "tractor", "name": "Tractor", "pose": {"x_mm": 0, "y_mm": 0}},
                {"id": "trailer", "name": "Trailer"},
            ],
            "joints": [
                {
                    "id": "hitch",
                    "parent_body_id": "tractor",
                    "child_body_id": "trailer",
                    "parent_anchor": {"x_mm": -1500, "y_mm": 0},
                    "child_anchor": {"x_mm": 2500, "y_mm": 0},
                    "articulation_deg": 15,
                    "maximum_articulation_deg": 32,
                }
            ],
            "joint_ranges": {
                "hitch": {"min_deg": -30, "max_deg": 30, "step_deg": 5}
            },
            "mounted_axles": [
                {
                    "body_id": "tractor",
                    "local_center": {"x_mm": 0, "y_mm": 0},
                    "axle": {
                        "id": "tractor_axle",
                        "track_mm": 2400,
                        "steering_mode": "FORCED_STEER",
                    },
                },
                {
                    "body_id": "trailer",
                    "local_center": {"x_mm": -1500, "y_mm": 0},
                    "axle": {
                        "id": "trailer_axle",
                        "track_mm": 2500,
                        "steering_mode": "FORCED_STEER",
                    },
                },
            ],
        }
        combination = _parse_vehicle_combination(raw)
        solution = solve_combination_kinematics(combination, root_turn_radius_mm=9000.0)
        payload = _combination_kinematic_payload(combination, solution, root_pose=None)

        self.assertEqual(payload["combination"]["body_count"], 2)
        self.assertEqual(payload["combination"]["joint_count"], 1)
        self.assertEqual(len(payload["ideal_steering"]["axles"]), 2)
        self.assertAlmostEqual(payload["kinematics"]["root_turn_radius_mm"], 9000.0)
        self.assertAlmostEqual(payload["kinematics"]["maximum_joint_closure_error_mm"], 0.0)
        self.assertEqual(payload["combination"]["joints"][0]["sweep_min_deg"], -30.0)
        self.assertEqual(payload["combination"]["joints"][0]["sweep_step_deg"], 5.0)
        self.assertEqual(payload["combination"]["joints"][0]["maximum_articulation_deg"], 32.0)

    def test_combination_graph_sweep_solves_every_requested_pose(self) -> None:
        combination = _parse_vehicle_combination(
            {
                "id": "two_body_sweep",
                "root_body_id": "rear",
                "bodies": [{"id": "rear"}, {"id": "front"}],
                "joints": [
                    {
                        "id": "hitch",
                        "parent_body_id": "rear",
                        "child_body_id": "front",
                        "parent_anchor": {"x_mm": 1000, "y_mm": 0},
                        "child_anchor": {"x_mm": -1000, "y_mm": 0},
                    }
                ],
                "mounted_axles": [
                    {"body_id": "rear", "local_center": {"x_mm": 0, "y_mm": 0}, "axle": {"id": "rear_axle", "track_mm": 2500}},
                    {"body_id": "front", "local_center": {"x_mm": 0, "y_mm": 0}, "axle": {"id": "front_axle", "track_mm": 2500}},
                ],
            }
        )
        points = []
        outputs = []
        assignments = []
        for body_id, axle_id in (("rear", "rear_axle"), ("front", "front_axle")):
            for side, y_mm in (("left", 100.0), ("right", -100.0)):
                pivot_id = f"{body_id}_{side}_pivot"
                endpoint_id = f"{body_id}_{side}_endpoint"
                output_id = f"{body_id}_{side}_output"
                points.extend(
                    [
                        {"id": pivot_id, "mode": "fixed", "x_mm": 0, "y_mm": y_mm, "body_id": body_id},
                        {"id": endpoint_id, "mode": "fixed", "x_mm": 100, "y_mm": y_mm, "body_id": body_id},
                    ]
                )
                outputs.append(
                    {"id": output_id, "pivot_point_id": pivot_id, "endpoint_point_id": endpoint_id, "neutral_angle_deg": 0}
                )
                assignments.append(MechanismSteeringAssignment(output_id, f"{axle_id}_{side}"))
        graph = _parse_mechanism_graph(
            {"id": "fixed_outputs", "points": points, "members": [], "angle_outputs": outputs}
        )

        sweep = build_combination_sweep_payload(
            combination,
            root_turn_radius_mm=9000.0,
            mechanism_graph=graph,
            mechanism_drivers=(),
            steering_assignments=tuple(assignments),
            beta_min_deg=-20.0,
            beta_max_deg=20.0,
            step_deg=10.0,
            primary_joint_id="hitch",
        )

        self.assertEqual(sweep["sample_count"], 5)
        self.assertEqual(sweep["solved_sample_count"], 5)
        self.assertEqual([sample["beta_deg"] for sample in sweep["samples"]], [-20.0, -10.0, 0.0, 10.0, 20.0])
        steering = sweep["samples"][0]["steering"]
        self.assertEqual(
            set(steering["ideal_wheel_angles_deg"]),
            {"rear_axle_left", "rear_axle_right", "front_axle_left", "front_axle_right"},
        )
        self.assertEqual(
            set(steering["actual_wheel_angles_deg"]),
            {"rear_axle_left", "rear_axle_right", "front_axle_left", "front_axle_right"},
        )
        self.assertIn("wheel_errors_deg", steering)
        self.assertIn("ideal_axle_center_angles_deg", steering)
        self.assertIn("actual_axle_center_angles_deg", steering)

    def test_combination_graph_sweep_evaluates_all_configured_joint_combinations(self) -> None:
        combination = _parse_vehicle_combination(
            {
                "id": "three_body_sweep",
                "root_body_id": "tractor",
                "bodies": [
                    {"id": "tractor", "body_length_mm": 6000, "body_width_mm": 1400},
                    {"id": "dolly", "body_length_mm": 4000, "body_width_mm": 1400},
                    {"id": "trailer", "body_length_mm": 6000, "body_width_mm": 1400},
                ],
                "joints": [
                    {
                        "id": "tractor_dolly",
                        "parent_body_id": "tractor",
                        "child_body_id": "dolly",
                        "parent_anchor": {"x_mm": 3000, "y_mm": 0},
                        "child_anchor": {"x_mm": -2000, "y_mm": 0},
                    },
                    {
                        "id": "dolly_trailer",
                        "parent_body_id": "dolly",
                        "child_body_id": "trailer",
                        "parent_anchor": {"x_mm": 2000, "y_mm": 0},
                        "child_anchor": {"x_mm": -3000, "y_mm": 0},
                    },
                ],
                "mounted_axles": [
                    {"body_id": body_id, "local_center": {"x_mm": 0, "y_mm": 0}, "axle": {"id": f"{body_id}_axle", "track_mm": 1200}}
                    for body_id in ("tractor", "dolly", "trailer")
                ],
            }
        )
        points = []
        outputs = []
        assignments = []
        for body_id in ("tractor", "dolly", "trailer"):
            axle_id = f"{body_id}_axle"
            for side, y_mm in (("left", 100.0), ("right", -100.0)):
                pivot_id = f"{body_id}_{side}_pivot"
                endpoint_id = f"{body_id}_{side}_endpoint"
                output_id = f"{body_id}_{side}_output"
                points.extend(
                    [
                        {"id": pivot_id, "mode": "fixed", "x_mm": 0, "y_mm": y_mm, "body_id": body_id},
                        {"id": endpoint_id, "mode": "fixed", "x_mm": 100, "y_mm": y_mm, "body_id": body_id},
                    ]
                )
                outputs.append({"id": output_id, "pivot_point_id": pivot_id, "endpoint_point_id": endpoint_id})
                assignments.append({"output_id": output_id, "wheel_id": f"{axle_id}_{side}"})
        graph = _parse_mechanism_graph(
            {"id": "fixed_three_body_outputs", "points": points, "members": [], "angle_outputs": outputs}
        )

        sweep = build_combination_sweep_payload(
            combination,
            root_turn_radius_mm=9000.0,
            mechanism_graph=graph,
            mechanism_drivers=(),
            steering_assignments=tuple(
                MechanismSteeringAssignment(item["output_id"], item["wheel_id"])
                for item in assignments
            ),
            beta_min_deg=-10.0,
            beta_max_deg=10.0,
            step_deg=10.0,
            clearance_target_mm=37.5,
            joint_ranges={
                "tractor_dolly": {"min_deg": -10.0, "max_deg": 10.0, "step_deg": 10.0},
                "dolly_trailer": {"min_deg": -10.0, "max_deg": 10.0, "step_deg": 10.0},
            },
        )

        self.assertEqual(sweep["sample_count"], 9)
        self.assertEqual(sweep["solved_sample_count"], 9)
        self.assertTrue(sweep["sampling_complete"])
        self.assertEqual(sweep["clearance_target_mm"], 37.5)
        self.assertEqual(sweep["joint_ids"], ["tractor_dolly", "dolly_trailer"])
        self.assertEqual(sweep["samples"][0]["joint_angles_deg"], {"tractor_dolly": -10.0, "dolly_trailer": -10.0})
        self.assertIn(
            {"tractor_dolly": 10.0, "dolly_trailer": 10.0},
            [sample["joint_angles_deg"] for sample in sweep["samples"]],
        )

        limited_combination = replace(
            combination,
            joints=tuple(
                replace(joint, maximum_articulation_deg=5.0)
                if joint.id == "dolly_trailer"
                else joint
                for joint in combination.joints
            ),
        )
        limited_sweep = build_combination_sweep_payload(
            limited_combination,
            root_turn_radius_mm=9000.0,
            mechanism_graph=graph,
            mechanism_drivers=(),
            steering_assignments=tuple(
                MechanismSteeringAssignment(item["output_id"], item["wheel_id"])
                for item in assignments
            ),
            beta_min_deg=-10.0,
            beta_max_deg=10.0,
            step_deg=10.0,
            joint_ranges={
                "tractor_dolly": {"min_deg": -10.0, "max_deg": 10.0, "step_deg": 10.0},
                "dolly_trailer": {"min_deg": -10.0, "max_deg": 10.0, "step_deg": 10.0},
            },
        )

        self.assertEqual(limited_sweep["sample_count"], 9)
        self.assertTrue(
            any(
                "DRAWBAR_LIMIT_EXCEEDED" in sample.get("failed_checks", [])
                for sample in limited_sweep["samples"]
            )
        )
        self.assertTrue(any("dolly_trailer" in violation["joint_angles_deg"] for violation in limited_sweep["violations"]))

    def test_payload_accepts_editable_reference_geometry(self) -> None:
        payload = build_demo_payload(15.0, wheelbase_mm=5000.0, track_mm=2800.0)

        self.assertGreaterEqual(payload["vehicle"]["body_width_mm"], 3500.0)
        self.assertEqual(payload["vehicle_combination"]["body_count"], 2)
        self.assertTrue(math.isclose(payload["vehicle_combination"]["mounted_axles"][0]["track_mm"], 2800.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(payload["vehicle_combination"]["joints"][0]["articulation_deg"], 15.0, abs_tol=1e-9))

    def test_custom_linkage_is_solved_and_serialized(self) -> None:
        rig = _parse_linkage_rig(
            {
                "id": "custom_test_linkage",
                "steering_arm_length_mm": 150.0,
                "companion_steering_arm_length_mm": 155.0,
            }
        )
        payload = build_demo_payload(15.0, linkage_rig=rig)

        self.assertEqual(payload["linkage"]["spec"]["id"], "custom_test_linkage")
        self.assertEqual(payload["linkage"]["spec"]["steering_arm_length_mm"], 150.0)
        self.assertEqual(payload["linkage"]["spec"]["companion_steering_arm_length_mm"], 155.0)
        self.assertIsNotNone(payload["linkage"]["state"]["companion_steering_angle_rad"])

    def test_custom_linkage_rejects_non_positive_lengths(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be positive"):
            _parse_linkage_rig({"tie_rod_length_mm": 0.0})

    def test_arbitrary_vehicle_config_drives_kinematic_payload(self) -> None:
        raw_config = {
            "id": "three_axle_layout",
            "name": "Three axle test layout",
            "body_length_mm": 6800.0,
            "body_width_mm": 3400.0,
            "origin": {"x_mm": 100.0, "y_mm": -50.0},
            "body_polygon": [
                {"x_mm": -3400.0, "y_mm": -1700.0},
                {"x_mm": 3400.0, "y_mm": -1700.0},
                {"x_mm": 3400.0, "y_mm": 1700.0},
                {"x_mm": -3400.0, "y_mm": 1700.0},
            ],
            "front_articulation_point": {"x_mm": 2500.0, "y_mm": 0.0},
            "rear_articulation_point": {"x_mm": -2500.0, "y_mm": 0.0},
            "kingpin_point": {"x_mm": 0.0, "y_mm": 0.0},
            "maximum_articulation_deg": 50.0,
            "axles": [
                {"id": "axle_1", "x_mm": -2500.0, "track_mm": 2600.0, "steering_mode": "FIXED", "steerable": False, "tire_width_mm": 385.0, "outside_diameter_mm": 1100.0},
                {"id": "axle_2", "x_mm": 0.0, "track_mm": 2700.0, "steering_mode": "FORCED_STEER", "load_kg": 4200.0},
                {"id": "axle_3", "x_mm": 2500.0, "track_mm": 2800.0, "steering_mode": "USER_DEFINED", "user_defined_steering_angle_deg": 2.0},
            ],
            "steering_synchronizations": [
                {"id": "axle_2_sync", "target_axle_id": "axle_2", "source_axle_id": "axle_3", "mode": "OPPOSITE_PHASE"},
            ],
        }
        parsed = _parse_vehicle_config(raw_config)
        assert parsed is not None
        vehicle, normalized = parsed
        payload = build_demo_payload(10.0, vehicle=vehicle)

        self.assertEqual(len(payload["axles"]), 3)
        self.assertEqual(payload["vehicle"]["axle_count"], 3)
        self.assertEqual(payload["vehicle_config"], normalized)
        self.assertEqual(payload["axles"][1]["load_kg"], 4200.0)
        self.assertIsNone(payload["vehicle_combination"])
        self.assertIsNotNone(payload["metrics"]["front_rear_phase_deg"])
        self.assertEqual(len(payload["actual_steering"]["axles"]), 3)
        self.assertIn("axle_2_left", payload["actual_steering"]["errors_deg"])
        self.assertEqual(payload["actual_steering"]["axles"][1]["synchronization_mode"], "OPPOSITE_PHASE")
        self.assertEqual(vehicle.wheels()[0].outside_diameter_mm, 1100.0)
        self.assertEqual(sum(item["id"].endswith("_tire") for item in payload["clearance"]["items"]), 2)
        self.assertEqual(normalized["origin"], {"x_mm": 100.0, "y_mm": -50.0})
        self.assertEqual(normalized["maximum_articulation_deg"], 50.0)
        self.assertEqual(payload["body_outline"][0], {"x_mm": -3300.0, "y_mm": -1750.0})
        with self.assertRaises(ArticulationLimitExceededError):
            build_demo_payload(55.0, vehicle=vehicle)

    def test_ideal_request_preserves_body_polygon_and_sync_alias(self) -> None:
        payload = _ideal_steering_request_payload(
            {
                "body_length_mm": 5000.0,
                "body_width_mm": 2800.0,
                "origin": {"x_mm": 100.0, "y_mm": 25.0},
                "body_polygon": [
                    {"x_mm": -2500.0, "y_mm": -1400.0},
                    {"x_mm": 2500.0, "y_mm": -1400.0},
                    {"x_mm": 2500.0, "y_mm": 1400.0},
                ],
                "axles": [
                    {"id": "front", "x_mm": 1600.0, "y_mm": 0.0, "track_mm": 2400.0},
                    {"id": "rear", "x_mm": -1600.0, "y_mm": 0.0, "track_mm": 2400.0},
                ],
                "steering_sync": [
                    {"id": "rear_phase", "target_axle_id": "rear", "mode": "OPPOSITE_PHASE"},
                ],
                "turn_radius_mm": 12000.0,
            }
        )

        self.assertEqual(payload["body_outline"][0], {"x_mm": -2400.0, "y_mm": -1375.0})
        self.assertEqual(payload["vehicle_config"]["steering_synchronizations"][0]["id"], "rear_phase")


if __name__ == "__main__":
    unittest.main()
