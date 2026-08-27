from __future__ import annotations

import math
import unittest

from easytowing.errors import ArticulationLimitExceededError
from easytowing.demo_server import _parse_linkage_rig, _parse_vehicle_config, build_demo_payload
from easytowing.demo_server import _ideal_steering_request_payload


class DemoPayloadTests(unittest.TestCase):
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
