from __future__ import annotations

import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from easytowing.pilot import validate_pilot_case


def _write_json(path: Path, payload: object) -> str:
    content = json.dumps(payload, sort_keys=True).encode("utf-8")
    path.write_bytes(content)
    return hashlib.sha256(content).hexdigest()


class PilotValidationTests(unittest.TestCase):
    def _package(self, root: Path) -> dict[str, object]:
        snapshot = {
            "metrics": {
                "max_abs_wheel_error_deg": 0.8,
                "max_abs_synchronization_error_deg": 0.3,
            },
            "mechanism_graph": {"state": {"maximum_residual_mm": 0.004}},
            "clearance": {"collision_detected": False, "minimum_clearance_mm": 28.0},
            "sweep_validation": {
                "status": "PASS",
                "sampling_complete": True,
                "sample_count": 3,
                "solved_sample_count": 3,
                "minimum_clearance_mm": 24.0,
                "max_abs_wheel_error_deg": 1.1,
                "max_abs_synchronization_error_deg": 0.4,
                "samples": [
                    {
                        "joint_angles_deg": {"hitch": 0.0},
                        "minimum_clearance_mm": 24.0,
                        "max_abs_wheel_error_deg": 1.1,
                        "max_abs_synchronization_error_deg": 0.4,
                        "maximum_mechanism_residual_mm": 0.006,
                        "collision_detected": False,
                        "steering": {
                            "ideal_wheel_angles_deg": {"left_wheel": 1.0, "right_wheel": -1.0},
                            "actual_wheel_angles_deg": {"left_wheel": 1.1, "right_wheel": -1.1},
                            "wheel_errors_deg": {"left_wheel": 0.1, "right_wheel": -0.1},
                            "ideal_axle_center_angles_deg": {"axle": 0.0},
                            "actual_axle_center_angles_deg": {"axle": 0.0},
                            "synchronization_errors_deg": {},
                        },
                    },
                ],
            },
        }
        snapshot_path = root / "engineering-snapshot.json"
        hand_path = root / "hand-calculation.json"
        reference_path = root / "approved-reference.json"
        cad_path = root / "approved-layout.dxf"
        snapshot_sha = _write_json(snapshot_path, snapshot)
        comparison = {
            "metrics": {
                "minimum_clearance_mm": 24.0,
                "max_abs_wheel_error_deg": 1.1,
                "max_abs_synchronization_error_deg": 0.4,
                "maximum_mechanism_residual_mm": 0.006,
            },
            "tolerances": {
                "minimum_clearance_mm": 0.1,
                "max_abs_wheel_error_deg": 0.01,
                "max_abs_synchronization_error_deg": 0.01,
                "maximum_mechanism_residual_mm": 0.001,
            },
            "steering_fields": [
                "ideal_wheel_angles_deg",
                "actual_wheel_angles_deg",
                "wheel_errors_deg",
                "ideal_axle_center_angles_deg",
                "actual_axle_center_angles_deg",
                "synchronization_errors_deg",
            ],
            "steering_tolerance_deg": 0.01,
            "steering_samples": [
                {
                    "joint_angles_deg": {"hitch": 0.0},
                    "steering": {
                        "ideal_wheel_angles_deg": {"left_wheel": 1.0, "right_wheel": -1.0},
                        "actual_wheel_angles_deg": {"left_wheel": 1.1, "right_wheel": -1.1},
                        "wheel_errors_deg": {"left_wheel": 0.1, "right_wheel": -0.1},
                        "ideal_axle_center_angles_deg": {"axle": 0.0},
                        "actual_axle_center_angles_deg": {"axle": 0.0},
                        "synchronization_errors_deg": {},
                    },
                },
            ],
        }
        hand_sha = _write_json(hand_path, comparison)
        reference_sha = _write_json(reference_path, comparison)
        cad_path.write_bytes(b"0\nSECTION\n2\nENTITIES\n0\nENDSEC\n0\nEOF\n")
        cad_sha = hashlib.sha256(cad_path.read_bytes()).hexdigest()
        return {
            "schema_version": 1,
            "case_id": "TEST-PILOT-01",
            "criteria": {
                "case_id": "TEST-PILOT-01",
                "minimum_clearance_mm": 20.0,
                "maximum_wheel_error_deg": 2.0,
                "maximum_synchronization_error_deg": 1.0,
                "maximum_mechanism_residual_mm": 0.01,
                "require_full_range": True,
            },
            "cad_source": {
                "path": cad_path.name,
                "sha256": cad_sha,
                "revision": "TEST-CAD-REV-A",
            },
            "engineering_snapshot": {"path": snapshot_path.name, "sha256": snapshot_sha},
            "comparisons": [
                {"id": "hand_calculation", "path": hand_path.name, "sha256": hand_sha},
                {"id": "approved_reference", "path": reference_path.name, "sha256": reference_sha},
            ],
        }

    def test_pilot_case_requires_hashed_evidence_and_two_comparisons(self) -> None:
        with TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            result = validate_pilot_case(self._package(root), base_dir=root)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["release_authority"], "none")
        self.assertEqual({comparison["id"] for comparison in result["comparisons"]}, {"hand_calculation", "approved_reference"})

    def test_pilot_case_fails_closed_on_cad_hash_mismatch(self) -> None:
        with TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self._package(root)
            manifest["cad_source"]["sha256"] = "0" * 64
            result = validate_pilot_case(manifest, base_dir=root)

        self.assertEqual(result["status"], "INVALID_PACKAGE")
        self.assertIn("SHA-256", result["message"])

    def test_pilot_case_fails_when_reference_metric_is_outside_tolerance(self) -> None:
        with TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self._package(root)
            reference_path = root / "approved-reference.json"
            reference_payload = json.loads(reference_path.read_text(encoding="utf-8"))
            reference_payload["metrics"]["max_abs_wheel_error_deg"] = 5.0
            reference_sha = _write_json(reference_path, reference_payload)
            manifest["comparisons"][1]["sha256"] = reference_sha
            result = validate_pilot_case(manifest, base_dir=root)

        self.assertEqual(result["status"], "FAIL")
        reference = next(comparison for comparison in result["comparisons"] if comparison["id"] == "approved_reference")
        self.assertEqual(reference["status"], "FAIL")

    def test_pilot_case_rejects_aggregate_only_comparison_evidence(self) -> None:
        with TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self._package(root)
            hand_path = root / "hand-calculation.json"
            hand_payload = json.loads(hand_path.read_text(encoding="utf-8"))
            hand_payload.pop("steering_fields")
            hand_payload.pop("steering_tolerance_deg")
            hand_payload.pop("steering_samples")
            hand_sha = _write_json(hand_path, hand_payload)
            manifest["comparisons"][0]["sha256"] = hand_sha
            result = validate_pilot_case(manifest, base_dir=root)

        self.assertEqual(result["status"], "INVALID_PACKAGE")
        self.assertIn("steering_fields", result["message"])


if __name__ == "__main__":
    unittest.main()
