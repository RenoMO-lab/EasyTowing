from __future__ import annotations

import unittest

from easytowing.acceptance import MonrocAcceptanceCriteria, evaluate_monroc_acceptance


def passing_snapshot() -> dict[str, object]:
    return {
        "metrics": {
            "max_abs_wheel_error_deg": 0.8,
            "max_abs_synchronization_error_deg": 0.3,
        },
        "mechanism_graph": {"state": {"maximum_residual_mm": 0.004}},
        "clearance": {"collision_detected": False, "minimum_clearance_mm": 28.0},
        "sweep_validation": {
            "status": "PASS",
            "sample_count": 3,
            "solved_sample_count": 3,
            "minimum_clearance_mm": 24.0,
            "max_abs_wheel_error_deg": 1.1,
            "max_abs_synchronization_error_deg": 0.4,
            "samples": [
                {
                    "minimum_clearance_mm": 24.0,
                    "max_abs_wheel_error_deg": 1.1,
                    "max_abs_synchronization_error_deg": 0.4,
                    "maximum_mechanism_residual_mm": 0.006,
                    "collision_detected": False,
                },
            ],
        },
    }


class AcceptanceTests(unittest.TestCase):
    def test_missing_criteria_are_not_configured(self) -> None:
        result = evaluate_monroc_acceptance(passing_snapshot(), None)
        self.assertEqual(result["status"], "NOT_CONFIGURED")
        self.assertFalse(result["configured"])

    def test_explicit_criteria_pass_saved_full_range_evidence(self) -> None:
        criteria = MonrocAcceptanceCriteria(
            case_id="MONROC-01",
            minimum_clearance_mm=20.0,
            maximum_wheel_error_deg=2.0,
            maximum_synchronization_error_deg=1.0,
        )
        result = evaluate_monroc_acceptance(passing_snapshot(), criteria)
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(all(check["status"] == "PASS" for check in result["checks"]))

    def test_missing_full_range_or_excess_error_fails_closed(self) -> None:
        snapshot = passing_snapshot()
        snapshot["metrics"] = {"max_abs_wheel_error_deg": 3.0}
        snapshot.pop("sweep_validation")
        result = evaluate_monroc_acceptance(
            snapshot,
            {
                "case_id": "MONROC-02",
                "minimum_clearance_mm": 20.0,
                "maximum_wheel_error_deg": 2.0,
                "maximum_synchronization_error_deg": 1.0,
            },
        )
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("STEERING_ACCURACY", [check["id"] for check in result["checks"]])
        self.assertIn("FULL_RANGE", [check["id"] for check in result["checks"]])

    def test_incomplete_multi_joint_sampling_cannot_pass_full_range(self) -> None:
        snapshot = passing_snapshot()
        snapshot["sweep_validation"]["sampling_complete"] = False

        result = evaluate_monroc_acceptance(
            snapshot,
            {
                "case_id": "MONROC-INCOMPLETE-GRID",
                "minimum_clearance_mm": 20.0,
                "maximum_wheel_error_deg": 2.0,
                "maximum_synchronization_error_deg": 1.0,
            },
        )

        self.assertEqual(result["status"], "FAIL")
        full_range = next(check for check in result["checks"] if check["id"] == "FULL_RANGE")
        self.assertEqual(full_range["status"], "FAIL")

    def test_criteria_reject_non_finite_limits(self) -> None:
        with self.assertRaises(ValueError):
            MonrocAcceptanceCriteria(
                case_id="MONROC-03",
                minimum_clearance_mm=float("nan"),
                maximum_wheel_error_deg=2.0,
                maximum_synchronization_error_deg=1.0,
            )


if __name__ == "__main__":
    unittest.main()
