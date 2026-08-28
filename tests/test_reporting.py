from __future__ import annotations

from io import BytesIO
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pypdf import PdfReader  # type: ignore  # noqa: E402

from easytowing.geometry import Point2D  # type: ignore  # noqa: E402
from easytowing.model import Axle, SteeringSynchronization, VehicleLayout  # type: ignore  # noqa: E402
from easytowing.reporting import build_dimensioned_svg, build_engineering_snapshot_csv, build_engineering_snapshot_dxf, build_engineering_snapshot_pdf, build_engineering_snapshot_png, build_engineering_snapshot_svg, build_export_bundle, build_export_csv, build_export_dxf, build_export_pdf, build_steering_curves_svg, build_swept_path_svg, evaluate_engineering_snapshot  # type: ignore  # noqa: E402


class ReportingTests(unittest.TestCase):
    def test_export_bundle_includes_comparison_and_optimization(self) -> None:
        bundle = build_export_bundle(0.0, "quick")

        self.assertIn("vehicle", bundle)
        self.assertIn("baseline", bundle)
        self.assertIn("optimized", bundle)
        self.assertIn("optimization", bundle)
        self.assertIn("comparison", bundle)
        self.assertIn("metrics", bundle["comparison"])
        self.assertGreater(len(bundle["comparison"]["metrics"]), 0)
        self.assertIn("actual_steering", bundle)
        self.assertIn("baseline", bundle["actual_steering"])
        self.assertTrue(any(row[0] == "Max synchronization error" for row in bundle["comparison"]["metrics"]))

    def test_export_csv_contains_expected_columns(self) -> None:
        csv_text = build_export_csv(0.0, "quick")
        header = csv_text.splitlines()[0]

        self.assertIn("beta_deg", header)
        self.assertIn("ideal_front_left_deg", header)
        self.assertIn("baseline_clearance_mm", header)
        self.assertIn("optimized_clearance_mm", header)
        self.assertIn("baseline_front_axle_left_actual_deg", header)

    def test_export_csv_contains_each_synchronization_channel(self) -> None:
        vehicle = VehicleLayout(
            id="sync_csv_case",
            name="Sync CSV case",
            axles=(
                Axle(id="rear", center=Point2D(-2180.0, 0.0), track_mm=2500.0),
                Axle(id="front", center=Point2D(2180.0, 0.0), track_mm=2500.0),
            ),
            body_length_mm=6160.0,
            body_width_mm=3200.0,
            steering_synchronizations=(
                SteeringSynchronization(
                    id="rear_sync",
                    target_axle_id="rear",
                    source_axle_id="front",
                    mode="OPPOSITE_PHASE",
                ),
            ),
        )

        header = build_export_csv(0.0, "quick", vehicle=vehicle).splitlines()[0]

        self.assertIn("baseline_rear_sync_sync_error_deg", header)
        self.assertIn("optimized_rear_sync_sync_error_deg", header)

    def test_export_dxf_contains_expected_entities(self) -> None:
        dxf_text = build_export_dxf(0.0, "quick")

        self.assertIn("SECTION", dxf_text)
        self.assertIn("ENTITIES", dxf_text)
        self.assertIn("LWPOLYLINE", dxf_text)
        self.assertIn("EasyTowing Engineering Sketch", dxf_text)
        self.assertIn("OPTIMIZED", dxf_text)

    def test_exports_include_companion_linkage_geometry(self) -> None:
        dxf_text = build_export_dxf(30.0, "quick")
        svg_text = build_dimensioned_svg(30.0, "quick")

        self.assertGreaterEqual(dxf_text.count("BASELINE\n"), 7)
        self.assertGreaterEqual(dxf_text.count("OPTIMIZED\n"), 7)
        self.assertIn("companion knuckle", svg_text)
        self.assertIn("Companion tie rod", svg_text)

    def test_export_pdf_contains_report_sections(self) -> None:
        pdf_bytes = build_export_pdf(0.0, "quick")

        self.assertTrue(pdf_bytes.startswith(b"%PDF"))
        self.assertGreater(len(pdf_bytes), 5000)

        reader = PdfReader(BytesIO(pdf_bytes))
        self.assertEqual(len(reader.pages), 2)
        extracted_text = "\n".join(page.extract_text() or "" for page in reader.pages)

        self.assertIn("EasyTowing Engineering Report", extracted_text)
        self.assertIn("Project and vehicle", extracted_text)
        self.assertIn("Steering sweep", extracted_text)
        self.assertIn("Swept path metrics", extracted_text)

    def test_steering_curves_svg_contains_legends_and_summary(self) -> None:
        svg_text = build_steering_curves_svg(0.0, "quick")

        self.assertIn("Steering curves", svg_text)
        self.assertIn("Wheel and linkage angles", svg_text)
        self.assertIn("Front-axle steering error", svg_text)
        self.assertIn("Optimized steer", svg_text)
        self.assertIn("Baseline error", svg_text)

    def test_swept_path_svg_contains_envelopes_and_tracks(self) -> None:
        svg_text = build_swept_path_svg(0.0, "quick")

        self.assertIn("Swept path preview", svg_text)
        self.assertIn("Left-turn envelope", svg_text)
        self.assertIn("Right-turn envelope", svg_text)
        self.assertIn("wheel-center trajectories", svg_text)
        self.assertIn("current pose", svg_text)

    def test_dimensioned_svg_contains_sketch_markers(self) -> None:
        svg_text = build_dimensioned_svg(0.0, "quick")

        self.assertIn("<svg", svg_text)
        self.assertIn("EasyTowing Engineering Sketch", svg_text)
        self.assertIn("Existing design = dashed; optimized design = solid.", svg_text)
        self.assertIn("dimension-main", svg_text)

    def test_multi_body_diagnostic_exports_are_explicitly_failed_and_traceable(self) -> None:
        snapshot = {
            "beta_deg": 20.0,
            "turn_radius_mm": 9000.0,
            "vehicle_combination": {
                "body_count": 2,
                "joint_count": 1,
                "mounted_axle_count": 1,
                "bodies": [
                    {
                        "id": "rear_body",
                        "name": "Rear body",
                        "pose": {"x_mm": -1000.0, "y_mm": 0.0, "yaw_rad": 0.0},
                        "body_length_mm": 1800.0,
                        "body_width_mm": 2600.0,
                        "body_polygon": [],
                    },
                    {
                        "id": "front_body",
                        "name": "Front body",
                        "pose": {"x_mm": 1000.0, "y_mm": 100.0, "yaw_rad": 0.1},
                        "body_length_mm": 1800.0,
                        "body_width_mm": 2600.0,
                        "body_polygon": [],
                    },
                ],
                "joints": [{
                    "id": "front_joint",
                    "parent_body_id": "rear_body",
                    "child_body_id": "front_body",
                    "parent_anchor": {"x_mm": 900.0, "y_mm": 0.0},
                    "child_anchor": {"x_mm": -900.0, "y_mm": 0.0},
                    "articulation_deg": 20.0,
                    "maximum_articulation_deg": 45.0,
                }],
            },
            "combination_kinematics": {"maximum_constraint_residual_mm": 0.0},
            "mechanism_graph": {
                "mechanism": {"points": [{"id": "pivot"}], "members": [{"id": "arm"}]},
                "state": {"maximum_residual_mm": 0.0},
            },
            "mechanism_mapping": {
                "steering_assignments": [{"wheel_id": "axle_left", "output_id": "left_output"}],
            },
            "axles": [
                {
                    "axle_id": "axle",
                    "left_wheel": {"wheel_id": "axle_left", "steering_angle_deg": 12.0},
                    "right_wheel": {"wheel_id": "axle_right", "steering_angle_deg": 10.0},
                }
            ],
            "actual_steering": {
                "wheel_angles_deg": {"axle_left": 11.0, "axle_right": 9.5},
                "errors_deg": {"axle_left": -1.0, "axle_right": -0.5},
            },
            "clearance": {"collision_detected": True, "minimum_clearance_mm": -4.0},
            "metrics": {"max_abs_wheel_error_deg": 1.0, "max_abs_synchronization_error_deg": 0.5},
        }

        evaluation = evaluate_engineering_snapshot(snapshot)
        csv_text = build_engineering_snapshot_csv(snapshot, project_name="Pilot", revision_id="rev_1")
        pdf_bytes = build_engineering_snapshot_pdf(snapshot, project_name="Pilot", revision_id="rev_1")

        self.assertEqual(evaluation["status"], "FAIL")
        self.assertEqual(
            [item["check_id"] for item in evaluation["guidance"]],
            ["COLLISION", "CLEARANCE"],
        )
        self.assertIn("engineering_status", csv_text)
        self.assertIn("left_output", csv_text)
        reader = PdfReader(BytesIO(pdf_bytes))
        self.assertEqual(len(reader.pages), 2)
        extracted_text = "\n".join(page.extract_text() or "" for page in reader.pages)
        self.assertIn("FAIL", extracted_text)
        self.assertIn("NOT APPROVED FOR MANUFACTURING", extracted_text)
        self.assertIn("left_output", extracted_text)

        svg_text = build_engineering_snapshot_svg(snapshot, project_name="Pilot", revision_id="rev_1")
        dxf_text = build_engineering_snapshot_dxf(snapshot, project_name="Pilot", revision_id="rev_1")
        png_bytes = build_engineering_snapshot_png(snapshot, project_name="Pilot", revision_id="rev_1")
        self.assertIn("EasyTowing Multi-body Engineering Sketch", svg_text)
        self.assertIn("Overall length", svg_text)
        self.assertIn("LWPOLYLINE", dxf_text)
        self.assertIn("DIAGNOSTIC ONLY", dxf_text)
        self.assertTrue(png_bytes.startswith(b"\x89PNG\r\n\x1a\n"))

    def test_multi_body_evaluation_rejects_missing_or_malformed_body_envelopes(self) -> None:
        snapshot = {
            "vehicle_combination": {
                "body_count": 2,
                "bodies": [
                    {
                        "id": "tractor",
                        "body_length_mm": 6000.0,
                        "body_width_mm": 2500.0,
                        "body_polygon": [],
                    },
                    {
                        "id": "trailer",
                        "body_length_mm": 0.0,
                        "body_width_mm": 0.0,
                        "body_polygon": [],
                    },
                ],
            },
            "combination_kinematics": {"maximum_constraint_residual_mm": 0.0},
            "mechanism_graph": {"state": {"maximum_residual_mm": 0.0}},
            "clearance": {"collision_detected": False, "minimum_clearance_mm": 25.0},
        }

        evaluation = evaluate_engineering_snapshot(snapshot)
        self.assertEqual(evaluation["status"], "FAIL")
        self.assertEqual(
            [check["id"] for check in evaluation["checks"] if not check["pass"]],
            ["MODEL_COMPLETENESS"],
        )

        snapshot["vehicle_combination"]["bodies"][1]["body_polygon"] = [
            {"x_mm": "not-a-number", "y_mm": 0.0},
            {"x_mm": 10.0, "y_mm": 0.0},
            {"x_mm": 0.0, "y_mm": 10.0},
        ]
        evaluation = evaluate_engineering_snapshot(snapshot)
        self.assertEqual(evaluation["status"], "FAIL")
        self.assertEqual(
            evaluation["checks"][0]["detail"],
            "body 'trailer' has no positive envelope dimensions or CAD outline",
        )

    def test_snapshot_reports_use_saved_clearance_target(self) -> None:
        snapshot = {
            "beta_deg": 0.0,
            "turn_radius_mm": 9000.0,
            "combination_kinematics": {"maximum_constraint_residual_mm": 0.0},
            "mechanism_graph": {"state": {"maximum_residual_mm": 0.0}},
            "clearance": {"collision_detected": False, "minimum_clearance_mm": 30.0},
            "metrics": {},
            "sweep_validation": {"clearance_target_mm": 50.0},
            "axles": [{
                "axle_id": "axle_1",
                "left_wheel": {"wheel_id": "axle_1_left", "steering_angle_deg": 0.0},
                "right_wheel": {"wheel_id": "axle_1_right", "steering_angle_deg": 0.0},
            }],
            "actual_steering": {
                "wheel_angles_deg": {"axle_1_left": 0.0, "axle_1_right": 0.0},
                "errors_deg": {"axle_1_left": 0.0, "axle_1_right": 0.0},
            },
        }

        self.assertEqual(evaluate_engineering_snapshot(snapshot)["status"], "PASS")
        csv_text = build_engineering_snapshot_csv(snapshot, project_name="Pilot", revision_id="rev_1")
        pdf_bytes = build_engineering_snapshot_pdf(snapshot, project_name="Pilot", revision_id="rev_1")

        self.assertIn(",FAIL,", csv_text)
        extracted_text = "\n".join(page.extract_text() or "" for page in PdfReader(BytesIO(pdf_bytes)).pages)
        self.assertIn("50.0 mm required", extracted_text)

    def test_missing_required_kinematics_evidence_fails_closed(self) -> None:
        evaluation = evaluate_engineering_snapshot(
            {
                "mechanism_graph": {"state": {"maximum_residual_mm": 0.0}},
                "clearance": {"collision_detected": False, "minimum_clearance_mm": 25.0},
            }
        )

        self.assertEqual(evaluation["status"], "FAIL")
        self.assertEqual([check["id"] for check in evaluation["checks"] if not check["pass"]], ["KINEMATICS"])


if __name__ == "__main__":
    unittest.main()
