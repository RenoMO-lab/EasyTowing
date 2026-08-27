from __future__ import annotations

from io import BytesIO
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pypdf import PdfReader  # type: ignore  # noqa: E402

from easytowing.geometry import Point2D  # type: ignore  # noqa: E402
from easytowing.model import Axle, SteeringSynchronization, VehicleLayout  # type: ignore  # noqa: E402
from easytowing.reporting import build_dimensioned_svg, build_export_bundle, build_export_csv, build_export_dxf, build_export_pdf, build_steering_curves_svg, build_swept_path_svg  # type: ignore  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
