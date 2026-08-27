from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.dxf_import import analyze_dxf_import, apply_dxf_role_overrides, serialize_dxf_import_report  # type: ignore  # noqa: E402
from easytowing.reporting import build_export_dxf  # type: ignore  # noqa: E402


class DxfImportTests(unittest.TestCase):
    def test_import_report_reconstructs_reference_export(self) -> None:
        dxf_text = build_export_dxf(0.0, "quick")
        report = analyze_dxf_import(dxf_text, source_name="reference-export.dxf")

        self.assertGreater(report.entity_count, 0)
        self.assertIn("LINE", report.counts_by_type)
        self.assertIn("CIRCLE", report.counts_by_type)
        self.assertIn("LWPOLYLINE", report.counts_by_type)
        self.assertIsNotNone(report.reconstructed_vehicle)

        vehicle = report.reconstructed_vehicle
        assert vehicle is not None
        self.assertEqual(len(vehicle.axles), 2)
        self.assertAlmostEqual(vehicle.body_length_mm, 6160.0, places=1)
        self.assertAlmostEqual(vehicle.body_width_mm, 3200.0, places=1)

        serialized = serialize_dxf_import_report(report)
        self.assertEqual(serialized["source_name"], "reference-export.dxf")
        self.assertIn("entities", serialized)
        self.assertIn("reconstructed_vehicle", serialized)
        self.assertIn("parametric_mechanism", serialized)
        self.assertIsNotNone(report.parametric_mechanism)
        assert report.parametric_mechanism is not None
        self.assertTrue(report.parametric_mechanism.components)

    def test_import_report_handles_polyline_vertices(self) -> None:
        dxf_text = "\n".join(
            [
                "0",
                "SECTION",
                "2",
                "ENTITIES",
                "0",
                "POLYLINE",
                "8",
                "BODY",
                "70",
                "1",
                "0",
                "VERTEX",
                "10",
                "0",
                "20",
                "0",
                "0",
                "VERTEX",
                "10",
                "100",
                "20",
                "0",
                "0",
                "VERTEX",
                "10",
                "100",
                "20",
                "50",
                "0",
                "VERTEX",
                "10",
                "0",
                "20",
                "50",
                "0",
                "SEQEND",
                "0",
                "ENDSEC",
                "0",
                "EOF",
            ]
        )
        report = analyze_dxf_import(dxf_text, source_name="polyline-test.dxf")

        self.assertEqual(report.entity_count, 1)
        self.assertEqual(report.entities[0].entity_type, "POLYLINE")
        self.assertEqual(report.entities[0].suggested_role, "body_envelope")
        self.assertEqual(report.entities[0].geometry["vertex_count"], 4)
        self.assertIsNotNone(report.reconstructed_vehicle)

    def test_manual_assignment_override_changes_reconstructed_layout(self) -> None:
        dxf_text = build_export_dxf(0.0, "quick")
        report = analyze_dxf_import(dxf_text, source_name="override-test.dxf")
        body_entity = next(entity for entity in report.entities if entity.suggested_role == "body_envelope")

        overridden = apply_dxf_role_overrides(report, {body_entity.index: None})

        self.assertIsNotNone(overridden.reconstructed_vehicle)
        vehicle = overridden.reconstructed_vehicle
        assert vehicle is not None
        self.assertAlmostEqual(vehicle.body_length_mm, 0.0, places=1)
        self.assertAlmostEqual(vehicle.body_width_mm, 0.0, places=1)
        self.assertEqual(len(vehicle.axles), 2)

        overridden_entity = next(entity for entity in overridden.entities if entity.index == body_entity.index)
        self.assertEqual(overridden_entity.assigned_role, "")
        self.assertEqual(overridden_entity.reason, "Manually cleared in the DXF assignment workflow.")

    def test_manual_assignment_builds_fixed_length_parametric_component(self) -> None:
        dxf_text = "\n".join(
            [
                "0", "SECTION", "2", "ENTITIES",
                "0", "CIRCLE", "8", "PIVOT", "10", "0", "20", "0", "40", "10",
                "0", "LINE", "8", "LINKAGE", "10", "0", "20", "0", "11", "300", "21", "400",
                "0", "ENDSEC", "0", "EOF",
            ]
        )
        report = analyze_dxf_import(dxf_text, source_name="parametric-test.dxf")
        line = next(entity for entity in report.entities if entity.entity_type == "LINE")
        overridden = apply_dxf_role_overrides(report, {line.index: "tie_rod"})

        mechanism = overridden.parametric_mechanism
        self.assertIsNotNone(mechanism)
        assert mechanism is not None
        tie_rods = [component for component in mechanism.components if component.role == "tie_rod"]
        self.assertEqual(len(tie_rods), 1)
        self.assertAlmostEqual(tie_rods[0].length_mm or 0.0, 500.0, places=6)
        self.assertEqual(tie_rods[0].point_a_id, "point_0_center")
        self.assertEqual(tie_rods[0].point_b_id, "point_1_b")

    def test_manual_assignment_rejects_unknown_role(self) -> None:
        report = analyze_dxf_import(build_export_dxf(0.0, "quick"), source_name="invalid-role.dxf")

        with self.assertRaisesRegex(ValueError, "Unsupported DXF role"):
            apply_dxf_role_overrides(report, {0: "not_a_mechanical_role"})


if __name__ == "__main__":
    unittest.main()
