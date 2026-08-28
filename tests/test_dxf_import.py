from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.dxf_import import analyze_dxf_import, apply_dxf_role_overrides, serialize_dxf_import_report  # type: ignore  # noqa: E402
from easytowing.reporting import build_export_dxf  # type: ignore  # noqa: E402


class DxfImportTests(unittest.TestCase):
    @staticmethod
    def _headered_line_dxf() -> str:
        return "\n".join(
            [
                "0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "1", "0", "ENDSEC",
                "0", "SECTION", "2", "ENTITIES",
                "0", "LINE", "8", "AXLE", "10", "0", "20", "-50", "11", "0", "21", "50",
                "0", "ENDSEC", "0", "EOF",
            ]
        )

    def test_header_units_are_detected_and_explicit_metadata_scales_geometry(self) -> None:
        report = analyze_dxf_import(
            self._headered_line_dxf(),
            source_name="inch-layout.dxf",
            source_units="in",
            coordinate_system="x_forward_y_left",
            confirm_metadata=True,
        )

        self.assertTrue(report.import_ready)
        self.assertEqual(report.detected_units, "in")
        self.assertEqual(report.source_units, "in")
        self.assertAlmostEqual(report.unit_scale_to_mm or 0.0, 25.4)
        self.assertEqual(report.coordinate_system, "x_forward_y_left")
        self.assertEqual(report.reconstructed_vehicle.axles[0].track_mm, 2540.0)  # type: ignore[union-attr]
        payload = serialize_dxf_import_report(report)
        self.assertEqual(payload["source_sha256"], report.source_sha256)
        self.assertEqual(payload["reconstructed_vehicle"]["cad_source"]["source_units"], "in")  # type: ignore[index]

    def test_coordinate_frame_mirror_is_applied_before_reconstruction(self) -> None:
        dxf_text = "\n".join(
            [
                "0", "SECTION", "2", "ENTITIES",
                "0", "LINE", "8", "AXLE", "10", "0", "20", "10", "11", "0", "21", "110",
                "0", "ENDSEC", "0", "EOF",
            ]
        )
        report = analyze_dxf_import(
            dxf_text,
            source_name="mirrored-layout.dxf",
            source_units="mm",
            coordinate_system="x_forward_y_right",
            confirm_metadata=True,
        )

        entity = report.entities[0]
        self.assertEqual(entity.geometry["start"].y_mm, -10.0)  # type: ignore[index]
        self.assertEqual(entity.geometry["end"].y_mm, -110.0)  # type: ignore[index]

    def test_missing_metadata_keeps_import_in_preview_only_state(self) -> None:
        report = analyze_dxf_import(self._headered_line_dxf(), source_name="unconfirmed.dxf")

        self.assertFalse(report.import_ready)
        self.assertEqual(report.detected_units, "in")
        self.assertEqual(report.source_units, "in")
        self.assertIsNone(report.coordinate_system)
        self.assertTrue(any("coordinate frame" in warning for warning in report.warnings))

    def test_unsupported_entities_block_confirmed_activation(self) -> None:
        dxf_text = "\n".join(
            [
                "0", "SECTION", "2", "ENTITIES",
                "0", "3DFACE", "8", "BODY",
                "0", "LINE", "8", "AXLE", "10", "0", "20", "-50", "11", "0", "21", "50",
                "0", "ENDSEC", "0", "EOF",
            ]
        )
        report = analyze_dxf_import(
            dxf_text,
            source_name="partial-layout.dxf",
            source_units="mm",
            coordinate_system="x_forward_y_left",
            confirm_metadata=True,
        )

        self.assertEqual(report.unsupported_entity_count, 1)
        self.assertFalse(report.import_ready)
        self.assertTrue(any("unsupported DXF" in warning for warning in report.warnings))

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
        assert report.reconstructed_vehicle is not None
        self.assertEqual(len(report.reconstructed_vehicle.body_polygon), 4)

    def test_invalid_body_outline_blocks_dxf_activation(self) -> None:
        dxf_text = "\n".join(
            [
                "0", "SECTION", "2", "ENTITIES",
                "0", "LWPOLYLINE", "8", "BODY", "70", "1", "90", "5",
                "10", "0", "20", "0",
                "10", "100", "20", "100",
                "10", "0", "20", "100",
                "10", "100", "20", "0",
                "10", "50", "20", "-100",
                "0", "ENDSEC", "0", "EOF",
            ]
        )

        report = analyze_dxf_import(
            dxf_text,
            source_name="invalid-outline.dxf",
            source_units="mm",
            coordinate_system="x_forward_y_left",
            confirm_metadata=True,
        )

        self.assertFalse(report.import_ready)
        self.assertIsNone(report.reconstructed_vehicle)
        self.assertTrue(any("self-intersects" in warning for warning in report.warnings))

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
