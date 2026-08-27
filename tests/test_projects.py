from __future__ import annotations

import sys
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.projects import ProjectStore  # type: ignore  # noqa: E402
from easytowing.demo_server import _parse_linkage_rig, _parse_vehicle_config  # type: ignore  # noqa: E402
from easytowing.steering import build_demo_solution  # type: ignore  # noqa: E402


class ProjectStoreTests(unittest.TestCase):
    def test_create_append_and_restore_project_revision(self) -> None:
        with TemporaryDirectory() as tmpdir:
            store_path = Path(tmpdir) / "projects.json"
            store = ProjectStore(store_path)

            linkage_config = {
                "id": "stored_linkage",
                "steering_arm_length_mm": 155.0,
                "companion_steering_pivot": None,
            }
            vehicle_config = {
                "id": "stored_three_axle",
                "name": "Stored three axle",
                "body_length_mm": 6800.0,
                "body_width_mm": 3400.0,
                "axles": [
                    {"id": "axle_1", "x_mm": -2500.0, "track_mm": 2600.0, "steering_mode": "FIXED", "steerable": False},
                    {"id": "axle_2", "x_mm": 0.0, "track_mm": 2700.0, "steering_mode": "FORCED_STEER"},
                    {"id": "axle_3", "x_mm": 2500.0, "track_mm": 2800.0, "steering_mode": "FORCED_STEER"},
                ],
            }
            vehicle, _normalized = _parse_vehicle_config(vehicle_config)
            assert vehicle is not None
            project = store.create_project(
                "Test Project",
                beta_deg=5.0,
                optimization_mode="quick",
                note="Initial",
                linkage_config=linkage_config,
                vehicle_config=vehicle_config,
                vehicle=vehicle,
            )
            self.assertEqual(project.name, "Test Project")
            self.assertEqual(len(project.revisions), 1)

            revision_one = project.revisions[0]
            self.assertAlmostEqual(revision_one.beta_deg, 5.0)
            self.assertEqual(revision_one.optimization_mode, "quick")
            self.assertIn("vehicle", revision_one.snapshot)

            revision_two = store.append_revision(
                project.id,
                beta_deg=10.0,
                optimization_mode="full",
                note="Second revision",
            )
            self.assertEqual(revision_two.note, "Second revision")
            self.assertEqual(store.get_project(project.id).active_revision_id, revision_two.id)

            restored = store.restore_revision(project.id, revision_one.id)
            self.assertEqual(restored.id, revision_one.id)
            self.assertEqual(store.get_project(project.id).active_revision_id, revision_one.id)

            reloaded = ProjectStore(store_path)
            reloaded_project = reloaded.get_project(project.id)
            self.assertIsNotNone(reloaded_project)
            self.assertEqual(len(reloaded_project.revisions), 2)
            self.assertEqual(reloaded_project.active_revision_id, revision_one.id)
            self.assertEqual(reloaded_project.revisions[0].linkage_config, linkage_config)
            self.assertEqual(reloaded_project.revisions[0].vehicle_config, vehicle_config)

    def test_custom_geometry_and_linkage_are_snapshot_inputs(self) -> None:
        with TemporaryDirectory() as tmpdir:
            store = ProjectStore(Path(tmpdir) / "projects.json")
            linkage_config = {
                "id": "snapshot_linkage",
                "steering_arm_length_mm": 155.0,
                "companion_steering_pivot": None,
            }
            project = store.create_project(
                "Custom Snapshot",
                linkage_config=linkage_config,
                wheelbase_mm=5000.0,
                track_mm=2800.0,
                linkage_rig=_parse_linkage_rig(linkage_config),
                vehicle=build_demo_solution(0.0, 5000.0, 2800.0)[0],
            )

            revision = project.revisions[0]
            self.assertEqual(revision.wheelbase_mm, 5000.0)
            self.assertEqual(revision.track_mm, 2800.0)
            self.assertEqual(revision.snapshot["baseline"]["spec"]["id"], "snapshot_linkage")
            self.assertEqual(revision.snapshot["baseline"]["spec"]["steering_arm_length_mm"], 155.0)
            self.assertEqual(revision.snapshot["vehicle"]["axles"][0]["track_mm"], 2800.0)

    def test_ensure_seed_project_creates_default_project(self) -> None:
        with TemporaryDirectory() as tmpdir:
            store = ProjectStore(Path(tmpdir) / "projects.json")
            project = store.ensure_seed_project()
            self.assertEqual(project.name, "Reference Demo Project")
            self.assertEqual(len(store.list_projects()), 1)


if __name__ == "__main__":
    unittest.main()
