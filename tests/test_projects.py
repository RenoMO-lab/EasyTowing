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
            self.assertIsNotNone(project.revisions[0].combination_config)
            self.assertEqual(project.revisions[0].root_turn_radius_mm, 9000.0)

    def test_local_project_store_scopes_projects_by_organization(self) -> None:
        with TemporaryDirectory() as tmpdir:
            store = ProjectStore(Path(tmpdir) / "projects.json")
            first = store.create_project("First", organization_id="org-first")
            second = store.create_project("Second", organization_id="org-second")

            self.assertEqual([project.id for project in store.list_projects("org-first")], [first.id])
            self.assertEqual([project.id for project in store.list_projects("org-second")], [second.id])
            self.assertIsNone(store.get_project(second.id, "org-first"))
            self.assertIsNone(store.get_active_revision(second.id, "org-first"))
            with self.assertRaises(KeyError):
                store.append_revision(
                    second.id,
                    organization_id="org-first",
                    beta_deg=0.0,
                    optimization_mode="quick",
                    note="Should be rejected",
                )

    def test_multi_body_mechanism_inputs_and_snapshot_survive_reload(self) -> None:
        with TemporaryDirectory() as tmpdir:
            store_path = Path(tmpdir) / "projects.json"
            store = ProjectStore(store_path)
            combination = {
                "id": "road_train",
                "root_body_id": "tractor",
                "bodies": [{"id": "tractor"}, {"id": "trailer"}],
            }
            graph = {
                "id": "steering_graph",
                "points": [{"id": "driver", "mode": "driven"}],
                "members": [],
            }
            drivers = [{"point_id": "driver", "input_id": "hitch"}]
            assignments = [{"output_id": "left_output", "wheel_id": "rear_left"}]
            snapshot = {"mechanism_graph": {"state": {"maximum_residual_mm": 0.0}}}

            project = store.create_project(
                "Road train",
                combination_config=combination,
                root_turn_radius_mm=9000.0,
                mechanism_graph_config=graph,
                mechanism_drivers=drivers,
                steering_assignments=assignments,
                engineering_snapshot=snapshot,
            )

            reloaded = ProjectStore(store_path).get_project(project.id)
            assert reloaded is not None
            revision = reloaded.revisions[0]
            self.assertEqual(revision.combination_config, combination)
            self.assertEqual(revision.root_turn_radius_mm, 9000.0)
            self.assertEqual(revision.mechanism_graph_config, graph)
            self.assertEqual(list(revision.mechanism_drivers), drivers)
            self.assertEqual(list(revision.steering_assignments), assignments)
            self.assertEqual(revision.snapshot, snapshot)

    def test_acceptance_evidence_is_persisted_on_the_revision(self) -> None:
        with TemporaryDirectory() as tmpdir:
            store_path = Path(tmpdir) / "projects.json"
            store = ProjectStore(store_path)
            project = store.create_project("Acceptance evidence")
            revision = project.revisions[0]
            acceptance = {
                "criteria": {"case_id": "MONROC-01", "minimum_clearance_mm": 20.0},
                "result": {"status": "PASS", "checks": []},
                "evaluated_by": "designer-1",
            }

            stored = store.record_acceptance(project.id, revision.id, acceptance)

            self.assertEqual(stored.snapshot["monroc_acceptance"], acceptance)
            reloaded = ProjectStore(store_path).get_project(project.id)
            assert reloaded is not None
            self.assertEqual(
                reloaded.revisions[0].snapshot["monroc_acceptance"],
                acceptance,
            )


if __name__ == "__main__":
    unittest.main()
