from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from easytowing.projects import ProjectStore  # type: ignore  # noqa: E402


class OptimizationAcceptanceTests(unittest.TestCase):
    def test_apply_optimized_design_creates_traceable_revision(self) -> None:
        with TemporaryDirectory() as tmpdir:
            store = ProjectStore(Path(tmpdir) / "projects.json")
            project = store.create_project("Acceptance Project")

            revision = store.append_revision(
                project.id,
                beta_deg=15.0,
                optimization_mode="quick",
                enabled_ids={"steering_arm_length_mm"},
                accepted_optimization=True,
                note="Applied optimized design",
                beta_min_deg=-30.0,
                beta_max_deg=40.0,
            )

            self.assertTrue(revision.accepted_optimization)
            self.assertEqual(revision.optimization_enabled_ids, ("steering_arm_length_mm",))
            self.assertEqual(revision.beta_min_deg, -30.0)
            self.assertEqual(revision.beta_max_deg, 40.0)
            self.assertEqual(revision.snapshot["optimization"]["mode"], "quick")
            enabled = {
                item["id"]: item["enabled"]
                for item in revision.snapshot["optimization"]["variables_before"]
            }
            self.assertTrue(enabled["steering_arm_length_mm"])
            self.assertFalse(enabled["tie_rod_length_mm"])

            reloaded = ProjectStore(Path(tmpdir) / "projects.json")
            restored = reloaded.get_project(project.id)
            self.assertIsNotNone(restored)
            self.assertTrue(restored.revisions[-1].accepted_optimization)
            self.assertEqual(restored.revisions[-1].beta_min_deg, -30.0)


if __name__ == "__main__":
    unittest.main()
