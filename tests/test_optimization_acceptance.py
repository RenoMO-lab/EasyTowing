from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from easytowing.errors import OptimizationNoFeasibleSolutionError  # type: ignore  # noqa: E402
from easytowing.linkage import build_reference_linkage_demo  # type: ignore  # noqa: E402
from easytowing.optimization import (  # type: ignore  # noqa: E402
    build_branch_hint,
    build_optimized_spec,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)
from easytowing.projects import ProjectStore  # type: ignore  # noqa: E402


class OptimizationAcceptanceTests(unittest.TestCase):
    def test_apply_optimized_design_creates_traceable_revision(self) -> None:
        with TemporaryDirectory() as tmpdir:
            store = ProjectStore(Path(tmpdir) / "projects.json")
            project = store.create_project("Acceptance Project")

            with self.assertRaises(OptimizationNoFeasibleSolutionError):
                store.append_revision(
                    project.id,
                    beta_deg=15.0,
                    optimization_mode="quick",
                    enabled_ids={"steering_arm_length_mm"},
                    accepted_optimization=True,
                    note="Unsafe baseline",
                )

            problem = build_reference_optimization_problem(mode="quick")
            result = optimize_linkage_problem(problem)
            optimized_spec = build_optimized_spec(problem.baseline_spec, result.optimized_variables)
            reference_rig = build_reference_linkage_demo()
            optimized_rig = replace(
                reference_rig,
                spec=optimized_spec,
                branch_hint=build_branch_hint(optimized_spec),
            )

            revision = store.append_revision(
                project.id,
                beta_deg=15.0,
                optimization_mode="quick",
                enabled_ids=set(),
                accepted_optimization=True,
                note="Applied optimized design",
                beta_min_deg=-30.0,
                beta_max_deg=40.0,
                linkage_rig=optimized_rig,
            )

            self.assertTrue(revision.accepted_optimization)
            self.assertEqual(revision.optimization_enabled_ids, ())
            self.assertEqual(revision.beta_min_deg, -30.0)
            self.assertEqual(revision.beta_max_deg, 40.0)
            self.assertEqual(revision.snapshot["optimization"]["mode"], "quick")
            enabled = {
                item["id"]: item["enabled"]
                for item in revision.snapshot["optimization"]["variables_before"]
            }
            self.assertFalse(enabled["steering_arm_length_mm"])
            self.assertFalse(enabled["tie_rod_length_mm"])
            self.assertTrue(revision.snapshot["optimization"]["baseline"]["feasible"])

            reloaded = ProjectStore(Path(tmpdir) / "projects.json")
            restored = reloaded.get_project(project.id)
            self.assertIsNotNone(restored)
            self.assertTrue(restored.revisions[-1].accepted_optimization)
            self.assertEqual(restored.revisions[-1].beta_min_deg, -30.0)


if __name__ == "__main__":
    unittest.main()
