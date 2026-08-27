from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from easytowing.design_cases import DesignCase
from easytowing.optimization import build_reference_optimization_problem
from easytowing.projects import ProjectStore


class DesignCaseTests(unittest.TestCase):
    def test_targets_resolve_to_signed_articulation(self) -> None:
        beta_case = DesignCase(id="nominal", name="Nominal", beta_deg=25.0)
        radius_case = DesignCase(id="radius", name="Radius", turn_radius_mm=-7000.0)
        diameter_case = DesignCase(
            id="reverse",
            name="Reverse envelope",
            outer_diameter_mm=14000.0,
            direction="right",
        )

        self.assertEqual(beta_case.resolved_beta_deg(4360.0), 25.0)
        self.assertLess(radius_case.resolved_beta_deg(4360.0), 0.0)
        self.assertLess(diameter_case.resolved_beta_deg(4360.0), 0.0)

    def test_optimizer_adds_enabled_cases_with_weights(self) -> None:
        cases = (
            DesignCase(id="nominal", name="Nominal", beta_deg=25.0, weight=3.0),
            DesignCase(id="disabled", name="Disabled", beta_deg=12.0, enabled=False),
        )
        problem = build_reference_optimization_problem(mode="quick", design_cases=cases)

        self.assertIn(25.0, problem.beta_samples_deg)
        self.assertNotIn(12.0, problem.beta_samples_deg)
        sample_index = problem.beta_samples_deg.index(25.0)
        self.assertEqual(problem.sample_weights[sample_index], 3.0)
        self.assertEqual(problem.design_cases, cases)

    def test_project_revision_round_trips_design_cases(self) -> None:
        case = DesignCase(id="max", name="Maximum articulation", beta_deg=45.0, weight=2.0)
        with TemporaryDirectory() as tmpdir:
            store = ProjectStore(Path(tmpdir) / "projects.json")
            project = store.create_project("Cases", design_cases=(case,))
            revision = project.revisions[0]
            self.assertEqual(revision.design_cases, (case,))
            self.assertEqual(revision.snapshot["optimization"]["design_cases"], [case.to_dict()])

            reloaded = ProjectStore(Path(tmpdir) / "projects.json")
            restored = reloaded.get_project(project.id)
            self.assertIsNotNone(restored)
            self.assertEqual(restored.revisions[0].design_cases, (case,))

    def test_case_requires_exactly_one_target(self) -> None:
        with self.assertRaises(ValueError):
            DesignCase(id="invalid", name="Invalid")
        with self.assertRaises(ValueError):
            DesignCase(id="invalid", name="Invalid", beta_deg=10.0, turn_radius_mm=5000.0)


if __name__ == "__main__":
    unittest.main()
