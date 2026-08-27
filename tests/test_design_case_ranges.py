from __future__ import annotations

import unittest

from easytowing.reporting import build_steering_sweep_bundle, build_swept_path_bundle


class DesignCaseRangeTests(unittest.TestCase):
    def test_steering_sweep_accepts_configured_articulation_bounds(self) -> None:
        bundle = build_steering_sweep_bundle(
            optimization_mode="quick",
            step_deg=10.0,
            beta_min_deg=-20.0,
            beta_max_deg=30.0,
        )

        self.assertEqual(bundle["beta_min_deg"], -20.0)
        self.assertEqual(bundle["beta_max_deg"], 30.0)
        self.assertEqual(bundle["sample_count"], 6)

    def test_swept_path_accepts_configured_articulation_bounds(self) -> None:
        bundle = build_swept_path_bundle(
            current_beta_deg=10.0,
            optimization_mode="quick",
            step_deg=10.0,
            beta_min_deg=-20.0,
            beta_max_deg=30.0,
        )

        self.assertEqual(bundle["sample_count"], 6)
        self.assertEqual(bundle["beta_min_deg"], -20.0)
        self.assertEqual(bundle["beta_max_deg"], 30.0)


if __name__ == "__main__":
    unittest.main()
