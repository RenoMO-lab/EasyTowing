from __future__ import annotations

import unittest

from easytowing.demo_server import _optimization_payload


class OptimizationSelectionTests(unittest.TestCase):
    def test_api_selection_controls_enabled_variables(self) -> None:
        payload = _optimization_payload("quick", {"steering_arm_length_mm"})
        variables = {variable["id"]: variable for variable in payload["variables_before"]}

        self.assertTrue(variables["steering_arm_length_mm"]["enabled"])
        self.assertFalse(variables["tie_rod_length_mm"]["enabled"])
        self.assertFalse(variables["bell_crank_pivot_x_mm"]["enabled"])


if __name__ == "__main__":
    unittest.main()
