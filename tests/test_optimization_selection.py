from __future__ import annotations

import unittest

from easytowing.demo_server import _optimization_payload


class OptimizationSelectionTests(unittest.TestCase):
    def test_api_selection_controls_enabled_variables(self) -> None:
        enabled_ids = {
            "bell_crank_input_arm_length_mm",
            "bell_crank_output_arm_length_mm",
            "steering_arm_length_mm",
            "input_rod_length_mm",
            "tie_rod_length_mm",
        }
        payload = _optimization_payload("quick", enabled_ids)
        variables = {variable["id"]: variable for variable in payload["variables_before"]}

        self.assertTrue(variables["steering_arm_length_mm"]["enabled"])
        self.assertTrue(variables["tie_rod_length_mm"]["enabled"])
        self.assertFalse(variables["bell_crank_pivot_x_mm"]["enabled"])


if __name__ == "__main__":
    unittest.main()
