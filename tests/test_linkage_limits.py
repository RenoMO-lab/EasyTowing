from __future__ import annotations

import unittest
from dataclasses import replace
import math

from easytowing.errors import SteeringLimitExceededError
from easytowing.linkage import (
    build_reference_linkage_demo,
    driver_point_arc,
    solve_planar_linkage,
)


class LinkageLimitTests(unittest.TestCase):
    def test_explicit_steering_stop_fails_the_linkage_state(self) -> None:
        rig = build_reference_linkage_demo()
        limited_spec = replace(rig.spec, steering_stop_deg=1.0)
        driver_point = driver_point_arc(
            rig.driver_arc_center,
            rig.driver_arc_radius_mm,
            math.radians(45.0),
        )

        with self.assertRaises(SteeringLimitExceededError):
            solve_planar_linkage(
                limited_spec,
                driver_point,
                branch_hint=rig.branch_hint,
            )


if __name__ == "__main__":
    unittest.main()
