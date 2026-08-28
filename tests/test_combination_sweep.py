from __future__ import annotations

import unittest

from easytowing.combination_sweep import (
    JointSweepRange,
    build_joint_sweep_grid,
    normalize_joint_sweep_ranges,
)
from easytowing.errors import SweepSampleLimitError


class CombinationSweepTests(unittest.TestCase):
    def test_cartesian_grid_contains_every_joint_endpoint_and_zero(self) -> None:
        ranges = (
            JointSweepRange("front_hitch", -10.0, 10.0, 10.0),
            JointSweepRange("rear_hitch", -5.0, 5.0, 5.0),
        )

        samples = build_joint_sweep_grid(ranges)

        self.assertEqual(len(samples), 9)
        self.assertEqual(samples[0], {"front_hitch": -10.0, "rear_hitch": -5.0})
        self.assertIn({"front_hitch": 0.0, "rear_hitch": 0.0}, samples)
        self.assertEqual(samples[-1], {"front_hitch": 10.0, "rear_hitch": 5.0})

    def test_cartesian_grid_is_serpentine_for_continuous_branch_solves(self) -> None:
        ranges = (
            JointSweepRange("front_hitch", -10.0, 10.0, 10.0),
            JointSweepRange("rear_hitch", -5.0, 5.0, 5.0),
        )

        samples = build_joint_sweep_grid(ranges)

        for previous, current in zip(samples, samples[1:]):
            changed = [
                joint_id
                for joint_id in previous
                if previous[joint_id] != current[joint_id]
            ]
            self.assertEqual(len(changed), 1)
            joint_id = changed[0]
            expected_step = 5.0 if joint_id == "rear_hitch" else 10.0
            self.assertEqual(abs(previous[joint_id] - current[joint_id]), expected_step)

    def test_defaults_cover_every_joint_when_ranges_are_omitted(self) -> None:
        ranges = normalize_joint_sweep_ranges(
            ("front_hitch", "rear_hitch"),
            None,
            default_min_deg=-20.0,
            default_max_deg=20.0,
            default_step_deg=10.0,
            primary_joint_id="rear_hitch",
        )

        self.assertEqual([item.joint_id for item in ranges], ["front_hitch", "rear_hitch"])
        self.assertEqual(ranges[0].minimum_deg, -20.0)
        self.assertEqual(ranges[1].maximum_deg, 20.0)

    def test_partial_ranges_fill_omitted_joints_with_defaults(self) -> None:
        ranges = normalize_joint_sweep_ranges(
            ("front_hitch", "rear_hitch"),
            {"rear_hitch": {"min_deg": -5.0, "max_deg": 5.0, "step_deg": 5.0}},
            default_min_deg=-20.0,
            default_max_deg=20.0,
            default_step_deg=10.0,
        )

        self.assertEqual(
            [item.to_dict() for item in ranges],
            [
                {"joint_id": "front_hitch", "min_deg": -20.0, "max_deg": 20.0, "step_deg": 10.0},
                {"joint_id": "rear_hitch", "min_deg": -5.0, "max_deg": 5.0, "step_deg": 5.0},
            ],
        )

    def test_oversized_grid_fails_without_truncation(self) -> None:
        with self.assertRaises(SweepSampleLimitError):
            build_joint_sweep_grid(
                (
                    JointSweepRange("front_hitch", -45.0, 45.0, 1.0),
                    JointSweepRange("rear_hitch", -45.0, 45.0, 1.0),
                ),
                maximum_samples=100,
            )


if __name__ == "__main__":
    unittest.main()
