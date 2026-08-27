from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from easytowing.collision import (  # type: ignore  # noqa: E402
    CapsuleEnvelope,
    CircleEnvelope,
    CollisionItem,
    analyze_clearance,
    axis_aligned_rectangle,
    clearance_between_items,
)
from easytowing.geometry import Point2D  # type: ignore  # noqa: E402


class ClearanceEngineTests(unittest.TestCase):
    def test_circle_circle_positive_clearance_and_margin_violation(self) -> None:
        item_a = CollisionItem("a", CircleEnvelope(Point2D(0.0, 0.0), 10.0), margin_mm=8.0)
        item_b = CollisionItem("b", CircleEnvelope(Point2D(40.0, 0.0), 10.0), margin_mm=15.0)

        pair = clearance_between_items(item_a, item_b)

        self.assertFalse(pair.overlaps)
        self.assertAlmostEqual(pair.raw_clearance_mm, 20.0, places=9)
        self.assertAlmostEqual(pair.clearance_mm, -3.0, places=9)
        self.assertTrue(pair.violates_margin)

    def test_capsule_capsule_overlap_is_negative(self) -> None:
        item_a = CollisionItem("a", CapsuleEnvelope(Point2D(-10.0, 0.0), Point2D(10.0, 0.0), 5.0))
        item_b = CollisionItem("b", CapsuleEnvelope(Point2D(0.0, -10.0), Point2D(0.0, 10.0), 5.0))

        pair = clearance_between_items(item_a, item_b)

        self.assertTrue(pair.overlaps)
        self.assertLess(pair.raw_clearance_mm, 0.0)
        self.assertLess(pair.clearance_mm, 0.0)
        self.assertEqual(pair.description, "capsule-capsule")

    def test_polygon_overlap_detected_and_minimum_pair_selected(self) -> None:
        rectangle = axis_aligned_rectangle(Point2D(0.0, 0.0), 100.0, 100.0)
        circle = CircleEnvelope(Point2D(0.0, 0.0), 15.0)
        far_circle = CircleEnvelope(Point2D(200.0, 0.0), 10.0)

        report = analyze_clearance(
            (
                CollisionItem("rectangle", rectangle),
                CollisionItem("circle", circle),
                CollisionItem("far_circle", far_circle),
            )
        )

        self.assertTrue(report.collision_detected)
        self.assertIsNotNone(report.minimum_pair)
        self.assertEqual(report.minimum_pair.item_a_id, "rectangle")
        self.assertEqual(report.minimum_pair.item_b_id, "circle")
        self.assertLess(report.minimum_clearance_mm or 0.0, 0.0)

    def test_excluded_pairs_are_not_reported(self) -> None:
        beam = CollisionItem(
            "axle_beam",
            CapsuleEnvelope(Point2D(0.0, -100.0), Point2D(0.0, 100.0), 10.0),
        )
        tire = CollisionItem(
            "wheel_tire",
            CircleEnvelope(Point2D(0.0, 0.0), 40.0),
            excluded_pair_ids=("axle_beam",),
        )

        report = analyze_clearance((beam, tire))

        self.assertEqual(report.pairs, ())
        self.assertIsNone(report.minimum_pair)


if __name__ == "__main__":
    unittest.main()
