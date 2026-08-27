from __future__ import annotations

import unittest

from easytowing.reporting import build_export_png


class PngExportTests(unittest.TestCase):
    def test_export_png_is_a_valid_snapshot(self) -> None:
        png_bytes = build_export_png(0.0, "quick")

        self.assertEqual(png_bytes[:8], b"\x89PNG\r\n\x1a\n")
        self.assertGreater(len(png_bytes), 10000)


if __name__ == "__main__":
    unittest.main()
