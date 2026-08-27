from __future__ import annotations

import sitecustomize
import unittest


class BootstrapTests(unittest.TestCase):
    def test_bootstrap_imports_sitecustomize(self) -> None:
        self.assertIsNotNone(sitecustomize)
