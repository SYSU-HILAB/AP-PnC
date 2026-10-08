"""Exercise the compiled planner binding, not a mock, on its export grid."""

import argparse
import importlib
import sys
import unittest
from pathlib import Path

import numpy as np

from tooling.env import artifact_path, get_git_root


class PlannerBindingsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.yaml = str(get_git_root() / "core/bringup/config/planning.yaml")
        cls.default = cls.binding.plan(cls.yaml)
        cls.explicit = cls.binding.plan(cls.yaml, sample_dt=0.073)
        output = artifact_path(".artifacts/tests/core-adapters/planner_samples.npz")
        output.parent.mkdir(parents=True, exist_ok=True)
        np.savez(output, **cls.default)

    def test_default_grid_and_exact_endpoint(self):
        self.check_export(self.default, 0.02)

    def test_explicit_grid_and_exact_endpoint(self):
        # Separate planning calls need not select the same optimizer solution.
        # Validate each exported trajectory against its own boundary conditions.
        self.check_export(self.explicit, 0.073)

    def check_export(self, data, dt):
        times = data["t"]
        self.assertGreater(len(times), 2)
        self.assertEqual(times[0], 0.0)
        self.assertEqual(times[-1], data["duration"])
        self.assertTrue(np.all(np.diff(times) > 0.0))
        self.assertTrue(np.all(np.diff(times) <= dt + 1e-12))
        np.testing.assert_allclose(times[:-1], np.arange(len(times) - 1) * dt, atol=1e-12)
        for key in ("p", "v", "a", "yb", "omega"):
            self.assertEqual(data[key].shape, (len(times), 3))
            self.assertTrue(np.isfinite(data[key]).all(), key)
        self.assertEqual(data["thrust"].shape, (len(times),))
        self.assertTrue(np.isfinite(data["thrust"]).all())
        np.testing.assert_allclose(np.linalg.norm(data["yb"], axis=1), 1.0, atol=1e-12)
        # The canonical circle returns to its start with zero terminal v/a.
        np.testing.assert_allclose(data["p"][0], data["p"][-1], atol=1e-8)
        np.testing.assert_allclose(data["v"][[0, -1]], 0.0, atol=1e-8)
        np.testing.assert_allclose(data["a"][[0, -1]], 0.0, atol=1e-8)

    def test_invalid_intervals_are_rejected_before_loading_yaml(self):
        for value in (0.0, -0.1, float("nan"), float("inf")):
            with self.subTest(sample_dt=value), self.assertRaises(ValueError):
                self.binding.plan("/nonexistent/planning.yaml", sample_dt=value)

    def test_relative_yaml_path_is_rejected(self):
        with self.assertRaises(ValueError):
            self.binding.plan("planning.yaml")

    def test_interval_longer_than_trajectory_exports_both_endpoints(self):
        data = self.binding.plan(self.yaml, sample_dt=1e6)
        np.testing.assert_array_equal(data["t"], [0.0, data["duration"]])
        self.assertEqual(data["p"].shape, (2, 3))

    def test_unrepresentable_grid_is_rejected(self):
        with self.assertRaises(ValueError):
            self.binding.plan(self.yaml, sample_dt=1e-300)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bindings-dir", type=Path, required=True)
    args = parser.parse_args()
    if not args.bindings_dir.is_absolute():
        parser.error("--bindings-dir must be absolute")
    sys.path.insert(0, str(args.bindings_dir))
    PlannerBindingsTests.binding = importlib.import_module("planner_bindings")
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(PlannerBindingsTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
