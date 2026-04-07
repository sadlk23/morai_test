"""Tests for the ROS1 wrapper package helper logic."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import tempfile
import unittest


def _load_wrapper_module():
    repo_root = Path(__file__).resolve().parents[2]
    module_path = repo_root / "ros1" / "alpamayo1_5_ros" / "scripts" / "run_competition_live_node.py"
    spec = importlib.util.spec_from_file_location("alpamayo_ros_wrapper", module_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RosWrapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.module = _load_wrapper_module()

    def test_resolve_repo_root_requires_src_package(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            with self.assertRaises(SystemExit):
                self.module.resolve_repo_root(temp_dir)

    def test_resolve_runtime_python_rejects_unknown_interpreter(self) -> None:
        with self.assertRaises(SystemExit):
            self.module.resolve_runtime_python("definitely-not-a-real-python-binary")

    def test_resolve_repo_root_can_discover_from_current_working_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            repo_root = Path(temp_dir)
            (repo_root / "src" / "alpamayo1_5").mkdir(parents=True)
            old_cwd = os.getcwd()
            try:
                os.chdir(str(repo_root))
                resolved = self.module.resolve_repo_root(None)
            finally:
                os.chdir(old_cwd)
            self.assertEqual(resolved, repo_root.resolve())

    def test_build_runtime_argv_includes_debug_and_arming_flags(self) -> None:
        argv = self.module.build_runtime_argv(
            runtime_python="python3.11",
            config_path=Path("configs/competition_morai_live.json"),
            passthrough=["--max-cycles", "2"],
            debug_only=False,
            enable_actuation=True,
            arm_actuation=True,
        )
        self.assertIn("--enable-actuation", argv)
        self.assertIn("--arm-actuation", argv)
        self.assertIn("--max-cycles", argv)

    def test_debug_only_env_flag_is_truthy(self) -> None:
        os.environ[self.module.ENV_DEBUG_ONLY] = "true"
        try:
            self.assertTrue(self.module._truthy_env(self.module.ENV_DEBUG_ONLY))
        finally:
            del os.environ[self.module.ENV_DEBUG_ONLY]


if __name__ == "__main__":
    unittest.main()
