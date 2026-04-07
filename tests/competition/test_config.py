"""Tests for competition config validation."""

from __future__ import annotations

import json
import tempfile
import unittest

from alpamayo1_5.competition.scripts.run_competition import apply_runtime_mode_overrides
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig, load_competition_config


class CompetitionConfigTest(unittest.TestCase):
    def test_default_json_config_loads(self) -> None:
        config = load_competition_config("configs/competition_camera_gps_imu.json")
        self.assertGreaterEqual(len(config.cameras), 1)
        self.assertEqual(config.controller.lateral_controller, "pure_pursuit")

    def test_invalid_duplicate_camera_names_fail(self) -> None:
        payload = CompetitionConfig().to_dict()
        payload["cameras"] = [
            {"name": "front", "topic": "/a"},
            {"name": "front", "topic": "/b"},
        ]
        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as handle:
            json.dump(payload, handle)
            handle.flush()
            with self.assertRaises(ValueError):
                load_competition_config(handle.name)

    def test_invalid_camera_message_type_fails(self) -> None:
        payload = CompetitionConfig().to_dict()
        payload["cameras"] = [
            {"name": "front", "topic": "/a", "message_type": "sensor_msgs/Foo"},
        ]
        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as handle:
            json.dump(payload, handle)
            handle.flush()
            with self.assertRaises(ValueError):
                load_competition_config(handle.name)

    def test_ros_output_requires_one_publish_mode(self) -> None:
        payload = CompetitionConfig().to_dict()
        payload["cameras"] = [
            {"name": "front", "topic": "/a", "message_type": "sensor_msgs/Image"},
        ]
        payload["ros_output"] = {
            "enabled": True,
            "publish_command_json": False,
            "publish_debug_json": False,
            "publish_actuation": False,
        }
        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as handle:
            json.dump(payload, handle)
            handle.flush()
            with self.assertRaises(ValueError):
                load_competition_config(handle.name)

    def test_publish_actuation_requires_arm_when_enabled(self) -> None:
        payload = CompetitionConfig().to_dict()
        payload["cameras"] = [
            {"name": "front", "topic": "/a", "message_type": "sensor_msgs/Image"},
        ]
        payload["live_input"]["enabled"] = True
        payload["ros_output"]["publish_actuation"] = True
        payload["ros_output"]["actuation_armed"] = False
        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as handle:
            json.dump(payload, handle)
            handle.flush()
            with self.assertRaises(ValueError):
                load_competition_config(handle.name)

    def test_debug_only_conflicts_with_actuation_flags(self) -> None:
        config = load_competition_config("configs/competition_morai_live.json")
        with self.assertRaises(ValueError):
            apply_runtime_mode_overrides(
                config,
                debug_only=True,
                enable_actuation=True,
                arm_actuation=False,
            )

    def test_live_safe_stop_publish_interval_must_be_positive(self) -> None:
        payload = CompetitionConfig().to_dict()
        payload["cameras"] = [
            {"name": "front", "topic": "/a", "message_type": "sensor_msgs/Image"},
        ]
        payload["live_input"]["safe_stop_publish_interval_s"] = 0.0
        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as handle:
            json.dump(payload, handle)
            handle.flush()
            with self.assertRaises(ValueError):
                load_competition_config(handle.name)


if __name__ == "__main__":
    unittest.main()
