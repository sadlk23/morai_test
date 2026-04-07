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

    def test_kcity_2026_config_loads(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        self.assertEqual(config.controller.pure_pursuit.wheelbase_m, 3.0)
        self.assertEqual(config.gps.topic, "/fix")
        self.assertIn("camera_image", config.cameras[0].fallback_topics)
        self.assertTrue(config.legacy_serial_bridge.enabled)
        self.assertFalse(config.legacy_serial_bridge.publish_enabled)
        self.assertEqual(config.legacy_serial_bridge.brake_mode, "normalized")
        self.assertEqual(config.optional_ego_topics.heading_message_type, "std_msgs/Float64")
        self.assertIn("std_msgs/Float32", config.optional_ego_topics.heading_fallback_message_types)
        self.assertIn("geometry_msgs/PointStamped", config.optional_ego_topics.utm_fallback_message_types)

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

    def test_debug_only_disables_legacy_serial_bridge_publish(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        config.legacy_serial_bridge.publish_enabled = True
        updated = apply_runtime_mode_overrides(config, debug_only=True)
        self.assertFalse(updated.legacy_serial_bridge.publish_enabled)

    def test_debug_only_conflicts_with_legacy_serial_enable_flag(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        with self.assertRaises(ValueError):
            apply_runtime_mode_overrides(
                config,
                debug_only=True,
                enable_legacy_serial_bridge=True,
            )

    def test_direct_actuation_override_still_works(self) -> None:
        config = load_competition_config("configs/competition_morai_live.json")
        updated = apply_runtime_mode_overrides(
            config,
            debug_only=False,
            enable_actuation=True,
            arm_actuation=True,
        )
        self.assertTrue(updated.ros_output.publish_actuation)
        self.assertTrue(updated.ros_output.actuation_armed)

    def test_enable_legacy_serial_bridge_override(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        config.legacy_serial_bridge.publish_enabled = False
        updated = apply_runtime_mode_overrides(config, enable_legacy_serial_bridge=True)
        self.assertTrue(updated.legacy_serial_bridge.enabled)
        self.assertTrue(updated.legacy_serial_bridge.publish_enabled)

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

    def test_invalid_legacy_brake_mode_fails(self) -> None:
        payload = CompetitionConfig().to_dict()
        payload["cameras"] = [
            {"name": "front", "topic": "/a", "message_type": "sensor_msgs/Image"},
        ]
        payload["legacy_serial_bridge"]["enabled"] = True
        payload["legacy_serial_bridge"]["brake_mode"] = "bad_mode"
        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as handle:
            json.dump(payload, handle)
            handle.flush()
            with self.assertRaises(ValueError):
                load_competition_config(handle.name)


if __name__ == "__main__":
    unittest.main()
