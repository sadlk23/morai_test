"""Tests for MORAI live topic subscription registry construction."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.integrations.morai.topic_registry import build_subscription_specs
from alpamayo1_5.competition.runtime.config_competition import load_competition_config


class TopicRegistryTest(unittest.TestCase):
    def test_kcity_2026_builds_primary_and_fallback_specs(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        config.vehicle_status.enabled = True
        config.competition_status.enabled = True
        config.competition_status.topic = "/competition/status"
        config.competition_status.message_type = "std_msgs/String"
        config.collision_data.enabled = True
        config.collision_data.topic = "/collision/data"
        config.collision_data.message_type = "std_msgs/Bool"
        specs = build_subscription_specs(config)
        topics = {spec.topic for spec in specs}
        sensor_kinds = {spec.sensor_kind for spec in specs}

        self.assertIn("/camera/front/image_raw", topics)
        self.assertIn("camera_image", topics)
        self.assertIn("/fix", topics)
        self.assertIn("/gps", topics)
        self.assertIn("/Local/heading", topics)
        self.assertIn("/Local/utm", topics)
        self.assertIn("/ERP/serial_data", topics)
        self.assertIn("/competition/status", topics)
        self.assertIn("/collision/data", topics)
        self.assertIn("camera", sensor_kinds)
        self.assertIn("gps", sensor_kinds)
        self.assertIn("imu", sensor_kinds)
        self.assertIn("optional_heading", sensor_kinds)
        self.assertIn("optional_utm", sensor_kinds)
        self.assertIn("vehicle_status", sensor_kinds)
        self.assertIn("competition_status", sensor_kinds)
        self.assertIn("collision_data", sensor_kinds)
        competition_status_specs = [spec for spec in specs if spec.sensor_kind == "competition_status"]
        collision_data_specs = [spec for spec in specs if spec.sensor_kind == "collision_data"]
        self.assertTrue(all(not spec.required for spec in competition_status_specs))
        self.assertTrue(all(not spec.required for spec in collision_data_specs))
        heading_specs = [
            spec for spec in specs if spec.sensor_kind == "optional_heading" and spec.topic == "/Local/heading"
        ]
        heading_types = {spec.message_type for spec in heading_specs}
        self.assertIn("std_msgs/Float64", heading_types)
        self.assertIn("std_msgs/Float32", heading_types)


if __name__ == "__main__":
    unittest.main()
