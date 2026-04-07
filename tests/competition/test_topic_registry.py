"""Tests for MORAI live topic subscription registry construction."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.integrations.morai.topic_registry import build_subscription_specs
from alpamayo1_5.competition.runtime.config_competition import load_competition_config


class TopicRegistryTest(unittest.TestCase):
    def test_kcity_2026_builds_primary_and_fallback_specs(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        specs = build_subscription_specs(config)
        topics = {spec.topic for spec in specs}
        sensor_kinds = {spec.sensor_kind for spec in specs}

        self.assertIn("/camera/front/image_raw", topics)
        self.assertIn("camera_image", topics)
        self.assertIn("/fix", topics)
        self.assertIn("/gps", topics)
        self.assertIn("/Local/heading", topics)
        self.assertIn("/Local/utm", topics)
        self.assertIn("camera", sensor_kinds)
        self.assertIn("gps", sensor_kinds)
        self.assertIn("imu", sensor_kinds)
        self.assertIn("optional_heading", sensor_kinds)
        self.assertIn("optional_utm", sensor_kinds)


if __name__ == "__main__":
    unittest.main()
