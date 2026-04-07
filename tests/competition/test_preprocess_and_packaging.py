"""Tests for preprocessing and model-input packaging."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.io.sync import SensorSynchronizer
from alpamayo1_5.competition.preprocess.image_preprocess import ImagePreprocessor
from alpamayo1_5.competition.preprocess.model_input import ModelInputPackager
from alpamayo1_5.competition.preprocess.sensor_fusion import SensorFusion
from alpamayo1_5.competition.preprocess.state_preprocess import StatePreprocessor
from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_packet


class PreprocessAndPackagingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_competition_config("configs/competition_camera_gps_imu.json")
        self.sync = SensorSynchronizer(self.config)
        self.image = ImagePreprocessor(self.config)
        self.state = StatePreprocessor()
        self.fusion = SensorFusion()
        self.packager = ModelInputPackager(self.config)

    def test_preprocess_emits_camera_summary(self) -> None:
        packet = make_mock_packet(self.config, frame_id=3, timestamp_s=0.3)
        synced = self.sync.synchronize(packet)
        image_features = self.image.preprocess(synced)
        self.assertEqual(image_features["camera_order"][0], "front_left")
        self.assertTrue(image_features["camera_mask"]["front"])
        self.assertEqual(image_features["image_summary"]["front"]["shape"], (4, 4, 3))

    def test_model_input_packaging_is_stable(self) -> None:
        packet = make_mock_packet(self.config, frame_id=4, timestamp_s=0.4, route_command="turn right in 10m")
        synced = self.sync.synchronize(packet)
        image_features = self.image.preprocess(synced)
        ego_state = self.state.estimate(packet.timestamp_s, packet.gps_fix, packet.imu_sample)
        planner_input = self.fusion.fuse(synced, ego_state, image_features)
        model_input = self.packager.build(planner_input)
        self.assertTrue(model_input.valid)
        self.assertEqual(len(model_input.image_payloads), len(self.config.cameras))
        self.assertEqual(model_input.target_resolution, (1280, 720))
        self.assertEqual(model_input.nav_text, "turn right in 10m")


if __name__ == "__main__":
    unittest.main()
