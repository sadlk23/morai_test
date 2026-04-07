"""Tests for planner output sanity and legacy-wrapper failure handling."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.planners.legacy_backend import LegacyAlpamayoPlannerBackend
from alpamayo1_5.competition.planners.lightweight_backend import LightweightWaypointPlannerBackend
from alpamayo1_5.competition.io.sync import SensorSynchronizer
from alpamayo1_5.competition.preprocess.image_preprocess import ImagePreprocessor
from alpamayo1_5.competition.preprocess.model_input import ModelInputPackager
from alpamayo1_5.competition.preprocess.sensor_fusion import SensorFusion
from alpamayo1_5.competition.preprocess.state_preprocess import StatePreprocessor
from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_packet


class PlannerAndWrapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_competition_config("configs/competition_camera_gps_imu.json")
        self.sync = SensorSynchronizer(self.config)
        self.image = ImagePreprocessor(self.config)
        self.state = StatePreprocessor()
        self.fusion = SensorFusion()
        self.packager = ModelInputPackager(self.config)

    def _planner_input(self):
        packet = make_mock_packet(self.config, frame_id=5, timestamp_s=0.5, route_command="turn left in 20m")
        synced = self.sync.synchronize(packet)
        image_features = self.image.preprocess(synced)
        ego_state = self.state.estimate(packet.timestamp_s, packet.gps_fix, packet.imu_sample)
        planner_input = self.fusion.fuse(synced, ego_state, image_features)
        planner_input.model_input_package = self.packager.build(planner_input)
        return planner_input

    def test_lightweight_planner_output_shape_and_value(self) -> None:
        planner_input = self._planner_input()
        backend = LightweightWaypointPlannerBackend(self.config.planner)
        plan = backend.plan(planner_input)
        self.assertTrue(plan.valid)
        self.assertEqual(len(plan.waypoints_xy), self.config.planner.num_waypoints)
        self.assertGreaterEqual(plan.target_speed_mps, self.config.planner.min_target_speed_mps)
        self.assertLessEqual(plan.target_speed_mps, self.config.planner.max_target_speed_mps)

    def test_legacy_wrapper_fails_closed_without_dependencies(self) -> None:
        planner_input = self._planner_input()
        legacy = LegacyAlpamayoPlannerBackend(self.config.planner, self.config.cameras)
        plan = legacy.plan(planner_input)
        self.assertFalse(plan.valid)
        self.assertIn("error", plan.diagnostics)
        self.assertIn("backend_status", plan.diagnostics)

    def test_legacy_wrapper_rejects_raw_image_payloads(self) -> None:
        planner_input = self._planner_input()
        planner_input.model_input_package.image_payloads = [
            b"raw-bytes" for _ in planner_input.model_input_package.camera_indices
        ]
        legacy = LegacyAlpamayoPlannerBackend(self.config.planner, self.config.cameras)
        legacy.wrapper._loaded = True
        legacy.wrapper._load_error = None
        legacy.wrapper._model = object()
        legacy.wrapper._processor = object()
        legacy.wrapper._torch = object()
        plan = legacy.plan(planner_input)
        self.assertFalse(plan.valid)
        self.assertEqual(plan.diagnostics["backend_status"], "invalid_live_model_input")
        self.assertIn("undecoded raw image payloads", plan.diagnostics["error"])


if __name__ == "__main__":
    unittest.main()
