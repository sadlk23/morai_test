"""Hardening-focused edge-case tests."""

from __future__ import annotations

import math
import tempfile
import unittest

from alpamayo1_5.competition.contracts import PlanResult
from alpamayo1_5.competition.controllers.controller_runtime import ControllerRuntime
from alpamayo1_5.competition.io.sync import SensorSynchronizer
from alpamayo1_5.competition.preprocess.image_preprocess import ImagePreprocessor
from alpamayo1_5.competition.preprocess.model_input import ModelInputPackager
from alpamayo1_5.competition.preprocess.sensor_fusion import SensorFusion
from alpamayo1_5.competition.preprocess.state_preprocess import StatePreprocessor
from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_packet
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline
from alpamayo1_5.competition.safety.safety_filter import SafetyFilter


class HardeningEdgeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_competition_config("configs/competition_camera_gps_imu.json")

    def test_future_timestamp_marks_sync_invalid(self) -> None:
        packet = make_mock_packet(self.config, frame_id=7, timestamp_s=1.0)
        packet.gps_fix.timestamp_s = 1.2
        synced = SensorSynchronizer(self.config).synchronize(packet)
        self.assertFalse(synced.valid)
        self.assertIn("gps:future_timestamp", synced.diagnostics["invalid_reasons"])

    def test_invalid_camera_shape_is_rejected_before_model(self) -> None:
        packet = make_mock_packet(self.config, frame_id=8, timestamp_s=0.8)
        packet.camera_frames["front"].shape = (4,)
        synced = SensorSynchronizer(self.config).synchronize(packet)
        image_features = ImagePreprocessor(self.config).preprocess(synced)
        ego_state = StatePreprocessor().estimate(packet.timestamp_s, packet.gps_fix, packet.imu_sample)
        planner_input = SensorFusion().fuse(synced, ego_state, image_features)
        model_input = ModelInputPackager(self.config).build(planner_input)
        self.assertFalse(planner_input.valid)
        self.assertFalse(model_input.valid)
        self.assertTrue(
            any(reason.startswith("front:unexpected_shape") for reason in model_input.diagnostics["invalid_reasons"])
        )

    def test_non_finite_controller_input_fails_closed(self) -> None:
        packet = make_mock_packet(self.config, frame_id=9, timestamp_s=0.9)
        synced = SensorSynchronizer(self.config).synchronize(packet)
        image_features = ImagePreprocessor(self.config).preprocess(synced)
        ego_state = StatePreprocessor().estimate(packet.timestamp_s, packet.gps_fix, packet.imu_sample)
        planner_input = SensorFusion().fuse(synced, ego_state, image_features)
        planner_input.ego_state.speed_mps = math.nan
        plan = PlanResult(
            frame_id=9,
            timestamp_s=0.9,
            planner_name="test",
            waypoints_xy=[(1.0, 0.0), (2.0, 0.0)],
            target_speed_mps=2.0,
            valid=True,
        )
        command = ControllerRuntime(self.config).compute(planner_input, plan, dt_s=0.1)
        decision = SafetyFilter(self.config).apply(planner_input, plan, command)
        self.assertFalse(command.valid)
        self.assertGreaterEqual(decision.command.brake, 1.0)

    def test_latency_toggle_disables_stage_breakdown(self) -> None:
        self.config.logging.enable_latency_profiling = False
        self.config.logging.log_dir = tempfile.mkdtemp(prefix="alpamayo_hardening_")
        pipeline = CompetitionRuntimePipeline(self.config)
        decision, snapshot = pipeline.run_cycle(make_mock_packet(self.config, frame_id=10, timestamp_s=1.0))
        self.assertEqual(decision.frame_id, 10)
        self.assertIn("total_cycle", snapshot.stage_latency_ms)
        self.assertNotIn("planner", snapshot.stage_latency_ms)


if __name__ == "__main__":
    unittest.main()
