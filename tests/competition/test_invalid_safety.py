"""Tests for invalid-plan safety behavior."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.contracts import PlanResult
from alpamayo1_5.competition.controllers.controller_runtime import ControllerRuntime
from alpamayo1_5.competition.io.sync import SensorSynchronizer
from alpamayo1_5.competition.preprocess.image_preprocess import ImagePreprocessor
from alpamayo1_5.competition.preprocess.sensor_fusion import SensorFusion
from alpamayo1_5.competition.preprocess.state_preprocess import StatePreprocessor
from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_packet
from alpamayo1_5.competition.safety.safety_filter import SafetyFilter


class InvalidSafetyTest(unittest.TestCase):
    def test_invalid_plan_triggers_brake(self) -> None:
        config = load_competition_config("configs/competition_camera_gps_imu.json")
        packet = make_mock_packet(config, frame_id=6, timestamp_s=0.6)
        sync = SensorSynchronizer(config).synchronize(packet)
        features = ImagePreprocessor(config).preprocess(sync)
        ego_state = StatePreprocessor().estimate(packet.timestamp_s, packet.gps_fix, packet.imu_sample)
        planner_input = SensorFusion().fuse(sync, ego_state, features)
        plan = PlanResult(
            frame_id=6,
            timestamp_s=0.6,
            planner_name="test",
            waypoints_xy=[],
            target_speed_mps=5.0,
            valid=False,
        )
        command = ControllerRuntime(config).compute(planner_input, plan, dt_s=0.1)
        decision = SafetyFilter(config).apply(planner_input, plan, command)
        self.assertGreaterEqual(decision.command.brake, 1.0)
        self.assertTrue(decision.fallback_used)
        self.assertIn(decision.intervention, {"emergency_stop", "reuse_last_valid_command"})


if __name__ == "__main__":
    unittest.main()
