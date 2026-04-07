"""Tests for controller and safety behavior."""

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


class ControllerAndSafetyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_competition_config("configs/competition_camera_gps_imu.json")
        self.sync = SensorSynchronizer(self.config)
        self.image = ImagePreprocessor(self.config)
        self.state = StatePreprocessor()
        self.fusion = SensorFusion()
        self.controller = ControllerRuntime(self.config)
        self.safety = SafetyFilter(self.config)

    def test_controller_generates_nominal_command(self) -> None:
        packet = make_mock_packet(self.config, frame_id=1, timestamp_s=0.1)
        synced = self.sync.synchronize(packet)
        features = self.image.preprocess(synced)
        ego_state = self.state.estimate(packet.timestamp_s, packet.gps_fix, packet.imu_sample)
        planner_input = self.fusion.fuse(synced, ego_state, features)
        plan = PlanResult(
            frame_id=1,
            timestamp_s=0.1,
            planner_name="test",
            waypoints_xy=[(1.0, 0.0), (2.0, 0.1), (3.0, 0.2)],
            target_speed_mps=3.0,
        )
        command = self.controller.compute(planner_input, plan, dt_s=0.1)
        decision = self.safety.apply(planner_input, plan, command)
        self.assertTrue(decision.command.valid)
        self.assertEqual(decision.intervention, "none")

    def test_stale_sensor_forces_conservative_mode(self) -> None:
        packet = make_mock_packet(self.config, frame_id=2, timestamp_s=1.0)
        packet.camera_frames["front"].timestamp_s = 0.0
        synced = self.sync.synchronize(packet)
        features = self.image.preprocess(synced)
        ego_state = self.state.estimate(packet.timestamp_s, packet.gps_fix, packet.imu_sample)
        planner_input = self.fusion.fuse(synced, ego_state, features)
        plan = PlanResult(
            frame_id=2,
            timestamp_s=1.0,
            planner_name="test",
            waypoints_xy=[(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
            target_speed_mps=4.0,
            confidence=0.9,
        )
        command = self.controller.compute(planner_input, plan, dt_s=0.1)
        decision = self.safety.apply(planner_input, plan, command)
        self.assertEqual(decision.intervention, "conservative_slowdown")
        self.assertIn("stale_sensors", decision.safety_flags)


if __name__ == "__main__":
    unittest.main()
