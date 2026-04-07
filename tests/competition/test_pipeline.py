"""End-to-end dry-run tests for the competition pipeline."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_packet
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline


class PipelineTest(unittest.TestCase):
    def test_pipeline_runs_one_cycle(self) -> None:
        config = load_competition_config("configs/competition_camera_gps_imu.json")
        pipeline = CompetitionRuntimePipeline(config)
        packet = make_mock_packet(config, frame_id=10, timestamp_s=1.0, route_command="turn left in 15m")
        decision, snapshot = pipeline.run_cycle(packet)
        self.assertEqual(decision.frame_id, 10)
        self.assertGreater(len(snapshot.waypoints_xy), 0)
        self.assertIn("planner", snapshot.stage_latency_ms)
        self.assertIn(decision.intervention, {"none", "conservative_slowdown", "command_clipped"})


if __name__ == "__main__":
    unittest.main()
