"""Tests for live MORAI runtime orchestration with mocked buffers."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.contracts import CameraFrame, GpsFix, ImuSample
from alpamayo1_5.competition.integrations.morai.live_runtime import LivePacketAssembler, MoraiLiveRuntime
from alpamayo1_5.competition.integrations.morai.subscribers import LiveSensorSnapshot
from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline


class _FakeSubscribers:
    def __init__(self, snapshot: LiveSensorSnapshot):
        self._snapshot = snapshot

    def snapshot(self) -> LiveSensorSnapshot:
        return self._snapshot


class LiveRuntimeTest(unittest.TestCase):
    def test_live_runtime_runs_one_cycle_with_mocked_buffers(self) -> None:
        config = load_competition_config("configs/competition_morai_live.json")
        pipeline = CompetitionRuntimePipeline(config, publishers=[])
        assembler = LivePacketAssembler(config, time_fn=lambda: 1.0)
        subscribers = _FakeSubscribers(
            LiveSensorSnapshot(
                camera_frames={
                    "front": CameraFrame(
                        camera_id="front",
                        timestamp_s=1.0,
                        frame_id=5,
                        image=[[[0, 0, 0] for _ in range(4)] for _ in range(4)],
                        shape=(4, 4, 3),
                        encoding="rgb8",
                    )
                },
                gps_fix=GpsFix(timestamp_s=1.0, latitude_deg=37.0, longitude_deg=127.0, speed_mps=2.0),
                imu_sample=ImuSample(timestamp_s=1.0, yaw_rad=0.0, yaw_rate_rps=0.0),
                route_command="keep lane",
            )
        )
        runtime = MoraiLiveRuntime(
            config,
            pipeline=pipeline,
            subscribers=subscribers,  # type: ignore[arg-type]
            assembler=assembler,
        )
        output = runtime.run_cycle_once()
        self.assertIsNotNone(output)
        assert output is not None
        decision, snapshot = output
        self.assertGreaterEqual(decision.command.throttle, 0.0)
        self.assertIn("planner", snapshot.stage_latency_ms)

    def test_live_runtime_can_wait_for_required_sensors_when_fail_closed_disabled(self) -> None:
        config = load_competition_config("configs/competition_morai_live.json")
        config.live_input.fail_closed_on_missing_required = False
        pipeline = CompetitionRuntimePipeline(config, publishers=[])
        assembler = LivePacketAssembler(config, time_fn=lambda: 1.0)
        subscribers = _FakeSubscribers(LiveSensorSnapshot())
        runtime = MoraiLiveRuntime(
            config,
            pipeline=pipeline,
            subscribers=subscribers,  # type: ignore[arg-type]
            assembler=assembler,
        )
        self.assertIsNone(runtime.run_cycle_once())


if __name__ == "__main__":
    unittest.main()
