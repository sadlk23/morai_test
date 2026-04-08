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


class _CapturePublisher:
    def __init__(self) -> None:
        self.decisions = []
        self.snapshots = []

    def publish(self, decision) -> None:
        self.decisions.append(decision)

    def publish_debug(self, snapshot) -> None:
        self.snapshots.append(snapshot)


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
        self.assertIn("live_system_state", decision.diagnostics)
        self.assertIn(decision.diagnostics["live_system_state"], {"debug_only", "publishing_actuation", "ready", "degraded"})
        self.assertIn("live_health", decision.diagnostics)
        self.assertIn("live_health", snapshot.diagnostics)
        self.assertIn("blocking_reasons", decision.diagnostics["live_health"])
        self.assertIn("optional_ego", decision.diagnostics)
        self.assertIn("legacy_serial_bridge", snapshot.diagnostics)
        self.assertIn("runtime_policy", decision.diagnostics)
        self.assertTrue(decision.diagnostics["runtime_policy"]["pedal_mode"])
        self.assertFalse(decision.diagnostics["runtime_policy"]["controls_gear_mode"])
        self.assertFalse(decision.diagnostics["runtime_policy"]["controls_external_mode"])
        self.assertIn("competition_profile", snapshot.diagnostics)

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

    def test_live_runtime_publishes_safe_stop_while_waiting_when_fail_closed_enabled(self) -> None:
        config = load_competition_config("configs/competition_morai_live.json")
        capture = _CapturePublisher()
        pipeline = CompetitionRuntimePipeline(config, publishers=[capture])
        assembler = LivePacketAssembler(config, time_fn=lambda: 2.0)
        subscribers = _FakeSubscribers(LiveSensorSnapshot())
        runtime = MoraiLiveRuntime(
            config,
            pipeline=pipeline,
            subscribers=subscribers,  # type: ignore[arg-type]
            assembler=assembler,
        )
        self.assertIsNone(runtime.run_cycle_once())
        self.assertEqual(len(capture.decisions), 1)
        self.assertEqual(capture.decisions[0].intervention, "live_input_wait_stop")
        self.assertIn("live_waiting_for_required_inputs", capture.decisions[0].safety_flags)
        self.assertEqual(capture.decisions[0].diagnostics["live_system_state"], "waiting")
        self.assertIn("live_health", capture.decisions[0].diagnostics)
        self.assertTrue(capture.decisions[0].diagnostics["live_health"]["timed_out"])

    def test_wait_stop_publish_interval_is_independent_from_warning_throttle(self) -> None:
        config = load_competition_config("configs/competition_morai_live.json")
        config.live_input.warn_throttle_s = 10.0
        config.live_input.safe_stop_publish_interval_s = 0.1
        capture = _CapturePublisher()
        pipeline = CompetitionRuntimePipeline(config, publishers=[capture])
        now = {"value": 2.0}
        assembler = LivePacketAssembler(config, time_fn=lambda: now["value"])
        subscribers = _FakeSubscribers(LiveSensorSnapshot())
        runtime = MoraiLiveRuntime(
            config,
            pipeline=pipeline,
            subscribers=subscribers,  # type: ignore[arg-type]
            assembler=assembler,
        )
        self.assertIsNone(runtime.run_cycle_once())
        now["value"] = 2.05
        self.assertIsNone(runtime.run_cycle_once())
        now["value"] = 2.15
        self.assertIsNone(runtime.run_cycle_once())
        self.assertEqual(len(capture.decisions), 2)
        self.assertTrue(all(item.intervention == "live_input_wait_stop" for item in capture.decisions))

    def test_optional_ego_topics_missing_does_not_block_runtime(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        pipeline = CompetitionRuntimePipeline(config, publishers=[])
        assembler = LivePacketAssembler(config, time_fn=lambda: 5.0)
        subscribers = _FakeSubscribers(
            LiveSensorSnapshot(
                camera_frames={
                    "front": CameraFrame(
                        camera_id="front",
                        timestamp_s=5.0,
                        frame_id=1,
                        image=[[[0, 0, 0] for _ in range(4)] for _ in range(4)],
                        shape=(4, 4, 3),
                        encoding="rgb8",
                    )
                },
                gps_fix=GpsFix(timestamp_s=5.0, latitude_deg=37.0, longitude_deg=127.0, speed_mps=1.5),
                imu_sample=ImuSample(timestamp_s=5.0, yaw_rad=0.0, yaw_rate_rps=0.0),
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
        self.assertIn("optional_ego", decision.diagnostics)
        self.assertFalse(decision.diagnostics["optional_ego"]["heading_available"])
        self.assertFalse(decision.diagnostics["optional_ego"]["utm_available"])
        self.assertIn("optional_ego", snapshot.diagnostics)
        self.assertEqual(decision.diagnostics["runtime_policy"]["gear_mode_policy"], "operator_managed")

    def test_vehicle_status_is_reflected_in_diagnostics_when_present(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        config.vehicle_status.enabled = True
        pipeline = CompetitionRuntimePipeline(config, publishers=[])
        assembler = LivePacketAssembler(config, time_fn=lambda: 7.0)
        subscribers = _FakeSubscribers(
            LiveSensorSnapshot(
                camera_frames={
                    "front": CameraFrame(
                        camera_id="front",
                        timestamp_s=7.0,
                        frame_id=2,
                        image=[[[0, 0, 0] for _ in range(4)] for _ in range(4)],
                        shape=(4, 4, 3),
                        encoding="rgb8",
                    )
                },
                gps_fix=GpsFix(timestamp_s=7.0, latitude_deg=37.0, longitude_deg=127.0, speed_mps=2.0),
                imu_sample=ImuSample(timestamp_s=7.0, yaw_rad=0.0, yaw_rate_rps=0.0),
                route_command="keep lane",
                vehicle_status={"speed_mps": 1.7, "gear": 0.0, "steer_rad": 0.02, "brake": 0.0},
                vehicle_status_timestamp_s=6.9,
                diagnostics={
                    "receive_counts": {"vehicle_status": 1},
                    "last_errors": {},
                    "last_error_timestamps_s": {},
                    "optional_ego": {},
                    "vehicle_status": {"available": True, "speed_mps": 1.7, "gear": 0.0},
                },
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
        self.assertTrue(decision.diagnostics["vehicle_status"]["available"])
        self.assertIn("speed_delta_mps", decision.diagnostics["command_status"])
        self.assertIn("vehicle_status", snapshot.diagnostics)
        self.assertTrue(decision.diagnostics["runtime_policy"]["vehicle_status_subscriber_enabled"])


if __name__ == "__main__":
    unittest.main()
