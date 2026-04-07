"""Tests for live packet assembly from ROS subscriber state."""

from __future__ import annotations

import unittest

from alpamayo1_5.competition.contracts import CameraFrame, GpsFix, ImuSample
from alpamayo1_5.competition.integrations.morai.live_runtime import LivePacketAssembler
from alpamayo1_5.competition.integrations.morai.subscribers import LiveSensorState
from alpamayo1_5.competition.runtime.config_competition import load_competition_config


class LivePacketAssemblyTest(unittest.TestCase):
    def test_assemble_live_packet(self) -> None:
        config = load_competition_config("configs/competition_morai_live.json")
        state = LiveSensorState()
        state.update_camera(
            "front",
            CameraFrame(
                camera_id="front",
                timestamp_s=10.0,
                frame_id=11,
                image=[[[0, 0, 0] for _ in range(2)] for _ in range(2)],
                shape=(2, 2, 3),
                encoding="rgb8",
                metadata={"decoded_rgb": True},
            ),
        )
        state.update_gps(GpsFix(timestamp_s=10.0, latitude_deg=37.0, longitude_deg=127.0))
        state.update_imu(ImuSample(timestamp_s=10.0, yaw_rad=0.1))
        state.update_route_command("keep lane", timestamp_s=10.0)

        assembler = LivePacketAssembler(config, time_fn=lambda: 10.05)
        packet = assembler.assemble(state.snapshot())
        self.assertIsNotNone(packet)
        assert packet is not None
        self.assertEqual(packet.frame_id, 0)
        self.assertIn("front", packet.camera_frames)
        self.assertEqual(packet.route_command, "keep lane")
        self.assertEqual(packet.metadata["live_blocking_reasons"], [])

    def test_missing_and_stale_detection(self) -> None:
        config = load_competition_config("configs/competition_morai_live.json")
        state = LiveSensorState()
        state.update_camera(
            "front",
            CameraFrame(
                camera_id="front",
                timestamp_s=5.0,
                frame_id=1,
                image=[[[0, 0, 0] for _ in range(2)] for _ in range(2)],
                shape=(2, 2, 3),
                encoding="rgb8",
                metadata={"decoded_rgb": True},
            ),
        )
        diagnostics = LivePacketAssembler(config, time_fn=lambda: 6.0).inspect_snapshot(state.snapshot())
        self.assertIn("gps", diagnostics.missing_required)
        self.assertIn("imu", diagnostics.missing_required)
        self.assertIn("front", diagnostics.stale_sensors)
        self.assertIn("packet_timeout", diagnostics.blocking_reasons)


if __name__ == "__main__":
    unittest.main()
