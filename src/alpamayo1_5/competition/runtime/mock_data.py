"""Mock sensor packets for dry-runs, tests, and latency benchmarks."""

from __future__ import annotations

from typing import Iterable

from alpamayo1_5.competition.contracts import CameraFrame, GpsFix, ImuSample, SensorPacket
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig


def make_mock_packet(
    config: CompetitionConfig,
    frame_id: int = 0,
    timestamp_s: float = 0.0,
    route_command: str | None = None,
) -> SensorPacket:
    """Construct a deterministic mock sensor packet."""

    camera_frames: dict[str, CameraFrame] = {}
    for index, camera in enumerate(config.cameras):
        fake_image = [[[index, index, index] for _ in range(4)] for _ in range(4)]
        camera_frames[camera.name] = CameraFrame(
            camera_id=camera.name,
            timestamp_s=timestamp_s,
            frame_id=frame_id,
            image=fake_image,
            shape=(4, 4, 3),
            encoding="mock_rgb8",
        )

    gps_fix = GpsFix(
        timestamp_s=timestamp_s,
        latitude_deg=37.0 + 1e-5 * frame_id,
        longitude_deg=-122.0 + 2e-5 * frame_id,
        speed_mps=2.5,
    )
    imu_sample = ImuSample(
        timestamp_s=timestamp_s,
        yaw_rad=0.01 * frame_id,
        yaw_rate_rps=0.02,
        accel_mps2=0.05,
    )
    return SensorPacket(
        frame_id=frame_id,
        timestamp_s=timestamp_s,
        camera_frames=camera_frames,
        gps_fix=gps_fix,
        imu_sample=imu_sample,
        route_command=route_command or config.replay.default_route_command,
    )


def make_mock_replay(
    config: CompetitionConfig,
    num_packets: int,
    route_command: str | None = None,
) -> Iterable[SensorPacket]:
    """Yield a deterministic sequence of mock sensor packets."""

    timestamp_s = 0.0
    for frame_id in range(num_packets):
        yield make_mock_packet(config, frame_id=frame_id, timestamp_s=timestamp_s, route_command=route_command)
        timestamp_s += config.replay.default_frame_interval_s
