"""Sensor synchronization utilities."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import SensorPacket, SynchronizedFrame
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig


class SensorSynchronizer:
    """Validate packet freshness and expose synchronized runtime inputs."""

    def __init__(self, config: CompetitionConfig):
        self.config = config

    def synchronize(self, packet: SensorPacket) -> SynchronizedFrame:
        """Validate staleness and required-sensor availability."""

        missing: list[str] = []
        stale: list[str] = []
        invalid: list[str] = []
        age_s: dict[str, float] = {}
        timestamp_s = packet.timestamp_s
        fresh_camera_count = 0

        for camera in self.config.cameras:
            frame = packet.camera_frames.get(camera.name)
            if frame is None:
                if camera.required:
                    missing.append(camera.name)
                continue
            frame_age = timestamp_s - frame.timestamp_s
            if frame_age < 0.0:
                invalid.append(f"{camera.name}:future_timestamp")
            frame_age = abs(frame_age)
            age_s[camera.name] = frame_age
            if frame_age > camera.max_staleness_s:
                stale.append(camera.name)
            else:
                fresh_camera_count += 1

        if packet.gps_fix is None and self.config.gps.required:
            missing.append("gps")
        elif packet.gps_fix is not None:
            gps_age = timestamp_s - packet.gps_fix.timestamp_s
            if gps_age < 0.0:
                invalid.append("gps:future_timestamp")
            gps_age = abs(gps_age)
            age_s["gps"] = gps_age
            if gps_age > self.config.gps.max_staleness_s:
                stale.append("gps")

        if packet.imu_sample is None and self.config.imu.required:
            missing.append("imu")
        elif packet.imu_sample is not None:
            imu_age = timestamp_s - packet.imu_sample.timestamp_s
            if imu_age < 0.0:
                invalid.append("imu:future_timestamp")
            imu_age = abs(imu_age)
            age_s["imu"] = imu_age
            if imu_age > self.config.imu.max_staleness_s:
                stale.append("imu")

        route_required = self.config.route_command.required and bool(self.config.route_command.topic)
        if route_required and not packet.route_command:
            missing.append("route_command")

        if self.config.use_lidar and self.config.lidar.required and packet.lidar_packet is None:
            missing.append("lidar")
        elif packet.lidar_packet is not None:
            lidar_age = timestamp_s - packet.lidar_packet.timestamp_s
            if lidar_age < 0.0:
                invalid.append("lidar:future_timestamp")
            lidar_age = abs(lidar_age)
            age_s["lidar"] = lidar_age
            if lidar_age > self.config.lidar.max_staleness_s:
                stale.append("lidar")

        if fresh_camera_count < self.config.safety.min_fresh_cameras:
            invalid.append("insufficient_fresh_cameras")

        if self.config.safety.invalidate_on_stale_ego_sensors:
            if "gps" in stale or "imu" in stale:
                invalid.append("stale_ego_sensor")

        return SynchronizedFrame(
            frame_id=packet.frame_id,
            timestamp_s=packet.timestamp_s,
            camera_frames=packet.camera_frames,
            gps_fix=packet.gps_fix,
            imu_sample=packet.imu_sample,
            lidar_packet=packet.lidar_packet,
            route_command=packet.route_command,
            missing_sensors=missing,
            stale_sensors=stale,
            sensor_age_s=age_s,
            valid=not missing and not invalid,
            diagnostics={
                "missing_required_sensors": missing,
                "stale_sensors": stale,
                "fresh_camera_count": fresh_camera_count,
                "invalid_reasons": invalid,
            },
        )
