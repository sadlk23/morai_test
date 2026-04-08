"""Live MORAI runtime loop around the existing competition pipeline."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from time import time
from typing import Callable

from alpamayo1_5.competition.contracts import CameraFrame, ControlCommand, DebugSnapshot, SafetyDecision, SensorPacket
from alpamayo1_5.competition.integrations.morai.legacy_serial_bridge import (
    LegacySerialDataPublisher,
    legacy_serial_bridge_diagnostics,
)
from alpamayo1_5.competition.integrations.morai.publishers import (
    MoraiCtrlCmdPublisher,
    RosDebugSnapshotPublisher,
    RosJsonCommandPublisher,
)
from alpamayo1_5.competition.integrations.morai.ros_message_utils import MoraiIntegrationUnavailable, import_rospy
from alpamayo1_5.competition.integrations.morai.subscribers import LiveSensorSnapshot, MoraiRosSubscriberManager
from alpamayo1_5.competition.io.udp_interface import UdpCommandPublisher
from alpamayo1_5.competition.runtime.config_competition import (
    CompetitionConfig,
    competition_profile_diagnostics,
    morai_udp_reference_diagnostics,
    runtime_policy_diagnostics,
)
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class LivePacketDiagnostics:
    """Useful live-assembly diagnostics for logging and tests."""

    missing_required: list[str]
    stale_sensors: list[str]
    route_command_stale: bool
    timed_out: bool
    blocking_reasons: list[str]


class LivePacketAssembler:
    """Convert latest live ROS samples into the existing SensorPacket contract."""

    def __init__(self, config: CompetitionConfig, time_fn: Callable[[], float] | None = None):
        self.config = config
        self._time_fn = time_fn or time
        self._next_frame_id = 0

    def _sensor_age(self, now_s: float, timestamp_s: float | None) -> float | None:
        if timestamp_s is None:
            return None
        return max(0.0, now_s - timestamp_s)

    def inspect_snapshot(self, snapshot: LiveSensorSnapshot, now_s: float | None = None) -> LivePacketDiagnostics:
        """Summarize missing and stale live inputs before packet assembly."""

        current_s = self._time_fn() if now_s is None else now_s
        missing_required: list[str] = []
        stale_sensors: list[str] = []
        latest_timestamp_s: float | None = None

        for camera in self.config.cameras:
            frame = snapshot.camera_frames.get(camera.name)
            if frame is None:
                if camera.required:
                    missing_required.append(camera.name)
                continue
            latest_timestamp_s = (
                frame.timestamp_s
                if latest_timestamp_s is None
                else max(latest_timestamp_s, frame.timestamp_s)
            )
            frame_age = self._sensor_age(current_s, frame.timestamp_s)
            if frame_age is not None and frame_age > camera.max_staleness_s:
                stale_sensors.append(camera.name)

        if snapshot.gps_fix is None:
            if self.config.gps.required:
                missing_required.append("gps")
        else:
            latest_timestamp_s = (
                snapshot.gps_fix.timestamp_s
                if latest_timestamp_s is None
                else max(latest_timestamp_s, snapshot.gps_fix.timestamp_s)
            )
            gps_age = self._sensor_age(current_s, snapshot.gps_fix.timestamp_s)
            if gps_age is not None and gps_age > self.config.gps.max_staleness_s:
                stale_sensors.append("gps")

        if snapshot.imu_sample is None:
            if self.config.imu.required:
                missing_required.append("imu")
        else:
            latest_timestamp_s = (
                snapshot.imu_sample.timestamp_s
                if latest_timestamp_s is None
                else max(latest_timestamp_s, snapshot.imu_sample.timestamp_s)
            )
            imu_age = self._sensor_age(current_s, snapshot.imu_sample.timestamp_s)
            if imu_age is not None and imu_age > self.config.imu.max_staleness_s:
                stale_sensors.append("imu")

        route_command_stale = False
        if snapshot.route_timestamp_s is not None:
            route_command_stale = (
                self._sensor_age(current_s, snapshot.route_timestamp_s) or 0.0
            ) > self.config.route_command.max_staleness_s
            latest_timestamp_s = (
                snapshot.route_timestamp_s
                if latest_timestamp_s is None
                else max(latest_timestamp_s, snapshot.route_timestamp_s)
            )

        timed_out = latest_timestamp_s is None or (
            self._sensor_age(current_s, latest_timestamp_s) or 0.0
        ) > self.config.live_input.packet_timeout_s
        blocking_reasons: list[str] = []
        if timed_out:
            blocking_reasons.append("packet_timeout")
        if missing_required:
            blocking_reasons.extend(["missing:%s" % item for item in missing_required])
        if stale_sensors:
            blocking_reasons.extend(["stale:%s" % item for item in stale_sensors])
        if route_command_stale:
            blocking_reasons.append("stale:route_command")

        return LivePacketDiagnostics(
            missing_required=missing_required,
            stale_sensors=stale_sensors,
            route_command_stale=route_command_stale,
            timed_out=timed_out,
            blocking_reasons=blocking_reasons,
        )

    def assemble(self, snapshot: LiveSensorSnapshot) -> SensorPacket | None:
        """Assemble the latest live ROS buffers into a SensorPacket."""

        if not snapshot.camera_frames and snapshot.gps_fix is None and snapshot.imu_sample is None:
            return None

        packet_time_s = self._time_fn()
        diagnostics = self.inspect_snapshot(snapshot, packet_time_s)
        ordered_frames: dict[str, CameraFrame] = {}
        for camera in self.config.cameras:
            frame = snapshot.camera_frames.get(camera.name)
            if frame is not None:
                ordered_frames[camera.name] = frame

        route_command = snapshot.route_command
        if diagnostics.route_command_stale:
            route_command = None

        packet = SensorPacket(
            frame_id=self._next_frame_id,
            timestamp_s=packet_time_s,
            camera_frames=ordered_frames,
            gps_fix=snapshot.gps_fix,
            imu_sample=snapshot.imu_sample,
            route_command=route_command,
            metadata={
                "source": "morai_live_ros",
                "live_missing_required": diagnostics.missing_required,
                "live_stale_sensors": diagnostics.stale_sensors,
                "live_timed_out": diagnostics.timed_out,
                "live_blocking_reasons": diagnostics.blocking_reasons,
                "receive_counts": snapshot.diagnostics.get("receive_counts", {}),
                "last_errors": snapshot.diagnostics.get("last_errors", {}),
                "last_error_timestamps_s": snapshot.diagnostics.get("last_error_timestamps_s", {}),
                "optional_ego": snapshot.diagnostics.get("optional_ego", {}),
                "vehicle_status": snapshot.diagnostics.get("vehicle_status", {}),
                "competition_status": snapshot.diagnostics.get("competition_status", {}),
                "collision_data": snapshot.diagnostics.get("collision_data", {}),
            },
        )
        self._next_frame_id += 1
        return packet


class MoraiLiveRuntime:
    """Drive the existing competition pipeline from live MORAI ROS topics."""

    def __init__(
        self,
        config: CompetitionConfig,
        pipeline: CompetitionRuntimePipeline | None = None,
        subscribers: MoraiRosSubscriberManager | None = None,
        assembler: LivePacketAssembler | None = None,
    ):
        self.config = config
        self.pipeline = pipeline or CompetitionRuntimePipeline(config, publishers=self._build_publishers())
        self.subscribers = subscribers or MoraiRosSubscriberManager(config)
        self.assembler = assembler or LivePacketAssembler(config, time_fn=self._select_time_fn())
        self._last_warning_s = 0.0
        self._last_wait_publish_s = 0.0

    def _select_time_fn(self) -> Callable[[], float]:
        if self.config.live_input.use_ros_time:
            try:
                rospy = import_rospy()
                return lambda: float(rospy.get_time())
            except MoraiIntegrationUnavailable:
                pass
        return time

    def _build_publishers(self) -> list[object]:
        publishers: list[object] = []
        if self.config.output_mode in {"ros", "dual"} and self.config.ros_output.enabled:
            if self.config.ros_output.publish_command_json:
                publishers.append(RosJsonCommandPublisher(self.config.ros_output))
            if self.config.ros_output.publish_debug_json:
                publishers.append(RosDebugSnapshotPublisher(self.config.ros_output))
            if self.config.ros_output.publish_actuation:
                publishers.append(MoraiCtrlCmdPublisher(self.config.ros_output))
            if (
                self.config.legacy_serial_bridge.enabled
                and self.config.legacy_serial_bridge.publish_enabled
            ):
                publishers.append(
                    LegacySerialDataPublisher(
                        self.config.legacy_serial_bridge,
                        self.config.ros_output,
                    )
                )
        if self.config.output_mode in {"udp", "dual"} and self.config.udp_output.enabled:
            publishers.append(UdpCommandPublisher(self.config.udp_output))
        return publishers

    def _system_state(
        self,
        health: LivePacketDiagnostics,
        live_snapshot: LiveSensorSnapshot,
        publishing_decision: bool = False,
    ) -> str:
        """Summarize the current live-runtime state for diagnostics."""

        if health.timed_out or health.missing_required:
            return "waiting"
        if live_snapshot.diagnostics.get("last_errors") or health.stale_sensors:
            return "degraded"
        if publishing_decision and self.config.ros_output.publish_actuation:
            return "publishing_actuation"
        if self.config.ros_output.publish_debug_json or self.config.ros_output.publish_command_json:
            if not self.config.ros_output.publish_actuation:
                return "debug_only"
        return "ready"

    def _health_summary(
        self,
        health: LivePacketDiagnostics,
        live_snapshot: LiveSensorSnapshot,
        publishing_decision: bool,
    ) -> dict[str, object]:
        """Build a structured live-health summary for debug snapshots and logs."""

        return {
            "system_state": self._system_state(health, live_snapshot, publishing_decision=publishing_decision),
            "missing_required": list(health.missing_required),
            "stale_sensors": list(health.stale_sensors),
            "route_command_stale": health.route_command_stale,
            "timed_out": health.timed_out,
            "blocking_reasons": list(health.blocking_reasons),
            "receive_counts": live_snapshot.diagnostics.get("receive_counts", {}),
            "last_errors": live_snapshot.diagnostics.get("last_errors", {}),
            "last_error_timestamps_s": live_snapshot.diagnostics.get("last_error_timestamps_s", {}),
            "optional_ego": self._optional_ego_summary(live_snapshot),
            "vehicle_status": self._vehicle_status_summary(live_snapshot),
            "competition_status": self._diagnostics_input_summary(
                dict(live_snapshot.diagnostics.get("competition_status", {})),
                live_snapshot.competition_status_timestamp_s,
                self.config.competition_status.max_staleness_s,
            ),
            "collision_data": self._diagnostics_input_summary(
                dict(live_snapshot.diagnostics.get("collision_data", {})),
                live_snapshot.collision_data_timestamp_s,
                self.config.collision_data.max_staleness_s,
            ),
            "legacy_serial_bridge": legacy_serial_bridge_diagnostics(self.config.legacy_serial_bridge),
            "competition_profile": competition_profile_diagnostics(self.config),
            "runtime_policy": runtime_policy_diagnostics(self.config),
            "morai_udp_reference": morai_udp_reference_diagnostics(self.config),
        }

    def _optional_ego_summary(self, live_snapshot: LiveSensorSnapshot) -> dict[str, object]:
        summary = dict(live_snapshot.diagnostics.get("optional_ego", {}))
        summary.setdefault("heading_available", live_snapshot.local_heading_rad is not None)
        summary.setdefault("utm_available", live_snapshot.local_utm_xy is not None)
        if live_snapshot.local_heading_timestamp_s is not None:
            summary.setdefault("local_heading_timestamp_s", live_snapshot.local_heading_timestamp_s)
        if live_snapshot.local_utm_timestamp_s is not None:
            summary.setdefault("local_utm_timestamp_s", live_snapshot.local_utm_timestamp_s)
        return summary

    def _vehicle_status_summary(self, live_snapshot: LiveSensorSnapshot) -> dict[str, object]:
        summary = dict(live_snapshot.diagnostics.get("vehicle_status", {}))
        summary.setdefault("available", live_snapshot.vehicle_status is not None)
        if live_snapshot.vehicle_status_timestamp_s is None:
            summary.setdefault("age_s", None)
            summary.setdefault("stale", False)
            return summary
        age_s = max(0.0, self.assembler._time_fn() - live_snapshot.vehicle_status_timestamp_s)
        summary["age_s"] = age_s
        summary["stale"] = age_s > self.config.vehicle_status.max_staleness_s
        return summary

    def _diagnostics_input_summary(
        self,
        summary: dict[str, object],
        timestamp_s: float | None,
        max_staleness_s: float,
    ) -> dict[str, object]:
        summary.setdefault("available", False)
        if timestamp_s is None:
            summary.setdefault("age_s", None)
            summary.setdefault("stale", False)
            return summary
        age_s = max(0.0, self.assembler._time_fn() - timestamp_s)
        summary["age_s"] = age_s
        summary["stale"] = age_s > max_staleness_s
        return summary

    def _command_status_summary(
        self,
        decision: SafetyDecision,
        live_snapshot: LiveSensorSnapshot,
    ) -> dict[str, object]:
        status = live_snapshot.vehicle_status
        if status is None:
            return {"available": False}
        summary: dict[str, object] = {"available": True}
        if status.get("speed_mps") is not None:
            summary["speed_delta_mps"] = float(decision.command.target_speed_mps) - float(status["speed_mps"])
        if status.get("steer_rad") is not None:
            summary["steer_delta_rad"] = float(decision.command.steering) - float(status["steer_rad"])
        if status.get("brake") is not None:
            summary["brake_delta"] = float(decision.command.brake) - float(status["brake"])
        if status.get("gear") is not None:
            summary["status_gear"] = status["gear"]
        return summary

    def run_cycle_once(self) -> tuple[object, object] | None:
        """Run one live cycle if at least one sensor sample has arrived."""

        live_snapshot = self.subscribers.snapshot()
        health = self.assembler.inspect_snapshot(live_snapshot)
        should_wait = health.timed_out or bool(health.missing_required)
        if should_wait and not self.config.live_input.fail_closed_on_missing_required:
            return None
        if should_wait and self.config.live_input.fail_closed_on_missing_required:
            self._publish_waiting_stop(live_snapshot, health)
            return None
        packet = self.assembler.assemble(live_snapshot)
        if packet is None:
            if self.config.live_input.fail_closed_on_missing_required:
                self._publish_waiting_stop(live_snapshot, health)
            return None
        decision, snapshot = self.pipeline.run_cycle(packet)
        health_summary = self._health_summary(health, live_snapshot, publishing_decision=True)
        decision.diagnostics["live_system_state"] = str(health_summary["system_state"])
        decision.diagnostics["blocking_reasons"] = list(health.blocking_reasons)
        decision.diagnostics["receive_counts"] = live_snapshot.diagnostics.get("receive_counts", {})
        decision.diagnostics["live_health"] = health_summary
        decision.diagnostics["optional_ego"] = health_summary.get("optional_ego", {})
        decision.diagnostics["vehicle_status"] = health_summary.get("vehicle_status", {})
        decision.diagnostics["competition_status"] = health_summary.get("competition_status", {})
        decision.diagnostics["collision_data"] = health_summary.get("collision_data", {})
        decision.diagnostics["command_status"] = self._command_status_summary(decision, live_snapshot)
        decision.diagnostics["legacy_serial_bridge"] = health_summary.get("legacy_serial_bridge", {})
        decision.diagnostics["competition_profile"] = health_summary.get("competition_profile", {})
        decision.diagnostics["runtime_policy"] = health_summary.get("runtime_policy", {})
        decision.diagnostics["morai_udp_reference"] = health_summary.get("morai_udp_reference", {})
        snapshot.diagnostics["live_system_state"] = str(health_summary["system_state"])
        snapshot.diagnostics["blocking_reasons"] = list(health.blocking_reasons)
        snapshot.diagnostics["receive_counts"] = live_snapshot.diagnostics.get("receive_counts", {})
        snapshot.diagnostics["last_errors"] = live_snapshot.diagnostics.get("last_errors", {})
        snapshot.diagnostics["last_error_timestamps_s"] = live_snapshot.diagnostics.get(
            "last_error_timestamps_s", {}
        )
        snapshot.diagnostics["live_health"] = health_summary
        snapshot.diagnostics["optional_ego"] = health_summary.get("optional_ego", {})
        snapshot.diagnostics["vehicle_status"] = health_summary.get("vehicle_status", {})
        snapshot.diagnostics["competition_status"] = health_summary.get("competition_status", {})
        snapshot.diagnostics["collision_data"] = health_summary.get("collision_data", {})
        snapshot.diagnostics["command_status"] = self._command_status_summary(decision, live_snapshot)
        snapshot.diagnostics["legacy_serial_bridge"] = health_summary.get("legacy_serial_bridge", {})
        snapshot.diagnostics["competition_profile"] = health_summary.get("competition_profile", {})
        snapshot.diagnostics["runtime_policy"] = health_summary.get("runtime_policy", {})
        snapshot.diagnostics["morai_udp_reference"] = health_summary.get("morai_udp_reference", {})
        return decision, snapshot

    def _publish_waiting_stop(
        self,
        live_snapshot: LiveSensorSnapshot,
        health: LivePacketDiagnostics,
    ) -> None:
        """Publish a safe stop while waiting for a valid live packet."""

        now_s = self.assembler._time_fn()
        if now_s - self._last_wait_publish_s < self.config.live_input.safe_stop_publish_interval_s:
            return
        self._last_wait_publish_s = now_s
        health_summary = self._health_summary(health, live_snapshot, publishing_decision=False)
        receive_counts = health_summary["receive_counts"]
        last_errors = health_summary["last_errors"]
        decision = SafetyDecision(
            frame_id=-1,
            timestamp_s=now_s,
            command=ControlCommand(
                frame_id=-1,
                timestamp_s=now_s,
                steering=0.0,
                throttle=0.0,
                brake=self.config.safety.emergency_brake_value,
                target_speed_mps=0.0,
                valid=False,
                reason="waiting_for_live_inputs",
            ),
            intervention="live_input_wait_stop",
            risk_level="high",
            safety_flags=["live_waiting_for_required_inputs"] + list(health.blocking_reasons),
            fallback_used=True,
            diagnostics={
                "blocking_reasons": list(health.blocking_reasons),
                "receive_counts": receive_counts,
                "last_errors": last_errors,
                "last_error_timestamps_s": health_summary["last_error_timestamps_s"],
                "live_system_state": str(health_summary["system_state"]),
                "live_health": health_summary,
                "optional_ego": health_summary.get("optional_ego", {}),
                "vehicle_status": health_summary.get("vehicle_status", {}),
                "competition_status": health_summary.get("competition_status", {}),
                "collision_data": health_summary.get("collision_data", {}),
                "legacy_serial_bridge": health_summary.get("legacy_serial_bridge", {}),
                "competition_profile": health_summary.get("competition_profile", {}),
                "runtime_policy": health_summary.get("runtime_policy", {}),
                "morai_udp_reference": health_summary.get("morai_udp_reference", {}),
            },
        )
        snapshot = DebugSnapshot(
            frame_id=-1,
            timestamp_s=now_s,
            diagnostics={
                "waiting_for_live_inputs": True,
                "blocking_reasons": list(health.blocking_reasons),
                "receive_counts": receive_counts,
                "last_errors": last_errors,
                "last_error_timestamps_s": health_summary["last_error_timestamps_s"],
                "live_system_state": str(health_summary["system_state"]),
                "live_health": health_summary,
                "optional_ego": health_summary.get("optional_ego", {}),
                "vehicle_status": health_summary.get("vehicle_status", {}),
                "competition_status": health_summary.get("competition_status", {}),
                "collision_data": health_summary.get("collision_data", {}),
                "legacy_serial_bridge": health_summary.get("legacy_serial_bridge", {}),
                "competition_profile": health_summary.get("competition_profile", {}),
                "runtime_policy": health_summary.get("runtime_policy", {}),
                "morai_udp_reference": health_summary.get("morai_udp_reference", {}),
            },
            safety_flags=["live_waiting_for_required_inputs"] + list(health.blocking_reasons),
        )
        publish_errors = self.pipeline._publish(decision, snapshot)
        if publish_errors:
            logger.warning("Failed to publish live wait-stop command: %s", publish_errors)

    def spin(self, max_cycles: int | None = None) -> int:
        """Run the live control loop until shutdown or ``max_cycles``."""

        rospy = import_rospy()
        rate = rospy.Rate(self.config.live_input.loop_hz)
        cycles = 0
        while not rospy.is_shutdown():
            output = self.run_cycle_once()
            current_time_s = float(rospy.get_time())
            if output is None and current_time_s - self._last_warning_s >= self.config.live_input.warn_throttle_s:
                live_snapshot = self.subscribers.snapshot()
                health = self.assembler.inspect_snapshot(live_snapshot, now_s=current_time_s)
                logger.warning(
                    "Waiting for live MORAI sensors. state=%s blocking_reasons=%s receive_counts=%s last_errors=%s",
                    self._system_state(health, live_snapshot, publishing_decision=False),
                    health.blocking_reasons,
                    live_snapshot.diagnostics.get("receive_counts", {}),
                    live_snapshot.diagnostics.get("last_errors", {}),
                )
                self._last_warning_s = current_time_s
            elif output is not None:
                decision, snapshot = output
                logger.info(
                    "live frame=%s state=%s intervention=%s steer=%.3f throttle=%.3f brake=%.3f total_ms=%.2f",
                    decision.frame_id,
                    decision.diagnostics.get("live_system_state", "unknown"),
                    decision.intervention,
                    decision.command.steering,
                    decision.command.throttle,
                    decision.command.brake,
                    snapshot.stage_latency_ms.get("total_cycle", -1.0),
                )
                cycles += 1
                if max_cycles is not None and cycles >= max_cycles:
                    break
            rate.sleep()
        return cycles


def run_live_runtime(config: CompetitionConfig, max_cycles: int | None = None) -> int:
    """Convenience wrapper used by the live CLI entrypoint."""

    runtime = MoraiLiveRuntime(config)
    return runtime.spin(max_cycles=max_cycles)
