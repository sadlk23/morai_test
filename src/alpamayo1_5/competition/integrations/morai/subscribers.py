"""Live ROS subscribers that convert MORAI topics into competition contracts."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
from threading import Lock
from typing import Any, Callable

from alpamayo1_5.competition.contracts import CameraFrame, GpsFix, ImuSample
from alpamayo1_5.competition.integrations.morai.message_mapping import (
    map_camera_message,
    map_gps_message,
    map_imu_message,
    map_route_message,
)
from alpamayo1_5.competition.integrations.morai.ros_message_utils import (
    MoraiIntegrationUnavailable,
    import_message_class,
    import_rospy,
)
from alpamayo1_5.competition.integrations.morai.topic_registry import build_subscription_specs
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class LiveSensorSnapshot:
    """Thread-safe snapshot of the latest converted live sensor values."""

    camera_frames: dict[str, CameraFrame] = field(default_factory=dict)
    gps_fix: GpsFix | None = None
    imu_sample: ImuSample | None = None
    route_command: str | None = None
    route_timestamp_s: float | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)


class LiveSensorState:
    """Latest-value store populated by ROS callbacks."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._camera_frames: dict[str, CameraFrame] = {}
        self._gps_fix: GpsFix | None = None
        self._imu_sample: ImuSample | None = None
        self._route_command: str | None = None
        self._route_timestamp_s: float | None = None
        self._receive_counts: dict[str, int] = {}
        self._last_errors: dict[str, str] = {}
        self._last_error_timestamps_s: dict[str, float] = {}

    def update_camera(self, camera_name: str, frame: CameraFrame) -> None:
        with self._lock:
            self._camera_frames[camera_name] = frame
            self._receive_counts[camera_name] = self._receive_counts.get(camera_name, 0) + 1
            self._last_errors.pop(camera_name, None)
            self._last_error_timestamps_s.pop(camera_name, None)

    def update_gps(self, gps_fix: GpsFix) -> None:
        with self._lock:
            self._gps_fix = gps_fix
            self._receive_counts["gps"] = self._receive_counts.get("gps", 0) + 1
            self._last_errors.pop("gps", None)
            self._last_error_timestamps_s.pop("gps", None)

    def update_imu(self, imu_sample: ImuSample) -> None:
        with self._lock:
            self._imu_sample = imu_sample
            self._receive_counts["imu"] = self._receive_counts.get("imu", 0) + 1
            self._last_errors.pop("imu", None)
            self._last_error_timestamps_s.pop("imu", None)

    def update_route_command(self, route_command: str, timestamp_s: float) -> None:
        with self._lock:
            self._route_command = route_command
            self._route_timestamp_s = timestamp_s
            self._receive_counts["route_command"] = self._receive_counts.get("route_command", 0) + 1
            self._last_errors.pop("route_command", None)
            self._last_error_timestamps_s.pop("route_command", None)

    def record_error(self, source: str, message: str, timestamp_s: float) -> None:
        """Store the latest callback error for diagnostics without crashing callbacks."""

        with self._lock:
            self._last_errors[source] = message
            self._last_error_timestamps_s[source] = timestamp_s

    def snapshot(self) -> LiveSensorSnapshot:
        with self._lock:
            return LiveSensorSnapshot(
                camera_frames=dict(self._camera_frames),
                gps_fix=self._gps_fix,
                imu_sample=self._imu_sample,
                route_command=self._route_command,
                route_timestamp_s=self._route_timestamp_s,
                diagnostics={
                    "receive_counts": dict(self._receive_counts),
                    "last_errors": dict(self._last_errors),
                    "last_error_timestamps_s": dict(self._last_error_timestamps_s),
                },
            )


class MoraiRosSubscriberManager:
    """ROS1 subscriber bundle for live MORAI-compatible runtime ingestion."""

    def __init__(self, config: CompetitionConfig, state: LiveSensorState | None = None):
        self.config = config
        self.state = state or LiveSensorState()
        self._rospy = import_rospy()
        self._last_warning_s: dict[str, float] = {}
        if not self._rospy.core.is_initialized():
            self._rospy.init_node(
                config.live_input.node_name,
                anonymous=True,
                disable_signals=True,
            )
        self._subscribers: list[Any] = []
        self._register_subscribers()

    def _warn_callback_error(self, source: str, message: str, timestamp_s: float) -> None:
        """Throttle repeated callback warnings for the same source."""

        last_warning_s = self._last_warning_s.get(source, -1e9)
        if timestamp_s - last_warning_s < self.config.live_input.warn_throttle_s:
            return
        self._last_warning_s[source] = timestamp_s
        logger.warning("%s", message)

    def _camera_callback(
        self,
        camera_name: str,
        camera_resolution: tuple[int, int],
        message_type: str,
    ) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            try:
                frame = map_camera_message(
                    message=message,
                    camera_name=camera_name,
                    frame_id=-1,
                    message_type=message_type,
                    expected_resolution=camera_resolution,
                )
                self.state.update_camera(camera_name, frame)
            except Exception as exc:
                stamp_s = getattr(getattr(message, "header", None), "stamp", None)
                timestamp_s = float(stamp_s.to_sec()) if hasattr(stamp_s, "to_sec") else self._rospy.get_time()
                error = "%s: %s" % (type(exc).__name__, exc)
                self._warn_callback_error(
                    camera_name,
                    "Failed to decode camera %s on %s: %s" % (camera_name, message_type, error),
                    timestamp_s,
                )
                self.state.record_error(camera_name, error, timestamp_s)

        return callback

    def _gps_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            try:
                self.state.update_gps(map_gps_message(message))
            except Exception as exc:
                error = "%s: %s" % (type(exc).__name__, exc)
                timestamp_s = self._rospy.get_time()
                self._warn_callback_error("gps", "Failed to decode GPS message: %s" % error, timestamp_s)
                self.state.record_error("gps", error, timestamp_s)

        return callback

    def _imu_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            try:
                self.state.update_imu(map_imu_message(message))
            except Exception as exc:
                error = "%s: %s" % (type(exc).__name__, exc)
                timestamp_s = self._rospy.get_time()
                self._warn_callback_error("imu", "Failed to decode IMU message: %s" % error, timestamp_s)
                self.state.record_error("imu", error, timestamp_s)

        return callback

    def _route_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            try:
                route_command = map_route_message(message)
                timestamp_s = getattr(getattr(message, "header", None), "stamp", None)
                if hasattr(timestamp_s, "to_sec"):
                    stamp_s = float(timestamp_s.to_sec())
                else:
                    stamp_s = self._rospy.get_time()
                self.state.update_route_command(route_command, stamp_s)
            except Exception as exc:
                error = "%s: %s" % (type(exc).__name__, exc)
                timestamp_s = self._rospy.get_time()
                self._warn_callback_error(
                    "route_command",
                    "Failed to decode route command: %s" % error,
                    timestamp_s,
                )
                self.state.record_error("route_command", error, timestamp_s)

        return callback

    def _register_subscribers(self) -> None:
        specs = build_subscription_specs(self.config)
        for spec in specs:
            try:
                message_cls = import_message_class(spec.message_type)
            except MoraiIntegrationUnavailable:
                if spec.required:
                    raise
                continue

            if spec.sensor_kind == "camera":
                camera_cfg = next(camera for camera in self.config.cameras if camera.name == spec.name)
                callback = self._camera_callback(
                    spec.name,
                    (camera_cfg.width, camera_cfg.height),
                    spec.message_type,
                )
            elif spec.sensor_kind == "gps":
                callback = self._gps_callback()
            elif spec.sensor_kind == "imu":
                callback = self._imu_callback()
            elif spec.sensor_kind == "route_command":
                callback = self._route_callback()
            else:
                continue

            self._subscribers.append(
                self._rospy.Subscriber(
                    spec.topic,
                    message_cls,
                    callback,
                    queue_size=self.config.live_input.subscriber_queue_size,
                )
            )

    def snapshot(self) -> LiveSensorSnapshot:
        """Return the latest thread-safe sensor snapshot."""

        return self.state.snapshot()
