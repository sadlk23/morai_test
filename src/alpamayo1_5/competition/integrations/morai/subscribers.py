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
    get_nested_attr,
    infer_message_type_name,
    import_message_class,
    import_rospy,
)
from alpamayo1_5.competition.integrations.morai.topic_registry import build_subscription_specs
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig

logger = logging.getLogger(__name__)


def _message_timestamp_s(message: Any, fallback_s: float) -> float:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return float(stamp.to_sec()) if hasattr(stamp, "to_sec") else fallback_s


def _extract_optional_heading_rad(message: Any) -> float:
    for candidate in ("data", "heading", "value"):
        value = getattr(message, candidate, None)
        if value is not None:
            return float(value)
    raise ValueError("could not parse heading from message")


def _extract_optional_utm(message: Any) -> dict[str, float]:
    if hasattr(message, "x") and hasattr(message, "y"):
        return {"x_m": float(message.x), "y_m": float(message.y)}
    data = getattr(message, "data", None)
    if isinstance(data, (list, tuple)) and len(data) >= 2:
        return {"x_m": float(data[0]), "y_m": float(data[1])}
    pose = getattr(message, "pose", None)
    position = getattr(pose, "position", None) if pose is not None else None
    if position is not None and hasattr(position, "x") and hasattr(position, "y"):
        return {"x_m": float(position.x), "y_m": float(position.y)}
    point = getattr(message, "point", None)
    if point is not None and hasattr(point, "x") and hasattr(point, "y"):
        return {"x_m": float(point.x), "y_m": float(point.y)}
    raise ValueError("could not parse UTM x/y from message")


def _optional_ego_diagnostics(
    local_heading_rad: float | None,
    local_heading_timestamp_s: float | None,
    local_utm_xy: dict[str, float] | None,
    local_utm_timestamp_s: float | None,
    local_utm_source_type: str | None,
    last_utm_error: str | None,
) -> dict[str, Any]:
    diagnostics: dict[str, Any] = {"utm_available": local_utm_xy is not None}
    if local_heading_rad is not None:
        diagnostics["local_heading_rad"] = local_heading_rad
    if local_heading_timestamp_s is not None:
        diagnostics["local_heading_timestamp_s"] = local_heading_timestamp_s
    if local_utm_xy is not None:
        diagnostics["local_utm_xy"] = dict(local_utm_xy)
    if local_utm_timestamp_s is not None:
        diagnostics["local_utm_timestamp_s"] = local_utm_timestamp_s
    if local_utm_source_type is not None:
        diagnostics["utm_source_type"] = local_utm_source_type
    if last_utm_error is not None:
        diagnostics["last_utm_error"] = last_utm_error
    return diagnostics


def _extract_vehicle_status(message: Any) -> dict[str, Any]:
    source_type = infer_message_type_name(message)
    status: dict[str, Any] = {"source_type": source_type}
    data = getattr(message, "data", None)
    if isinstance(data, (list, tuple)):
        if len(data) >= 7:
            status.update(
                {
                    "control_mode": float(data[0]),
                    "e_stop": float(data[1]),
                    "gear": float(data[2]),
                    "speed_mps": float(data[3]),
                    "steer_rad": float(data[4]),
                    "brake": float(data[5]),
                    "alive": float(data[6]),
                }
            )
            return status
        raise ValueError("vehicle status array must contain at least 7 entries")

    speed_value = None
    for candidate in ("speed", "velocity", "vel", "speed_mps"):
        value = get_nested_attr(message, candidate, None)
        if value is not None:
            speed_value = float(value)
            break
    if speed_value is not None:
        status["speed_mps"] = speed_value
    for field_name, candidates in {
        "gear": ("gear", "gearNo"),
        "brake": ("brake", "brake_pct"),
        "steer_rad": ("steer", "steering", "wheel_angle"),
    }.items():
        for candidate in candidates:
            value = get_nested_attr(message, candidate, None)
            if value is not None:
                status[field_name] = float(value)
                break
    if len(status) == 1:
        raise ValueError("could not parse vehicle status fields from message")
    return status


def _vehicle_status_diagnostics(
    vehicle_status: dict[str, Any] | None,
    vehicle_status_timestamp_s: float | None,
    last_vehicle_status_error: str | None,
) -> dict[str, Any]:
    diagnostics: dict[str, Any] = {"available": vehicle_status is not None}
    if vehicle_status is not None:
        diagnostics.update(dict(vehicle_status))
    if vehicle_status_timestamp_s is not None:
        diagnostics["timestamp_s"] = vehicle_status_timestamp_s
    if last_vehicle_status_error is not None:
        diagnostics["last_error"] = last_vehicle_status_error
    return diagnostics


@dataclass(slots=True)
class LiveSensorSnapshot:
    """Thread-safe snapshot of the latest converted live sensor values."""

    camera_frames: dict[str, CameraFrame] = field(default_factory=dict)
    gps_fix: GpsFix | None = None
    imu_sample: ImuSample | None = None
    route_command: str | None = None
    route_timestamp_s: float | None = None
    local_heading_rad: float | None = None
    local_heading_timestamp_s: float | None = None
    local_utm_xy: dict[str, float] | None = None
    local_utm_timestamp_s: float | None = None
    vehicle_status: dict[str, Any] | None = None
    vehicle_status_timestamp_s: float | None = None
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
        self._local_heading_rad: float | None = None
        self._local_heading_timestamp_s: float | None = None
        self._local_utm_xy: dict[str, float] | None = None
        self._local_utm_timestamp_s: float | None = None
        self._local_utm_source_type: str | None = None
        self._last_utm_error: str | None = None
        self._vehicle_status: dict[str, Any] | None = None
        self._vehicle_status_timestamp_s: float | None = None
        self._last_vehicle_status_error: str | None = None
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

    def update_local_heading(self, heading_rad: float, timestamp_s: float) -> None:
        with self._lock:
            self._local_heading_rad = heading_rad
            self._local_heading_timestamp_s = timestamp_s
            self._receive_counts["local_heading"] = self._receive_counts.get("local_heading", 0) + 1
            self._last_errors.pop("local_heading", None)
            self._last_error_timestamps_s.pop("local_heading", None)

    def update_local_utm(self, utm_xy: dict[str, float], timestamp_s: float) -> None:
        with self._lock:
            self._local_utm_xy = {"x_m": float(utm_xy["x_m"]), "y_m": float(utm_xy["y_m"])}
            self._local_utm_timestamp_s = timestamp_s
            self._local_utm_source_type = str(utm_xy.get("source_type", "")) or None
            self._last_utm_error = None
            self._receive_counts["local_utm"] = self._receive_counts.get("local_utm", 0) + 1
            self._last_errors.pop("local_utm", None)
            self._last_error_timestamps_s.pop("local_utm", None)

    def update_vehicle_status(self, vehicle_status: dict[str, Any], timestamp_s: float) -> None:
        with self._lock:
            self._vehicle_status = dict(vehicle_status)
            self._vehicle_status_timestamp_s = timestamp_s
            self._last_vehicle_status_error = None
            self._receive_counts["vehicle_status"] = self._receive_counts.get("vehicle_status", 0) + 1
            self._last_errors.pop("vehicle_status", None)
            self._last_error_timestamps_s.pop("vehicle_status", None)

    def record_local_utm_error(self, message: str) -> None:
        with self._lock:
            self._last_utm_error = message

    def record_vehicle_status_error(self, message: str) -> None:
        with self._lock:
            self._last_vehicle_status_error = message

    def record_error(self, source: str, message: str, timestamp_s: float) -> None:
        """Store the latest callback error for diagnostics without crashing callbacks."""

        with self._lock:
            self._last_errors[source] = message
            self._last_error_timestamps_s[source] = timestamp_s

    def snapshot(self) -> LiveSensorSnapshot:
        with self._lock:
            optional_ego = _optional_ego_diagnostics(
                self._local_heading_rad,
                self._local_heading_timestamp_s,
                self._local_utm_xy,
                self._local_utm_timestamp_s,
                self._local_utm_source_type,
                self._last_utm_error,
            )
            vehicle_status = _vehicle_status_diagnostics(
                self._vehicle_status,
                self._vehicle_status_timestamp_s,
                self._last_vehicle_status_error,
            )
            return LiveSensorSnapshot(
                camera_frames=dict(self._camera_frames),
                gps_fix=self._gps_fix,
                imu_sample=self._imu_sample,
                route_command=self._route_command,
                route_timestamp_s=self._route_timestamp_s,
                local_heading_rad=self._local_heading_rad,
                local_heading_timestamp_s=self._local_heading_timestamp_s,
                local_utm_xy=dict(self._local_utm_xy) if self._local_utm_xy is not None else None,
                local_utm_timestamp_s=self._local_utm_timestamp_s,
                vehicle_status=dict(self._vehicle_status) if self._vehicle_status is not None else None,
                vehicle_status_timestamp_s=self._vehicle_status_timestamp_s,
                diagnostics={
                    "receive_counts": dict(self._receive_counts),
                    "last_errors": dict(self._last_errors),
                    "last_error_timestamps_s": dict(self._last_error_timestamps_s),
                    "optional_ego": optional_ego,
                    "vehicle_status": vehicle_status,
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
                stamp_s = _message_timestamp_s(message, float(self._rospy.get_time()))
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

    def _optional_heading_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            try:
                heading_rad = _extract_optional_heading_rad(message)
                timestamp_s = _message_timestamp_s(message, float(self._rospy.get_time()))
                self.state.update_local_heading(heading_rad, timestamp_s)
            except Exception as exc:
                error = "%s: %s" % (type(exc).__name__, exc)
                timestamp_s = self._rospy.get_time()
                self._warn_callback_error(
                    "local_heading",
                    "Failed to decode optional local heading: %s" % error,
                    timestamp_s,
                )
                self.state.record_error("local_heading", error, timestamp_s)

        return callback

    def _optional_utm_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            try:
                utm_xy = _extract_optional_utm(message)
                utm_xy["source_type"] = infer_message_type_name(message)
                timestamp_s = _message_timestamp_s(message, float(self._rospy.get_time()))
                self.state.update_local_utm(utm_xy, timestamp_s)
            except Exception as exc:
                error = "%s: %s" % (type(exc).__name__, exc)
                timestamp_s = self._rospy.get_time()
                self._warn_callback_error(
                    "local_utm",
                    "Failed to decode optional local UTM: %s" % error,
                    timestamp_s,
                )
                self.state.record_local_utm_error(error)
                self.state.record_error("local_utm", error, timestamp_s)

        return callback

    def _vehicle_status_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            try:
                vehicle_status = _extract_vehicle_status(message)
                timestamp_s = _message_timestamp_s(message, float(self._rospy.get_time()))
                self.state.update_vehicle_status(vehicle_status, timestamp_s)
            except Exception as exc:
                error = "%s: %s" % (type(exc).__name__, exc)
                timestamp_s = self._rospy.get_time()
                self._warn_callback_error(
                    "vehicle_status",
                    "Failed to decode optional vehicle status: %s" % error,
                    timestamp_s,
                )
                self.state.record_vehicle_status_error(error)
                self.state.record_error("vehicle_status", error, timestamp_s)

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
            elif spec.sensor_kind == "optional_heading":
                callback = self._optional_heading_callback()
            elif spec.sensor_kind == "optional_utm":
                callback = self._optional_utm_callback()
            elif spec.sensor_kind == "vehicle_status":
                callback = self._vehicle_status_callback()
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
