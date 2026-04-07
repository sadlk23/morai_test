"""Live ROS subscribers that convert MORAI topics into competition contracts."""

from __future__ import annotations

from dataclasses import dataclass, field
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

    def update_camera(self, camera_name: str, frame: CameraFrame) -> None:
        with self._lock:
            self._camera_frames[camera_name] = frame
            self._receive_counts[camera_name] = self._receive_counts.get(camera_name, 0) + 1

    def update_gps(self, gps_fix: GpsFix) -> None:
        with self._lock:
            self._gps_fix = gps_fix
            self._receive_counts["gps"] = self._receive_counts.get("gps", 0) + 1

    def update_imu(self, imu_sample: ImuSample) -> None:
        with self._lock:
            self._imu_sample = imu_sample
            self._receive_counts["imu"] = self._receive_counts.get("imu", 0) + 1

    def update_route_command(self, route_command: str, timestamp_s: float) -> None:
        with self._lock:
            self._route_command = route_command
            self._route_timestamp_s = timestamp_s
            self._receive_counts["route_command"] = self._receive_counts.get("route_command", 0) + 1

    def snapshot(self) -> LiveSensorSnapshot:
        with self._lock:
            return LiveSensorSnapshot(
                camera_frames=dict(self._camera_frames),
                gps_fix=self._gps_fix,
                imu_sample=self._imu_sample,
                route_command=self._route_command,
                route_timestamp_s=self._route_timestamp_s,
                diagnostics={"receive_counts": dict(self._receive_counts)},
            )


class MoraiRosSubscriberManager:
    """ROS1 subscriber bundle for live MORAI-compatible runtime ingestion."""

    def __init__(self, config: CompetitionConfig, state: LiveSensorState | None = None):
        self.config = config
        self.state = state or LiveSensorState()
        self._rospy = import_rospy()
        if not self._rospy.core.is_initialized():
            self._rospy.init_node(
                config.live_input.node_name,
                anonymous=True,
                disable_signals=True,
            )
        self._subscribers: list[Any] = []
        self._register_subscribers()

    def _camera_callback(self, camera_name: str, camera_resolution: tuple[int, int]) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            frame = map_camera_message(
                message=message,
                camera_name=camera_name,
                frame_id=-1,
                expected_resolution=camera_resolution,
            )
            self.state.update_camera(camera_name, frame)

        return callback

    def _gps_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            self.state.update_gps(map_gps_message(message))

        return callback

    def _imu_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            self.state.update_imu(map_imu_message(message))

        return callback

    def _route_callback(self) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            route_command = map_route_message(message)
            timestamp_s = getattr(getattr(message, "header", None), "stamp", None)
            if hasattr(timestamp_s, "to_sec"):
                stamp_s = float(timestamp_s.to_sec())
            else:
                stamp_s = self._rospy.get_time()
            self.state.update_route_command(route_command, stamp_s)

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
                callback = self._camera_callback(spec.name, (camera_cfg.width, camera_cfg.height))
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
