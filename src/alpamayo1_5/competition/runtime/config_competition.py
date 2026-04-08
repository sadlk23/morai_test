"""Competition runtime configuration and validation."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any

SUPPORTED_CAMERA_MESSAGE_TYPES = {"sensor_msgs/Image", "sensor_msgs/CompressedImage"}
SUPPORTED_GPS_MESSAGE_TYPES = {"sensor_msgs/NavSatFix", "morai_msgs/GPSMessage"}
SUPPORTED_IMU_MESSAGE_TYPES = {"sensor_msgs/Imu"}
SUPPORTED_ROUTE_MESSAGE_TYPES = {"std_msgs/String"}
DEFAULT_MORAI_ACTUATION_MESSAGE_TYPE = "morai_msgs/CtrlCmd"


def _default_list() -> list[Any]:
    return []


@dataclass(slots=True)
class CameraConfig:
    """Camera input configuration."""

    name: str
    topic: str
    message_type: str = "sensor_msgs/Image"
    frame_id: str = ""
    camera_index: int | None = None
    required: bool = True
    width: int = 1280
    height: int = 720
    max_staleness_s: float = 0.2
    fallback_topics: list[str] = field(default_factory=_default_list)


@dataclass(slots=True)
class GpsConfig:
    """GPS configuration."""

    topic: str = "/alpamayo/gps"
    message_type: str = "sensor_msgs/NavSatFix"
    required: bool = True
    max_staleness_s: float = 0.2
    fallback_topics: list[str] = field(default_factory=_default_list)


@dataclass(slots=True)
class ImuConfig:
    """IMU configuration."""

    topic: str = "/alpamayo/imu"
    message_type: str = "sensor_msgs/Imu"
    required: bool = True
    max_staleness_s: float = 0.2


@dataclass(slots=True)
class LidarConfig:
    """Optional LiDAR configuration."""

    topic: str = "/alpamayo/lidar"
    message_type: str = "sensor_msgs/PointCloud2"
    required: bool = False
    max_staleness_s: float = 0.25


@dataclass(slots=True)
class RouteCommandConfig:
    """Optional route or navigation command topic."""

    topic: str = ""
    message_type: str = "std_msgs/String"
    required: bool = False
    max_staleness_s: float = 1.0


@dataclass(slots=True)
class PlannerConfig:
    """Planner configuration."""

    backend: str = "lightweight"
    checkpoint_path: str | None = None
    legacy_model_id: str = "nvidia/Alpamayo-1.5-10B"
    device: str = "cuda"
    use_diffusion: bool = False
    use_nav: bool = True
    use_lidar: bool = False
    precision: str = "bf16"
    input_image_width: int = 1280
    input_image_height: int = 720
    history_steps: int = 16
    history_dt_s: float = 0.1
    num_waypoints: int = 20
    waypoint_dt_s: float = 0.2
    min_target_speed_mps: float = 0.0
    max_target_speed_mps: float = 8.0
    low_confidence_threshold: float = 0.35
    top_p: float = 0.95
    temperature: float = 0.6
    max_generation_length: int = 256
    allow_backend_fallback: bool = True


@dataclass(slots=True)
class PurePursuitConfig:
    """Pure Pursuit controller configuration."""

    wheelbase_m: float = 2.7
    min_lookahead_m: float = 2.0
    max_lookahead_m: float = 8.0
    speed_to_lookahead_gain: float = 0.8


@dataclass(slots=True)
class StanleyConfig:
    """Stanley controller configuration."""

    wheelbase_m: float = 2.7
    gain: float = 1.2
    softening_speed_mps: float = 0.5


@dataclass(slots=True)
class PidConfig:
    """PID gains for longitudinal speed control."""

    kp: float = 0.5
    ki: float = 0.05
    kd: float = 0.1
    integral_limit: float = 5.0
    output_limit: float = 1.0


@dataclass(slots=True)
class ControllerConfig:
    """Controller stack configuration."""

    lateral_controller: str = "pure_pursuit"
    pure_pursuit: PurePursuitConfig = field(default_factory=PurePursuitConfig)
    stanley: StanleyConfig = field(default_factory=StanleyConfig)
    longitudinal_pid: PidConfig = field(default_factory=PidConfig)
    max_steering: float = 1.0
    max_throttle: float = 1.0
    max_brake: float = 1.0
    brake_deadband: float = 0.05


@dataclass(slots=True)
class SafetyConfig:
    """Safety thresholds and fallback policy."""

    max_plan_age_s: float = 0.3
    max_sensor_age_s: float = 0.25
    max_abs_steering: float = 1.0
    max_target_speed_mps: float = 10.0
    max_curvature_for_full_speed: float = 0.08
    min_confidence: float = 0.3
    max_stop_probability_without_brake: float = 0.5
    reuse_last_command_horizon_s: float = 0.3
    emergency_brake_value: float = 1.0
    conservative_speed_mps: float = 1.0
    min_fresh_cameras: int = 1
    invalidate_on_stale_ego_sensors: bool = True


@dataclass(slots=True)
class RosOutputConfig:
    """ROS output topics."""

    enabled: bool = True
    command_topic: str = "/alpamayo/control_cmd_json"
    debug_topic: str = "/alpamayo/debug_snapshot"
    node_name: str = "alpamayo_competition_runtime"
    queue_size: int = 1
    publish_command_json: bool = True
    publish_debug_json: bool = True
    publish_actuation: bool = False
    require_actuation_arm: bool = True
    actuation_armed: bool = False
    actuation_topic: str = "/ctrl_cmd"
    actuation_message_type: str = "morai_msgs/CtrlCmd"
    command_mode: str = "pedal"


@dataclass(slots=True)
class LegacySerialBridgeConfig:
    """Legacy moo-compatible serial-data bridge output."""

    enabled: bool = False
    topic: str = "/Control/serial_data"
    message_type: str = "std_msgs/Float32MultiArray"
    publish_enabled: bool = False
    default_control_mode: int = 1
    default_gear: int = 0
    include_alive_counter: bool = True
    e_stop_on_intervention: bool = True
    brake_mode: str = "auto"
    brake_output_max: float = 1.0


@dataclass(slots=True)
class CompetitionProfileConfig:
    """Competition metadata and operational policy notes."""

    competition_name: str = ""
    map_name: str = ""
    vehicle_model: str = ""
    wheelbase_m: float | None = None
    ros_distro: str = ""
    os_version: str = ""
    desktop_only: bool = False
    rosbridge_optional: bool = True
    max_gps: int | None = None
    max_imu: int | None = None
    max_camera: int | None = None
    max_lidar: int | None = None
    camera_pitch_limit_deg: float | None = None
    direct_actuation_topic: str = ""
    direct_actuation_message_type: str = ""
    direct_actuation_longitudinal_type: int | None = None
    direct_actuation_command_mode: str = ""
    participant_controls_gear_mode: bool = False
    participant_controls_external_mode: bool = False
    initial_vehicle_state: str = ""
    operator_transition_state: str = ""


@dataclass(slots=True)
class OptionalEgoTopicsConfig:
    """Optional helper ego topics used for diagnostics only."""

    heading_topic: str = ""
    heading_message_type: str = "std_msgs/Float64"
    heading_fallback_message_types: list[str] = field(default_factory=_default_list)
    utm_topic: str = ""
    utm_message_type: str = "std_msgs/Float32MultiArray"
    utm_fallback_message_types: list[str] = field(default_factory=_default_list)


@dataclass(slots=True)
class VehicleStatusConfig:
    """Optional vehicle-status input used for diagnostics only."""

    enabled: bool = False
    topic: str = "/ERP/serial_data"
    message_type: str = "std_msgs/Float32MultiArray"
    required: bool = False
    max_staleness_s: float = 0.5


@dataclass(slots=True)
class DiagnosticsInputConfig:
    """Diagnostics-only optional ROS input.

    The runtime never treats these channels as bring-up blockers.
    Historical JSON may still contain a deprecated ``required`` key; the loader
    accepts and ignores it for backward compatibility.
    """

    enabled: bool = False
    topic: str = ""
    message_type: str = ""
    max_staleness_s: float = 0.5


@dataclass(slots=True)
class MoraiUdpReferenceConfig:
    """Historical MORAI UDP reference values for operator diagnostics.

    These values are not active runtime behavior by themselves. They exist so
    the debug snapshot can surface venue-reference information, including older
    LAN-based competition channels seen in prior MORAI workspaces.
    """

    user_ip: str = "127.0.0.1"
    host_ip: str = "127.0.0.1"
    multi_ip: str = ""
    camera_host_port: int = 1231
    camera_user_port: int = 1232
    lidar_host_port: int = 2369
    lidar_user_port: int = 2368
    gps_host_port: int = 1233
    gps_user_port: int = 1234
    imu_host_port: int = 1235
    imu_user_port: int = 1236
    ctrl_cmd_host_port: int = 9091
    ctrl_cmd_user_port: int = 9092
    vehicle_status_host_port: int = 7701
    vehicle_status_user_port: int = 7702
    competition_status_host_port: int | None = None
    competition_status_user_port: int | None = None
    collision_data_host_port: int | None = None
    collision_data_user_port: int | None = None
    get_traffic_host_port: int = 7602
    get_traffic_user_port: int = 7502
    set_traffic_host_port: int = 7801
    set_traffic_user_port: int = 7800
    object_info_host_port: int = 7605
    object_info_user_port: int = 7505
    planner_path_file_name: str = "kcity.txt"
    user_serial_port: str = "/dev/ttyUSB0"


@dataclass(slots=True)
class LiveInputConfig:
    """Live ROS ingestion configuration."""

    enabled: bool = False
    adapter: str = "morai"
    node_name: str = "alpamayo_morai_live_runtime"
    loop_hz: float = 10.0
    subscriber_queue_size: int = 1
    packet_timeout_s: float = 0.5
    use_ros_time: bool = True
    fail_closed_on_missing_required: bool = True
    safe_stop_publish_interval_s: float = 0.1
    warn_throttle_s: float = 2.0


@dataclass(slots=True)
class UdpOutputConfig:
    """UDP output configuration."""

    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 5005


@dataclass(slots=True)
class LoggingConfig:
    """Logging and debug-dump configuration."""

    log_dir: str = "artifacts/competition_logs"
    write_metrics_jsonl: bool = True
    write_debug_jsonl: bool = True
    write_command_history_jsonl: bool = True
    verbose_debug: bool = False
    save_last_valid_plan: bool = True
    enable_latency_profiling: bool = True


@dataclass(slots=True)
class ReplayConfig:
    """Offline replay and dry-run settings."""

    enabled: bool = True
    default_route_command: str = "keep lane"
    default_frame_interval_s: float = 0.1


@dataclass(slots=True)
class CompetitionConfig:
    """Top-level competition configuration."""

    planner_hz: float = 10.0
    control_hz: float = 20.0
    use_lidar: bool = False
    output_mode: str = "ros"
    cameras: list[CameraConfig] = field(default_factory=_default_list)
    gps: GpsConfig = field(default_factory=GpsConfig)
    imu: ImuConfig = field(default_factory=ImuConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    route_command: RouteCommandConfig = field(default_factory=RouteCommandConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    live_input: LiveInputConfig = field(default_factory=LiveInputConfig)
    ros_output: RosOutputConfig = field(default_factory=RosOutputConfig)
    legacy_serial_bridge: LegacySerialBridgeConfig = field(default_factory=LegacySerialBridgeConfig)
    competition_profile: CompetitionProfileConfig = field(default_factory=CompetitionProfileConfig)
    optional_ego_topics: OptionalEgoTopicsConfig = field(default_factory=OptionalEgoTopicsConfig)
    vehicle_status: VehicleStatusConfig = field(default_factory=VehicleStatusConfig)
    competition_status: DiagnosticsInputConfig = field(default_factory=DiagnosticsInputConfig)
    collision_data: DiagnosticsInputConfig = field(default_factory=DiagnosticsInputConfig)
    morai_udp_reference: MoraiUdpReferenceConfig = field(default_factory=MoraiUdpReferenceConfig)
    udp_output: UdpOutputConfig = field(default_factory=UdpOutputConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    replay: ReplayConfig = field(default_factory=ReplayConfig)

    def validate(self) -> None:
        """Validate critical configuration invariants."""

        errors: list[str] = []
        if self.planner_hz <= 0:
            errors.append("planner_hz must be > 0")
        if self.control_hz <= 0:
            errors.append("control_hz must be > 0")
        if self.control_hz < self.planner_hz:
            errors.append("control_hz should be >= planner_hz")
        if not self.cameras:
            errors.append("at least one camera configuration is required")

        seen_names: set[str] = set()
        seen_topics: set[str] = set()
        for camera in self.cameras:
            if camera.name in seen_names:
                errors.append(f"duplicate camera name: {camera.name}")
            seen_names.add(camera.name)
            if not camera.topic:
                errors.append(f"camera {camera.name} topic must not be empty")
            elif camera.topic in seen_topics:
                errors.append(f"duplicate camera topic: {camera.topic}")
            else:
                seen_topics.add(camera.topic)
            if not camera.message_type:
                errors.append(f"camera {camera.name} message_type must not be empty")
            elif camera.message_type not in SUPPORTED_CAMERA_MESSAGE_TYPES:
                errors.append(
                    f"camera {camera.name} message_type must be one of: "
                    + ", ".join(sorted(SUPPORTED_CAMERA_MESSAGE_TYPES))
                )
            if camera.width <= 0 or camera.height <= 0:
                errors.append(f"camera {camera.name} must have positive resolution")
            if camera.max_staleness_s <= 0:
                errors.append(f"camera {camera.name} max_staleness_s must be > 0")
            for fallback_topic in camera.fallback_topics:
                if not fallback_topic:
                    errors.append(f"camera {camera.name} fallback topic must not be empty")
                elif fallback_topic == camera.topic:
                    errors.append(
                        f"camera {camera.name} fallback topic duplicates primary topic: {fallback_topic}"
                    )
                elif fallback_topic in seen_topics:
                    errors.append(f"duplicate camera fallback topic: {fallback_topic}")
                else:
                    seen_topics.add(fallback_topic)

        if not self.gps.topic:
            errors.append("gps.topic must not be empty")
        if not self.gps.message_type:
            errors.append("gps.message_type must not be empty")
        elif self.gps.message_type not in SUPPORTED_GPS_MESSAGE_TYPES:
            errors.append(
                "gps.message_type must be one of: " + ", ".join(sorted(SUPPORTED_GPS_MESSAGE_TYPES))
            )
        if self.gps.topic in seen_topics:
            errors.append(f"gps.topic duplicates another topic: {self.gps.topic}")
        else:
            seen_topics.add(self.gps.topic)
        if self.gps.max_staleness_s <= 0:
            errors.append("gps.max_staleness_s must be > 0")
        for fallback_topic in self.gps.fallback_topics:
            if not fallback_topic:
                errors.append("gps.fallback_topics entries must not be empty")
            elif fallback_topic == self.gps.topic:
                errors.append(f"gps fallback topic duplicates primary topic: {fallback_topic}")
            elif fallback_topic in seen_topics:
                errors.append(f"gps fallback topic duplicates another topic: {fallback_topic}")
            else:
                seen_topics.add(fallback_topic)

        if not self.imu.topic:
            errors.append("imu.topic must not be empty")
        if not self.imu.message_type:
            errors.append("imu.message_type must not be empty")
        elif self.imu.message_type not in SUPPORTED_IMU_MESSAGE_TYPES:
            errors.append(
                "imu.message_type must be one of: " + ", ".join(sorted(SUPPORTED_IMU_MESSAGE_TYPES))
            )
        if self.imu.topic in seen_topics:
            errors.append(f"imu.topic duplicates another topic: {self.imu.topic}")
        else:
            seen_topics.add(self.imu.topic)
        if self.imu.max_staleness_s <= 0:
            errors.append("imu.max_staleness_s must be > 0")

        if self.route_command.required and not self.route_command.topic:
            errors.append("route_command.topic must not be empty when required")
        if self.route_command.topic:
            if self.route_command.topic in seen_topics:
                errors.append(
                    f"route_command.topic duplicates another topic: {self.route_command.topic}"
                )
            else:
                seen_topics.add(self.route_command.topic)
        if self.route_command.topic and not self.route_command.message_type:
            errors.append("route_command.message_type must not be empty when route topic is set")
        elif self.route_command.topic and self.route_command.message_type not in SUPPORTED_ROUTE_MESSAGE_TYPES:
            errors.append(
                "route_command.message_type must be one of: "
                + ", ".join(sorted(SUPPORTED_ROUTE_MESSAGE_TYPES))
            )
        if self.route_command.max_staleness_s <= 0:
            errors.append("route_command.max_staleness_s must be > 0")
        if self.optional_ego_topics.heading_topic:
            if self.optional_ego_topics.heading_topic in seen_topics:
                errors.append(
                    "optional_ego_topics.heading_topic duplicates another topic: "
                    + self.optional_ego_topics.heading_topic
                )
            else:
                seen_topics.add(self.optional_ego_topics.heading_topic)
            if not self.optional_ego_topics.heading_message_type:
                errors.append(
                    "optional_ego_topics.heading_message_type must not be empty when heading_topic is set"
                )
            elif "/" not in self.optional_ego_topics.heading_message_type:
                errors.append(
                    "optional_ego_topics.heading_message_type must use package/MessageName syntax"
                )
            for fallback_message_type in self.optional_ego_topics.heading_fallback_message_types:
                if not fallback_message_type:
                    errors.append(
                        "optional_ego_topics.heading_fallback_message_types entries must not be empty"
                    )
                elif "/" not in fallback_message_type:
                    errors.append(
                        "optional_ego_topics.heading_fallback_message_types must use package/MessageName syntax"
                    )
        if self.optional_ego_topics.utm_topic:
            if self.optional_ego_topics.utm_topic in seen_topics:
                errors.append(
                    "optional_ego_topics.utm_topic duplicates another topic: "
                    + self.optional_ego_topics.utm_topic
                )
            else:
                seen_topics.add(self.optional_ego_topics.utm_topic)
            if not self.optional_ego_topics.utm_message_type:
                errors.append(
                    "optional_ego_topics.utm_message_type must not be empty when utm_topic is set"
                )
            elif "/" not in self.optional_ego_topics.utm_message_type:
                errors.append(
                    "optional_ego_topics.utm_message_type must use package/MessageName syntax"
                )
            for fallback_message_type in self.optional_ego_topics.utm_fallback_message_types:
                if not fallback_message_type:
                    errors.append("optional_ego_topics.utm_fallback_message_types entries must not be empty")
                elif "/" not in fallback_message_type:
                    errors.append(
                        "optional_ego_topics.utm_fallback_message_types must use package/MessageName syntax"
                    )
        if self.vehicle_status.enabled:
            if not self.vehicle_status.topic:
                errors.append("vehicle_status.topic must not be empty when enabled=true")
            elif self.vehicle_status.topic in seen_topics:
                errors.append(
                    "vehicle_status.topic duplicates another topic: " + self.vehicle_status.topic
                )
            else:
                seen_topics.add(self.vehicle_status.topic)
            if not self.vehicle_status.message_type:
                errors.append("vehicle_status.message_type must not be empty when enabled=true")
            elif "/" not in self.vehicle_status.message_type:
                errors.append("vehicle_status.message_type must use package/MessageName syntax")
            if self.vehicle_status.max_staleness_s <= 0:
                errors.append("vehicle_status.max_staleness_s must be > 0 when enabled=true")
        for field_name in ("competition_status", "collision_data"):
            diagnostics_input = getattr(self, field_name)
            if diagnostics_input.enabled:
                if not diagnostics_input.topic:
                    errors.append(f"{field_name}.topic must not be empty when enabled=true")
                elif diagnostics_input.topic in seen_topics:
                    errors.append(f"{field_name}.topic duplicates another topic: {diagnostics_input.topic}")
                else:
                    seen_topics.add(diagnostics_input.topic)
                if not diagnostics_input.message_type:
                    errors.append(f"{field_name}.message_type must not be empty when enabled=true")
                elif "/" not in diagnostics_input.message_type:
                    errors.append(f"{field_name}.message_type must use package/MessageName syntax")
                if diagnostics_input.max_staleness_s <= 0:
                    errors.append(f"{field_name}.max_staleness_s must be > 0 when enabled=true")

        if self.output_mode not in {"ros", "udp", "dual"}:
            errors.append("output_mode must be one of: ros, udp, dual")
        if self.planner.backend not in {"legacy_alpamayo", "lightweight"}:
            errors.append("planner.backend must be one of: legacy_alpamayo, lightweight")
        if self.planner.precision not in {"fp32", "fp16", "bf16"}:
            errors.append("planner.precision must be one of: fp32, fp16, bf16")
        if self.planner.input_image_width <= 0 or self.planner.input_image_height <= 0:
            errors.append("planner input image dimensions must be > 0")
        if self.planner.history_steps <= 0:
            errors.append("planner.history_steps must be > 0")
        if self.planner.history_dt_s <= 0:
            errors.append("planner.history_dt_s must be > 0")
        if self.planner.num_waypoints <= 1:
            errors.append("planner.num_waypoints must be > 1")
        if self.planner.waypoint_dt_s <= 0:
            errors.append("planner.waypoint_dt_s must be > 0")
        if self.planner.min_target_speed_mps < 0:
            errors.append("planner.min_target_speed_mps must be >= 0")
        if self.planner.max_target_speed_mps < self.planner.min_target_speed_mps:
            errors.append("planner.max_target_speed_mps must be >= min_target_speed_mps")
        if self.controller.lateral_controller not in {"pure_pursuit", "stanley"}:
            errors.append("controller.lateral_controller must be pure_pursuit or stanley")
        if not 0 <= self.safety.min_confidence <= 1:
            errors.append("safety.min_confidence must be within [0, 1]")
        if self.safety.min_fresh_cameras <= 0:
            errors.append("safety.min_fresh_cameras must be > 0")
        if self.safety.min_fresh_cameras > len(self.cameras):
            errors.append("safety.min_fresh_cameras cannot exceed configured camera count")
        if self.live_input.adapter not in {"morai", "generic_ros"}:
            errors.append("live_input.adapter must be morai or generic_ros")
        if self.live_input.loop_hz <= 0:
            errors.append("live_input.loop_hz must be > 0")
        if self.live_input.subscriber_queue_size <= 0:
            errors.append("live_input.subscriber_queue_size must be > 0")
        if self.live_input.packet_timeout_s <= 0:
            errors.append("live_input.packet_timeout_s must be > 0")
        if self.live_input.safe_stop_publish_interval_s <= 0:
            errors.append("live_input.safe_stop_publish_interval_s must be > 0")
        if self.live_input.warn_throttle_s <= 0:
            errors.append("live_input.warn_throttle_s must be > 0")
        if self.udp_output.enabled and self.udp_output.port <= 0:
            errors.append("udp_output.port must be > 0 when UDP is enabled")
        if self.competition_profile.wheelbase_m is not None:
            if self.competition_profile.wheelbase_m <= 0:
                errors.append("competition_profile.wheelbase_m must be > 0 when set")
            if abs(self.controller.pure_pursuit.wheelbase_m - self.competition_profile.wheelbase_m) > 1e-6:
                errors.append(
                    "competition_profile.wheelbase_m must match controller.pure_pursuit.wheelbase_m"
                )
            if abs(self.controller.stanley.wheelbase_m - self.competition_profile.wheelbase_m) > 1e-6:
                errors.append(
                    "competition_profile.wheelbase_m must match controller.stanley.wheelbase_m"
                )
        for sensor_limit_name in ("max_gps", "max_imu", "max_camera", "max_lidar"):
            sensor_limit = getattr(self.competition_profile, sensor_limit_name)
            if sensor_limit is not None and sensor_limit <= 0:
                errors.append(f"competition_profile.{sensor_limit_name} must be > 0 when set")
        if (
            self.competition_profile.camera_pitch_limit_deg is not None
            and self.competition_profile.camera_pitch_limit_deg < 0
        ):
            errors.append("competition_profile.camera_pitch_limit_deg must be >= 0 when set")
        if self.output_mode == "ros" and not self.ros_output.enabled:
            errors.append("output_mode=ros requires ros_output.enabled=true")
        if self.output_mode == "udp" and not self.udp_output.enabled:
            errors.append("output_mode=udp requires udp_output.enabled=true")
        if self.ros_output.queue_size <= 0:
            errors.append("ros_output.queue_size must be > 0")
        legacy_serial_active = (
            self.legacy_serial_bridge.enabled and self.legacy_serial_bridge.publish_enabled
        )
        if self.output_mode in {"ros", "dual"} and self.ros_output.enabled:
            if not (
                self.ros_output.publish_command_json
                or self.ros_output.publish_debug_json
                or self.ros_output.publish_actuation
                or legacy_serial_active
            ):
                errors.append(
                    "ros_output.enabled=true requires at least one of publish_command_json, "
                    "publish_debug_json, publish_actuation, or legacy_serial_bridge.publish_enabled"
                )
        if self.ros_output.publish_command_json and not self.ros_output.command_topic:
            errors.append("ros_output.command_topic must not be empty when publish_command_json=true")
        if self.ros_output.publish_debug_json and not self.ros_output.debug_topic:
            errors.append("ros_output.debug_topic must not be empty when publish_debug_json=true")
        if self.ros_output.publish_actuation:
            if not self.ros_output.actuation_topic:
                errors.append(
                    "ros_output.actuation_topic must not be empty when publish_actuation=true"
                )
            if not self.ros_output.actuation_message_type:
                errors.append(
                    "ros_output.actuation_message_type must not be empty when publish_actuation=true"
                )
            if "/" not in self.ros_output.actuation_message_type:
                errors.append(
                    "ros_output.actuation_message_type must use package/MessageName syntax"
                )
            if (
                self.live_input.adapter == "morai"
                and self.ros_output.actuation_message_type != DEFAULT_MORAI_ACTUATION_MESSAGE_TYPE
            ):
                errors.append(
                    "morai live adapter currently supports actuation_message_type="
                    + DEFAULT_MORAI_ACTUATION_MESSAGE_TYPE
                )
            if self.ros_output.require_actuation_arm and not self.ros_output.actuation_armed:
                errors.append(
                    "ros_output.publish_actuation=true requires ros_output.actuation_armed=true "
                    "when require_actuation_arm=true"
                )
            if not self.live_input.fail_closed_on_missing_required:
                errors.append(
                    "publish_actuation=true requires live_input.fail_closed_on_missing_required=true"
                )
            if self.safety.emergency_brake_value <= 0:
                errors.append("publish_actuation=true requires safety.emergency_brake_value > 0")
        if self.ros_output.command_mode not in {"pedal", "velocity"}:
            errors.append("ros_output.command_mode must be pedal or velocity")
        if self.ros_output.command_mode == "velocity" and not self.ros_output.publish_actuation:
            errors.append("ros_output.command_mode=velocity requires publish_actuation=true")
        if (
            self.competition_profile.direct_actuation_command_mode
            and self.competition_profile.direct_actuation_command_mode != self.ros_output.command_mode
        ):
            errors.append(
                "competition_profile.direct_actuation_command_mode must match ros_output.command_mode"
            )
        if (
            self.competition_profile.direct_actuation_topic
            and self.competition_profile.direct_actuation_topic != self.ros_output.actuation_topic
        ):
            errors.append(
                "competition_profile.direct_actuation_topic must match ros_output.actuation_topic"
            )
        if (
            self.competition_profile.direct_actuation_message_type
            and self.competition_profile.direct_actuation_message_type
            != self.ros_output.actuation_message_type
        ):
            errors.append(
                "competition_profile.direct_actuation_message_type must match "
                "ros_output.actuation_message_type"
            )
        if (
            self.competition_profile.direct_actuation_longitudinal_type == 1
            and self.ros_output.command_mode != "pedal"
        ):
            errors.append(
                "competition_profile.direct_actuation_longitudinal_type=1 requires "
                "ros_output.command_mode=pedal"
            )
        if self.legacy_serial_bridge.publish_enabled and not self.legacy_serial_bridge.enabled:
            errors.append("legacy_serial_bridge.publish_enabled=true requires legacy_serial_bridge.enabled=true")
        if self.legacy_serial_bridge.enabled:
            if not self.legacy_serial_bridge.topic:
                errors.append("legacy_serial_bridge.topic must not be empty when enabled=true")
            if self.legacy_serial_bridge.message_type != "std_msgs/Float32MultiArray":
                errors.append(
                    "legacy_serial_bridge.message_type must be std_msgs/Float32MultiArray"
                )
            if self.legacy_serial_bridge.brake_mode not in {"auto", "normalized", "erp_200"}:
                errors.append("legacy_serial_bridge.brake_mode must be auto, normalized, or erp_200")
            if self.legacy_serial_bridge.default_control_mode < 0:
                errors.append("legacy_serial_bridge.default_control_mode must be >= 0")
            if self.legacy_serial_bridge.brake_output_max <= 0:
                errors.append("legacy_serial_bridge.brake_output_max must be > 0")
            if (
                self.legacy_serial_bridge.publish_enabled
                and self.output_mode not in {"ros", "dual"}
            ):
                errors.append(
                    "legacy_serial_bridge.publish_enabled=true requires output_mode ros or dual"
                )
            if self.legacy_serial_bridge.publish_enabled and not self.ros_output.enabled:
                errors.append(
                    "legacy_serial_bridge.publish_enabled=true requires ros_output.enabled=true"
                )

        if not self.morai_udp_reference.user_ip:
            errors.append("morai_udp_reference.user_ip must not be empty")
        if not self.morai_udp_reference.host_ip:
            errors.append("morai_udp_reference.host_ip must not be empty")
        reference_ports = [
            ("camera_host_port", self.morai_udp_reference.camera_host_port),
            ("camera_user_port", self.morai_udp_reference.camera_user_port),
            ("lidar_host_port", self.morai_udp_reference.lidar_host_port),
            ("lidar_user_port", self.morai_udp_reference.lidar_user_port),
            ("gps_host_port", self.morai_udp_reference.gps_host_port),
            ("gps_user_port", self.morai_udp_reference.gps_user_port),
            ("imu_host_port", self.morai_udp_reference.imu_host_port),
            ("imu_user_port", self.morai_udp_reference.imu_user_port),
            ("ctrl_cmd_host_port", self.morai_udp_reference.ctrl_cmd_host_port),
            ("ctrl_cmd_user_port", self.morai_udp_reference.ctrl_cmd_user_port),
            ("vehicle_status_host_port", self.morai_udp_reference.vehicle_status_host_port),
            ("vehicle_status_user_port", self.morai_udp_reference.vehicle_status_user_port),
            ("get_traffic_host_port", self.morai_udp_reference.get_traffic_host_port),
            ("get_traffic_user_port", self.morai_udp_reference.get_traffic_user_port),
            ("set_traffic_host_port", self.morai_udp_reference.set_traffic_host_port),
            ("set_traffic_user_port", self.morai_udp_reference.set_traffic_user_port),
            ("object_info_host_port", self.morai_udp_reference.object_info_host_port),
            ("object_info_user_port", self.morai_udp_reference.object_info_user_port),
        ]
        for port_name, port_value in reference_ports:
            if port_value <= 0:
                errors.append(f"morai_udp_reference.{port_name} must be > 0")
        optional_reference_ports = [
            ("competition_status_host_port", self.morai_udp_reference.competition_status_host_port),
            ("competition_status_user_port", self.morai_udp_reference.competition_status_user_port),
            ("collision_data_host_port", self.morai_udp_reference.collision_data_host_port),
            ("collision_data_user_port", self.morai_udp_reference.collision_data_user_port),
        ]
        for port_name, port_value in optional_reference_ports:
            if port_value is not None and port_value <= 0:
                errors.append(f"morai_udp_reference.{port_name} must be > 0 when set")
        camera_indices = [camera.camera_index for camera in self.cameras if camera.camera_index is not None]
        if len(camera_indices) != len(set(camera_indices)):
            errors.append("camera.camera_index values must be unique when provided")
        if self.planner.checkpoint_path is not None and not Path(self.planner.checkpoint_path).exists():
            errors.append(f"planner.checkpoint_path does not exist: {self.planner.checkpoint_path}")
        if self.planner.backend == "legacy_alpamayo" and not (
            self.planner.checkpoint_path or self.planner.legacy_model_id
        ):
            errors.append(
                "legacy_alpamayo backend requires planner.checkpoint_path or planner.legacy_model_id"
            )
        if self.planner.backend == "legacy_alpamayo":
            missing_indices = [camera.name for camera in self.cameras if camera.camera_index is None]
            if missing_indices:
                errors.append(
                    "legacy_alpamayo backend requires camera_index for all configured cameras: "
                    + ", ".join(missing_indices)
                )

        if errors:
            raise ValueError("Invalid competition config:\n- " + "\n- ".join(errors))

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable configuration dictionary."""

        return asdict(self)


def _build_camera_list(raw: list[dict[str, Any]]) -> list[CameraConfig]:
    return [CameraConfig(**item) for item in raw]


def _build_diagnostics_input_config(raw: dict[str, Any]) -> DiagnosticsInputConfig:
    allowed_keys = {"enabled", "topic", "message_type", "max_staleness_s"}
    normalized = {key: value for key, value in raw.items() if key in allowed_keys}
    return DiagnosticsInputConfig(**normalized)


def _build_config(raw: dict[str, Any]) -> CompetitionConfig:
    controller_raw = raw.get("controller", {})
    return CompetitionConfig(
        planner_hz=raw.get("planner_hz", 10.0),
        control_hz=raw.get("control_hz", 20.0),
        use_lidar=raw.get("use_lidar", False),
        output_mode=raw.get("output_mode", "ros"),
        cameras=_build_camera_list(raw.get("cameras", [])),
        gps=GpsConfig(**raw.get("gps", {})),
        imu=ImuConfig(**raw.get("imu", {})),
        lidar=LidarConfig(**raw.get("lidar", {})),
        route_command=RouteCommandConfig(**raw.get("route_command", {})),
        planner=PlannerConfig(**raw.get("planner", {})),
        controller=ControllerConfig(
            lateral_controller=controller_raw.get("lateral_controller", "pure_pursuit"),
            pure_pursuit=PurePursuitConfig(**controller_raw.get("pure_pursuit", {})),
            stanley=StanleyConfig(**controller_raw.get("stanley", {})),
            longitudinal_pid=PidConfig(**controller_raw.get("longitudinal_pid", {})),
            max_steering=controller_raw.get("max_steering", 1.0),
            max_throttle=controller_raw.get("max_throttle", 1.0),
            max_brake=controller_raw.get("max_brake", 1.0),
            brake_deadband=controller_raw.get("brake_deadband", 0.05),
        ),
        safety=SafetyConfig(**raw.get("safety", {})),
        live_input=LiveInputConfig(**raw.get("live_input", {})),
        ros_output=RosOutputConfig(**raw.get("ros_output", {})),
        legacy_serial_bridge=LegacySerialBridgeConfig(**raw.get("legacy_serial_bridge", {})),
        competition_profile=CompetitionProfileConfig(**raw.get("competition_profile", {})),
        optional_ego_topics=OptionalEgoTopicsConfig(**raw.get("optional_ego_topics", {})),
        vehicle_status=VehicleStatusConfig(**raw.get("vehicle_status", {})),
        competition_status=_build_diagnostics_input_config(raw.get("competition_status", {})),
        collision_data=_build_diagnostics_input_config(raw.get("collision_data", {})),
        morai_udp_reference=MoraiUdpReferenceConfig(**raw.get("morai_udp_reference", {})),
        udp_output=UdpOutputConfig(**raw.get("udp_output", {})),
        logging=LoggingConfig(**raw.get("logging", {})),
        replay=ReplayConfig(**raw.get("replay", {})),
    )


def load_competition_config(path: str | Path) -> CompetitionConfig:
    """Load and validate a competition config from JSON."""

    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    config = _build_config(raw)
    config.validate()
    return config


def save_competition_config(path: str | Path, config: CompetitionConfig) -> None:
    """Persist a competition config to JSON."""

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(config.to_dict(), indent=2), encoding="utf-8")


def competition_profile_diagnostics(config: CompetitionConfig) -> dict[str, Any]:
    """Return structured competition metadata for debug snapshots."""

    profile = config.competition_profile
    return {
        "competition_name": profile.competition_name,
        "map_name": profile.map_name,
        "vehicle_model": profile.vehicle_model,
        "wheelbase_m": profile.wheelbase_m,
        "ros_distro": profile.ros_distro,
        "os_version": profile.os_version,
        "desktop_only": profile.desktop_only,
        "rosbridge_optional": profile.rosbridge_optional,
        "sensor_limits": {
            "gps": profile.max_gps,
            "imu": profile.max_imu,
            "camera": profile.max_camera,
            "lidar": profile.max_lidar,
        },
        "camera_pitch_limit_deg": profile.camera_pitch_limit_deg,
    }


def runtime_policy_diagnostics(config: CompetitionConfig) -> dict[str, Any]:
    """Expose runtime policy decisions that operators should verify on site."""

    profile = config.competition_profile
    primary_output_path = "debug_only"
    if config.legacy_serial_bridge.enabled and config.legacy_serial_bridge.publish_enabled:
        primary_output_path = "legacy_serial_bridge"
    elif config.ros_output.publish_actuation:
        primary_output_path = "direct_actuation"
    return {
        "command_mode": config.ros_output.command_mode,
        "pedal_mode": config.ros_output.command_mode == "pedal",
        "primary_output_path": primary_output_path,
        "direct_actuation_enabled": config.ros_output.publish_actuation,
        "direct_actuation_topic": config.ros_output.actuation_topic,
        "direct_actuation_message_type": config.ros_output.actuation_message_type,
        "direct_actuation_longitudinal_type": 1 if config.ros_output.command_mode == "pedal" else 2,
        "direct_actuation_uses_accel_brake": config.ros_output.command_mode == "pedal",
        "legacy_bridge_enabled": config.legacy_serial_bridge.enabled,
        "legacy_bridge_publish_enabled": config.legacy_serial_bridge.publish_enabled,
        "legacy_bridge_topic": config.legacy_serial_bridge.topic,
        "vehicle_status_subscriber_enabled": config.vehicle_status.enabled,
        "vehicle_status_topic": config.vehicle_status.topic,
        "competition_status_subscriber_enabled": config.competition_status.enabled,
        "collision_data_subscriber_enabled": config.collision_data.enabled,
        "controls_gear_mode": profile.participant_controls_gear_mode,
        "controls_external_mode": profile.participant_controls_external_mode,
        "gear_mode_policy": (
            "operator_managed"
            if not profile.participant_controls_gear_mode and not profile.participant_controls_external_mode
            else "team_managed"
        ),
        "initial_vehicle_state": profile.initial_vehicle_state,
        "operator_transition_state": profile.operator_transition_state,
    }


def morai_udp_reference_diagnostics(config: CompetitionConfig) -> dict[str, Any]:
    """Expose non-binding MORAI UDP reference values for operator diagnostics."""

    reference = config.morai_udp_reference
    return {
        "user_ip": reference.user_ip,
        "host_ip": reference.host_ip,
        "multi_ip": reference.multi_ip,
        "ctrl_cmd_host_port": reference.ctrl_cmd_host_port,
        "ctrl_cmd_user_port": reference.ctrl_cmd_user_port,
        "vehicle_status_host_port": reference.vehicle_status_host_port,
        "vehicle_status_user_port": reference.vehicle_status_user_port,
        "competition_status_host_port": reference.competition_status_host_port,
        "competition_status_user_port": reference.competition_status_user_port,
        "collision_data_host_port": reference.collision_data_host_port,
        "collision_data_user_port": reference.collision_data_user_port,
        "imu_host_port": reference.imu_host_port,
        "imu_user_port": reference.imu_user_port,
    }
