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


@dataclass(slots=True)
class GpsConfig:
    """GPS configuration."""

    topic: str = "/alpamayo/gps"
    message_type: str = "sensor_msgs/NavSatFix"
    required: bool = True
    max_staleness_s: float = 0.2


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
    actuation_topic: str = "/ctrl_cmd"
    actuation_message_type: str = "morai_msgs/CtrlCmd"
    command_mode: str = "pedal"


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
        if self.live_input.warn_throttle_s <= 0:
            errors.append("live_input.warn_throttle_s must be > 0")
        if self.udp_output.enabled and self.udp_output.port <= 0:
            errors.append("udp_output.port must be > 0 when UDP is enabled")
        if self.output_mode == "ros" and not self.ros_output.enabled:
            errors.append("output_mode=ros requires ros_output.enabled=true")
        if self.output_mode == "udp" and not self.udp_output.enabled:
            errors.append("output_mode=udp requires udp_output.enabled=true")
        if self.ros_output.queue_size <= 0:
            errors.append("ros_output.queue_size must be > 0")
        if self.output_mode in {"ros", "dual"} and self.ros_output.enabled:
            if not (
                self.ros_output.publish_command_json
                or self.ros_output.publish_debug_json
                or self.ros_output.publish_actuation
            ):
                errors.append(
                    "ros_output.enabled=true requires at least one of publish_command_json, "
                    "publish_debug_json, or publish_actuation"
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
        if self.ros_output.command_mode not in {"pedal", "velocity"}:
            errors.append("ros_output.command_mode must be pedal or velocity")
        if self.ros_output.command_mode == "velocity" and not self.ros_output.publish_actuation:
            errors.append("ros_output.command_mode=velocity requires publish_actuation=true")
        if self.planner.checkpoint_path is not None and not Path(self.planner.checkpoint_path).exists():
            errors.append(f"planner.checkpoint_path does not exist: {self.planner.checkpoint_path}")
        if self.planner.backend == "legacy_alpamayo" and not (
            self.planner.checkpoint_path or self.planner.legacy_model_id
        ):
            errors.append(
                "legacy_alpamayo backend requires planner.checkpoint_path or planner.legacy_model_id"
            )

        if errors:
            raise ValueError("Invalid competition config:\n- " + "\n- ".join(errors))

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable configuration dictionary."""

        return asdict(self)


def _build_camera_list(raw: list[dict[str, Any]]) -> list[CameraConfig]:
    return [CameraConfig(**item) for item in raw]


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
