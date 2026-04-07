"""Typed runtime contracts for the competition stack."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


def _default_dict() -> dict[str, Any]:
    return {}


def _default_list() -> list[Any]:
    return []


@dataclass(slots=True)
class CameraFrame:
    """Single camera observation for one runtime tick."""

    camera_id: str
    timestamp_s: float
    frame_id: int
    image: Any | None = None
    shape: tuple[int, ...] | None = None
    encoding: str | None = None
    metadata: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class GpsFix:
    """GPS observation used by the state estimator."""

    timestamp_s: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0
    speed_mps: float | None = None
    track_rad: float | None = None
    covariance: tuple[float, ...] | None = None
    metadata: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class ImuSample:
    """IMU observation used by the state estimator."""

    timestamp_s: float
    yaw_rad: float | None = None
    yaw_rate_rps: float | None = None
    accel_mps2: float | None = None
    quaternion_xyzw: tuple[float, float, float, float] | None = None
    metadata: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class LidarPacket:
    """Optional LiDAR observation."""

    timestamp_s: float
    points: Any | None = None
    metadata: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class SensorPacket:
    """Raw sensor packet for one competition tick."""

    frame_id: int
    timestamp_s: float
    camera_frames: dict[str, CameraFrame] = field(default_factory=_default_dict)
    gps_fix: GpsFix | None = None
    imu_sample: ImuSample | None = None
    lidar_packet: LidarPacket | None = None
    route_command: str | None = None
    metadata: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class SynchronizedFrame:
    """Timestamp-aligned sensor view consumed by preprocessing."""

    frame_id: int
    timestamp_s: float
    camera_frames: dict[str, CameraFrame] = field(default_factory=_default_dict)
    gps_fix: GpsFix | None = None
    imu_sample: ImuSample | None = None
    lidar_packet: LidarPacket | None = None
    route_command: str | None = None
    missing_sensors: list[str] = field(default_factory=_default_list)
    stale_sensors: list[str] = field(default_factory=_default_list)
    sensor_age_s: dict[str, float] = field(default_factory=_default_dict)
    valid: bool = True
    diagnostics: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class EgoState:
    """Estimated ego state in a local planar frame."""

    timestamp_s: float
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    speed_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    accel_mps2: float = 0.0
    valid: bool = True
    diagnostics: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class PlannerInput:
    """Unified planner input after synchronization and preprocessing."""

    frame_id: int
    timestamp_s: float
    synchronized: SynchronizedFrame
    ego_state: EgoState
    route_command: str | None = None
    camera_order: list[str] = field(default_factory=_default_list)
    camera_mask: dict[str, bool] = field(default_factory=_default_dict)
    image_summary: dict[str, Any] = field(default_factory=_default_dict)
    fused_features: dict[str, Any] = field(default_factory=_default_dict)
    model_input_package: Any | None = None
    valid: bool = True
    diagnostics: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class ModelInputPackage:
    """Structured model-facing payload built from preprocessed runtime data."""

    frame_id: int
    timestamp_s: float
    camera_order: list[str] = field(default_factory=_default_list)
    camera_indices: list[int] = field(default_factory=_default_list)
    image_payloads: list[Any] = field(default_factory=_default_list)
    nav_text: str | None = None
    ego_history_xy: list[tuple[float, float]] = field(default_factory=_default_list)
    ego_speed_mps: float = 0.0
    target_resolution: tuple[int, int] | None = None
    valid: bool = True
    diagnostics: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class PlanResult:
    """Interpretable planner output."""

    frame_id: int
    timestamp_s: float
    planner_name: str
    waypoints_xy: list[tuple[float, float]]
    target_speed_mps: float
    confidence: float = 1.0
    stop_probability: float = 0.0
    risk_score: float = 0.0
    valid: bool = True
    used_fallback: bool = False
    diagnostics: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class ControlCommand:
    """Low-level command after controller conversion."""

    frame_id: int
    timestamp_s: float
    steering: float
    throttle: float
    brake: float
    target_speed_mps: float
    valid: bool = True
    saturated: bool = False
    source_plan: str | None = None
    reason: str | None = None
    diagnostics: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class SafetyDecision:
    """Final actuation decision after safety checks."""

    frame_id: int
    timestamp_s: float
    command: ControlCommand
    intervention: str = "none"
    risk_level: str = "nominal"
    safety_flags: list[str] = field(default_factory=_default_list)
    fallback_used: bool = False
    diagnostics: dict[str, Any] = field(default_factory=_default_dict)


@dataclass(slots=True)
class DebugSnapshot:
    """Structured debug payload for logs and offline replay."""

    frame_id: int
    timestamp_s: float
    stage_latency_ms: dict[str, float] = field(default_factory=_default_dict)
    fused_feature_summary: dict[str, Any] = field(default_factory=_default_dict)
    waypoints_xy: list[tuple[float, float]] = field(default_factory=_default_list)
    target_speed_mps: float = 0.0
    controller_output: dict[str, Any] = field(default_factory=_default_dict)
    safety_flags: list[str] = field(default_factory=_default_list)
    diagnostics: dict[str, Any] = field(default_factory=_default_dict)

    def to_dict(self) -> dict[str, Any]:
        """Convert the snapshot into a JSON-serializable dictionary."""

        return asdict(self)
