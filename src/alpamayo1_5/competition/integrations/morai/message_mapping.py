"""Mapping helpers from ROS/MORAI messages into competition contracts."""

from __future__ import annotations

import math
from typing import Any

from alpamayo1_5.competition.contracts import CameraFrame, ControlCommand, GpsFix, ImuSample
from alpamayo1_5.competition.integrations.morai.image_decode import decode_ros_image_message
from alpamayo1_5.competition.integrations.morai.ros_message_utils import (
    get_header_frame_id,
    get_nested_attr,
    get_stamp_seconds,
    infer_message_type_name,
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Compute yaw from a quaternion."""

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _channels_from_encoding(encoding: str | None) -> int | None:
    if not encoding:
        return None
    normalized = encoding.lower()
    if "mono" in normalized:
        return 1
    if "rgba" in normalized or "bgra" in normalized:
        return 4
    if "rgb" in normalized or "bgr" in normalized:
        return 3
    return None


def map_camera_message(
    message: Any,
    camera_name: str,
    frame_id: int,
    message_type: str,
    expected_resolution: tuple[int, int] | None = None,
) -> CameraFrame:
    """Convert a ROS image-like message into a :class:`CameraFrame`."""

    timestamp_s = get_stamp_seconds(message)
    decoded_image, source_encoding = decode_ros_image_message(message, message_type=message_type)
    encoding = getattr(message, "encoding", None)
    width = getattr(message, "width", None)
    height = getattr(message, "height", None)
    channels = _channels_from_encoding(encoding)

    if width is None or height is None:
        if expected_resolution is not None:
            width, height = expected_resolution
    shape: tuple[int, ...] | None = None
    if width is not None and height is not None:
        if channels is None and getattr(message, "format", None):
            channels = 3
        if channels is not None:
            shape = (int(height), int(width), int(channels))

    metadata = {
        "ros_message_type": infer_message_type_name(message),
        "ros_frame_id": get_header_frame_id(message),
        "compressed_format": getattr(message, "format", None),
        "source_encoding": source_encoding,
        "decoded_rgb": True,
    }
    return CameraFrame(
        camera_id=camera_name,
        timestamp_s=timestamp_s,
        frame_id=frame_id,
        image=decoded_image,
        shape=tuple(int(dim) for dim in decoded_image.shape),
        encoding=encoding or getattr(message, "format", None) or "rgb8",
        metadata=metadata,
    )


def map_gps_message(message: Any) -> GpsFix:
    """Convert a ROS GPS-like message into :class:`GpsFix`."""

    timestamp_s = get_stamp_seconds(message)
    latitude_deg = get_nested_attr(message, "latitude", None)
    longitude_deg = get_nested_attr(message, "longitude", None)
    altitude_m = get_nested_attr(message, "altitude", 0.0)
    if latitude_deg is None:
        latitude_deg = get_nested_attr(message, "lat", 0.0)
    if longitude_deg is None:
        longitude_deg = get_nested_attr(message, "lon", 0.0)

    covariance = tuple(getattr(message, "position_covariance", []) or []) or None
    speed_mps = get_nested_attr(message, "speed", None)
    if speed_mps is None:
        speed_mps = get_nested_attr(message, "velocity", None)
    track_rad = get_nested_attr(message, "track_rad", None)
    if track_rad is None:
        track_rad = get_nested_attr(message, "heading", None)

    return GpsFix(
        timestamp_s=timestamp_s,
        latitude_deg=float(latitude_deg),
        longitude_deg=float(longitude_deg),
        altitude_m=float(altitude_m or 0.0),
        speed_mps=float(speed_mps) if speed_mps is not None else None,
        track_rad=float(track_rad) if track_rad is not None else None,
        covariance=covariance,
        metadata={
            "ros_message_type": infer_message_type_name(message),
            "ros_frame_id": get_header_frame_id(message),
        },
    )


def map_imu_message(message: Any) -> ImuSample:
    """Convert a ROS IMU-like message into :class:`ImuSample`."""

    timestamp_s = get_stamp_seconds(message)
    angular_velocity_z = get_nested_attr(message, "angular_velocity.z", None)
    linear_acceleration_x = get_nested_attr(message, "linear_acceleration.x", None)
    quat = get_nested_attr(message, "orientation", None)
    quaternion_xyzw: tuple[float, float, float, float] | None = None
    yaw_rad: float | None = None
    if quat is not None and all(hasattr(quat, attr) for attr in ("x", "y", "z", "w")):
        quaternion_xyzw = (
            float(quat.x),
            float(quat.y),
            float(quat.z),
            float(quat.w),
        )
        yaw_rad = _yaw_from_quaternion(*quaternion_xyzw)

    return ImuSample(
        timestamp_s=timestamp_s,
        yaw_rad=yaw_rad,
        yaw_rate_rps=float(angular_velocity_z) if angular_velocity_z is not None else None,
        accel_mps2=float(linear_acceleration_x) if linear_acceleration_x is not None else None,
        quaternion_xyzw=quaternion_xyzw,
        metadata={
            "ros_message_type": infer_message_type_name(message),
            "ros_frame_id": get_header_frame_id(message),
        },
    )


def map_route_message(message: Any) -> str:
    """Extract route-command text from a ROS message."""

    for candidate in ("data", "command", "route_command", "text"):
        value = getattr(message, candidate, None)
        if value is not None:
            return str(value)
    return str(message)


def inspect_control_message_contract(message: Any, command_mode: str = "pedal") -> dict[str, Any]:
    """Inspect a MORAI control message instance for required compatibility fields."""

    long_mode_field = next(
        (field_name for field_name in ("longlCmdType", "longiCmdType") if hasattr(message, field_name)),
        None,
    )
    steering_field = next(
        (field_name for field_name in ("steering", "front_steer") if hasattr(message, field_name)),
        None,
    )
    contract = {
        "command_mode": command_mode,
        "message_type": infer_message_type_name(message),
        "longitudinal_mode_field": long_mode_field,
        "steering_field": steering_field,
        "rear_steer_field": "rear_steer" if hasattr(message, "rear_steer") else None,
        "accel_field": "accel" if hasattr(message, "accel") else None,
        "brake_field": "brake" if hasattr(message, "brake") else None,
        "velocity_field": "velocity" if hasattr(message, "velocity") else None,
        "acceleration_field": "acceleration" if hasattr(message, "acceleration") else None,
        "missing_fields": [],
    }
    if long_mode_field is None:
        contract["missing_fields"].append("longlCmdType|longiCmdType")
    if steering_field is None:
        contract["missing_fields"].append("steering|front_steer")
    if command_mode == "pedal":
        if contract["accel_field"] is None:
            contract["missing_fields"].append("accel")
        if contract["brake_field"] is None:
            contract["missing_fields"].append("brake")
    elif command_mode == "velocity":
        if contract["velocity_field"] is None:
            contract["missing_fields"].append("velocity")
    else:
        contract["missing_fields"].append("unsupported_command_mode:%s" % command_mode)
    contract["compatible"] = not contract["missing_fields"]
    return contract


def validate_control_message_contract(message: Any, command_mode: str = "pedal") -> dict[str, Any]:
    """Validate the target MORAI message contract and raise on mismatch."""

    contract = inspect_control_message_contract(message, command_mode=command_mode)
    if contract["missing_fields"]:
        raise ValueError(
            "CtrlCmd contract mismatch for %s mode on %s. Missing fields: %s"
            % (
                command_mode,
                contract["message_type"],
                ", ".join(contract["missing_fields"]),
            )
        )
    return contract


def populate_control_message(
    message: Any,
    command: ControlCommand,
    command_mode: str = "pedal",
) -> Any:
    """Populate a ROS command message from a runtime control command."""

    contract = validate_control_message_contract(message, command_mode=command_mode)
    setattr(message, str(contract["longitudinal_mode_field"]), 1 if command_mode == "pedal" else 2)
    setattr(message, str(contract["steering_field"]), float(command.steering))
    if contract["rear_steer_field"] is not None and getattr(message, "rear_steer", None) is not None:
        setattr(message, str(contract["rear_steer_field"]), 0.0)
    if contract["accel_field"] is not None:
        setattr(message, str(contract["accel_field"]), float(command.throttle))
    if contract["brake_field"] is not None:
        setattr(message, str(contract["brake_field"]), float(command.brake))
    if command_mode == "velocity" and contract["velocity_field"] is not None:
        setattr(message, str(contract["velocity_field"]), float(command.target_speed_mps) * 3.6)
    if contract["acceleration_field"] is not None:
        setattr(message, str(contract["acceleration_field"]), float(command.throttle))
    return message
