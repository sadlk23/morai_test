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


def populate_control_message(
    message: Any,
    command: ControlCommand,
    command_mode: str = "pedal",
) -> Any:
    """Populate a ROS command message from a runtime control command."""

    if hasattr(message, "longlCmdType"):
        message.longlCmdType = 1 if command_mode == "pedal" else 2
    if hasattr(message, "steering"):
        message.steering = float(command.steering)
    if hasattr(message, "front_steer"):
        message.front_steer = float(command.steering)
    if hasattr(message, "rear_steer") and getattr(message, "rear_steer", None) is not None:
        message.rear_steer = 0.0
    if hasattr(message, "accel"):
        message.accel = float(command.throttle)
    if hasattr(message, "brake"):
        message.brake = float(command.brake)
    if command_mode == "velocity" and hasattr(message, "velocity"):
        message.velocity = float(command.target_speed_mps) * 3.6
    if hasattr(message, "acceleration"):
        message.acceleration = float(command.throttle)
    return message
