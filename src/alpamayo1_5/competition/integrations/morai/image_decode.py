"""ROS image decoding helpers for live MORAI integration."""

from __future__ import annotations

from io import BytesIO
from typing import Any

import numpy as np


SUPPORTED_RAW_ENCODINGS = {"rgb8", "bgr8", "rgba8", "bgra8", "mono8"}
SUPPORTED_COMPRESSED_MESSAGE_TYPES = {"sensor_msgs/CompressedImage"}


def _ensure_uint8_length(data: bytes, expected_size: int, message_label: str) -> np.ndarray:
    """Convert a ROS byte payload into a flat uint8 array with size checks."""

    buffer = np.frombuffer(data, dtype=np.uint8)
    if buffer.size < expected_size:
        raise ValueError(
            "%s payload shorter than expected: got %d bytes, expected at least %d"
            % (message_label, int(buffer.size), int(expected_size))
        )
    return buffer


def _normalize_rgb(array: np.ndarray, encoding: str) -> np.ndarray:
    """Normalize common ROS image encodings into HWC RGB uint8 arrays."""

    normalized = encoding.lower()
    if normalized == "rgb8":
        return array
    if normalized == "bgr8":
        return array[..., ::-1]
    if normalized == "rgba8":
        return array[..., :3]
    if normalized == "bgra8":
        return array[..., [2, 1, 0]]
    if normalized == "mono8":
        return np.repeat(array[..., None], 3, axis=2)
    raise ValueError("unsupported raw image encoding: %s" % encoding)


def decode_raw_image_message(message: Any) -> tuple[np.ndarray, str]:
    """Decode a ``sensor_msgs/Image``-like payload into an RGB uint8 array."""

    encoding = str(getattr(message, "encoding", "") or "").lower()
    if encoding not in SUPPORTED_RAW_ENCODINGS:
        raise ValueError("unsupported raw image encoding: %s" % (encoding or "<empty>"))

    width = int(getattr(message, "width", 0) or 0)
    height = int(getattr(message, "height", 0) or 0)
    if width <= 0 or height <= 0:
        raise ValueError("raw image message must have positive width and height")

    channels = 1 if encoding == "mono8" else (4 if "a" in encoding else 3)
    expected_row_bytes = width * channels
    step = int(getattr(message, "step", expected_row_bytes) or expected_row_bytes)
    if step < expected_row_bytes:
        raise ValueError(
            "raw image step is smaller than width * channels: %d < %d" % (step, expected_row_bytes)
        )

    payload = _ensure_uint8_length(
        getattr(message, "data", b"") or b"",
        step * height,
        "raw image",
    )
    row_major = payload[: step * height].reshape(height, step)
    useful = row_major[:, :expected_row_bytes]
    if channels == 1:
        image = useful.reshape(height, width)
    else:
        image = useful.reshape(height, width, channels)
    return _normalize_rgb(image, encoding), encoding


def decode_compressed_image_message(message: Any) -> tuple[np.ndarray, str]:
    """Decode a ``sensor_msgs/CompressedImage``-like payload into an RGB array."""

    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError(
            "Compressed image decoding requires Pillow. Install the 'pillow' dependency "
            "or use sensor_msgs/Image topics."
        ) from exc
    image = Image.open(BytesIO(getattr(message, "data", b"") or b""))
    rgb = image.convert("RGB")
    return np.asarray(rgb, dtype=np.uint8), str(getattr(message, "format", "") or "compressed")


def decode_ros_image_message(message: Any, message_type: str) -> tuple[np.ndarray, str]:
    """Decode a ROS image message into a consistent HWC RGB uint8 representation."""

    if message_type == "sensor_msgs/Image":
        return decode_raw_image_message(message)
    if message_type == "sensor_msgs/CompressedImage":
        return decode_compressed_image_message(message)
    raise ValueError("unsupported image message type: %s" % message_type)
