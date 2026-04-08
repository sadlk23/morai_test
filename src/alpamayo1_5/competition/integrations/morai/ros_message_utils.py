"""ROS message import and timestamp helpers for MORAI integration."""

from __future__ import annotations

from importlib import import_module
from time import time
from typing import Any


class MoraiIntegrationUnavailable(RuntimeError):
    """Raised when required ROS or MORAI pieces are unavailable."""


def import_rospy() -> Any:
    """Import rospy lazily so dry-run/test paths stay dependency-light."""

    try:
        import rospy  # type: ignore
    except ImportError as exc:  # pragma: no cover - exercised only on ROS hosts
        raise MoraiIntegrationUnavailable(
            "rospy is not available in this environment. "
            "Source the ROS1 Noetic workspace before live MORAI bring-up."
        ) from exc
    return rospy


def _message_import_hint(message_type: str) -> str:
    if message_type == "morai_msgs/CtrlCmd":
        return (
            "This usually means the active catkin workspace does not contain the expected "
            "morai_msgs, the wrong overlay is sourced, or the venue workspace drifted. "
            "Run `rosmsg show morai_msgs/CtrlCmd` and verify the active workspace contract "
            "before enabling direct actuation."
        )
    if message_type.startswith("morai_msgs/"):
        return (
            "Verify that the active catkin workspace contains the expected morai_msgs package "
            "and that the correct ROS overlay is sourced."
        )
    return "Verify that the required ROS message package exists in the sourced workspace."


def import_message_class(message_type: str) -> type[Any]:
    """Import a ROS message class from ``package/MessageName`` syntax."""

    if "/" not in message_type:
        raise MoraiIntegrationUnavailable(
            f"Unsupported ROS message type format: {message_type}. Expected package/MessageName."
        )
    package_name, class_name = message_type.split("/", maxsplit=1)
    try:
        module = import_module(f"{package_name}.msg")
        return getattr(module, class_name)
    except (ImportError, AttributeError) as exc:  # pragma: no cover - ROS host dependent
        raise MoraiIntegrationUnavailable(
            f"ROS message class {message_type} is not available in this environment. "
            f"{_message_import_hint(message_type)}"
        ) from exc


def infer_message_type_name(message: Any) -> str:
    """Return a best-effort ``package/Message`` identifier for diagnostics."""

    cls = message.__class__
    module = getattr(cls, "__module__", "")
    package = module.split(".")[0] if module else "unknown"
    return f"{package}/{cls.__name__}"


def get_nested_attr(obj: Any, path: str, default: Any = None) -> Any:
    """Read ``a.b.c`` style attributes without raising."""

    current = obj
    for part in path.split("."):
        if current is None or not hasattr(current, part):
            return default
        current = getattr(current, part)
    return current


def get_stamp_seconds(message: Any, fallback: float | None = None) -> float:
    """Extract header stamp seconds from a ROS message or return a fallback."""

    stamp = get_nested_attr(message, "header.stamp", None)
    if stamp is not None:
        if hasattr(stamp, "to_sec"):
            return float(stamp.to_sec())
        secs = getattr(stamp, "secs", None)
        nsecs = getattr(stamp, "nsecs", None)
        if secs is not None:
            return float(secs) + float(nsecs or 0) * 1e-9
    return float(fallback if fallback is not None else time())


def get_header_frame_id(message: Any) -> str:
    """Extract the ROS header frame id when present."""

    return str(get_nested_attr(message, "header.frame_id", "") or "")
