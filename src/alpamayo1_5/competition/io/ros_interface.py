"""ROS1 publisher adapter with optional imports."""

from __future__ import annotations

import json

from alpamayo1_5.competition.contracts import DebugSnapshot, SafetyDecision
from alpamayo1_5.competition.runtime.config_competition import RosOutputConfig


class RosInterfaceUnavailable(RuntimeError):
    """Raised when ROS functionality is requested without rospy installed."""


class RosCommandPublisher:
    """ROS1 publisher wrapper with graceful dependency errors."""

    def __init__(self, config: RosOutputConfig):
        self.config = config
        try:
            import rospy  # type: ignore
            from std_msgs.msg import String  # type: ignore
        except ImportError as exc:
            raise RosInterfaceUnavailable(
                "rospy/std_msgs are not available in this environment"
            ) from exc

        self._rospy = rospy
        self._string_cls = String
        if not rospy.core.is_initialized():
            rospy.init_node(config.node_name, anonymous=True, disable_signals=True)
        self._command_pub = rospy.Publisher(config.command_topic, String, queue_size=1)
        self._debug_pub = rospy.Publisher(config.debug_topic, String, queue_size=1)

    def publish(self, decision: SafetyDecision) -> None:
        payload = {
            "frame_id": decision.frame_id,
            "timestamp_s": decision.timestamp_s,
            "steering": decision.command.steering,
            "throttle": decision.command.throttle,
            "brake": decision.command.brake,
            "target_speed_mps": decision.command.target_speed_mps,
            "intervention": decision.intervention,
            "risk_level": decision.risk_level,
            "flags": decision.safety_flags,
        }
        self._command_pub.publish(self._string_cls(data=json.dumps(payload)))

    def publish_debug(self, snapshot: DebugSnapshot) -> None:
        self._debug_pub.publish(self._string_cls(data=json.dumps(snapshot.to_dict())))
