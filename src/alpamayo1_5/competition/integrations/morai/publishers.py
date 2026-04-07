"""ROS publishers for MORAI actuation and debug output."""

from __future__ import annotations

import json
from typing import Any

from alpamayo1_5.competition.contracts import DebugSnapshot, SafetyDecision
from alpamayo1_5.competition.integrations.morai.message_mapping import populate_control_message
from alpamayo1_5.competition.integrations.morai.ros_message_utils import (
    import_message_class,
    import_rospy,
)
from alpamayo1_5.competition.runtime.config_competition import RosOutputConfig


class RosDebugSnapshotPublisher:
    """Publish structured debug snapshots as JSON strings."""

    def __init__(self, config: RosOutputConfig):
        self._rospy = import_rospy()
        if not self._rospy.core.is_initialized():  # pragma: no cover - ROS host dependent
            self._rospy.init_node(config.node_name, anonymous=True, disable_signals=True)
        self._string_cls = import_message_class("std_msgs/String")
        self._debug_pub = self._rospy.Publisher(
            config.debug_topic,
            self._string_cls,
            queue_size=config.queue_size,
        )

    def publish_debug(self, snapshot: DebugSnapshot) -> None:
        self._debug_pub.publish(self._string_cls(data=json.dumps(snapshot.to_dict())))


class RosJsonCommandPublisher:
    """Publish the final safety decision as a JSON string for observability."""

    def __init__(self, config: RosOutputConfig):
        self._rospy = import_rospy()
        if not self._rospy.core.is_initialized():  # pragma: no cover - ROS host dependent
            self._rospy.init_node(config.node_name, anonymous=True, disable_signals=True)
        self._string_cls = import_message_class("std_msgs/String")
        self._command_pub = self._rospy.Publisher(
            config.command_topic,
            self._string_cls,
            queue_size=config.queue_size,
        )

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


class MoraiCtrlCmdPublisher:
    """Publish actual actuation messages for MORAI-compatible drive loops."""

    def __init__(self, config: RosOutputConfig):
        self._rospy = import_rospy()
        if not self._rospy.core.is_initialized():  # pragma: no cover - ROS host dependent
            self._rospy.init_node(config.node_name, anonymous=True, disable_signals=True)
        self._message_cls = import_message_class(config.actuation_message_type)
        self._command_mode = config.command_mode
        self._publisher = self._rospy.Publisher(
            config.actuation_topic,
            self._message_cls,
            queue_size=config.queue_size,
        )

    def build_message(self, decision: SafetyDecision) -> Any:
        """Build the ROS actuation message without publishing it."""

        return populate_control_message(
            self._message_cls(),
            decision.command,
            command_mode=self._command_mode,
        )

    def publish(self, decision: SafetyDecision) -> None:
        self._publisher.publish(self.build_message(decision))
