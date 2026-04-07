"""ROS1 JSON debug publishers kept for generic runtime compatibility."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import DebugSnapshot, SafetyDecision
from alpamayo1_5.competition.integrations.morai.publishers import (
    RosDebugSnapshotPublisher,
    RosJsonCommandPublisher,
)
from alpamayo1_5.competition.runtime.config_competition import RosOutputConfig


class RosInterfaceUnavailable(RuntimeError):
    """Raised when ROS functionality is requested without rospy installed."""


class RosCommandPublisher:
    """Compatibility wrapper around separated JSON command/debug publishers."""

    def __init__(self, config: RosOutputConfig):
        self.config = config
        try:
            self._command_publisher = (
                RosJsonCommandPublisher(config) if config.publish_command_json else None
            )
            self._debug_publisher = (
                RosDebugSnapshotPublisher(config) if config.publish_debug_json else None
            )
        except RuntimeError as exc:
            raise RosInterfaceUnavailable(str(exc)) from exc

    def publish(self, decision: SafetyDecision) -> None:
        if self._command_publisher is not None:
            self._command_publisher.publish(decision)

    def publish_debug(self, snapshot: DebugSnapshot) -> None:
        if self._debug_publisher is not None:
            self._debug_publisher.publish_debug(snapshot)
