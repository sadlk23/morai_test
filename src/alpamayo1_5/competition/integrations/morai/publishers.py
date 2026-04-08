"""ROS publishers for MORAI actuation and debug output."""

from __future__ import annotations

import json
import logging
from typing import Any

from alpamayo1_5.competition.contracts import DebugSnapshot, SafetyDecision
from alpamayo1_5.competition.integrations.morai.message_mapping import (
    inspect_control_message_contract,
    populate_control_message,
    validate_control_message_contract,
)
from alpamayo1_5.competition.integrations.morai.ros_message_utils import (
    import_message_class,
    import_rospy,
)
from alpamayo1_5.competition.runtime.config_competition import RosOutputConfig

logger = logging.getLogger(__name__)


class MoraiActuationContractError(RuntimeError):
    """Raised when MORAI CtrlCmd fields do not match runtime expectations."""


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
        self.config = config
        if config.require_actuation_arm and not config.actuation_armed:
            raise ValueError(
                "MoraiCtrlCmdPublisher requires ros_output.actuation_armed=true when require_actuation_arm=true"
            )
        self._rospy = import_rospy()
        if not self._rospy.core.is_initialized():  # pragma: no cover - ROS host dependent
            self._rospy.init_node(config.node_name, anonymous=True, disable_signals=True)
        self._message_cls = import_message_class(config.actuation_message_type)
        self._command_mode = config.command_mode
        self._contract_summary = self._startup_self_check()
        self._publisher = self._rospy.Publisher(
            config.actuation_topic,
            self._message_cls,
            queue_size=config.queue_size,
        )
        logger.info(
            "Initialized MORAI actuation publisher topic=%s message_type=%s command_mode=%s armed=%s",
            config.actuation_topic,
            config.actuation_message_type,
            config.command_mode,
            config.actuation_armed,
        )
        logger.info("MORAI actuation contract=%s", self._contract_summary)
        logger.info(
            "Direct MORAI actuation only publishes steering/longitudinal command fields; "
            "simulator gear and ExternalCtrl state remain operator-managed."
        )

    def _startup_self_check(self) -> dict[str, Any]:
        """Fail fast when the target CtrlCmd contract is incompatible."""

        message = self._message_cls()
        contract = inspect_control_message_contract(message, command_mode=self._command_mode)
        try:
            validate_control_message_contract(message, command_mode=self._command_mode)
        except ValueError as exc:
            expected_mode = (
                "longi type 1 pedal mode (accel/brake + steering)"
                if self._command_mode == "pedal"
                else "velocity mode (velocity + steering)"
            )
            raise MoraiActuationContractError(
                "Direct actuation self-check failed for %s on topic %s: %s. "
                "Competition direct actuation expects %s. "
                "Run `rosmsg show %s` and confirm pedal mode needs %s or velocity mode needs %s."
                % (
                    self.config.actuation_message_type,
                    self.config.actuation_topic,
                    exc,
                    expected_mode,
                    self.config.actuation_message_type,
                    "longlCmdType|longiCmdType + steering|front_steer + accel + brake",
                    "longlCmdType|longiCmdType + steering|front_steer + velocity",
                )
            ) from exc
        return contract

    def _validate_message_shape(self, message: Any) -> None:
        """Fail early if the configured actuation message lacks expected fields."""

        validate_control_message_contract(message, command_mode=self._command_mode)

    def build_message(self, decision: SafetyDecision) -> Any:
        """Build the ROS actuation message without publishing it."""

        message = self._message_cls()
        try:
            self._validate_message_shape(message)
            message = populate_control_message(
                message,
                decision.command,
                command_mode=self._command_mode,
            )
        except ValueError as exc:
            raise MoraiActuationContractError(
                "Failed to build direct actuation message frame=%s topic=%s type=%s mode=%s: %s"
                % (
                    decision.frame_id,
                    self.config.actuation_topic,
                    self.config.actuation_message_type,
                    self._command_mode,
                    exc,
                )
            ) from exc
        return message

    def publish(self, decision: SafetyDecision) -> None:
        self._publisher.publish(self.build_message(decision))
