"""Legacy moo-compatible /Control/serial_data bridge publisher."""

from __future__ import annotations

import logging

from alpamayo1_5.competition.contracts import SafetyDecision
from alpamayo1_5.competition.integrations.morai.ros_message_utils import (
    import_message_class,
    import_rospy,
)
from alpamayo1_5.competition.runtime.config_competition import (
    LegacySerialBridgeConfig,
    RosOutputConfig,
)

logger = logging.getLogger(__name__)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def _should_e_stop(decision: SafetyDecision, config: LegacySerialBridgeConfig) -> float:
    if not config.e_stop_on_intervention:
        return 0.0
    intervention = (decision.intervention or "").lower()
    risk_level = (decision.risk_level or "").lower()
    if decision.fallback_used:
        return 1.0
    if "stop" in intervention or "emergency" in intervention:
        return 1.0
    if risk_level in {"high", "critical"}:
        return 1.0
    return 0.0


def build_legacy_serial_payload(
    decision: SafetyDecision,
    config: LegacySerialBridgeConfig,
    alive_counter: int = 0,
) -> list[float]:
    """Build moo-compatible serial_data payload from typed runtime output.

    Runtime brake is normalized in [0, 1]. This bridge scales it by
    ``config.brake_output_max`` so it can remain [0, 1] (default) or be mapped
    into [0, 200] when configured for ERP-style consumers.
    """

    speed_mps = max(0.0, float(decision.command.target_speed_mps))
    steer_rad = float(decision.command.steering)
    brake_normalized = _clamp(float(decision.command.brake), 0.0, 1.0)
    brake_out = brake_normalized * float(config.brake_output_max)
    alive_value = float(alive_counter if config.include_alive_counter else 0)
    return [
        float(config.default_control_mode),
        _should_e_stop(decision, config),
        float(config.default_gear),
        speed_mps,
        steer_rad,
        brake_out,
        alive_value,
    ]


class LegacySerialDataPublisher:
    """Publish legacy Float32MultiArray serial_data output."""

    def __init__(self, bridge_config: LegacySerialBridgeConfig, ros_output_config: RosOutputConfig):
        self._bridge_config = bridge_config
        if not bridge_config.enabled or not bridge_config.publish_enabled:
            raise ValueError("LegacySerialDataPublisher requires enabled=true and publish_enabled=true")
        self._rospy = import_rospy()
        if not self._rospy.core.is_initialized():  # pragma: no cover - ROS host dependent
            self._rospy.init_node(ros_output_config.node_name, anonymous=True, disable_signals=True)
        self._message_cls = import_message_class(bridge_config.message_type)
        self._publisher = self._rospy.Publisher(
            bridge_config.topic,
            self._message_cls,
            queue_size=ros_output_config.queue_size,
        )
        self._alive_counter = 0
        logger.info(
            "Initialized legacy serial bridge topic=%s type=%s include_alive=%s brake_output_max=%.1f",
            bridge_config.topic,
            bridge_config.message_type,
            bridge_config.include_alive_counter,
            bridge_config.brake_output_max,
        )

    def build_message(self, decision: SafetyDecision):
        message = self._message_cls()
        message.data = build_legacy_serial_payload(
            decision,
            self._bridge_config,
            alive_counter=self._alive_counter,
        )
        if self._bridge_config.include_alive_counter:
            self._alive_counter = (self._alive_counter + 1) % 256
        return message

    def publish(self, decision: SafetyDecision) -> None:
        self._publisher.publish(self.build_message(decision))
