"""Low-level command clipping and numeric guards."""

from __future__ import annotations

import math

from alpamayo1_5.competition.contracts import ControlCommand
from alpamayo1_5.competition.runtime.config_competition import ControllerConfig, SafetyConfig


def _finite(value: float) -> bool:
    return math.isfinite(value)


class CommandGuard:
    """Clamp and validate controller outputs."""

    def __init__(self, controller_config: ControllerConfig, safety_config: SafetyConfig):
        self.controller_config = controller_config
        self.safety_config = safety_config

    def guard(self, command: ControlCommand) -> tuple[ControlCommand, list[str]]:
        flags: list[str] = []
        if not all(
            _finite(value)
            for value in [command.steering, command.throttle, command.brake, command.target_speed_mps]
        ):
            command.steering = 0.0
            command.throttle = 0.0
            command.brake = self.safety_config.emergency_brake_value
            command.target_speed_mps = 0.0
            command.valid = False
            command.reason = "non_finite_command"
            flags.append("non_finite_command")
            return command, flags

        if command.throttle > 0.0 and command.brake > 0.0:
            if command.brake >= command.throttle:
                command.throttle = 0.0
            else:
                command.brake = 0.0
            flags.append("conflicting_longitudinal_command")

        if abs(command.steering) > self.safety_config.max_abs_steering:
            command.steering = max(
                -self.safety_config.max_abs_steering,
                min(self.safety_config.max_abs_steering, command.steering),
            )
            command.saturated = True
            flags.append("steering_clipped")

        if command.throttle < 0.0:
            command.throttle = 0.0
            flags.append("throttle_clipped")
        if command.brake < 0.0:
            command.brake = 0.0
            flags.append("brake_clipped")
        if command.throttle > self.controller_config.max_throttle:
            command.throttle = self.controller_config.max_throttle
            command.saturated = True
            flags.append("throttle_clipped")
        if command.brake > self.controller_config.max_brake:
            command.brake = self.controller_config.max_brake
            command.saturated = True
            flags.append("brake_clipped")
        return command, flags
