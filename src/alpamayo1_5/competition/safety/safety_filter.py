"""Safety filtering and fallback management."""

from __future__ import annotations

from dataclasses import replace

from alpamayo1_5.competition.contracts import ControlCommand, PlanResult, PlannerInput, SafetyDecision
from alpamayo1_5.competition.safety.command_guard import CommandGuard
from alpamayo1_5.competition.safety.risk_rules import assess_plan_risk
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig


class SafetyFilter:
    """Final safety layer before command publishing."""

    def __init__(self, config: CompetitionConfig):
        self.config = config
        self.guard = CommandGuard(config.controller, config.safety)
        self._last_valid_command: ControlCommand | None = None
        self._last_valid_timestamp_s: float | None = None

    def _stop_command(self, planner_input: PlannerInput, reason: str) -> ControlCommand:
        return ControlCommand(
            frame_id=planner_input.frame_id,
            timestamp_s=planner_input.timestamp_s,
            steering=0.0,
            throttle=0.0,
            brake=self.config.safety.emergency_brake_value,
            target_speed_mps=0.0,
            valid=False,
            saturated=True,
            reason=reason,
        )

    def apply(
        self,
        planner_input: PlannerInput,
        plan: PlanResult,
        command: ControlCommand,
    ) -> SafetyDecision:
        """Safety-filter a controller command and choose fallbacks if needed."""

        risk_score, flags = assess_plan_risk(planner_input, plan, self.config.safety)
        intervention = "none"
        fallback_used = False

        if not plan.valid or "invalid_waypoints" in flags or "missing_required_sensors" in flags:
            age_ok = (
                self._last_valid_command is not None
                and self._last_valid_timestamp_s is not None
                and planner_input.timestamp_s - self._last_valid_timestamp_s
                <= self.config.safety.reuse_last_command_horizon_s
            )
            if age_ok:
                command = replace(
                    self._last_valid_command,
                    frame_id=planner_input.frame_id,
                    timestamp_s=planner_input.timestamp_s,
                    reason="reuse_last_valid_command",
                )
                command.throttle = min(command.throttle, 0.2)
                command.brake = max(command.brake, 0.0)
                intervention = "reuse_last_valid_command"
                fallback_used = True
                flags.append("fallback_reuse_last_valid")
            else:
                command = self._stop_command(planner_input, "invalid_or_missing_plan")
                intervention = "emergency_stop"
                fallback_used = True
        elif "low_confidence" in flags or "stale_sensors" in flags or "high_curvature" in flags:
            command.target_speed_mps = min(command.target_speed_mps, self.config.safety.conservative_speed_mps)
            command.throttle = min(command.throttle, 0.2)
            if command.target_speed_mps <= 0.2 or "high_stop_probability" in flags:
                command.throttle = 0.0
                command.brake = max(command.brake, 0.4)
            intervention = "conservative_slowdown"

        command, clip_flags = self.guard.guard(command)
        flags.extend(clip_flags)

        if command.valid and not fallback_used:
            self._last_valid_command = ControlCommand(
                frame_id=command.frame_id,
                timestamp_s=command.timestamp_s,
                steering=command.steering,
                throttle=command.throttle,
                brake=command.brake,
                target_speed_mps=command.target_speed_mps,
                valid=command.valid,
                saturated=command.saturated,
                source_plan=command.source_plan,
                reason=command.reason,
                diagnostics=dict(command.diagnostics),
            )
            self._last_valid_timestamp_s = command.timestamp_s

        if not command.valid and intervention == "none":
            intervention = "invalid_command_stop"

        if intervention == "none" and clip_flags:
            intervention = "command_clipped"

        risk_level = "nominal"
        if risk_score >= 0.9:
            risk_level = "critical"
        elif risk_score >= 0.6:
            risk_level = "elevated"

        return SafetyDecision(
            frame_id=planner_input.frame_id,
            timestamp_s=planner_input.timestamp_s,
            command=command,
            intervention=intervention,
            risk_level=risk_level,
            safety_flags=flags,
            fallback_used=fallback_used,
            diagnostics={
                "plan_confidence": plan.confidence,
                "plan_risk_score": plan.risk_score,
                "evaluated_risk_score": risk_score,
            },
        )
