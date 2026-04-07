"""Controller stack for converting plans into actuation commands."""

from __future__ import annotations

import math

from alpamayo1_5.competition.contracts import ControlCommand, PlanResult, PlannerInput
from alpamayo1_5.competition.controllers.pid import PIDController
from alpamayo1_5.competition.controllers.pure_pursuit import PurePursuitController
from alpamayo1_5.competition.controllers.stanley import StanleyController
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig


class ControllerRuntime:
    """Owns lateral and longitudinal control for the competition stack."""

    def __init__(self, config: CompetitionConfig):
        self.config = config
        self.pure_pursuit = PurePursuitController(config.controller.pure_pursuit)
        self.stanley = StanleyController(config.controller.stanley)
        self.speed_pid = PIDController(
            kp=config.controller.longitudinal_pid.kp,
            ki=config.controller.longitudinal_pid.ki,
            kd=config.controller.longitudinal_pid.kd,
            integral_limit=config.controller.longitudinal_pid.integral_limit,
            output_limit=config.controller.longitudinal_pid.output_limit,
        )

    def _lateral_command(self, waypoints_xy: list[tuple[float, float]], speed_mps: float) -> float:
        if self.config.controller.lateral_controller == "stanley":
            return self.stanley.command(waypoints_xy, speed_mps)
        return self.pure_pursuit.command(waypoints_xy, speed_mps)

    def compute(self, planner_input: PlannerInput, plan: PlanResult, dt_s: float) -> ControlCommand:
        """Convert a waypoint/speed plan into low-level steering/throttle/brake."""

        if not plan.valid:
            self.speed_pid.reset()
            return ControlCommand(
                frame_id=planner_input.frame_id,
                timestamp_s=planner_input.timestamp_s,
                steering=0.0,
                throttle=0.0,
                brake=1.0,
                target_speed_mps=0.0,
                valid=False,
                saturated=True,
                source_plan=plan.planner_name,
                reason="invalid_plan",
            )

        current_speed = planner_input.ego_state.speed_mps
        if not math.isfinite(current_speed) or not math.isfinite(plan.target_speed_mps):
            self.speed_pid.reset()
            return ControlCommand(
                frame_id=planner_input.frame_id,
                timestamp_s=planner_input.timestamp_s,
                steering=0.0,
                throttle=0.0,
                brake=1.0,
                target_speed_mps=0.0,
                valid=False,
                saturated=True,
                source_plan=plan.planner_name,
                reason="non_finite_controller_input",
            )
        steering = self._lateral_command(plan.waypoints_xy, current_speed)
        speed_error = plan.target_speed_mps - current_speed
        accel_cmd = self.speed_pid.step(speed_error, dt_s)
        throttle = max(0.0, accel_cmd)
        brake = max(0.0, -accel_cmd)
        if brake < self.config.controller.brake_deadband:
            brake = 0.0

        saturated = False
        steering = max(-self.config.controller.max_steering, min(self.config.controller.max_steering, steering))
        if throttle > self.config.controller.max_throttle:
            throttle = self.config.controller.max_throttle
            saturated = True
        if brake > self.config.controller.max_brake:
            brake = self.config.controller.max_brake
            saturated = True

        return ControlCommand(
            frame_id=planner_input.frame_id,
            timestamp_s=planner_input.timestamp_s,
            steering=steering,
            throttle=throttle,
            brake=brake,
            target_speed_mps=plan.target_speed_mps,
            valid=True,
            saturated=saturated,
            source_plan=plan.planner_name,
            diagnostics={
                "speed_error_mps": speed_error,
                "current_speed_mps": current_speed,
            },
        )
