"""Simple PID controller."""

from __future__ import annotations


class PIDController:
    """Longitudinal PID for target speed tracking."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float,
        output_limit: float,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self) -> None:
        """Reset the integrator state."""

        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error: float, dt_s: float) -> float:
        """Advance the PID controller by one step."""

        dt_s = max(1e-3, dt_s)
        self.integral += error * dt_s
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.prev_error) / dt_s
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))
