"""Deterministic waypoint generation for the lightweight planner backend."""

from __future__ import annotations

import math


def infer_route_curvature(route_command: str | None) -> float:
    """Convert route keywords into a gentle signed curvature prior."""

    if not route_command:
        return 0.0
    text = route_command.lower()
    if "u-turn" in text or "uturn" in text:
        return 0.18
    if "left" in text:
        return 0.08
    if "right" in text:
        return -0.08
    return 0.0


class HeuristicWaypointHead:
    """Generate a stable local waypoint arc from route intent and ego speed."""

    def __init__(self, num_waypoints: int, dt_s: float):
        self.num_waypoints = num_waypoints
        self.dt_s = dt_s

    def generate(
        self,
        speed_mps: float,
        route_command: str | None = None,
        nominal_speed_mps: float = 3.0,
    ) -> list[tuple[float, float]]:
        """Generate local-frame waypoints in meters."""

        curvature = infer_route_curvature(route_command)
        speed = max(0.5, speed_mps if speed_mps > 0.1 else nominal_speed_mps)
        heading = 0.0
        x_m = 0.0
        y_m = 0.0
        waypoints: list[tuple[float, float]] = []

        for _ in range(self.num_waypoints):
            distance = speed * self.dt_s
            heading += curvature * distance
            x_m += distance * math.cos(heading)
            y_m += distance * math.sin(heading)
            waypoints.append((x_m, y_m))

        return waypoints
