"""Stanley lateral control."""

from __future__ import annotations

import math

from alpamayo1_5.competition.runtime.config_competition import StanleyConfig


class StanleyController:
    """Compute steering from local-frame cross-track and heading error."""

    def __init__(self, config: StanleyConfig):
        self.config = config

    def command(self, waypoints_xy: list[tuple[float, float]], speed_mps: float) -> float:
        """Compute a Stanley steering command."""

        if len(waypoints_xy) < 2:
            return 0.0

        nearest = min(waypoints_xy, key=lambda point: math.hypot(point[0], point[1]))
        next_point = waypoints_xy[min(1, len(waypoints_xy) - 1)]
        path_heading = math.atan2(next_point[1] - nearest[1], next_point[0] - nearest[0])
        heading_error = math.atan2(math.sin(path_heading), math.cos(path_heading))
        cross_track = nearest[1]
        correction = math.atan2(
            self.config.gain * cross_track,
            self.config.softening_speed_mps + max(0.0, speed_mps),
        )
        return heading_error + correction
