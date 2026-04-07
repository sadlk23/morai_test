"""Pure Pursuit lateral control."""

from __future__ import annotations

import math

from alpamayo1_5.competition.runtime.config_competition import PurePursuitConfig


class PurePursuitController:
    """Compute steering for a local waypoint path."""

    def __init__(self, config: PurePursuitConfig):
        self.config = config

    def _lookahead_distance(self, speed_mps: float) -> float:
        distance = self.config.min_lookahead_m + self.config.speed_to_lookahead_gain * speed_mps
        return max(self.config.min_lookahead_m, min(self.config.max_lookahead_m, distance))

    def command(self, waypoints_xy: list[tuple[float, float]], speed_mps: float) -> float:
        """Compute a steering command from local-frame waypoints."""

        if not waypoints_xy:
            return 0.0
        lookahead = self._lookahead_distance(speed_mps)
        chosen = waypoints_xy[-1]
        for point in waypoints_xy:
            if math.hypot(point[0], point[1]) >= lookahead:
                chosen = point
                break
        alpha = math.atan2(chosen[1], max(1e-6, chosen[0]))
        return math.atan2(2.0 * self.config.wheelbase_m * math.sin(alpha), lookahead)
