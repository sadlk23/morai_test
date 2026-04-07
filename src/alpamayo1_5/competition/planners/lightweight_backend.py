"""Deterministic lightweight planner backend."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import PlanResult, PlannerInput
from alpamayo1_5.competition.planners.base import PlannerBackend
from alpamayo1_5.competition.planners.behavior_head import BehaviorHead
from alpamayo1_5.competition.planners.postprocess import is_valid_waypoint_set
from alpamayo1_5.competition.planners.waypoint_head import HeuristicWaypointHead
from alpamayo1_5.competition.runtime.config_competition import PlannerConfig


class LightweightWaypointPlannerBackend(PlannerBackend):
    """Safe, dependency-light fallback planner."""

    name = "lightweight_waypoint"

    def __init__(self, config: PlannerConfig):
        self.config = config
        self.waypoint_head = HeuristicWaypointHead(config.num_waypoints, config.waypoint_dt_s)
        self.behavior_head = BehaviorHead(config)

    def plan(self, planner_input: PlannerInput) -> PlanResult:
        waypoints_xy = self.waypoint_head.generate(
            speed_mps=planner_input.ego_state.speed_mps,
            route_command=planner_input.route_command,
        )
        behavior = self.behavior_head.evaluate(planner_input, waypoints_xy)
        valid = planner_input.valid and is_valid_waypoint_set(waypoints_xy)
        return PlanResult(
            frame_id=planner_input.frame_id,
            timestamp_s=planner_input.timestamp_s,
            planner_name=self.name,
            waypoints_xy=waypoints_xy,
            target_speed_mps=behavior["target_speed_mps"],
            confidence=behavior["confidence"],
            stop_probability=behavior["stop_probability"],
            risk_score=behavior["risk_score"],
            valid=valid,
            diagnostics={
                "backend": self.name,
                "route_command": planner_input.route_command,
                "camera_mask": planner_input.camera_mask,
            },
        )
