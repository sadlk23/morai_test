"""Heuristic behavior scoring for interpretable plan metadata."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import PlannerInput
from alpamayo1_5.competition.planners.postprocess import compute_path_curvatures, derive_target_speed
from alpamayo1_5.competition.runtime.config_competition import PlannerConfig


class BehaviorHead:
    """Compute target speed, confidence, and stop probability."""

    def __init__(self, config: PlannerConfig):
        self.config = config

    def evaluate(
        self,
        planner_input: PlannerInput,
        waypoints_xy: list[tuple[float, float]],
    ) -> dict[str, float]:
        route_text = (planner_input.route_command or "").lower()
        target_speed = derive_target_speed(
            waypoints_xy,
            self.config.min_target_speed_mps,
            self.config.max_target_speed_mps,
            max_curvature_for_full_speed=0.08,
        )

        if "stop" in route_text:
            target_speed = min(target_speed, 0.2)
            stop_probability = 0.95
        elif "yield" in route_text:
            target_speed = min(target_speed, max(0.8, 0.5 * target_speed))
            stop_probability = 0.45
        else:
            stop_probability = 0.05

        available_cameras = sum(1 for present in planner_input.camera_mask.values() if present)
        expected_cameras = max(1, len(planner_input.camera_mask))
        camera_ratio = available_cameras / expected_cameras
        curvature = max(compute_path_curvatures(waypoints_xy))
        confidence = max(0.1, min(1.0, 0.55 + 0.35 * camera_ratio - min(0.25, curvature)))
        risk_score = min(1.0, max(stop_probability, curvature * 2.5, 1.0 - confidence))
        return {
            "target_speed_mps": target_speed,
            "confidence": confidence,
            "stop_probability": stop_probability,
            "risk_score": risk_score,
        }
