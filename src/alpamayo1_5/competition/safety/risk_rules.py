"""Risk heuristics for the competition runtime."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import PlanResult, PlannerInput
from alpamayo1_5.competition.planners.postprocess import compute_path_curvatures, is_valid_waypoint_set
from alpamayo1_5.competition.runtime.config_competition import SafetyConfig


def assess_plan_risk(
    planner_input: PlannerInput,
    plan: PlanResult,
    safety_config: SafetyConfig,
) -> tuple[float, list[str]]:
    """Estimate risk score and safety flags for a plan."""

    flags: list[str] = []
    risk = 0.0

    if planner_input.synchronized.stale_sensors:
        flags.append("stale_sensors")
        risk = max(risk, 0.7)
    if planner_input.diagnostics.get("invalid_reasons"):
        flags.append("invalid_runtime_inputs")
        risk = max(risk, 0.95)
    if planner_input.synchronized.missing_sensors:
        flags.append("missing_required_sensors")
        risk = max(risk, 0.9)
    if not is_valid_waypoint_set(plan.waypoints_xy):
        flags.append("invalid_waypoints")
        risk = 1.0
    if plan.confidence < safety_config.min_confidence:
        flags.append("low_confidence")
        risk = max(risk, 0.75)
    if plan.stop_probability > safety_config.max_stop_probability_without_brake:
        flags.append("high_stop_probability")
        risk = max(risk, plan.stop_probability)

    if plan.waypoints_xy:
        max_curvature = max(compute_path_curvatures(plan.waypoints_xy))
        if max_curvature > safety_config.max_curvature_for_full_speed:
            flags.append("high_curvature")
            risk = max(risk, min(1.0, max_curvature))

    risk = max(risk, plan.risk_score)
    return risk, flags
