"""Adapter around the original Alpamayo inference path."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import PlanResult, PlannerInput
from alpamayo1_5.competition.planners.base import PlannerBackend
from alpamayo1_5.competition.planners.model_wrapper import AlpamayoCompatibilityWrapper
from alpamayo1_5.competition.planners.postprocess import derive_target_speed, is_valid_waypoint_set
from alpamayo1_5.competition.runtime.config_competition import CameraConfig, PlannerConfig


class LegacyAlpamayoPlannerBackend(PlannerBackend):
    """Wrap the research model behind the competition planner interface."""

    name = "legacy_alpamayo"

    def __init__(self, config: PlannerConfig, cameras: list[CameraConfig]):
        self.config = config
        self.cameras = cameras
        self.wrapper = AlpamayoCompatibilityWrapper(config)

    def plan(self, planner_input: PlannerInput) -> PlanResult:
        self.wrapper.ensure_loaded()
        if not self.wrapper.is_available():
            return PlanResult(
                frame_id=planner_input.frame_id,
                timestamp_s=planner_input.timestamp_s,
                planner_name=self.name,
                waypoints_xy=[],
                target_speed_mps=0.0,
                confidence=0.0,
                stop_probability=1.0,
                risk_score=1.0,
                valid=False,
                diagnostics={"error": self.wrapper.load_error or "model_unavailable"},
            )

        try:
            model_input = planner_input.model_input_package
            if model_input is None:
                return PlanResult(
                    frame_id=planner_input.frame_id,
                    timestamp_s=planner_input.timestamp_s,
                    planner_name=self.name,
                    waypoints_xy=[],
                    target_speed_mps=0.0,
                    confidence=0.0,
                    stop_probability=1.0,
                    risk_score=1.0,
                    valid=False,
                    diagnostics={"error": "missing_model_input_package"},
                )
            wrapper_output = self.wrapper.forward(model_input)
            waypoints_xy = wrapper_output.waypoints_xy
            target_speed = derive_target_speed(
                waypoints_xy,
                self.config.min_target_speed_mps,
                self.config.max_target_speed_mps,
                max_curvature_for_full_speed=0.08,
            )
            confidence = 0.6 if is_valid_waypoint_set(waypoints_xy) else 0.0
            return PlanResult(
                frame_id=planner_input.frame_id,
                timestamp_s=planner_input.timestamp_s,
                planner_name=self.name,
                waypoints_xy=waypoints_xy,
                target_speed_mps=target_speed,
                confidence=confidence,
                stop_probability=0.1 if target_speed > 0.3 else 0.9,
                risk_score=max(0.0, 1.0 - confidence),
                valid=is_valid_waypoint_set(waypoints_xy),
                diagnostics={
                    "backend": self.name,
                    **wrapper_output.diagnostics,
                },
            )
        except Exception as exc:
            return PlanResult(
                frame_id=planner_input.frame_id,
                timestamp_s=planner_input.timestamp_s,
                planner_name=self.name,
                waypoints_xy=[],
                target_speed_mps=0.0,
                confidence=0.0,
                stop_probability=1.0,
                risk_score=1.0,
                valid=False,
                diagnostics={"error": f"{type(exc).__name__}: {exc}"},
            )
