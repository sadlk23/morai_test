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

    def _invalid_result(
        self,
        planner_input: PlannerInput,
        error: str,
        backend_status: str,
        model_input: object | None = None,
    ) -> PlanResult:
        diagnostics = {
            "backend": self.name,
            "backend_status": backend_status,
            "error": error,
            "configured_cameras": [camera.name for camera in self.cameras],
            "route_command_present": bool(planner_input.route_command),
        }
        if model_input is not None:
            diagnostics["model_input_present"] = True
            if hasattr(model_input, "camera_order"):
                diagnostics["camera_order"] = list(getattr(model_input, "camera_order"))
            if hasattr(model_input, "camera_indices"):
                diagnostics["camera_indices"] = list(getattr(model_input, "camera_indices"))
            if hasattr(model_input, "diagnostics"):
                diagnostics["model_input_diagnostics"] = getattr(model_input, "diagnostics")
        else:
            diagnostics["model_input_present"] = False
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
            diagnostics=diagnostics,
        )

    def plan(self, planner_input: PlannerInput) -> PlanResult:
        self.wrapper.ensure_loaded()
        if not self.wrapper.is_available():
            return self._invalid_result(
                planner_input,
                self.wrapper.load_error or "model_unavailable",
                backend_status="backend_unavailable",
                model_input=planner_input.model_input_package,
            )

        try:
            model_input = planner_input.model_input_package
            if model_input is None:
                return self._invalid_result(
                    planner_input,
                    "missing_model_input_package",
                    backend_status="missing_model_input",
                )
            input_diagnostics = self.wrapper.validate_model_input(model_input)
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
                    "backend_status": "ok",
                    "route_command_present": bool(planner_input.route_command),
                    "validated_model_input": input_diagnostics,
                    **wrapper_output.diagnostics,
                },
            )
        except ValueError as exc:
            return self._invalid_result(
                planner_input,
                f"{type(exc).__name__}: {exc}",
                backend_status="invalid_live_model_input",
                model_input=planner_input.model_input_package,
            )
        except Exception as exc:
            return self._invalid_result(
                planner_input,
                f"{type(exc).__name__}: {exc}",
                backend_status="runtime_error",
                model_input=planner_input.model_input_package,
            )
