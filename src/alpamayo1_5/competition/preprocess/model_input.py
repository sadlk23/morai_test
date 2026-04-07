"""Model-input packaging for planner backends."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import ModelInputPackage, PlannerInput
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig


class ModelInputPackager:
    """Build a stable model-facing package from planner input.

    The compatibility wrapper can delegate final pixel normalization to the
    original Alpamayo/HF processor, but this packager owns ordering, masking,
    route transfer, history construction, and shape/debug metadata.
    """

    def __init__(self, config: CompetitionConfig):
        self.config = config

    def build(self, planner_input: PlannerInput) -> ModelInputPackage:
        """Construct the model-facing payload."""

        camera_order: list[str] = []
        camera_indices: list[int] = []
        image_payloads: list[object] = []
        invalid_reasons: list[str] = []
        invalid_shapes = planner_input.fused_features.get("invalid_camera_shapes", {})
        for camera_name, reason in invalid_shapes.items():
            invalid_reasons.append(f"{camera_name}:{reason}")

        for camera in self.config.cameras:
            if not planner_input.camera_mask.get(camera.name):
                continue
            frame = planner_input.synchronized.camera_frames.get(camera.name)
            if frame is None:
                continue
            shape = planner_input.image_summary.get(camera.name, {}).get("shape")
            shape_valid = planner_input.image_summary.get(camera.name, {}).get("shape_valid", False)
            if not shape_valid:
                invalid_reasons.append(f"{camera.name}:invalid_shape")
                continue
            camera_order.append(camera.name)
            camera_indices.append(
                camera.camera_index if camera.camera_index is not None else len(camera_indices)
            )
            image_payloads.append(frame.image)

        ego_history_xy: list[tuple[float, float]] = []
        for idx in range(self.config.planner.history_steps):
            offset = (
                -(self.config.planner.history_steps - 1 - idx)
                * planner_input.ego_state.speed_mps
                * self.config.planner.history_dt_s
            )
            ego_history_xy.append((offset, 0.0))

        return ModelInputPackage(
            frame_id=planner_input.frame_id,
            timestamp_s=planner_input.timestamp_s,
            camera_order=camera_order,
            camera_indices=camera_indices,
            image_payloads=image_payloads,
            nav_text=planner_input.route_command if self.config.planner.use_nav else None,
            ego_history_xy=ego_history_xy,
            ego_speed_mps=planner_input.ego_state.speed_mps,
            target_resolution=(
                self.config.planner.input_image_width,
                self.config.planner.input_image_height,
            ),
            valid=planner_input.valid and len(image_payloads) >= self.config.safety.min_fresh_cameras and not invalid_reasons,
            diagnostics={
                "camera_count": len(image_payloads),
                "camera_order": camera_order,
                "camera_indices": camera_indices,
                "target_resolution": [
                    self.config.planner.input_image_width,
                    self.config.planner.input_image_height,
                ],
                "history_steps": self.config.planner.history_steps,
                "history_dt_s": self.config.planner.history_dt_s,
                "invalid_reasons": invalid_reasons,
                "normalization": "delegated_to_legacy_processor_for_compatibility",
            },
        )
