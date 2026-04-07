"""Sensor fusion for the competition planner input."""

from __future__ import annotations

from alpamayo1_5.competition.contracts import EgoState, PlannerInput, SynchronizedFrame


class SensorFusion:
    """Assemble planner-facing fused input from preprocessed components."""

    def fuse(
        self,
        synchronized: SynchronizedFrame,
        ego_state: EgoState,
        image_features: dict[str, object],
    ) -> PlannerInput:
        camera_order = list(image_features.get("camera_order", []))
        camera_mask = dict(image_features.get("camera_mask", {}))
        image_summary = dict(image_features.get("image_summary", {}))
        invalid_shapes = dict(image_features.get("invalid_shapes", {}))
        fused_features = {
            "camera_count": sum(1 for present in camera_mask.values() if present),
            "camera_mask": camera_mask,
            "has_gps": synchronized.gps_fix is not None,
            "has_imu": synchronized.imu_sample is not None,
            "has_lidar": synchronized.lidar_packet is not None,
            "invalid_camera_shapes": invalid_shapes,
        }
        invalid_reasons = list(synchronized.diagnostics.get("invalid_reasons", []))
        if invalid_shapes:
            invalid_reasons.append("invalid_camera_shape")

        return PlannerInput(
            frame_id=synchronized.frame_id,
            timestamp_s=synchronized.timestamp_s,
            synchronized=synchronized,
            ego_state=ego_state,
            route_command=synchronized.route_command,
            camera_order=camera_order,
            camera_mask=camera_mask,
            image_summary=image_summary,
            fused_features=fused_features,
            valid=synchronized.valid and ego_state.valid and not invalid_shapes,
            diagnostics={
                "missing_sensors": synchronized.missing_sensors,
                "stale_sensors": synchronized.stale_sensors,
                "ego_state_valid": ego_state.valid,
                "invalid_reasons": invalid_reasons,
            },
        )
