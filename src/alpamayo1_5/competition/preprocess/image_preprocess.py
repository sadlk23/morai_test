"""Image preprocessing utilities."""

from __future__ import annotations

from typing import Any

from alpamayo1_5.competition.contracts import SynchronizedFrame
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig


def infer_shape(image: Any) -> tuple[int, ...] | None:
    """Infer the shape tuple from common array-like objects."""

    shape = getattr(image, "shape", None)
    if isinstance(shape, tuple):
        return tuple(int(dim) for dim in shape)
    if isinstance(shape, list):
        return tuple(int(dim) for dim in shape)
    return None


class ImagePreprocessor:
    """Build normalized camera summaries for downstream planning."""

    def __init__(self, config: CompetitionConfig):
        self.config = config

    def preprocess(self, synchronized: SynchronizedFrame) -> dict[str, Any]:
        camera_order: list[str] = []
        camera_mask: dict[str, bool] = {}
        image_summary: dict[str, Any] = {}
        invalid_shapes: dict[str, str] = {}

        for camera_cfg in self.config.cameras:
            camera_order.append(camera_cfg.name)
            frame = synchronized.camera_frames.get(camera_cfg.name)
            available = frame is not None
            camera_mask[camera_cfg.name] = available
            if not available:
                image_summary[camera_cfg.name] = {
                    "present": False,
                    "shape": None,
                    "timestamp_s": None,
                }
                continue

            shape = frame.shape or infer_shape(frame.image)
            shape_valid = isinstance(shape, tuple) and len(shape) in {3, 4}
            if shape_valid and len(shape) == 3:
                shape_valid = shape[-1] == 3 or shape[0] == 3
            if shape_valid and len(shape) == 4:
                shape_valid = shape[-1] == 3 or shape[1] == 3
            if not shape_valid:
                invalid_shapes[camera_cfg.name] = f"unexpected_shape:{shape}"
                camera_mask[camera_cfg.name] = False
            image_summary[camera_cfg.name] = {
                "present": True,
                "shape": shape,
                "shape_valid": shape_valid,
                "timestamp_s": frame.timestamp_s,
                "encoding": frame.encoding,
                "frame_id": frame.frame_id,
                "decoded_rgb": bool(frame.metadata.get("decoded_rgb", False)),
                "source_encoding": frame.metadata.get("source_encoding"),
            }

        return {
            "camera_order": camera_order,
            "camera_mask": camera_mask,
            "image_summary": image_summary,
            "invalid_shapes": invalid_shapes,
        }
