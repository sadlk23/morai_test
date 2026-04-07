"""Stable Alpamayo-compatible model wrapper boundary."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from alpamayo1_5.competition.contracts import ModelInputPackage
from alpamayo1_5.competition.runtime.config_competition import PlannerConfig


@dataclass(slots=True)
class WrapperForwardResult:
    """Normalized wrapper output before planner postprocess."""

    waypoints_xy: list[tuple[float, float]]
    diagnostics: dict[str, Any]


class AlpamayoCompatibilityWrapper:
    """Hide the original Alpamayo loading and prompt-building details."""

    def __init__(self, config: PlannerConfig):
        self.config = config
        self._loaded = False
        self._load_error: str | None = None
        self._model: Any | None = None
        self._processor: Any | None = None
        self._torch: Any | None = None

    @property
    def load_error(self) -> str | None:
        """Expose deferred load failures for diagnostics."""

        return self._load_error

    def _resolve_dtype(self, torch_module: Any) -> Any:
        if self.config.precision == "fp32":
            return torch_module.float32
        if self.config.precision == "fp16":
            return torch_module.float16
        return torch_module.bfloat16

    def ensure_loaded(self) -> None:
        """Lazy-load model and processor only when the legacy backend is used."""

        if self._loaded:
            return
        self._loaded = True
        try:
            import torch
            from alpamayo1_5 import helper
            from alpamayo1_5.models.alpamayo1_5 import Alpamayo1_5
        except Exception as exc:
            self._load_error = f"dependency import failed: {exc}"
            return

        try:
            model_name = self.config.checkpoint_path or self.config.legacy_model_id
            dtype = self._resolve_dtype(torch)
            self._model = Alpamayo1_5.from_pretrained(model_name, dtype=dtype).to(self.config.device)
            self._processor = helper.get_processor(self._model.tokenizer)
            self._torch = torch
        except Exception as exc:
            self._load_error = f"model load failed: {type(exc).__name__}: {exc}"

    def is_available(self) -> bool:
        """Return whether the compatibility path is loaded and usable."""

        self.ensure_loaded()
        return self._load_error is None and self._model is not None and self._processor is not None and self._torch is not None

    def validate_model_input(self, model_input: ModelInputPackage) -> dict[str, Any]:
        """Validate model-facing inputs before entering the heavy legacy path."""

        diagnostics: dict[str, Any] = {
            "camera_order": list(model_input.camera_order),
            "camera_indices": list(model_input.camera_indices),
            "camera_count": len(model_input.image_payloads),
            "nav_text_present": bool(model_input.nav_text),
            "target_resolution": model_input.target_resolution,
        }
        if not model_input.valid:
            raise ValueError("invalid model input package")
        if not model_input.image_payloads:
            raise ValueError("model input package has no images")
        if len(model_input.image_payloads) != len(model_input.camera_indices):
            raise ValueError("image payload count does not match camera index count")
        if len(model_input.image_payloads) != len(model_input.camera_order):
            raise ValueError("image payload count does not match camera order length")
        if any(isinstance(image, (bytes, bytearray, memoryview)) for image in model_input.image_payloads):
            raise ValueError("model input package contains undecoded raw image payloads")
        for index, image in enumerate(model_input.image_payloads):
            shape = getattr(image, "shape", None)
            if shape is None or len(shape) != 3:
                raise ValueError("image payload at index %d is not an HWC decoded image array" % index)
            if shape[-1] != 3:
                raise ValueError("image payload at index %d must have 3 RGB channels" % index)
        return diagnostics

    def _camera_tensor(self, image: Any) -> Any:
        torch = self._torch
        if isinstance(image, (bytes, bytearray, memoryview)):
            raise ValueError("expected decoded image array, got raw byte payload")
        tensor = image if torch.is_tensor(image) else torch.as_tensor(image)
        if tensor.ndim == 3:
            tensor = tensor.unsqueeze(0)
        if tensor.ndim != 4:
            raise ValueError(f"expected image tensor with 3 or 4 dims, got {tensor.ndim}")
        if tensor.shape[-1] == 3 and tensor.shape[1] != 3:
            tensor = tensor.permute(0, 3, 1, 2)
        if tensor.shape[1] != 3:
            raise ValueError(f"expected channel-first tensor after normalization, got {tuple(tensor.shape)}")
        if tensor.shape[0] == 1:
            tensor = tensor.repeat(4, 1, 1, 1)
        return tensor[:4]

    def _build_history(self, model_input: ModelInputPackage) -> tuple[Any, Any]:
        torch = self._torch
        hist_xyz = torch.zeros((1, 1, len(model_input.ego_history_xy), 3), dtype=torch.float32)
        for idx, (x_m, y_m) in enumerate(model_input.ego_history_xy):
            hist_xyz[0, 0, idx, 0] = x_m
            hist_xyz[0, 0, idx, 1] = y_m
        hist_rot = torch.eye(3, dtype=torch.float32).view(1, 1, 1, 3, 3).repeat(
            1, 1, len(model_input.ego_history_xy), 1, 1
        )
        return hist_xyz, hist_rot

    def _camera_indices(self, model_input: ModelInputPackage) -> Any:
        return self._torch.tensor(model_input.camera_indices, dtype=self._torch.int64)

    def forward(self, model_input: ModelInputPackage) -> WrapperForwardResult:
        """Run the original Alpamayo trajectory sampling path through a stable wrapper API."""

        if not self.is_available():
            raise RuntimeError(self._load_error or "legacy wrapper unavailable")
        validation_diagnostics = self.validate_model_input(model_input)

        torch = self._torch
        from alpamayo1_5 import helper

        image_tensors = [self._camera_tensor(image) for image in model_input.image_payloads]
        image_frames = torch.stack(image_tensors, dim=0)
        hist_xyz, hist_rot = self._build_history(model_input)
        messages = helper.create_message(
            image_frames.flatten(0, 1),
            camera_indices=self._camera_indices(model_input),
            nav_text=model_input.nav_text,
        )
        inputs = self._processor.apply_chat_template(
            messages,
            tokenize=True,
            add_generation_prompt=False,
            continue_final_message=True,
            return_dict=True,
            return_tensors="pt",
        )
        model_inputs = helper.to_device(
            {
                "tokenized_data": inputs,
                "ego_history_xyz": hist_xyz,
                "ego_history_rot": hist_rot,
            },
            self.config.device,
        )
        with torch.autocast(self.config.device, dtype=self._resolve_dtype(torch)):
            pred_xyz, _pred_rot, extra = self._model.sample_trajectories_from_data_with_vlm_rollout(
                data=model_inputs,
                top_p=self.config.top_p,
                temperature=self.config.temperature,
                num_traj_samples=1,
                max_generation_length=self.config.max_generation_length,
                return_extra=True,
            )
        if pred_xyz.ndim < 5 or pred_xyz.shape[-1] < 2:
            raise ValueError(f"unexpected trajectory tensor shape: {tuple(pred_xyz.shape)}")
        waypoint_tensor = pred_xyz[0, 0, 0, :, :2].detach().cpu().tolist()
        cot = ""
        if extra and "cot" in extra:
            try:
                cot = str(extra["cot"][0][0][0])
            except Exception:
                cot = str(extra["cot"])
        return WrapperForwardResult(
            waypoints_xy=[(float(x), float(y)) for x, y in waypoint_tensor],
            diagnostics={
                **validation_diagnostics,
                "cot_preview": cot[:240],
                "use_diffusion_compatibility_path": True,
            },
        )
