"""Metrics storage and latency aggregation."""

from __future__ import annotations

import json
from pathlib import Path
from statistics import mean
from typing import Any


class JsonlWriter:
    """Append JSON lines to a target file."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, payload: dict[str, Any]) -> None:
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, ensure_ascii=True) + "\n")


class StageStats:
    """Aggregate repeated stage-latency observations."""

    def __init__(self) -> None:
        self.samples: dict[str, list[float]] = {}

    def update(self, stage_latency_ms: dict[str, float]) -> None:
        for stage_name, latency_ms in stage_latency_ms.items():
            self.samples.setdefault(stage_name, []).append(latency_ms)

    def summary(self) -> dict[str, dict[str, float]]:
        result: dict[str, dict[str, float]] = {}
        for stage_name, values in self.samples.items():
            if not values:
                continue
            sorted_values = sorted(values)
            p95_index = min(len(sorted_values) - 1, max(0, int(round(0.95 * (len(sorted_values) - 1)))))
            result[stage_name] = {
                "avg_ms": mean(values),
                "max_ms": max(values),
                "p95_ms": sorted_values[p95_index],
            }
        return result
