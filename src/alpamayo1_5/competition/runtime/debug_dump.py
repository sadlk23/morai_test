"""Debug snapshot persistence."""

from __future__ import annotations

from pathlib import Path

from alpamayo1_5.competition.contracts import DebugSnapshot, PlanResult
from alpamayo1_5.competition.runtime.metrics import JsonlWriter


class DebugDumper:
    """Persist debug snapshots and last valid plans."""

    def __init__(self, log_dir: str):
        root = Path(log_dir)
        root.mkdir(parents=True, exist_ok=True)
        self.snapshot_writer = JsonlWriter(root / "debug_snapshots.jsonl")
        self.command_writer = JsonlWriter(root / "command_history.jsonl")
        self.last_plan_path = root / "last_valid_plan.json"

    def write_snapshot(self, snapshot: DebugSnapshot) -> None:
        self.snapshot_writer.write(snapshot.to_dict())

    def write_last_valid_plan(self, plan: PlanResult) -> None:
        payload = {
            "frame_id": plan.frame_id,
            "timestamp_s": plan.timestamp_s,
            "planner_name": plan.planner_name,
            "waypoints_xy": plan.waypoints_xy,
            "target_speed_mps": plan.target_speed_mps,
            "confidence": plan.confidence,
            "stop_probability": plan.stop_probability,
            "risk_score": plan.risk_score,
            "diagnostics": plan.diagnostics,
        }
        self.last_plan_path.write_text(__import__("json").dumps(payload, indent=2), encoding="utf-8")

    def write_command(self, payload: dict[str, object]) -> None:
        """Append a command-history record."""

        self.command_writer.write(payload)
