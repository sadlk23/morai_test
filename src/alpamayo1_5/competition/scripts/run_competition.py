"""Entrypoint for competition runtime execution.

Phase 1 focuses on replay-friendly dry-runs and a stable runtime skeleton. Live
ROS subscribers can feed the same `CompetitionRuntimePipeline` by producing
`SensorPacket` objects that match the runtime contracts.
"""

from __future__ import annotations

import argparse

from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_replay
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the Alpamayo competition runtime")
    parser.add_argument("--config", default="configs/competition_camera_gps_imu.json")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run the pipeline on mock replay data instead of waiting for live IO",
    )
    parser.add_argument("--frames", type=int, default=10)
    args = parser.parse_args()

    config = load_competition_config(args.config)
    pipeline = CompetitionRuntimePipeline(config)

    if args.dry_run:
        for decision, _snapshot in pipeline.run_replay(make_mock_replay(config, args.frames)):
            print(
                f"frame={decision.frame_id} intervention={decision.intervention} "
                f"steer={decision.command.steering:.3f} "
                f"throttle={decision.command.throttle:.3f} "
                f"brake={decision.command.brake:.3f}"
            )
        return

    print("Live sensor ingestion is not wired in this workspace yet.")
    print("Use --dry-run for replay validation or feed SensorPacket objects into CompetitionRuntimePipeline.")


if __name__ == "__main__":
    main()
