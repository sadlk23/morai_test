"""Run a dry-run cycle through the competition stack."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_replay
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline


def main() -> None:
    parser = argparse.ArgumentParser(description="Dry-run the Alpamayo competition pipeline")
    parser.add_argument(
        "--config",
        default="configs/competition_camera_gps_imu.json",
        help="Path to competition JSON config",
    )
    parser.add_argument("--frames", type=int, default=1, help="Number of mock frames to replay")
    parser.add_argument("--route", default=None, help="Optional override route command")
    args = parser.parse_args()

    config = load_competition_config(args.config)
    pipeline = CompetitionRuntimePipeline(config)
    results = pipeline.run_replay(make_mock_replay(config, args.frames, route_command=args.route))
    decision, snapshot = results[-1]
    payload = {
        "decision": {
            "frame_id": decision.frame_id,
            "intervention": decision.intervention,
            "risk_level": decision.risk_level,
            "command": {
                "steering": decision.command.steering,
                "throttle": decision.command.throttle,
                "brake": decision.command.brake,
                "target_speed_mps": decision.command.target_speed_mps,
            },
            "flags": decision.safety_flags,
        },
        "snapshot": snapshot.to_dict(),
    }
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
