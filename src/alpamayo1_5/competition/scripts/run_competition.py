"""Entrypoint for competition runtime execution."""

from __future__ import annotations

import argparse
import logging

from alpamayo1_5.competition.integrations.morai.live_runtime import run_live_runtime
from alpamayo1_5.competition.integrations.morai.ros_message_utils import MoraiIntegrationUnavailable
from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_replay
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    parser = argparse.ArgumentParser(description="Run the Alpamayo competition runtime")
    parser.add_argument("--config", default="configs/competition_camera_gps_imu.json")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run the pipeline on mock replay data instead of waiting for live IO",
    )
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument(
        "--max-cycles",
        type=int,
        default=None,
        help="Optional live-loop cycle limit for integration tests",
    )
    args = parser.parse_args()

    config = load_competition_config(args.config)

    if args.dry_run:
        pipeline = CompetitionRuntimePipeline(config)
        for decision, _snapshot in pipeline.run_replay(make_mock_replay(config, args.frames)):
            print(
                f"frame={decision.frame_id} intervention={decision.intervention} "
                f"steer={decision.command.steering:.3f} "
                f"throttle={decision.command.throttle:.3f} "
                f"brake={decision.command.brake:.3f}"
            )
        return

    if not config.live_input.enabled:
        raise SystemExit(
            "Live mode requires config.live_input.enabled=true. "
            "Use --dry-run for replay validation or provide a live MORAI config."
        )
    try:
        cycles = run_live_runtime(config, max_cycles=args.max_cycles)
    except MoraiIntegrationUnavailable as exc:
        raise SystemExit(
            "Live MORAI integration is environment-gated in this workspace: "
            f"{exc}"
        ) from exc
    print(f"live_runtime_cycles={cycles}")


if __name__ == "__main__":
    main()
