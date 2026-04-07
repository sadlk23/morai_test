"""Entrypoint for competition runtime execution."""

from __future__ import annotations

import argparse
import logging

from alpamayo1_5.competition.integrations.morai.live_runtime import run_live_runtime
from alpamayo1_5.competition.integrations.morai.publishers import MoraiActuationContractError
from alpamayo1_5.competition.integrations.morai.ros_message_utils import MoraiIntegrationUnavailable
from alpamayo1_5.competition.runtime.config_competition import CompetitionConfig, load_competition_config
from alpamayo1_5.competition.runtime.mock_data import make_mock_replay
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline


def apply_runtime_mode_overrides(
    config: CompetitionConfig,
    debug_only: bool = False,
    enable_actuation: bool = False,
    arm_actuation: bool = False,
    enable_legacy_serial_bridge: bool = False,
) -> CompetitionConfig:
    """Apply launch-time overrides and revalidate the config."""

    if debug_only and (enable_actuation or arm_actuation or enable_legacy_serial_bridge):
        raise ValueError(
            "--debug-only cannot be combined with --enable-actuation, --arm-actuation, "
            "or --enable-legacy-serial-bridge"
        )
    if debug_only:
        config.ros_output.publish_actuation = False
        config.ros_output.actuation_armed = False
        config.legacy_serial_bridge.publish_enabled = False
    if enable_actuation:
        config.ros_output.publish_actuation = True
    if arm_actuation:
        config.ros_output.actuation_armed = True
    if enable_legacy_serial_bridge:
        config.legacy_serial_bridge.enabled = True
        config.legacy_serial_bridge.publish_enabled = True
    config.validate()
    return config


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
    parser.add_argument(
        "--debug-only",
        action="store_true",
        help="Disable actuation publishing even if configured, while keeping debug outputs active",
    )
    parser.add_argument(
        "--enable-actuation",
        action="store_true",
        help="Enable actuation publishing for this run",
    )
    parser.add_argument(
        "--arm-actuation",
        action="store_true",
        help="Explicitly arm actuation publishing for this run",
    )
    parser.add_argument(
        "--enable-legacy-serial-bridge",
        action="store_true",
        help="Enable legacy /Control/serial_data Float32MultiArray publishing",
    )
    args = parser.parse_args()

    try:
        config = load_competition_config(args.config)
        config = apply_runtime_mode_overrides(
            config,
            debug_only=args.debug_only,
            enable_actuation=args.enable_actuation,
            arm_actuation=args.arm_actuation,
            enable_legacy_serial_bridge=args.enable_legacy_serial_bridge,
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

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
    except MoraiActuationContractError as exc:
        raise SystemExit(str(exc)) from exc
    print(f"live_runtime_cycles={cycles}")


if __name__ == "__main__":
    main()
