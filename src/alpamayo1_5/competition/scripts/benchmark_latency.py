"""Benchmark stage latency for the competition runtime."""

from __future__ import annotations

import argparse
import json

from alpamayo1_5.competition.runtime.config_competition import load_competition_config
from alpamayo1_5.competition.runtime.metrics import StageStats
from alpamayo1_5.competition.runtime.mock_data import make_mock_replay
from alpamayo1_5.competition.runtime.pipeline import CompetitionRuntimePipeline


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark competition runtime latency")
    parser.add_argument("--config", default="configs/competition_camera_gps_imu.json")
    parser.add_argument("--frames", type=int, default=25)
    args = parser.parse_args()

    config = load_competition_config(args.config)
    pipeline = CompetitionRuntimePipeline(config)
    stats = StageStats()
    for _decision, snapshot in pipeline.run_replay(make_mock_replay(config, args.frames)):
        stats.update(snapshot.stage_latency_ms)

    print(json.dumps(stats.summary(), indent=2))


if __name__ == "__main__":
    main()
