#!/usr/bin/env python3
"""ROS1 wrapper node for the Alpamayo competition live runtime.

This wrapper makes the repository launchable from a catkin workspace while the
actual runtime implementation stays in the main Python package under `src/`.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def main() -> None:
    if sys.version_info < (3, 10):
        raise SystemExit(
            "alpamayo1_5_ros requires Python 3.10+ for the current competition runtime. "
            "Stock ROS1 Noetic Python 3.8 is not sufficient for direct execution. "
            "Use a newer interpreter environment or keep ROS1 as the transport layer only."
        )
    repo_root = _repo_root()
    sys.path.insert(0, str(repo_root / "src"))

    from alpamayo1_5.competition.scripts.run_competition import main as run_main

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config", default=str(repo_root / "configs" / "competition_morai_live.json"))
    args, passthrough = parser.parse_known_args()
    sys.argv = [sys.argv[0], "--config", args.config] + passthrough
    run_main()


if __name__ == "__main__":
    main()
