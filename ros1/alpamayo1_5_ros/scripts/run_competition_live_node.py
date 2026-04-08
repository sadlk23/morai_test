#!/usr/bin/env python3
"""ROS1 wrapper node for the Alpamayo competition live runtime.

This wrapper makes the repository launchable from a catkin workspace while the
actual runtime implementation stays in the main Python package under `src/`.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

DEFAULT_CONFIG_RELATIVE = Path("configs") / "competition_morai_live.json"
ENV_REPO_ROOT = "ALPAMAYO_REPO_ROOT"
ENV_CONFIG_PATH = "ALPAMAYO_CONFIG_PATH"
ENV_RUNTIME_PYTHON = "ALPAMAYO_RUNTIME_PYTHON"
ENV_DEBUG_ONLY = "ALPAMAYO_DEBUG_ONLY"
ENV_ENABLE_ACTUATION = "ALPAMAYO_ENABLE_ACTUATION"
ENV_ARM_ACTUATION = "ALPAMAYO_ARM_ACTUATION"
ENV_ENABLE_LEGACY_SERIAL_BRIDGE = "ALPAMAYO_ENABLE_LEGACY_SERIAL_BRIDGE"


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _is_repo_root(path: Path) -> bool:
    return (path / "src" / "alpamayo1_5").exists()


def _discover_repo_root() -> Path | None:
    candidates: list[Path] = []
    cwd = Path.cwd().resolve()
    candidates.append(cwd)
    candidates.extend(cwd.parents)
    script_path = Path(__file__).resolve()
    candidates.extend(script_path.parents)
    for candidate in candidates:
        if _is_repo_root(candidate):
            return candidate
    return None


def _truthy_env(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


def resolve_repo_root(cli_value: str | None = None) -> Path:
    """Resolve the repository root using CLI, env, or relative path."""

    candidate = cli_value or os.environ.get(ENV_REPO_ROOT)
    if candidate:
        repo_root = Path(candidate).expanduser().resolve()
    else:
        repo_root = _discover_repo_root() or _repo_root()
    if not _is_repo_root(repo_root):
        raise SystemExit(
            "Could not resolve Alpamayo repo root. Set --repo-root or %s to a checkout that contains src/alpamayo1_5."
            % ENV_REPO_ROOT
        )
    return repo_root


def resolve_config_path(repo_root: Path, cli_value: str | None = None) -> Path:
    """Resolve the runtime config path using CLI, env, or repo-relative default."""

    candidate = cli_value or os.environ.get(ENV_CONFIG_PATH)
    if candidate:
        config_candidate = Path(candidate).expanduser()
        if not config_candidate.is_absolute():
            config_candidate = repo_root / config_candidate
        config_path = config_candidate.resolve()
    else:
        config_path = repo_root / DEFAULT_CONFIG_RELATIVE
    if not config_path.exists():
        raise SystemExit(
            "Could not find competition config at %s. Set --config or %s explicitly."
            % (config_path, ENV_CONFIG_PATH)
        )
    return config_path


def resolve_runtime_python(cli_value: str | None = None) -> str:
    """Resolve the interpreter used to execute the live runtime."""

    runtime_python = cli_value or os.environ.get(ENV_RUNTIME_PYTHON, sys.executable)
    if Path(runtime_python).expanduser().exists():
        return str(Path(runtime_python).expanduser().resolve())
    if shutil.which(runtime_python):
        return runtime_python
    raise SystemExit(
        "Could not resolve runtime interpreter %r. Set --runtime-python or %s to a valid Python 3.10+ executable."
        % (runtime_python, ENV_RUNTIME_PYTHON)
    )


def validate_runtime_flags(
    debug_only: bool,
    enable_actuation: bool,
    arm_actuation: bool,
    enable_legacy_serial_bridge: bool,
) -> None:
    """Keep wrapper launch flags aligned with runtime policy before handoff."""

    if debug_only and (enable_actuation or arm_actuation or enable_legacy_serial_bridge):
        raise SystemExit(
            "--debug-only cannot be combined with --enable-actuation, --arm-actuation, "
            "or --enable-legacy-serial-bridge"
        )


def build_runtime_argv(
    runtime_python: str,
    config_path: Path,
    passthrough: list[str],
    debug_only: bool = False,
    enable_actuation: bool = False,
    arm_actuation: bool = False,
    enable_legacy_serial_bridge: bool = False,
) -> list[str]:
    """Build the runtime command that launches the main competition script."""

    argv = [
        runtime_python,
        "-m",
        "alpamayo1_5.competition.scripts.run_competition",
        "--config",
        str(config_path),
    ]
    if debug_only:
        argv.append("--debug-only")
    if enable_actuation:
        argv.append("--enable-actuation")
    if arm_actuation:
        argv.append("--arm-actuation")
    if enable_legacy_serial_bridge:
        argv.append("--enable-legacy-serial-bridge")
    return argv + list(passthrough)


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--repo-root", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument("--runtime-python", default=None)
    parser.add_argument("--debug-only", action="store_true")
    parser.add_argument("--enable-actuation", action="store_true")
    parser.add_argument("--arm-actuation", action="store_true")
    parser.add_argument("--enable-legacy-serial-bridge", action="store_true")
    args, passthrough = parser.parse_known_args()
    repo_root = resolve_repo_root(args.repo_root)
    config_path = resolve_config_path(repo_root, args.config)
    runtime_python = resolve_runtime_python(args.runtime_python)
    debug_only = args.debug_only or _truthy_env(ENV_DEBUG_ONLY)
    enable_actuation = args.enable_actuation or _truthy_env(ENV_ENABLE_ACTUATION)
    arm_actuation = args.arm_actuation or _truthy_env(ENV_ARM_ACTUATION)
    enable_legacy_serial_bridge = args.enable_legacy_serial_bridge or _truthy_env(
        ENV_ENABLE_LEGACY_SERIAL_BRIDGE
    )
    validate_runtime_flags(
        debug_only=debug_only,
        enable_actuation=enable_actuation,
        arm_actuation=arm_actuation,
        enable_legacy_serial_bridge=enable_legacy_serial_bridge,
    )

    runtime_argv = build_runtime_argv(
        runtime_python=runtime_python,
        config_path=config_path,
        passthrough=passthrough,
        debug_only=debug_only,
        enable_actuation=enable_actuation,
        arm_actuation=arm_actuation,
        enable_legacy_serial_bridge=enable_legacy_serial_bridge,
    )
    runtime_env = os.environ.copy()
    runtime_env["PYTHONPATH"] = str(repo_root / "src") + os.pathsep + runtime_env.get("PYTHONPATH", "")

    if sys.version_info < (3, 10):
        if not runtime_python or runtime_python == sys.executable:
            raise SystemExit(
                "alpamayo1_5_ros was launched under Python %d.%d, but the competition runtime requires Python 3.10+. "
                "Set --runtime-python or %s to a Python 3.10+ interpreter."
                % (sys.version_info.major, sys.version_info.minor, ENV_RUNTIME_PYTHON)
            )
        sys.stderr.write(
            "alpamayo1_5_ros: handing off from Python %d.%d to runtime interpreter %s\n"
            % (sys.version_info.major, sys.version_info.minor, runtime_python)
        )
        raise SystemExit(subprocess.call(runtime_argv, env=runtime_env))

    sys.path.insert(0, str(repo_root / "src"))
    from alpamayo1_5.competition.scripts.run_competition import main as run_main

    sys.argv = [sys.argv[0]] + runtime_argv[3:]
    os.environ.update(runtime_env)
    run_main()


if __name__ == "__main__":
    main()
