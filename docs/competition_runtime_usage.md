# Competition Runtime Usage

## Purpose

The phase-1 competition runtime is the operational path for modular
camera/GPS/IMU driving experiments. It is intentionally separated from the
original research notebook flow.

## Main Entry Points

- Dry-run:
  - `python -m alpamayo1_5.competition.scripts.dry_run --config configs/competition_camera_gps_imu.json --frames 3`
- Runtime loop:
  - `python -m alpamayo1_5.competition.scripts.run_competition --config configs/competition_camera_gps_imu.json --dry-run --frames 10`
- Latency benchmark:
  - `python -m alpamayo1_5.competition.scripts.benchmark_latency --config configs/competition_camera_gps_imu.json --frames 25`

## Runtime Flow

1. `SensorPacket` input
2. `SensorSynchronizer`
3. `ImagePreprocessor` and `StatePreprocessor`
4. `SensorFusion`
5. `ModelInputPackager`
6. Planner backend
7. `ControllerRuntime`
8. `SafetyFilter`
9. ROS/UDP publisher adapter
10. Debug and metrics dump

## Planner Modes

- `lightweight`
  - deterministic phase-1 default
  - no heavy model dependency
  - best for integration testing and latency-stable bring-up
- `legacy_alpamayo`
  - compatibility path around the original Alpamayo release model
  - remains isolated behind `AlpamayoCompatibilityWrapper`
  - may require heavy dependencies and gated checkpoints

## Config Notes

Primary config file:

- `configs/competition_camera_gps_imu.json`

Important fields:

- `planner.backend`
- `planner.use_nav`
- `planner.precision`
- `planner.input_image_width` / `planner.input_image_height`
- `controller.lateral_controller`
- `safety.*`
- `output_mode`
- `logging.*`

## Debug Artifacts

By default the runtime writes:

- `artifacts/competition_logs/debug_snapshots.jsonl`
- `artifacts/competition_logs/metrics.jsonl`
- `artifacts/competition_logs/command_history.jsonl`
- `artifacts/competition_logs/last_valid_plan.json`

## Known Phase-1 Limitations

- Live ROS subscribers are not wired to real simulator message schemas in this workspace yet.
- Legacy Alpamayo execution is dependency-gated and may fail closed when `torch`, `transformers`, or checkpoints are unavailable.
- The default lightweight planner is a deterministic competition baseline, not a newly trained lightweight neural head.
