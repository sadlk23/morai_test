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
- Live MORAI runtime:
  - `python -m alpamayo1_5.competition.scripts.run_competition --config configs/competition_morai_live.json`
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

Live simulator wiring reuses the same pipeline through the MORAI adapter layer:

1. ROS subscribers convert simulator messages into `CameraFrame`, `GpsFix`, `ImuSample`
2. `LivePacketAssembler` builds a `SensorPacket`
3. `CompetitionRuntimePipeline.run_cycle(...)` executes unchanged
4. debug JSON and actuation publishers publish separately

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
- `configs/competition_morai_live.json`

Important fields:

- `planner.backend`
- `planner.use_nav`
- `planner.precision`
- `planner.input_image_width` / `planner.input_image_height`
- `live_input.*`
- `route_command.*`
- `cameras[*].message_type`
- `gps.message_type`
- `imu.message_type`
- `controller.lateral_controller`
- `safety.*`
- `output_mode`
- `ros_output.publish_command_json`
- `ros_output.publish_debug_json`
- `ros_output.publish_actuation`
- `logging.*`

## Debug Artifacts

By default the runtime writes:

- `artifacts/competition_logs/debug_snapshots.jsonl`
- `artifacts/competition_logs/metrics.jsonl`
- `artifacts/competition_logs/command_history.jsonl`
- `artifacts/competition_logs/last_valid_plan.json`

## Known Phase-1 Limitations

- Legacy Alpamayo execution is dependency-gated and may fail closed when `torch`, `transformers`, or checkpoints are unavailable.
- The default lightweight planner is a deterministic competition baseline, not a newly trained lightweight neural head.
- Live MORAI integration is now implemented behind `competition.integrations.morai`, but exact end-to-end validation still depends on the local ROS workspace providing the real message packages and topic graph.
- The repository still declares Python 3.12 and uses modern type-syntax features, so stock Ubuntu 20.04 + ROS1 Noetic Python 3.8 deployment is not yet a drop-in path.
