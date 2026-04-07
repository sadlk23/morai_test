# MORAI Live Usage

## Entry Point

```bash
python -m alpamayo1_5.competition.scripts.run_competition --config configs/competition_morai_live.json
```

Dedicated wrapper:

```bash
python -m alpamayo1_5.competition.scripts.run_competition_live --config configs/competition_morai_live.json
```

## What The Live Adapter Owns

The MORAI adapter layer:

- subscribes to live ROS topics
- converts ROS messages into `CameraFrame`, `GpsFix`, `ImuSample`, and route text
- assembles a `SensorPacket`
- feeds the existing `CompetitionRuntimePipeline`
- publishes debug JSON and actuation commands separately

## Expected ROS Packages

The target ROS workspace should provide the packages referenced by config:

- `rospy`
- `std_msgs`
- image/GPS/IMU message packages that match `message_type`

For direct MORAI actuation publishing:

- `morai_msgs/CtrlCmd` or another configured actuation message type

## Important Ubuntu 20.04 + ROS1 Noetic Reality Check

Stock ROS1 Noetic commonly runs on Python 3.8, but this repository currently:

- declares `requires-python = "==3.12.*"` in `pyproject.toml`
- uses modern Python syntax such as `A | B` union types throughout the runtime code

That means the current repository is **not** a drop-in stock-Noetic Python 3.8
node yet. The live MORAI adapter path is implemented, but one of the following
must still be done before a real Ubuntu 20.04 + ROS1 Noetic deployment:

1. backport the runtime code to Python 3.8-compatible syntax and packaging
2. run the live MORAI adapter in a ROS setup that supports a newer Python interpreter
3. split the system into a ROS-facing Python 3.8 bridge and a newer-Python runtime process

Default MORAI mapping in this repository:

- `front_steer <- steering`
- `accel <- throttle`
- `brake <- brake`
- `longlCmdType <- 1` in pedal mode
- `velocity <- target_speed_mps * 3.6` only when `command_mode=velocity`

## Bring-Up Checklist

1. Verify actual simulator topic names and update `configs/competition_morai_live.json`.
2. Verify actual ROS message types and update `message_type` fields if needed.
3. Confirm the actuation topic and message type used by the target MORAI workspace.
4. Start with `planner.backend=lightweight`.
5. Confirm debug JSON publication before enabling real actuation.
6. Monitor `artifacts/morai_live_logs/` for metrics, snapshots, and command history.

## Environment-Dependent Limits

This repository now contains the live MORAI adapter path, but actual simulator
validation still depends on the local ROS workspace:

- the real message packages must be installed
- the real topic graph must match config
- final actuation semantics must be confirmed against the target MORAI setup
- Python compatibility with the target ROS1 Noetic environment must be resolved first
