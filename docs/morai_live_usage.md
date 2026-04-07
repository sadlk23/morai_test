# MORAI Live Usage

## Entry Point

```bash
python -m alpamayo1_5.competition.scripts.run_competition --config configs/competition_morai_live.json
```

Dedicated wrapper:

```bash
python -m alpamayo1_5.competition.scripts.run_competition_live --config configs/competition_morai_live.json
```

ROS1 catkin wrapper:

```bash
roslaunch alpamayo1_5_ros run_competition_live.launch
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

- `morai_msgs/CtrlCmd`

## Important Ubuntu 20.04 + ROS1 Noetic Reality Check

Stock ROS1 Noetic commonly runs on Python 3.8, but the current competition runtime now targets:

- `Python >=3.10,<3.13` in `pyproject.toml`
- a ROS1 wrapper package under `ros1/alpamayo1_5_ros/`

This means the repository is still **not** a drop-in stock-Noetic Python 3.8
node. The implemented deployment choice is:

1. keep the competition runtime on Python 3.10+
2. provide a ROS1 wrapper package for catkin integration
3. fail early with a clear message when the wrapper is started under Python 3.8

Default MORAI mapping in this repository:

- `front_steer <- steering`
- `accel <- throttle`
- `brake <- brake`
- `longlCmdType <- 1` in pedal mode
- `velocity <- target_speed_mps * 3.6` only when `command_mode=velocity`

## Supported Live Image Inputs

Supported camera ROS message types:

- `sensor_msgs/Image`
- `sensor_msgs/CompressedImage`

Supported raw image encodings:

- `rgb8`
- `bgr8`
- `rgba8`
- `bgra8`
- `mono8`

Internal normalization:

- decoded to `H x W x 3` RGB `uint8`
- stored in `CameraFrame.image`
- `CameraFrame.metadata["decoded_rgb"] = true`

Compressed image note:

- compressed image decoding requires Pillow at runtime
- if Pillow is unavailable, compressed image topics fail clearly and raw image topics should be used instead

## Bring-Up Checklist

1. Verify actual simulator topic names and update `configs/competition_morai_live.json`.
2. Verify actual ROS message types and update `message_type` fields if needed.
3. Confirm the actuation topic and message type used by the target MORAI workspace.
4. Start with `planner.backend=lightweight`.
5. Set `ros_output.publish_actuation=false` and confirm debug JSON first.
6. Confirm the runtime publishes `live_input_wait_stop` while sensors are still missing.
7. Enable real actuation only after JSON/debug behavior matches expectations.
8. Monitor `artifacts/morai_live_logs/` for metrics, snapshots, and command history.

## Debug-First Bring-Up

Recommended first pass:

1. `planner.backend=lightweight`
2. `ros_output.publish_command_json=true`
3. `ros_output.publish_debug_json=true`
4. `ros_output.publish_actuation=false`
5. confirm camera/GPS/IMU rates and stale warnings
6. only then switch `publish_actuation=true`

## ROS1 Wrapper Package

The repository now includes a minimal catkin wrapper package:

- `ros1/alpamayo1_5_ros/package.xml`
- `ros1/alpamayo1_5_ros/CMakeLists.txt`
- `ros1/alpamayo1_5_ros/scripts/run_competition_live_node.py`
- `ros1/alpamayo1_5_ros/launch/run_competition_live.launch`

This wrapper improves ROS launch practicality, but it still depends on the
runtime being executed with Python 3.10+.

## Environment-Dependent Limits

This repository now contains the live MORAI adapter path, but actual simulator
validation still depends on the local ROS workspace:

- the real message packages must be installed
- the real topic graph must match config
- final actuation semantics must be confirmed against the target MORAI setup
- stock ROS1 Noetic Python 3.8 still cannot run the competition runtime directly
