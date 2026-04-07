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

Debug-first ROS1 wrapper with an explicit Python 3.10+ runtime interpreter:

```bash
roslaunch alpamayo1_5_ros run_competition_live.launch \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=true
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
3. let the ROS1 wrapper hand off to a configured Python 3.10+ interpreter
4. fail early with a clear message when the wrapper is started under Python 3.8 without that interpreter handoff

Supported wrapper env vars:

- `ALPAMAYO_REPO_ROOT`
- `ALPAMAYO_CONFIG_PATH`
- `ALPAMAYO_RUNTIME_PYTHON`
- `ALPAMAYO_DEBUG_ONLY`
- `ALPAMAYO_ENABLE_ACTUATION`
- `ALPAMAYO_ARM_ACTUATION`

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
5. Keep `ros_output.publish_actuation=false` or launch with `debug_only:=true`.
6. Confirm the runtime publishes `live_input_wait_stop` while sensors are still missing.
7. Verify `live_system_state` moves from `waiting` to `debug_only` or `ready`.
8. Verify `CameraFrame.metadata["decoded_rgb"] = true` in debug snapshots.
9. Enable real actuation only after JSON/debug behavior matches expectations.
10. Monitor `artifacts/morai_live_logs/` for metrics, snapshots, and command history.

`live_input_wait_stop` note:

- waiting-stop publication is intentionally separate from warning throttling
- `live_input.safe_stop_publish_interval_s` controls how often the safe stop command is republished
- `live_input.warn_throttle_s` controls how often waiting warnings are logged
- for simulator bring-up, keep `safe_stop_publish_interval_s` short enough to maintain a stable stop state

## Debug-First Bring-Up

Recommended first pass:

1. `planner.backend=lightweight`
2. `ros_output.publish_command_json=true`
3. `ros_output.publish_debug_json=true`
4. `ros_output.publish_actuation=false`
5. confirm camera/GPS/IMU rates and stale warnings
6. only then switch `publish_actuation=true`

Recommended command sequence:

```bash
python -m alpamayo1_5.competition.scripts.run_competition \
  --config configs/competition_morai_live.json \
  --debug-only
```

Then, only after verifying debug output:

```bash
python -m alpamayo1_5.competition.scripts.run_competition \
  --config configs/competition_morai_live.json \
  --enable-actuation \
  --arm-actuation
```

`--enable-actuation` without `--arm-actuation` now fails early by design.

`--debug-only` cannot be combined with actuation flags.

## ROS1 Wrapper Package

The repository now includes a minimal catkin wrapper package:

- `ros1/alpamayo1_5_ros/package.xml`
- `ros1/alpamayo1_5_ros/CMakeLists.txt`
- `ros1/alpamayo1_5_ros/scripts/run_competition_live_node.py`
- `ros1/alpamayo1_5_ros/launch/run_competition_live.launch`

This wrapper improves ROS launch practicality, but it still depends on the
runtime being executed with Python 3.10+.

Practical catkin bring-up:

1. Source your ROS workspace.
2. Make sure `morai_msgs`, `sensor_msgs`, and `std_msgs` are installed.
3. Use `runtime_python:=/path/to/python3.10+` when launching from a Python 3.8 Noetic shell.
4. Start with `debug_only:=true`.
5. Enable `enable_actuation:=true arm_actuation:=true` only after debug validation.

Example:

```bash
roslaunch alpamayo1_5_ros run_competition_live.launch \
  repo_root:=/path/to/alpamayo1.5-main \
  config:=/path/to/alpamayo1.5-main/configs/competition_morai_live.json \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=true
```

If `repo_root` and `config` are omitted, the wrapper now tries:

1. explicit env vars
2. current working directory discovery
3. script-relative fallback

For actual MORAI bring-up, passing `repo_root` and `config` explicitly is still recommended.

## Exact First Steps On Ubuntu 20.04 + ROS1 Noetic + MORAI

1. Install or activate a Python 3.10+ environment for the competition runtime.
2. Install the project dependencies into that Python 3.10+ environment.
3. Source ROS and your catkin workspace:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

4. Confirm the simulator topics and types:

```bash
rostopic list
rostopic type /camera/front/image_raw
rostopic type /gps
rostopic type /imu
rostopic type /ctrl_cmd
```

5. Edit `configs/competition_morai_live.json` so camera, GPS, IMU, route, and command topics match your workspace.
6. Keep the planner backend as `lightweight`.
7. Launch debug-only first:

```bash
roslaunch alpamayo1_5_ros run_competition_live.launch \
  repo_root:=/path/to/alpamayo1.5-main \
  config:=/path/to/alpamayo1.5-main/configs/competition_morai_live.json \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=true
```

8. Check that:
   - sensor callbacks increment receive counts
   - `live_system_state` moves out of `waiting`
   - decoded camera frames report `decoded_rgb=true`
   - debug JSON topics are publishing
9. Only after that, enable actuation explicitly:

```bash
roslaunch alpamayo1_5_ros run_competition_live.launch \
  repo_root:=/path/to/alpamayo1.5-main \
  config:=/path/to/alpamayo1.5-main/configs/competition_morai_live.json \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=false \
  enable_actuation:=true \
  arm_actuation:=true
```

## Environment-Dependent Limits

This repository now contains the live MORAI adapter path, but actual simulator
validation still depends on the local ROS workspace:

- the real message packages must be installed
- the real topic graph must match config
- final actuation semantics must be confirmed against the target MORAI setup
- stock ROS1 Noetic Python 3.8 still cannot run the competition runtime directly without the wrapper handing off to Python 3.10+
- `morai_msgs/CtrlCmd` must exist in the local ROS workspace before real actuation publishing can be enabled
- `legacy_alpamayo` still needs heavy dependencies, checkpoint availability, and a GPU-capable runtime environment
