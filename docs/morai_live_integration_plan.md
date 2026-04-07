# MORAI Live Integration Plan

## 1. Current Live-IO Gap Summary

The repository already contains a working phase-1 competition runtime skeleton:

- `CompetitionRuntimePipeline` with explicit stages
- typed contracts such as `SensorPacket`, `PlanResult`, and `SafetyDecision`
- dry-run and mock replay scripts
- latency/debug logging
- controller and safety layers

The remaining gap before a live MORAI loop is simulator IO:

- live ROS subscribers were not yet converting simulator messages into `SensorPacket`
- `run_competition.py` still stopped at a placeholder message outside dry-run
- the ROS publisher path was JSON-oriented (`std_msgs/String`) and useful for debugging, but not yet separated from real actuation publishing

## 2. Actual Message Types Found In Workspace

Directly found in this repository/workspace:

- `rospy` imports are referenced in `src/alpamayo1_5/competition/io/ros_interface.py`
- `std_msgs/String` is referenced for JSON command/debug output

Not found in this repository/workspace:

- `morai_msgs`
- `sensor_msgs/NavSatFix`
- `sensor_msgs/Imu`
- `sensor_msgs/Image`
- `sensor_msgs/CompressedImage`
- any checked-in ROS message package defining MORAI control messages

Implication:

- this workspace can implement an honest MORAI adapter boundary now
- exact live completeness still depends on the target ROS workspace providing the real message packages at runtime

## 3. Actual Topic Names Found In Workspace

Topics already present in the current config:

- camera topics under `/alpamayo/camera/.../image_raw`
- GPS topic `/alpamayo/gps`
- IMU topic `/alpamayo/imu`
- debug JSON output topic `/alpamayo/debug_snapshot`
- JSON command output topic `/alpamayo/control_cmd_json`

Topics not found anywhere else in the repo:

- a checked-in MORAI-native actuation topic contract
- a checked-in route/nav live topic

## 4. Missing Assumptions

The following remain environment-dependent and must not be hidden:

- whether the target workspace exposes `morai_msgs/CtrlCmd`
- whether GPS comes in as `sensor_msgs/NavSatFix`, `morai_msgs/GPSMessage`, or another wrapper
- whether camera topics use `sensor_msgs/Image` or `sensor_msgs/CompressedImage`
- the exact route/nav topic, if any

## 5. Adapter Design

Add a simulator-specific integration layer under:

- `src/alpamayo1_5/competition/integrations/morai/`

Responsibilities:

- `message_mapping.py`
  - convert ROS messages into runtime contracts
- `subscribers.py`
  - own ROS subscribers and latest-message buffers
- `live_runtime.py`
  - assemble `SensorPacket` objects and drive the existing pipeline loop
- `publishers.py`
  - keep debug JSON publishing separate from real actuation publishing
- `topic_registry.py`
  - build live topic subscriptions from config
- `ros_message_utils.py`
  - dynamic message imports and timestamp helpers

The generic runtime pipeline remains unchanged in responsibility. MORAI logic is isolated at the adapter boundary.

## 6. Command Publication Design

Two ROS output paths are kept separate:

- debug/observability path
  - JSON command publisher
  - JSON debug snapshot publisher
- real actuation path
  - MORAI-compatible control publisher using a runtime-imported ROS message class

Current implementation target:

- use `morai_msgs/CtrlCmd` when available in the target ROS environment
- default to `command_mode=pedal`
  - steering -> `front_steer`
  - throttle -> `accel`
  - brake -> `brake`
  - `longlCmdType=1` when that field exists

This keeps the first live milestone focused on safe low-level pedal commands instead of velocity-mode assumptions.

## 7. Testing Strategy For Live Integration

Add dependency-light tests that do not require a real ROS master:

- ROS-like fake message -> contract mapping
- latest-buffer snapshot -> `SensorPacket` assembly
- stale/missing sensor handling in live assembly
- `SafetyDecision.command` -> actuation message mapping using a fake `CtrlCmd` class
- live runtime single-cycle execution with mocked subscriber buffers

Dry-run and existing runtime tests must continue to pass unchanged.

## 8. What Can Be Completed Now Vs Environment-Dependent

Can be completed now:

- live MORAI adapter package
- config extensions for live ROS IO
- live packet assembly into `SensorPacket`
- live runtime entrypoint
- explicit separation of debug vs actuation ROS publishing
- unit tests with mocked ROS-like messages

Still environment-dependent:

- importing real MORAI message packages
- validating exact MORAI topic names in the target simulator workspace
- validating actual live command publication against a running ROS master and MORAI instance
