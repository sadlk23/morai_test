# Final Reality Check

## 1. Already Implemented And Good

The repository already has the core pieces that should now be protected from unnecessary churn:

- original Alpamayo 1.5 research and inference code
- modular competition runtime centered on `CompetitionRuntimePipeline`
- isolated MORAI integration layer under `competition/integrations/morai/`
- decoded ROS image handling for raw and compressed image messages
- explicit split between debug JSON publishing and actuation publishing
- debug-first MORAI live config defaults
- explicit actuation arming controls in config, CLI, and launch usage
- ROS1 wrapper package with interpreter handoff strategy
- `live_system_state` diagnostics
- safer wait-stop republish interval
- explicit legacy Alpamayo live input validation and backend status reporting
- tests for config, live runtime, MORAI mapping, wrapper helpers, and legacy-input validation

These areas are materially stronger than in earlier stages and should not be redesigned.

## 2. Material Improvements Compared With Earlier Stages

Compared with the earlier dry-run-only state, the repository now supports:

- real live ROS subscribers mapped into `SensorPacket`
- real live packet assembly
- real decoded image arrays in the live path instead of metadata-only placeholders
- debug-first MORAI bring-up with explicit actuation arming
- wait-stop behavior while live inputs are incomplete
- ROS1 wrapper entry through catkin launch
- explicit diagnostics for waiting, degraded, debug-only, ready, and actuation-publishing states
- safer and more explicit legacy Alpamayo live failure modes

## 3. Remaining Code-Level Issues

After the final pass, there are no high-priority structural code gaps left in the repository.

The main code-level issues that were worth fixing in this pass were:

1. reducing fragile ROS launch path assumptions
2. throttling repeated callback decode warnings
3. exposing a more structured live-health summary in diagnostics

Those are now implemented.

## 4. Remaining Environment-Level Issues

These are not repository-only problems and should be documented honestly instead of hidden:

- `rospy`, `sensor_msgs`, `std_msgs`, and `morai_msgs` must exist in the local ROS workspace
- actual topic names and message types must match the MORAI workspace
- final actuation semantics for `morai_msgs/CtrlCmd` still need local confirmation
- Python 3.10+ runtime must exist alongside a ROS1 Noetic environment that may still start under Python 3.8
- legacy Alpamayo live mode still depends on heavy model packages, checkpoint availability, and suitable GPU resources

## 5. What Should Still Be Fixed Now

At the repository level, only small environment-specific adjustments remain:

- verify the actual MORAI topic graph in the target workspace
- confirm `morai_msgs/CtrlCmd` semantics in the target simulator build
- validate heavy legacy Alpamayo execution on the final GPU/runtime machine

## 6. What Should Not Be Changed Anymore

- the modular competition runtime architecture
- the MORAI integration boundary
- the debug-first live safety flow
- the explicit actuation arming model
- the lightweight backend as the default bring-up path
- the preserved legacy Alpamayo backend

## 7. Exact Bring-Up Sequence

### Debug-Only

1. source ROS and catkin workspaces
2. verify topics and message types with `rostopic list` and `rostopic type`
3. edit `configs/competition_morai_live.json`
4. launch with debug-only:

```bash
roslaunch alpamayo1_5_ros run_competition_live.launch \
  repo_root:=/path/to/alpamayo1.5-main \
  config:=/path/to/alpamayo1.5-main/configs/competition_morai_live.json \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=true
```

### Safe Waiting / Wait-Stop

While required live inputs are missing or timed out:

- `live_input_wait_stop` should be published
- `live_system_state` should report `waiting`
- `blocking_reasons` should explain which sensor or timeout is blocking progress

### Real Actuation Enablement

Only after debug validation:

```bash
roslaunch alpamayo1_5_ros run_competition_live.launch \
  repo_root:=/path/to/alpamayo1.5-main \
  config:=/path/to/alpamayo1.5-main/configs/competition_morai_live.json \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=false \
  enable_actuation:=true \
  arm_actuation:=true
```

## 8. Exact Known Environment Dependencies

- ROS packages:
  - `rospy`
  - `sensor_msgs`
  - `std_msgs`
- MORAI message package presence:
  - `morai_msgs/CtrlCmd`
- topic graph matching:
  - camera, GPS, IMU, route, and command topics must match local MORAI workspace names
- Python interpreter strategy:
  - wrapper launches from ROS1 environment and hands off to Python 3.10+ runtime
- legacy heavy backend dependencies:
  - `torch`
  - `transformers`
  - checkpoint/model access
  - GPU-capable runtime

## 9. File-By-File Action Plan

- `ros1/alpamayo1_5_ros/launch/run_competition_live.launch`
  - already updated to allow wrapper-driven repo/config discovery when launch args are omitted
- `ros1/alpamayo1_5_ros/scripts/run_competition_live_node.py`
  - already updated to prefer cwd/env-based repo discovery before script-relative fallback
- `src/alpamayo1_5/competition/integrations/morai/subscribers.py`
  - already updated to throttle repeated callback decode warnings
- `src/alpamayo1_5/competition/integrations/morai/live_runtime.py`
  - already updated to expose richer live-health diagnostics
- tests and docs
  - updated only where those changes improved bring-up practicality
