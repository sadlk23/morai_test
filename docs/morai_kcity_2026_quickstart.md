# MORAI K-City 2026 Quickstart

## Competition Assumptions

- OS: Ubuntu 20.04
- ROS: ROS1 Noetic
- Host: desktop PC only
- rosbridge: optional
- Map: `R-KR_PG_K-City_2025`
- Vehicle: `2023_Hyundai_ioniq5`
- Wheelbase baseline: `3.0 m`
- MORAI control: UDP Ego Ctrl Cmd, longi type 1 (accel/brake)
- Sensor limits: GPS `1`, IMU `1`, Camera `2`, 3D LiDAR `2`
- Camera pitch limit: `+/-30 deg`

Use:

- config: `configs/competition_morai_kcity_2026.json`
- launch: `ros1/alpamayo1_5_ros/launch/run_competition_kcity_2026.launch`

Historical venue reference from `sadlk23/sim` is documented separately in
`docs/morai_sim_workspace_reference.md`. That is a historical reference only and must not override the active default unless the venue confirms it.
The paired LAN reference artifact is `docs/morai_sim_2025_final_udp_profile.json`.

## morai_msgs Workspace Policy

For the K-City direct `/ctrl_cmd` path, treat `morai_msgs` as a managed workspace dependency rather than a hopeful environment assumption.

Preferred field setup:

- bring a validated `catkin_ws/src/morai_msgs`
- rebuild or source the same workspace that will launch `alpamayo1_5_ros`

Fallback field setup:

- use the venue-provided `morai_msgs`
- but do not drive until `rosmsg show morai_msgs/CtrlCmd` matches the expected pedal-mode contract

Why this is mandatory:

- venue ROS workspaces drift
- simulator PC and operator PC can source different overlays
- `/ctrl_cmd` may exist even when `morai_msgs/CtrlCmd` is the wrong definition

## Default Input Topics

- camera primary: `/camera/front/image_raw`
- camera fallback: `camera_image`
- gps primary: `/fix`
- gps fallback: `/gps`
- imu: `/imu`
- optional helper (debug-only): `/Local/heading` (`std_msgs/Float64` primary, `std_msgs/Float32` fallback), `/Local/utm`
- optional vehicle status (diagnostics-only optional input when enabled): `/ERP/serial_data` by default placeholder
- optional competition status (diagnostics-only optional input when enabled): topic and type are venue-defined
- optional collision data (diagnostics-only optional input when enabled): topic and type are venue-defined

Optional helper topics and diagnostics-only optional inputs are not bring-up blockers.
Missing helper topics must not crash runtime.
Competition Vehicle Status and Ego Vehicle Status can differ by event program, so the on-site topic and message type must be confirmed before enabling diagnostics.
Historical `sim` LAN values are historical reference only and are not the active default in this config.

## Output Modes

1. Direct MORAI actuation
- topic: `/ctrl_cmd`
- type: `morai_msgs/CtrlCmd`
- competition rule: longi type `1`, pedal mode, `accel` + `brake` + `steering`
- simulator gear and ExternalCtrl mode are not owned by participant code
- the active default remains this path even when historical LAN references exist

`morai_udp_reference` in the debug snapshot is operator diagnostics only. It is useful for checking venue-reference IPs and ports, including historical `competition_status` and `collision_data` channels, but it is not a bring-up blocker.

2. Legacy moo serial bridge
- topic: `/Control/serial_data`
- type: `std_msgs/Float32MultiArray`
- format: `[control_mode, e_stop, gear, speed_mps, steer_rad, brake, alive]`

Brake conversion in legacy bridge:

- internal runtime brake is normalized `[0,1]`
- explicit `brake_mode` wins over `brake_output_max`
- `brake_mode=normalized` forces `0~1`
- `brake_mode=erp_200` forces `0~200`
- `brake_mode=auto` keeps backward compatibility and infers from `brake_output_max`

Example when moo side expects `0~200` brake:

```json
"legacy_serial_bridge": {
  "enabled": true,
  "publish_enabled": true,
  "topic": "/Control/serial_data",
  "message_type": "std_msgs/Float32MultiArray",
  "brake_mode": "erp_200",
  "brake_output_max": 200.0
}
```

## Direct Actuation Self-Check

When `enable_actuation:=true`, runtime now checks `morai_msgs/CtrlCmd` at startup and fails fast if the contract does not match.

Check this before driving:

```bash
source /opt/ros/noetic/setup.bash
rosmsg show morai_msgs/CtrlCmd
```

Expected pedal mode fields:

- longi type `1`
- `longlCmdType` or `longiCmdType`
- `steering` or `front_steer`
- `accel`
- `brake`

Expected velocity mode fields:

- `longlCmdType` or `longiCmdType`
- `steering` or `front_steer`
- `velocity`

Runtime policy in the K-City config stays on pedal mode by default. If `rosmsg show morai_msgs/CtrlCmd` does not expose the pedal-mode fields above, do not drive until the workspace message package is corrected.
This is even more important when the venue workspace bundles its own `morai_msgs`, as seen in the historical `sim` workspace.
If direct actuation startup reports "wrong morai_msgs in workspace" or "CtrlCmd missing accel/brake fields", stop and fix the sourced workspace before retrying.

## Gear And Mode Policy

- Initial simulator state is `Keyboard + P`
- Race start transition to `D + ExternalCtrl` is owned by the operator or judging program
- Participant code must not force gear changes
- Participant code must not force Keyboard or ExternalCtrl mode transitions
- This repository only publishes `/ctrl_cmd` actuation and optional diagnostics

## Bring-Up Order (Debug-Only First)

1. Source ROS/catkin and verify Python 3.10 runtime path.
2. Start simulator with K-City map and Ioniq5 vehicle profile.
3. Launch debug-only runtime.
4. Verify `rosmsg show morai_msgs/CtrlCmd`, topic/type/hz, and debug JSON output.
5. Only then enable direct actuation and arm.
6. If needed, additionally enable legacy bridge publishing.

## Competition-Day Commands

Debug only:

```bash
roslaunch alpamayo1_5_ros run_competition_kcity_2026.launch \
  repo_root:=<repo> \
  config:=<config> \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=true
```

Direct actuation enabled:

```bash
roslaunch alpamayo1_5_ros run_competition_kcity_2026.launch \
  repo_root:=<repo> \
  config:=<config> \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=false \
  enable_actuation:=true \
  arm_actuation:=true
```

Legacy bridge enabled:

```bash
roslaunch alpamayo1_5_ros run_competition_kcity_2026.launch \
  repo_root:=<repo> \
  config:=<config> \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=false \
  enable_legacy_serial_bridge:=true
```

Direct + legacy together:

```bash
roslaunch alpamayo1_5_ros run_competition_kcity_2026.launch \
  repo_root:=<repo> \
  config:=<config> \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=false \
  enable_actuation:=true \
  arm_actuation:=true \
  enable_legacy_serial_bridge:=true
```

## ROS Validation Checklist

```bash
rosmsg show morai_msgs/CtrlCmd
rostopic list
rostopic type /camera/front/image_raw
rostopic type camera_image
rostopic type /fix
rostopic type /gps
rostopic type /imu
rostopic type /ERP/serial_data
rostopic type /Local/heading
rostopic type /Local/utm
rostopic type /ctrl_cmd
rostopic type /Control/serial_data
rostopic echo /alpamayo/debug_snapshot
rostopic hz /camera/front/image_raw
rostopic hz /fix
rostopic hz /imu
```

## Trouble Points

- desktop-only field setup means laptop assumptions, power profiles, and USB layout can diverge from the venue PC
- Python 3.10 handoff missing from ROS1 launch shell
- ROS workspace message package mismatch (`morai_msgs/CtrlCmd`, helper-topic message types)
- wrong `morai_msgs` in workspace even though `/ctrl_cmd` topic exists
- topic mismatch between simulator and config (camera/gps names)
- message type mismatch (`NavSatFix`, `Imu`, `Float32MultiArray`, `CtrlCmd`)
- `CtrlCmd` field mismatch now fails fast during direct actuation startup
- `vehicle_status` is a diagnostics-only optional input and is not a bring-up blocker, but the on-site topic/type must be confirmed
- `competition_status` and `collision_data` are diagnostics-only optional inputs and may not exist until the venue confirms the topic contract
- stale sensor warnings due to low frequency or timestamp drift
- actuation not armed (`enable_actuation` without `arm_actuation`)
- display, ethernet, and simulator-PC network link issues can block topic graph visibility on site
- ROS overlay order mistakes can make the runtime see a different `morai_msgs` than the operator expects
- LAN-based historical profiles from `sim` are useful operator references but must not be copied blindly into active config
