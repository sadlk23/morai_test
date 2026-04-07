# MORAI K-City 2026 Quickstart

## Competition Assumptions

- OS: Ubuntu 20.04
- ROS: ROS1 Noetic
- Host: desktop PC
- Map: `R-KR_PG_K-City_2025`
- Vehicle: `2023_Hyundai_ioniq5`
- Wheelbase baseline: `3.0 m`
- MORAI control: UDP Ego Ctrl Cmd, longi type 1 (accel/brake)

Use:

- config: `configs/competition_morai_kcity_2026.json`
- launch: `ros1/alpamayo1_5_ros/launch/run_competition_kcity_2026.launch`

## Default Input Topics

- camera primary: `/camera/front/image_raw`
- camera fallback: `camera_image`
- gps primary: `/fix`
- gps fallback: `/gps`
- imu: `/imu`
- optional helper (debug-only): `/Local/heading`, `/Local/utm`
- optional vehicle status (diagnostics-only when enabled): `/ERP/serial_data` by default placeholder

Optional helper topics are non-blocking. Missing helper topics must not crash runtime.

## Output Modes

1. Direct MORAI actuation
- topic: `/ctrl_cmd`
- type: `morai_msgs/CtrlCmd`

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
rosmsg show morai_msgs/CtrlCmd
```

Expected pedal mode fields:

- `longlCmdType` or `longiCmdType`
- `steering` or `front_steer`
- `accel`
- `brake`

Expected velocity mode fields:

- `longlCmdType` or `longiCmdType`
- `steering` or `front_steer`
- `velocity`

## Bring-Up Order (Debug-Only First)

1. Source ROS/catkin and verify Python 3.10 runtime path.
2. Start simulator with K-City map and Ioniq5 vehicle profile.
3. Launch debug-only runtime.
4. Verify topic/type/hz and debug JSON output.
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

- Python 3.10 handoff missing from ROS1 launch shell
- topic mismatch between simulator and config (camera/gps names)
- message type mismatch (`NavSatFix`, `Imu`, `Float32MultiArray`, `CtrlCmd`)
- `CtrlCmd` field mismatch now fails fast during direct actuation startup
- `vehicle_status` is diagnostics-only and is not a bring-up blocker
- stale sensor warnings due to low frequency or timestamp drift
- actuation not armed (`enable_actuation` without `arm_actuation`)
