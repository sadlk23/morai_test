# MORAI Live Usage

## Scope

This repository keeps the existing MORAI competition runtime skeleton and now supports two control outputs:

1. Direct MORAI actuation (`/ctrl_cmd`, `morai_msgs/CtrlCmd`)
2. Legacy moo-compatible serial bridge (`/Control/serial_data`, `std_msgs/Float32MultiArray`)

Both can be enabled together, and debug-first behavior is preserved.

Target competition assumptions for the single K-City bring-up path:

- Ubuntu 20.04
- ROS1 Noetic
- desktop PC only
- rosbridge optional
- map `R-KR_PG_K-City_2025`
- vehicle `2023_Hyundai_ioniq5`
- wheelbase `3.0 m`
- sensor limits: GPS `1`, IMU `1`, Camera `2`, 3D LiDAR `2`
- camera pitch limit: `+/-30 deg`

## Runtime And ROS Wrapper

Runtime entrypoint:

```bash
python -m alpamayo1_5.competition.scripts.run_competition --config configs/competition_morai_live.json
```

ROS1 wrapper node:

```bash
roslaunch alpamayo1_5_ros run_competition_live.launch
```

K-City 2026 wrapper:

```bash
roslaunch alpamayo1_5_ros run_competition_kcity_2026.launch
```

## Python/ROS1 Constraint

Target field environment is Ubuntu 20.04 + ROS1 Noetic, but the runtime requires Python 3.10+.
Use wrapper handoff:

- wrapper under ROS1 launches
- runtime executes with `runtime_python:=/usr/bin/python3.10` (or another Python 3.10+ path)

Supported wrapper env vars:

- `ALPAMAYO_REPO_ROOT`
- `ALPAMAYO_CONFIG_PATH`
- `ALPAMAYO_RUNTIME_PYTHON`
- `ALPAMAYO_DEBUG_ONLY`
- `ALPAMAYO_ENABLE_ACTUATION`
- `ALPAMAYO_ARM_ACTUATION`
- `ALPAMAYO_ENABLE_LEGACY_SERIAL_BRIDGE`

## Topic Compatibility

Primary live inputs:

- camera: `/camera/front/image_raw` (supports fallback subscription to `camera_image`)
- gps: `/fix` (K-City 2026 config) with fallback `/gps`
- imu: `/imu`

Optional helper topics (debug only, non-blocking):

- `/Local/heading` with `std_msgs/Float64` primary and `std_msgs/Float32` fallback
- `/Local/utm`
- optional `vehicle_status` diagnostics topic when configured

If optional helper topics are absent, runtime does not fail.
Competition Vehicle Status and Ego Vehicle Status can differ by venue program, so the on-site topic and message type must still be checked before relying on diagnostics.

## Output Paths

Direct MORAI output:

- topic: `/ctrl_cmd`
- type: `morai_msgs/CtrlCmd`
- longi type 1 in pedal mode
- longitudinal fields: `accel` and `brake`
- lateral field: `steering` or `front_steer`
- participant code does not own simulator gear or ExternalCtrl transitions

Legacy moo bridge output:

- topic: `/Control/serial_data`
- type: `std_msgs/Float32MultiArray`
- payload: `[control_mode, e_stop, gear, speed_mps, steer_rad, brake, alive]`

Brake conversion rule in legacy bridge:

- runtime brake is treated as normalized `[0, 1]`
- `brake_mode=normalized` forces `0~1`
- `brake_mode=erp_200` forces `0~200`
- `brake_mode=auto` infers from `brake_output_max` for backward compatibility
- if `brake_mode` and `brake_output_max` disagree, `brake_mode` wins and a warning is emitted

## Safety/Arming Policy

- Default is debug-first
- Actuation is allowed only when explicitly armed
- `--debug-only` disables direct actuation and legacy serial bridge publishing
- runtime policy diagnostics expose pedal mode, direct actuation enablement, legacy bridge state, vehicle-status subscriber state, and operator-managed gear/mode ownership

## Gear And External Control Policy

- Initial simulator state is `Keyboard + P`
- Start transition to `D + ExternalCtrl` is owned by the operator or judging program
- Team code must not force gear changes
- Team code must not force Keyboard or ExternalCtrl mode changes
- This repository intentionally limits itself to `/ctrl_cmd` actuation plus optional diagnostics

Allowed actuation command pattern:

```bash
python -m alpamayo1_5.competition.scripts.run_competition \
  --config configs/competition_morai_live.json \
  --enable-actuation \
  --arm-actuation
```

Legacy bridge command flag:

```bash
python -m alpamayo1_5.competition.scripts.run_competition \
  --config configs/competition_morai_kcity_2026.json \
  --enable-legacy-serial-bridge
```

## Verification Checklist

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

## Common Trouble Points

- desktop-only venue PCs may expose different display, ethernet, and USB conditions than a development laptop
- Python handoff mismatch: wrapper starts under Python 3.8 but `runtime_python` is missing
- topic mismatch: simulator publishes `camera_image` or `/gps` while config points elsewhere
- message package mismatch: topic type differs from config `message_type`, or the ROS workspace has the wrong `morai_msgs`
- `CtrlCmd` contract mismatch: direct actuation now fails fast if required fields are missing
- `vehicle_status` topic/type can differ by venue setup and must be checked on site even though it is diagnostics-only
- stale sensor warnings: timestamp/Hz mismatch causes waiting or degraded states
- actuation arming: publish flags set without `arm_actuation` or debug-only still enabled
