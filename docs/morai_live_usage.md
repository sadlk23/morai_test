# MORAI Live Usage

## Scope

This repository keeps the existing MORAI competition runtime skeleton and now supports two control outputs:

1. Direct MORAI actuation (`/ctrl_cmd`, `morai_msgs/CtrlCmd`)
2. Legacy moo-compatible serial bridge (`/Control/serial_data`, `std_msgs/Float32MultiArray`)

Both can be enabled together, and debug-first behavior is preserved.

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

- `/Local/heading`
- `/Local/utm`

If optional helper topics are absent, runtime does not fail.

## Output Paths

Direct MORAI output:

- topic: `/ctrl_cmd`
- type: `morai_msgs/CtrlCmd`
- longi mode: type 1 in pedal mode

Legacy moo bridge output:

- topic: `/Control/serial_data`
- type: `std_msgs/Float32MultiArray`
- payload: `[control_mode, e_stop, gear, speed_mps, steer_rad, brake, alive]`

Brake conversion rule in legacy bridge:

- runtime brake is treated as normalized `[0, 1]`
- bridge output is `clamp(brake,0,1) * legacy_serial_bridge.brake_output_max`
- default `brake_output_max=1.0` keeps normalized output
- set `brake_output_max=200.0` if target consumer expects ERP-style 0-200

## Safety/Arming Policy

- Default is debug-first
- Actuation is allowed only when explicitly armed
- `--debug-only` disables direct actuation and legacy serial bridge publishing

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
rostopic list
rostopic type /camera/front/image_raw
rostopic type camera_image
rostopic type /fix
rostopic type /gps
rostopic type /imu
rostopic type /ctrl_cmd
rostopic type /Control/serial_data
rostopic hz /camera/front/image_raw
rostopic hz /fix
rostopic hz /imu
```

## Common Trouble Points

- Python handoff mismatch: wrapper starts under Python 3.8 but `runtime_python` is missing
- topic mismatch: simulator publishes `camera_image` or `/gps` while config points elsewhere
- message type mismatch: topic type differs from config `message_type`
- stale sensor warnings: timestamp/Hz mismatch causes waiting or degraded states
- actuation arming: publish flags set without `arm_actuation` or debug-only still enabled
