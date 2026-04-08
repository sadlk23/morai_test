# MORAI ERP Runtime

## Scope

This branch is ERP-oriented.

It adds an ERP-specific MORAI runtime profile without replacing the main Ioniq5 direct runtime on `main`.

Active ERP files:

- config: `configs/competition_morai_erp.json`
- launch: `ros1/alpamayo1_5_ros/launch/run_competition_erp.launch`

## How It Differs From The Main Runtime

- main branch default: K-City and Ioniq5 oriented
- main branch default output path: `/ctrl_cmd`
- ERP branch default output path: `/Control/serial_data`
- `/ctrl_cmd` remains available as an optional path, but it is not the active default in this ERP profile

## Default ERP Flow

- sensors -> runtime -> `/Control/serial_data`
- status <- `/ERP/serial_data`

The legacy serial bridge still uses the typed internal `SafetyDecision.command` representation, so planner and controller output flows through the same runtime architecture as the direct path.

## Topics

Required live inputs:

- `/camera/front/image_raw` or fallback `camera_image`
- `/fix`
- `/imu`

ERP-oriented output and diagnostics:

- `/Control/serial_data`
- `/ERP/serial_data`
- `/Local/heading`
- `/Local/utm`

## `/ctrl_cmd` And `/Control/serial_data`

- `/Control/serial_data` is the active default output path in this ERP profile
- `/ctrl_cmd` is still configured and can be enabled explicitly for comparison or fallback experiments
- this branch does not delete or replace the direct MORAI actuation implementation
- gear and ExternalCtrl remain operator-managed
- the active default remains `/Control/serial_data`; `/ctrl_cmd` stays as an optional comparison or fallback path

## Brake Scaling Rule

ERP default brake scaling is `erp_200`.

Why:

- `moo`'s `serial_ros2_bridge.py` treats `msg.data[5]` as ERP brake in the `1~200` family
- `moo`'s `morai_udp_control.py` divides `msg.data[5]` by `200` before sending MORAI UDP brake
- this means the `/Control/serial_data` contract is strongly aligned with ERP-style `0~200` brake output rather than normalized `0~1`

The runtime still computes brake internally as normalized `[0, 1]`, and the ERP bridge scales it to `0~200` right before `/Control/serial_data` publish.

If a downstream ERP bridge expects normalized brake instead, switch to:

```json
"legacy_serial_bridge": {
  "brake_mode": "normalized",
  "brake_output_max": 1.0
}
```

## Bring-Up Order

1. Start with debug-only
2. Confirm `/fix`, `/imu`, and camera input rates
3. Confirm `/ERP/serial_data` status is arriving
4. Disable debug-only and let the ERP legacy bridge publish to `/Control/serial_data`
5. Use direct `/ctrl_cmd` only if you explicitly want the optional path
6. If direct actuation is needed, arm it explicitly

Launch precedence note:

- `run_competition_erp.launch` leaves `enable_legacy_serial_bridge:=false` by default
- that launch arg does not disable the ERP config default
- with `debug_only:=false`, the ERP config still publishes `/Control/serial_data`
- `debug_only:=true` is the knob that suppresses bridge publish for safe bring-up

## Commands

Debug only:

```bash
roslaunch alpamayo1_5_ros run_competition_erp.launch \
  repo_root:=<repo> \
  config:=configs/competition_morai_erp.json \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=true
```

ERP bridge publish enabled:

```bash
roslaunch alpamayo1_5_ros run_competition_erp.launch \
  repo_root:=<repo> \
  config:=configs/competition_morai_erp.json \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=false
```

Optional direct actuation on top of ERP diagnostics:

```bash
roslaunch alpamayo1_5_ros run_competition_erp.launch \
  repo_root:=<repo> \
  config:=configs/competition_morai_erp.json \
  runtime_python:=/usr/bin/python3.10 \
  debug_only:=false \
  enable_actuation:=true \
  arm_actuation:=true
```

## Validation Checklist

```bash
rostopic type /Control/serial_data
rostopic type /ERP/serial_data
rostopic hz /fix
rostopic hz /imu
rostopic echo /alpamayo/debug_snapshot
```

## Diagnostics Notes

- `vehicle_status` is enabled by default in the ERP config
- command/status mismatch diagnostics use `/ERP/serial_data` when it is available
- `morai_udp_reference` remains operator diagnostics only
- optional helper topics remain non-blocking
- the current ERP branch inherits the `3.0 m` baseline from the K-City profile because `moo` alone does not strongly confirm a better ERP-specific wheelbase

## Risks To Confirm On Site

- ERP vehicle wheelbase and controller tuning may need adjustment from the inherited 3.0 m baseline
- downstream ERP bridge may still use a variant of the ERP brake contract, so confirm the exact `/Control/serial_data` expectation on site
- `/ERP/serial_data` payload meaning must be confirmed in the venue workspace
- direct `/ctrl_cmd` remains optional and should not be mistaken for the ERP default path
