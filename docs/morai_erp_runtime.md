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

## Brake Scaling Rule

ERP default brake scaling stays on `normalized`.

Why:

- the runtime already produces normalized brake internally
- `/ERP/serial_data` status diagnostics are easier to compare against command output when the same normalized scale is used
- this avoids assuming an ERP `0~200` consumer unless the downstream bridge explicitly requires it

If the downstream ERP bridge expects `0~200`, switch to:

```json
"legacy_serial_bridge": {
  "brake_mode": "erp_200",
  "brake_output_max": 200.0
}
```

## Bring-Up Order

1. Start with debug-only
2. Confirm `/fix`, `/imu`, and camera input rates
3. Confirm `/ERP/serial_data` status is arriving
4. Disable debug-only and let the ERP legacy bridge publish to `/Control/serial_data`
5. Use direct `/ctrl_cmd` only if you explicitly want the optional path
6. If direct actuation is needed, arm it explicitly

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

## Risks To Confirm On Site

- ERP vehicle wheelbase and controller tuning may need adjustment from the inherited 3.0 m baseline
- downstream ERP bridge may expect `erp_200` brake scaling instead of normalized
- `/ERP/serial_data` payload meaning must be confirmed in the venue workspace
- direct `/ctrl_cmd` remains optional and should not be mistaken for the ERP default path
