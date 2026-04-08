# MORAI Sim Workspace Reference

## Why This Exists

`sadlk23/sim` is treated here as a historical competition reference from a ROS1 workspace that reflected a more field-like setup than a localhost-only development profile.

This repository remains the active runtime:

- `morai_test` stays the main runtime
- direct MORAI actuation still uses `/ctrl_cmd` and `morai_msgs/CtrlCmd`
- pedal mode and longi type `1` remain the active default
- gear and ExternalCtrl remain operator-managed

## morai_msgs In The Historical Workspace

The historical `sim` workspace carried `morai_msgs` directly in the catkin workspace, including files such as:

- `CtrlCmd.msg`
- `EgoVehicleStatus.msg`
- `CollisionData.msg`

That matters because on-site ROS workspaces can drift. Even when our runtime logic is correct, the venue message package can still differ.

For that reason, on-site verification remains mandatory:

```bash
rosmsg show morai_msgs/CtrlCmd
```

## Historical LAN Profile

The historical `sim` final setup was LAN-based rather than localhost-based.

Historical reference values:

- `user_ip`: `192.168.0.22`
- `host_ip`: `192.168.0.2`
- `multi_ip`: `192.168.0.100`
- `ctrl_cmd`: `3300 / 3301`
- `vehicle_status`: `1233 / 1234`
- `competition_status`: `3314 / 3315`
- `collision_data`: `5677 / 5678`
- `imu`: `1235 / 1236`

These values are reference-only. They are not the current runtime defaults.

## What To Do With This Reference

Use it to prepare for venue conditions where:

- the simulator PC and team PC are on a LAN
- status or judging signals are supplied outside the basic sensor topic graph
- message package mismatches are more likely because the workspace bundles `morai_msgs`

Do not hardcode these historical values into the current K-City config unless the venue explicitly confirms them.

## What Not To Copy Blindly

- private LAN IPs
- old bridge or ctrl mode assumptions
- any gear or ExternalCtrl control logic
- any output path that would replace `/ctrl_cmd` as the active default

## On-Site Checklist

```bash
rosmsg show morai_msgs/CtrlCmd
rostopic type /ctrl_cmd
rostopic type /ERP/serial_data
rostopic type /Local/heading
rostopic type /Local/utm
rostopic hz /camera/front/image_raw
rostopic hz /fix
rostopic hz /imu
```

## Related Files

- reference LAN profile: `docs/morai_sim_2025_final_udp_profile.json`
- active runtime config: `configs/competition_morai_kcity_2026.json`
