# Runtime Architecture

## Overview

The competition runtime is organized as a modular batch-1 control stack:

1. `io`
2. `preprocess`
3. `planners`
4. `controllers`
5. `safety`
6. `runtime`

This separates live system concerns from the research inference code that originally shipped with Alpamayo 1.5.

## Stage-by-Stage Flow

### Sensor IO

`SensorPacket` carries raw sensor data for one control tick:

- camera frames
- GPS sample
- IMU sample
- optional LiDAR packet
- optional route command
- timestamps and metadata

`SensorSynchronizer` checks freshness and produces `SynchronizedFrame`, explicitly tracking stale or missing sensor inputs.

### Preprocessing

`ImagePreprocessor` builds camera summaries and input-shape metadata without forcing a heavyweight image stack.

`StatePreprocessor` estimates:

- local XY pose
- heading
- speed
- yaw rate
- acceleration

`SensorFusion` combines synchronized camera/state data into `PlannerInput`.

`ModelInputPackager` turns the fused planner input into a stable
`ModelInputPackage` that preserves camera ordering, route text, local ego
history, and target model resolution metadata. The legacy Alpamayo compatibility
wrapper uses this package as its only runtime-facing input.

### Planner Layer

`PlannerBackend` is the stable planning interface.

Implemented backends:

- `LegacyAlpamayoPlannerBackend`
  - wraps the original Alpamayo inference path
  - optional and dependency-heavy
- `LightweightWaypointPlannerBackend`
  - deterministic fallback baseline
  - suitable for dry-run, pipeline integration, and safe debugging

Every backend returns `PlanResult`:

- future waypoints
- target speed
- confidence
- stop probability
- risk score
- diagnostics

### Controller Layer

`ControllerRuntime` converts `PlanResult` into `ControlCommand`.

Supported controllers:

- lateral: Pure Pursuit
- lateral optional: Stanley
- longitudinal: PID

### Safety Layer

`SafetyFilter` validates plan and command quality and enforces safe fallback behavior.

It checks:

- stale sensors
- invalid waypoint geometry
- low confidence
- excessive curvature
- NaN/Inf command values
- output saturation

Fallback behavior:

- reuse last valid command for a short guard interval
- clamp command and reduce speed
- emergency stop

### Output Layer

Primary output interface:

- ROS1 command publisher

Secondary interface:

- UDP publisher

### Logging and Debug

Runtime instrumentation includes:

- per-stage latency
- frame/timestamp tracking
- predicted waypoints
- target speed
- controller output
- safety interventions
- persistent JSONL metrics/debug snapshots

## MORAI Integration Boundary

Simulator-specific wiring is isolated under:

- `src/alpamayo1_5/competition/integrations/morai/message_mapping.py`
- `src/alpamayo1_5/competition/integrations/morai/subscribers.py`
- `src/alpamayo1_5/competition/integrations/morai/live_runtime.py`
- `src/alpamayo1_5/competition/integrations/morai/publishers.py`

This preserves a clean split:

- generic runtime logic
  - contracts
  - sync
  - preprocess
  - planner
  - controller
  - safety
  - metrics/debug
- simulator-specific logic
  - ROS message imports
  - live topic subscriptions
  - packet assembly
  - MORAI actuation publishing

Live MORAI flow:

1. ROS camera/GPS/IMU/route topics are subscribed
2. camera messages are decoded into RGB image arrays before contract conversion
3. messages are converted into runtime contracts
4. `LivePacketAssembler` builds a `SensorPacket`
5. `CompetitionRuntimePipeline` runs unchanged
6. debug JSON and actuation outputs publish separately

## Startup And Fail-Closed Behavior

The live runtime now distinguishes:

- waiting for first valid live inputs
- missing required sensors
- stale required sensors
- packet timeout

When `live_input.fail_closed_on_missing_required=true`, the runtime publishes a
safe stop command with `intervention="live_input_wait_stop"` while waiting for a
valid live packet. This is intended to make simulator bring-up behavior explicit
instead of silently doing nothing.

The live runtime also reports a high-level `live_system_state` in safety and
debug diagnostics:

- `waiting`
- `degraded`
- `debug_only`
- `ready`
- `publishing_actuation`

This is intended to make simulator bring-up easier to diagnose from logs and
JSON debug output.
