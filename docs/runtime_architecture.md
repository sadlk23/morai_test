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
