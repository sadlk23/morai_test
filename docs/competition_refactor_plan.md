# Alpamayo 1.5 Competition Refactor Plan

## 1. Current Codebase Audit Summary

Alpamayo 1.5 currently ships as a research inference package centered on a heavyweight VLM-plus-diffusion trajectory generator. The main execution path is:

1. Load replay data from `physical_ai_av`
2. Build a prompt with camera images and optional navigation text
3. Run Qwen3-VL-based autoregressive rollout
4. Hand VLM cache into an expert model
5. Sample future actions with diffusion
6. Decode actions into future trajectory

The repository does not yet contain a competition runtime loop, sensor synchronization layer, controller stack, safety filter, ROS/UDP command interface, or structured runtime telemetry.

## 2. Current Data Flow

Existing offline flow:

- `load_physical_aiavdataset.py`
  - Replays synchronized multi-camera frames
  - Loads egomotion history/future
  - Converts world poses into a local ego frame at `t0`
- `helper.py`
  - Builds camera-index-aware chat messages
  - Supports optional nav text and VQA-style prompts
- `models/alpamayo1_5.py`
  - Runs VLM rollout
  - Samples future actions with diffusion
  - Converts latent action sequence into trajectory via `action_space`

## 3. Problems, Risks, and Bottlenecks

- The core runtime depends on a 10B-class model and sampled diffusion, which is not latency-stable enough for a competition control loop.
- Planning is tightly coupled to text generation and transformer KV-cache behavior.
- There is no deterministic batch-1 pipeline abstraction for real-time execution.
- There is no low-level controller, command shaping, or actuator safety guard.
- There is no failure-management policy for stale sensors, missing cameras, invalid waypoints, or low-confidence outputs.
- Replay loading is tied to a specific research dataset interface instead of generic live sensor IO.
- Notebook flows demonstrate usage patterns, but critical runtime concepts are not modularized into source modules.
- The local development environment is not yet bootstrapped with the repo's heavy ML dependencies, so safe fallbacks and dependency-light validation paths are required.

## 4. What Should Be Preserved

- `geometry/rotation.py` utilities
- `action_space/unicycle_accel_curvature.py` and supporting math
- `helper.py` prompt/message construction logic
- `nav_utils.py` navigation token handling and prompt-conditioning helpers
- visualization helpers that are useful for replay/debug workflows
- the existing trajectory-oriented output concept

## 5. What Should Be Refactored

- Introduce a competition-owned runtime package under `src/alpamayo1_5/competition/`
- Wrap the current Alpamayo model behind a planner backend interface instead of letting it own runtime flow
- Move all sensor synchronization, preprocessing, control, safety, output publishing, metrics, and debug handling into dedicated competition modules
- Add dependency-light fallback planning so the stack can be exercised even when the heavyweight model is unavailable
- Add typed runtime contracts and config validation

## 6. What Should Be Deprecated or Isolated

- Notebook-only inference patterns as a runtime dependency
- Direct reliance on diffusion as the default runtime abstraction
- `test_inference.py` as the implied production entrypoint
- VQA flow in competition runtime

These are retained for research/demo use but explicitly isolated from the competition pipeline.

## 7. Proposed Target Architecture

Primary runtime dataflow:

`sensor reception -> synchronization -> preprocessing -> fused planner input -> planner backend -> waypoint/speed postprocess -> classical controller -> safety filter -> ROS/UDP publisher -> logging`

## 8. Runtime Design

- Planner runs at 10 Hz by default
- Controller runs at 20 Hz by default
- Runtime uses explicit typed payloads between stages
- ROS1 Noetic is the primary output interface
- UDP is supported as a sibling adapter
- The legacy Alpamayo model is available through a backend adapter
- A lightweight deterministic waypoint backend is provided for fallback, dry-runs, and baseline integration testing

## 9. Latency and Failure Analysis

High-risk components:

- Hugging Face model loading
- VLM autoregressive rollout
- diffusion sampling
- image tensor construction and copies

Safe fallback policy:

1. reuse last valid plan briefly when permitted
2. clamp command and reduce speed
3. issue controlled stop

## 10. Migration and Testing Strategy

1. Add typed contracts and config validation
2. Add generic IO/synchronization modules
3. Add preprocessing and planner input assembly
4. Wrap existing Alpamayo inference as `LegacyAlpamayoPlannerBackend`
5. Add lightweight deterministic planner backend
6. Add controller stack
7. Add safety filter
8. Add ROS/UDP publishers
9. Add dry-run and latency benchmark scripts
10. Add sanity tests and documentation

Validation targets:

- config validation
- planner backend smoke tests
- controller tests on synthetic waypoint sets
- safety filter tests for clipping and emergency stop
- end-to-end dry-run over mock sensor packets
- latency benchmark using repeated dry-run cycles

## Implementation Note

The implemented phase-1 runtime follows the planned modular structure, with one
practical adjustment: the default "lightweight planner head" is currently a
deterministic waypoint baseline rather than a newly trained neural planner
checkpoint. The original Alpamayo release model is isolated behind a dedicated
compatibility wrapper and remains available as an optional heavy backend when
dependencies and weights are provisioned.
