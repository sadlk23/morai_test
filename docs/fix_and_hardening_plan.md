# Fix And Hardening Plan

## 1. Current Repository Status

The repository already contains:

- original Alpamayo 1.5 research/inference code
- modular competition runtime
- MORAI live integration adapter path
- dry-run and latency benchmark scripts
- live packet assembly and MORAI mapping tests
- separated JSON debug publishing and MORAI actuation publishing

The goal is not to redesign the repository. The goal is to harden the existing
live MORAI path for realistic deployment and make the remaining constraints
explicit.

## 2. What Is Already Implemented

- `CompetitionRuntimePipeline` remains the main orchestration boundary
- `SensorPacket` is the runtime input contract
- `competition.integrations.morai.subscribers` converts ROS callbacks into latest-value buffers
- `competition.integrations.morai.live_runtime.LivePacketAssembler` builds `SensorPacket`
- `competition.integrations.morai.publishers` separates debug JSON vs actuation publishing
- `run_competition.py` can now enter a live runtime path
- `run_competition_live.py` and `run_competition_live.launch` exist
- lightweight planner remains the safe live default
- legacy Alpamayo remains available behind a compatibility wrapper

## 3. Remaining Blockers

Ranked by deployment risk:

1. Python 3.12 requirement and Python 3.10+ syntax conflict with stock Ubuntu
   20.04 + ROS1 Noetic Python 3.8 deployments.
2. Live image ingestion still carries raw ROS payloads plus shape metadata, not
   a guaranteed decoded image representation for the legacy Alpamayo path.
3. ROS launch/package assumptions are still more Python-module-friendly than
   catkin/roslaunch-friendly.
4. Live startup, timeout, stale-sensor, and route-command waiting behavior needs
   stronger diagnostics and clearer fail-closed semantics.
5. Legacy Alpamayo live usability remains limited until decoded images and
   deployment/runtime compatibility are stronger.

## 4. Python / ROS Compatibility Decision

Decision:

- preserve the main repository and research code as-is
- make the **competition live path** more Python 3.8-friendly
- avoid backporting the full research stack
- if the live path still cannot be made fully Python 3.8-safe without excessive
  churn, document and add a thin ROS-facing bridge boundary as the fallback
  deployment pattern

Immediate implementation strategy:

1. audit competition runtime files for Python 3.8-incompatible syntax
2. reduce compatibility blockers in the live competition path where practical
3. keep heavy research dependencies isolated from the ROS-facing live runtime
4. document the exact Ubuntu 20.04 + ROS1 Noetic deployment options

Known incompatible patterns already found:

- `A | B` union types
- `list[str]`, `dict[str, ...]`, `tuple[...]`, `set[...]`
- broad usage of Python 3.9+/3.10+ typing syntax across competition modules
- the competition runtime previously declared `requires-python = "==3.12.*"` and has now been relaxed to `>=3.10,<3.13`

## 5. Image Decoding / Live Payload Decision

Decision:

- camera ROS messages must be explicitly decoded into usable image arrays before
  entering the planner path
- support both `sensor_msgs/Image` and `sensor_msgs/CompressedImage`
- normalize into a consistent internal representation that preserves:
  - decoded pixel data
  - shape
  - encoding metadata
  - timestamps and frame ids
- fail clearly on unsupported encodings or malformed payloads

This is required both for live robustness and for making the legacy Alpamayo
live path plausible.

## 6. ROS Package / Launch Practicality Issues

Current issue summary:

- launch files exist, but catkin/roslaunch packaging assumptions remain weak
- the current repository is still more naturally launched as a Python module
  than as a conventional ROS1 package node
- missing ROS packages currently fail honestly, but the deployment path needs to
  be documented more concretely

Planned action:

- harden entrypoint error messages
- clarify launch expectations in docs
- document the exact ROS workspace requirements
- decide whether a dedicated catkin wrapper package is required or whether the
  current module-first flow is sufficient for phase-1

## 7. Legacy Alpamayo Live Usability Audit

Already good:

- camera ordering/indexing are preserved into `ModelInputPackage`
- route text propagates into planner input and model input
- wrapper fails closed when heavy dependencies are unavailable

Still weak:

- live image payloads are not yet guaranteed to be decoded into the format the
  legacy wrapper expects
- live deployment on ROS1 Noetic remains blocked by Python compatibility unless
  narrowed to the lightweight path or bridged

## 8. File-By-File Action Plan

- `pyproject.toml`
  - clarify or soften the live-runtime deployment strategy
- `src/alpamayo1_5/competition/runtime/config_competition.py`
  - strengthen validation for live ROS and image message combinations
- `src/alpamayo1_5/competition/integrations/morai/ros_message_utils.py`
  - add explicit decode/import helpers
- `src/alpamayo1_5/competition/integrations/morai/message_mapping.py`
  - move from metadata-only camera handling to decoded image handling
- `src/alpamayo1_5/competition/integrations/morai/subscribers.py`
  - improve callback diagnostics and startup robustness
- `src/alpamayo1_5/competition/integrations/morai/live_runtime.py`
  - harden startup waiting, packet timeout, route staleness, and logs
- `src/alpamayo1_5/competition/integrations/morai/publishers.py`
  - clarify command mode semantics and publish-time diagnostics
- `src/alpamayo1_5/competition/preprocess/image_preprocess.py`
  - validate decoded image representation more explicitly
- `src/alpamayo1_5/competition/planners/model_wrapper.py`
  - verify decoded live image compatibility and improve diagnostics
- `src/alpamayo1_5/competition/planners/legacy_backend.py`
  - surface exact live compatibility limitations clearly
- `src/alpamayo1_5/competition/scripts/run_competition.py`
  - improve ROS/deployment error flow
- `src/alpamayo1_5/competition/scripts/run_competition_live.py`
  - keep explicit live entry behavior
- `src/alpamayo1_5/competition/launch/run_competition_live.launch`
  - make launch assumptions more explicit
- `configs/competition_morai_live.json`
  - tighten topic/message defaults and publish-mode clarity
- `docs/morai_live_usage.md`
  - turn into an operational bring-up guide
- `docs/testing_and_validation.md`
  - add live image decode and deployment validation coverage
- `docs/runtime_architecture.md`
  - reflect the hardened live adapter boundary
- `README.md`
  - surface the practical live-runtime status and constraints

## 9. Validation Plan

Gate A:

- this document exists
- blocker ranking is explicit
- Python/ROS strategy is chosen

Gate B:

- raw ROS image messages decode into usable internal image data
- compressed image messages decode too, when payload is valid
- unsupported encodings fail clearly

Gate C:

- live packet assembly still works
- startup waiting behavior is explicit
- stale/missing sensors are visible in diagnostics

Gate D:

- debug publishing and actuation publishing remain separate
- command mapping semantics are explicit and tested

Gate E:

- dry-run still works
- existing pipeline tests still pass
- new live decode / live hardening tests pass

Gate F:

- docs explain what works now
- docs explain what remains environment-dependent
- Ubuntu 20.04 + ROS1 Noetic deployment guidance is clearer and more actionable
