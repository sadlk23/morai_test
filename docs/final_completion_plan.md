# Final Completion Plan

## 1. Current Repository Status

The repository already includes:

- original Alpamayo 1.5 research/inference code
- modular competition runtime with `CompetitionRuntimePipeline`
- MORAI live integration package
- live ROS subscribers and publishers
- live packet assembly
- raw and compressed ROS image decoding
- competition configs
- tests for MORAI mapping, live packet assembly, and live runtime behavior
- ROS1 wrapper package for catkin launch integration
- hardening and usage documentation

## 2. What Is Already Good

- live MORAI integration is isolated under `competition/integrations/morai/`
- debug JSON publishing and actuation publishing are already separated
- image decoding is real, not metadata-only
- wait-stop behavior exists
- config validation exists
- lightweight planner is a stable default
- legacy Alpamayo path is preserved

## 3. Remaining Blockers Ranked By Real Deployment Risk

1. Python 3.10+ runtime vs stock ROS1 Noetic Python 3.8 deployment mismatch
2. actuation enablement is still too easy to misconfigure compared with a debug-first bring-up workflow
3. ROS wrapper/package path assumptions are still fragile for normal catkin workspaces
4. live system-state visibility is still too implicit during startup/degraded operation
5. legacy Alpamayo live path needs clearer distinction between:
   - unavailable environment
   - invalid live input payload
   - otherwise valid but dependency-gated execution

## 4. Chosen Python / ROS Deployment Strategy

Chosen strategy:

- keep the main competition runtime on Python 3.10+
- keep the ROS1 wrapper package lightweight and more ROS-workspace-friendly
- add explicit interpreter handoff support so the wrapper can re-exec into a configured Python 3.10+ runtime interpreter
- keep Python 3.8 failure explicit and actionable when no configured runtime interpreter exists

This avoids rewriting the whole repository for Python 3.8 while making the ROS-facing launch path closer to real Noetic deployment.

## 5. Chosen MORAI Bring-Up Strategy

Bring-up sequence:

1. start in debug-only mode
2. verify live sensors and decoded image flow
3. verify `live_input_wait_stop` while inputs are incomplete
4. verify steady live debug JSON output
5. only then explicitly arm and enable actuation

The repository should make debug-first the safe default.

## 6. Legacy Alpamayo Live Viability Audit

Already acceptable:

- decoded images are now available to the live path
- camera ordering and camera indices are preserved
- nav/route text propagates into `ModelInputPackage`

Still needs improvement:

- clearer model-input validation before heavy model invocation
- diagnostics that distinguish invalid live payload from unavailable dependencies
- clearer reporting of backend status in live conditions

## 7. ROS Wrapper / Package Practicality Audit

Current weak points:

- repo-root assumptions are still mostly path-relative
- config path handling should support overrides
- interpreter selection should support a configured Python 3.10+ runtime
- launch args should support debug-only and actuation enablement cleanly

## 8. File-By-File Action Plan

- `src/alpamayo1_5/competition/runtime/config_competition.py`
  - add actuation arming config and stricter safety validation
- `src/alpamayo1_5/competition/integrations/morai/live_runtime.py`
  - add live system-state summary and clearer degraded/ready states
- `src/alpamayo1_5/competition/integrations/morai/publishers.py`
  - harden actuation arming and publish-time diagnostics
- `src/alpamayo1_5/competition/planners/model_wrapper.py`
  - add explicit model-input validation for live legacy use
- `src/alpamayo1_5/competition/planners/legacy_backend.py`
  - report backend readiness and invalid-input vs unavailable-env more clearly
- `src/alpamayo1_5/competition/scripts/run_competition.py`
  - add explicit debug-only / actuation enable / arm-actuation CLI control
- `ros1/alpamayo1_5_ros/scripts/run_competition_live_node.py`
  - add env-driven repo/config/runtime-python overrides and interpreter handoff
- `ros1/alpamayo1_5_ros/launch/run_competition_live.launch`
  - add launch args for config, repo root, runtime python, debug mode, actuation enablement
- `configs/competition_morai_live.json`
  - make debug-first safer by default
- tests
  - add config safety tests, wrapper launch/env tests, live system-state/arming tests
- docs
  - operationalize debug-first bring-up and interpreter-handoff deployment

## 9. Validation Plan

Gate A:

- this document exists
- blocker ranking is explicit
- deployment strategy is chosen clearly

Gate B:

- live camera decode path still works
- live packet assembly still works
- waiting/degraded state is clearer

Gate C:

- unsafe actuation config combinations fail early
- debug-only default is safer

Gate D:

- ROS wrapper supports clearer repo/config/runtime-python handling
- ROS launch path is more practical

Gate E:

- legacy backend live-path diagnostics are clearer
- failure modes distinguish input issues vs env issues

Gate F:

- dry-run still works
- existing tests still work
- new tests pass

## 10. What Will Remain Environment-Dependent

Even after this pass, the following still depend on the local MORAI / ROS environment:

- presence of `rospy`, `sensor_msgs`, `std_msgs`, and `morai_msgs`
- actual topic names in the simulator workspace
- final confirmation of actuation semantics against the target MORAI setup
- heavy legacy Alpamayo dependencies and checkpoints
- the ability to provide a Python 3.10+ interpreter environment alongside ROS1 Noetic
