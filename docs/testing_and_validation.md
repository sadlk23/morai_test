# Testing And Validation

## Validation Gates

### Gate A: Config and interfaces

Validated by:

- `tests/competition/test_config.py`

Checks:

- config loads
- duplicate camera names fail
- runtime validation is enforced before execution
- unsafe actuation enablement without arming fails
- conflicting live CLI mode overrides fail early

### Gate B: IO and preprocess

Validated by:

- `tests/competition/test_preprocess_and_packaging.py`

Checks:

- synchronized mock packets produce stable camera summaries
- model input packaging preserves order, nav text, and target resolution metadata

### Gate C: Model wrapper and planner

Validated by:

- `tests/competition/test_planner_and_wrapper.py`

Checks:

- lightweight planner emits the configured waypoint count and sane target speed
- legacy wrapper fails closed when heavy dependencies are unavailable
- legacy wrapper distinguishes invalid live image payloads from environment failures

### Gate D: Controller and safety

Validated by:

- `tests/competition/test_controller_and_safety.py`
- `tests/competition/test_invalid_safety.py`

Checks:

- nominal controller path works
- stale sensors trigger conservative behavior
- invalid plans trigger braking fallback

### Gate E: End-to-end runtime

Validated by:

- `tests/competition/test_pipeline.py`
- `tests/competition/test_live_runtime.py`
- `tests/competition/test_ros_wrapper.py`
- `python -m alpamayo1_5.competition.scripts.dry_run --config configs/competition_camera_gps_imu.json --frames 2`

Checks:

- one complete cycle executes
- planner/controller/safety outputs are produced
- per-stage latency is recorded
- mocked live subscriber buffers can be assembled and executed through the same pipeline
- ROS1 wrapper helper logic validates repo/config/interpreter assumptions before launch

### Gate E2: Live MORAI adapter boundary

Validated by:

- `tests/competition/test_morai_mapping.py`
- `tests/competition/test_live_packet_assembly.py`
- raw image decode path in `tests/competition/test_morai_mapping.py`
- compressed image decode path in `tests/competition/test_morai_mapping.py` when Pillow is installed

Checks:

- ROS-like image/GPS/IMU/route messages map into runtime contracts
- supported raw image encodings decode into RGB arrays
- compressed image decoding is covered when the Pillow dependency is present
- live packet assembly preserves configured camera ordering
- stale or missing live sensors are surfaced before pipeline execution

### Gate F: Latency instrumentation

Validated by:

- `python -m alpamayo1_5.competition.scripts.benchmark_latency --config configs/competition_camera_gps_imu.json --frames 5`

Outputs:

- average latency
- max latency
- p95 latency

## Test Command

```bash
PYTHONPATH=src python -m unittest discover -s tests/competition -p "test_*.py"
```

On Windows PowerShell:

```powershell
$env:PYTHONPATH='src'
python -m unittest discover -s tests\competition -p "test_*.py"
```

## Current Assumptions

- Dependency-light tests are the default validation path in this workspace.
- Legacy Alpamayo heavy-model validation must be run on a properly provisioned environment.
- ROS publish behavior is validated structurally via adapters, not against a live ROS master here.
- MORAI actuation publication still requires the target ROS workspace to provide the actual message package configured in `ros_output.actuation_message_type`.
- `sensor_msgs/CompressedImage` live tests require Pillow; the test is skipped when Pillow is unavailable in the local environment.
- ROS1 Noetic Python 3.8 deployment is supported through the wrapper handoff strategy, not by running the competition runtime natively under Python 3.8.
