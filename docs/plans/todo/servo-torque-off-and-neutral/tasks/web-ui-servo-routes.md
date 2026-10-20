---
title: Web UI servo routes
status: todo
repo: ros2-lerobot-sverve-platform
value: "The browser can read each group's torque and neutral state and run release, record, save and enable."
files: [nodes/web_ui/web_ui/servo_calibration.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/imu_calibration.py, nodes/web_ui/tests/test_servo_calibration.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_servo_calibration.py tests/test_imu_calibration.py tests/test_ros_thread_affinity.py -q && uv run --frozen poe lint"
depends_on: [bridge-runtime-neutral-save]
owns: [servo-calibration-api]
uses: [torque-state-topic, neutral-services, arm-stale-marker]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI servo routes

## Requirement

- Config (`config.py`): `CalibrationConfig.servo_groups: list[ServoGroupConfig]` (default empty). `ServoGroupConfig` (pydantic): `id: Literal["arm", "base", "head"]` (unique), `label: str`, `namespace: str` (bridge group namespace, e.g. `follower`), `neutral: bool = False`, `stale_marker: str | None = None` (arm only), `state_stale_s: float = 3.0` (> 0), `service_timeout_s: float = 15.0` (> 0). Derived names: state topic `/{namespace}/torque_state`, services `/{namespace}/servo/release_torque|enable_torque|record_neutral|save_neutral`.
- `bridge.py`: an always-on `std_msgs/String` subscription per configured group on its state topic (reliable + transient_local, depth 1), created through the existing always-on path (ROS-thread affinity: created on the bridge timer by apply_demand); the latest parsed payload and its monotonic receipt time are cached; `servo_torque_sample(namespace) -> tuple[dict, float] | None`. Unparsable payloads are logged and dropped.
- New `servo_calibration.py` with `register_servo_calibration_routes(app, config, bridge_node, battery_block)` (registered in `server.py` next to the IMU routes): `GET /api/calibration/servos` -> `{"groups": [{"id", "label", "neutral", "state": <payload> | null, "age_s": float | null, "stale": bool, "calibration_stale": [items] | null}]}` (`calibration_stale` from the marker file for a group with `stale_marker`, null when absent); `POST /api/calibration/servos/{group_id}/{action}` with action `release`, `enable`, `record`, `save` calls the matching Trigger through `bridge_node.trigger_async` and answers `{"ok", "message"}` (200 / 409 on success false; 404 for an unknown group or action, or record/save on a group without `neutral`; 503 without a bridge). `save` is refused under battery cut-off via `battery_block`; release and enable never are (D8).
- `imu_calibration.py`: `GET /api/calibration/config` adds `"servos": true` when `servo_groups` is non-empty.
- Tests first, seen failing, in new `tests/test_servo_calibration.py` (fake bridge as in `tests/test_imu_calibration.py`): config validation (unique ids, derived names); GET with fresh, stale and missing samples and the marker; each POST action calls the derived service; 404/409/503 paths; save blocked under cut-off, release not; config endpoint flag.
- `README.md` `## Calibrate page (hidden)` gets `### Servo torque and neutral` (config, routes, payload).

## Acceptance conditions

- a1: servo_groups config validates and derives topic and service names [evidence: test: nodes/web_ui/tests/test_servo_calibration.py]
- a2: GET /api/calibration/servos reports cached state, age, stale flag and arm calibration_stale [evidence: test: nodes/web_ui/tests/test_servo_calibration.py]
- a3: POST actions call the derived Trigger services with the documented status codes; save is blocked under battery cut-off [evidence: test: nodes/web_ui/tests/test_servo_calibration.py]
- a4: register_servo_calibration_routes exists [evidence: symbol: nodes/web_ui/web_ui/servo_calibration.py::register_servo_calibration_routes]
- a5: README documents the section [evidence: doc-section: nodes/web_ui/README.md#Servo torque and neutral]
- a6: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/servo_calibration.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/imu_calibration.py`
- `nodes/web_ui/tests/test_servo_calibration.py`
- `nodes/web_ui/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
