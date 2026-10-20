---
title: Web UI quiet released groups
status: todo
repo: ros2-lerobot-sverve-platform
value: "The web UI cannot itself command a released group: no slider, jog, home, grasp or nav goal reaches it."
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/servo_calibration.py, nodes/web_ui/tests/test_servo_quiet.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_servo_quiet.py tests/test_head_control.py tests/test_arm_api.py -q && uv run --frozen poe lint"
depends_on: [web-ui-servo-routes]
owns: []
uses: [servo-calibration-api]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI quiet released groups

## Requirement

- `ServoGroupConfig` gains `command_topics: list[str]` (web_ui topics that command the group: arm -> the arm tab `arm_command_topic`; head -> `head_command_topic`; base -> the map_nav `goal_topic`).
- `servo_calibration.released_command_error(config, bridge_node, topic) -> str | None`: the error text `<label> servo torque is off (Calibrate page)` when the topic belongs to a group whose cached state is fresh and `released`; None otherwise (unknown or stale refuses nothing).
- `server.py` WS publish chain: a publish to such a topic is rejected and logged (`ws_servo_released_rejected`, client id, topic) and the client gets the same error envelope as a head range rejection; nothing is published.
- `server.py` routes `/api/arm/home`, `/api/arm/take_control`, `/api/arm/jog` and `/api/grasp` answer 409 `{"ok": false, "message": ...}` while the arm group is released; `/api/arm/release` and `/api/grasp/stop` stay allowed.
- Tests first, seen failing, in new `tests/test_servo_quiet.py`: WS publish to the head/arm/goal topic rejected while released, allowed when enabled, unknown or stale; arm routes 409 while released; release and stop routes allowed.
- `README.md` `### Servo torque and neutral` documents the quieting.

## Acceptance conditions

- a1: WS publishes to a released group's command topics are rejected and logged; unknown state allows [evidence: test: nodes/web_ui/tests/test_servo_quiet.py]
- a2: arm motion routes answer 409 while the arm is released; release and stop stay allowed [evidence: test: nodes/web_ui/tests/test_servo_quiet.py]
- a3: released_command_error exists [evidence: symbol: nodes/web_ui/web_ui/servo_calibration.py::released_command_error]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/servo_calibration.py`
- `nodes/web_ui/tests/test_servo_quiet.py`
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
