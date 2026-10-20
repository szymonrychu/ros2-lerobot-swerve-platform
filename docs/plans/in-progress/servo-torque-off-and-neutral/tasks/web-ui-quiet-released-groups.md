---
title: Web UI quiet released groups
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--web-ui-quiet-released-groups
branch: servo-torque-off-and-neutral--web-ui-quiet-released-groups
verified_head: ""
run: servo-torque-off-and-neutral
provides: []
acs_for_audit: []
process_gaps: []
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

- Uses part `servo-calibration-api` (`ServoGroupConfig`, `BridgeNode.servo_torque_sample`).
- `ServoGroupConfig.command_topics: list[str]`; `released_command_error(config, bridge_node, topic) -> str | None`.

## Expected outcome

The web UI itself cannot command a released group: no head slider, arm jog, home, grasp or nav goal reaches it.

## Steps

- Write `tests/test_servo_quiet.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_servo_quiet.py -q`; expect failures.
- Implement; run `uv run --frozen pytest tests/test_servo_quiet.py tests/test_head_control.py tests/test_arm_api.py -q`; then the verify command; commit `feat(web_ui): quiet commands to released servo groups`.

## Ruling

- 2026-10-20 accepted: Verifier agents died / empty merge-base diff after early merge to main; manual verify: mcp 26 passed + lint clean, web_ui 55 passed + lint clean, both trees clean; all merged to main and full suites green (mcp 2800, web_ui 1024, vitest 1196, feetech 425, root 1857)

## Notes

- The WS publish chain is in `server.py` around lines 1300-1320 (battery cut-off and head range checks); add the check there, same shape.
- `call_arm_trigger` (server.py:1055) is the arm route helper.

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on web-ui-servo-routes
