---
title: MCP approach tools front only
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Approach and frontier looking work from the D435, and stop honestly when it cannot see the target."
files: [nodes/mcp_server/mcp_server/approach_tools.py, nodes/mcp_server/mcp_server/frontier_tools.py, nodes/mcp_server/mcp_server/target_tracker.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/tests/test_approach_tools.py, nodes/mcp_server/tests/test_approach.py, nodes/mcp_server/tests/test_frontier_tools.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_approach_tools.py tests/test_approach.py tests/test_frontier_tools.py tests/test_target_tracker.py -q && uv run poe lint"
depends_on: [mcp-camera-tools-front-only]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# MCP approach tools front only

## Requirement

Goal G7. User decision 4. Design D8.

- `mcp_server/approach_tools.py`: `CAMERAS` replaced by camera_scene's `CAMERAS` ("front" only). The servoed approach tracks on front frames only; the `camera == "gripper"` arm-joint branch is removed. When the D435 loses the target near the robot (inside min-Z 0.15 m or out of the colour FOV), the approach stops with that reason. It never falls back to a placeholder pixel.
- `mcp_server/frontier_tools.py`: the `camera == "gripper"` branch of `look_toward_frontier` is removed; `config.py` `look_camera` becomes `Literal["front"]`.
- `mcp_server/target_tracker.py`: front frames only (no camera-specific code expected; adjust if present).
- Tests: `tests/test_approach_tools.py`, `tests/test_approach.py`, `tests/test_frontier_tools.py`: gripper cases removed or turned into front cases (named in the commit). New `test_approach_tools.py::test_lost_target_inside_min_z_stops_with_reason`.

## Acceptance conditions

- AC1: approach tools offer and track only the front camera [evidence: test:nodes/mcp_server/tests/test_approach_tools.py::test_approach_offers_only_front]
- AC2: a target lost inside the D435 min-Z stops the approach with that reason [evidence: test:nodes/mcp_server/tests/test_approach_tools.py::test_lost_target_inside_min_z_stops_with_reason]
- AC3: look_toward_frontier uses the front camera [evidence: test:nodes/mcp_server/tests/test_frontier_tools.py::test_look_toward_frontier_uses_front]
- AC4: approach and frontier suites pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_approach_tools.py tests/test_approach.py tests/test_frontier_tools.py tests/test_target_tracker.py -q]

## Files

- `nodes/mcp_server/mcp_server/approach_tools.py`
- `nodes/mcp_server/mcp_server/frontier_tools.py`
- `nodes/mcp_server/mcp_server/target_tracker.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_approach_tools.py`
- `nodes/mcp_server/tests/test_approach.py`
- `nodes/mcp_server/tests/test_frontier_tools.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the new tests and convert gripper cases; run the four test files; expect failures.
- Change approach, frontier, config and tracker; rerun; expect pass.
- Run the verify command; commit `refactor(mcp_server): approach and frontier tools on the front D435 camera`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
