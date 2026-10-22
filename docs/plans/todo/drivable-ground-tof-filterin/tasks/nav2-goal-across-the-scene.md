---
title: Nav2 goal across the scene
status: todo
repo: ros2-lerobot-sverve-platform
value: Proof that the robot drives around the box on the ground layer instead of finding no path
files: [docs/runbooks/ground-nav2-check.md, tests/test_ground_nav2_check_runbook.py, tests/README.md]
verify: "uv run pytest tests/test_ground_nav2_check_runbook.py -q && uv run poe lint"
depends_on: [ground-voxel-layer-on]
owns: []
uses: [ground-layer-on]
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Nav2 goal across the scene

## Requirement

Live task, driving with the user present: with `ground_voxel_layer` on, Nav2 reaches a short goal across the user's
scene (design D25, D28; approval G1). It uses part ground-layer-on.

Precondition, checked first and stopped on when not met (report `blocked`, `whoCanClear: user`):
- ground-voxel-layer-on is merged and the orchestrator has deployed nav2_bringup (live step 4);
- the user is at the robot, has undocked it and can stop it (web UI Stop or the e-stop); the scene is in place and the
  light on; the area beyond the scene is clear.

- New runbook `docs/runbooks/ground-nav2-check.md`:
  - `## Safety`: the user present at the robot for every goal; goals at most 1.5 m from the start pose; Nav2's speed
    cap (0.25 m/s) unchanged; one goal at a time; stop on any contact or unexpected motion;
  - `## Procedure`: through the robot MCP tools of mcp_server only (`take_control` subsystem base, `move_relative` or
    `navigate_to_pose`, `release_control`); before each goal, the local costmap through the web_ui (Map tab or its
    websocket) is checked for lethal cells on the board and the empty floor (cells inside the `truth.yaml` polygons);
    goal A: a pose on the far side of the scene whose straight line passes the box, so the path must go around it;
    goal B: back to the start pose;
  - `## Acceptance`: both goals end SUCCEEDED within their timeout; the robot never touches the box, the ramp or the
    board edge (the user confirms); no lethal cell on the board or the empty floor in the local costmap during the
    run; the box is marked lethal;
  - `## Results`: date, goal, result, duration, distance, lethal cells counted on board / empty floor / box, the
    user's confirmation, PASS/FAIL per row.
- `tests/test_ground_nav2_check_runbook.py`: `test_runbook_sections`, `test_runbook_requires_user_present_and_short_goals`
  (the Safety section names the user's presence and the 1.5 m limit), `test_runbook_uses_mcp_and_web_ui_only` (no
  `ros2`, `rclpy` or `ssh`); `tests/README.md` entry.
- A goal that fails names the Nav2 result and the costmap cells that blocked it; the task then escalates with them.

## Acceptance conditions

- a1: the runbook has its Safety, Procedure, Acceptance and Results sections [evidence: test:tests/test_ground_nav2_check_runbook.py::test_runbook_sections]
- a2: the runbook requires the user present and goals of at most 1.5 m [evidence: test:tests/test_ground_nav2_check_runbook.py::test_runbook_requires_user_present_and_short_goals]
- a3: the runbook drives through mcp_server and reads through web_ui only [evidence: test:tests/test_ground_nav2_check_runbook.py::test_runbook_uses_mcp_and_web_ui_only]
- a4: the Results hold both goals measured on the robot [evidence: doc-section:docs/runbooks/ground-nav2-check.md#Results]
- a5: Nav2 reached the goal across the scene and back with the user present, no contact, and no lethal cell on the board or the empty floor [evidence: judgement]

## Files

- `docs/runbooks/ground-nav2-check.md`
- `tests/test_ground_nav2_check_runbook.py`
- `tests/README.md`

## Interfaces

Uses part ground-layer-on (the deployed Nav2 params). Drives only through mcp_server's guarded tools
(`take_control`, `move_relative` / `navigate_to_pose`, `release_control`; base lease, battery and tilt gates) and
reads the costmap through web_ui. Writes `docs/runbooks/ground-nav2-check.md` and its test.

## Expected outcome

`uv run pytest tests/test_ground_nav2_check_runbook.py -q` PASS; the runbook Results show goals A and B SUCCEEDED with
the user present, no contact, and no lethal cell on the board or the empty floor.

## Steps

- Write `tests/test_ground_nav2_check_runbook.py`; it FAILS (no runbook).
- Write the runbook Safety, Procedure and Acceptance; the test PASSES.
- With the user at the robot: count lethal cells, send goal A (<= 1.5 m), then goal B; record each result.
- Fill Results; `uv run pytest tests/test_ground_nav2_check_runbook.py -q && uv run poe lint` PASS; commit
  `docs(runbooks): Nav2 goal across the ground scene`.

## Notes

- Live task at live step 4 (design D29): the only driving in this epic. Needs the user present at the robot, the
  robot undocked by the user, the scene in place, the light on; short slow goals only (Nav2 caps 0.25 m/s).
- The robot MCP server must be reachable (the orchestrator's `robot` MCP tools or the mcp client pattern of
  `nodes/claude_agent/claude_agent/robot_stop.py::call_robot_tool`); a refused lease or battery gate is reported,
  never worked around.
- Deploying mcp_server kills a running agent session (MEMORY): nothing is deployed in this task.
