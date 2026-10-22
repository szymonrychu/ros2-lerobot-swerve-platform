---
title: StopBox backing and turns
status: todo
repo: ros2-lerobot-sverve-platform
value: "The robot can back away from a nearby obstacle, and a turn that would hit one is refused before it moves"
files: [nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/look_around.py, nodes/nav2_bringup/config/nav2_params.yaml, nodes/mcp_server/tests/test_move_relative_clearance.py, docs/runbooks/agent-retro-fixes.md, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_move_relative_clearance.py -q && uv run poe lint"
depends_on: [collision-source-stall-fix, mcp-head-kinematics-applied]
owns: [move-relative-clearance]
uses: []
model: opus
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# StopBox backing and turns

## Requirement

Stop StopBox from cutting short backing moves, and refuse turns that would sweep into an obstacle before moving
(robot agent retrospective 2026-10-22 16:28, "Explore": "StopBox fired immediately on move_relative dx -0.35 (backing
up) and on dyaw -0.6 near the wardrobe corner (front_left obstacle at 0.34 m), yet drive vy -0.12 then vx 0.1/vy -0.1
worked at once"). Design D56; approval R4.

Context: `move_relative` (`nodes/mcp_server/mcp_server/ros_iface.py:move_relative`) sends a Nav2 navigate goal; the
controller is RotationShimController around MPPI with `rotate_to_heading_once` and "front first" (nav2_params.yaml), so
a goal behind the robot starts with an in-place turn, which selects StopBox's `rotate` sub-polygon (bounding box + 2 cm,
front edge x 0.3425) instead of `back`; `drive()` publishes twists directly and selects the directional sub-polygons.
StopBox stops in the saved logs (`.logs/agent-retro-2026-10-22/nav2_bringup.log`): 16:24:39, 16:25:02-16:25:09,
16:26:11-16:26:20, 16:27:49-16:29:12 CEST.

- Diagnose first: from the saved logs and Loki, which StopBox sub-polygon tripped at each stop and what Nav2 commanded
  (in-place rotation or backing); write it to `docs/runbooks/agent-retro-fixes.md` section
  `## StopBox on backing and turning`.
- Backing and sideways moves: a `move_relative` whose translation is <= 0.5 m with dx < 0 or a sideways component and
  dyaw 0 is executed without an in-place turn first (the holonomic base backs or slides straight, for example through
  Nav2's BackUp / DriveOnHeading behaviours or a timed, odometry-checked twist on cmd_vel_nav like `drive()`; the
  builder picks within the collision monitor chain: every command still passes velocity_smoother and
  collision_monitor). Backing away from an obstacle 0.34 m front-left then selects the `back` sub-polygon and is not
  stopped.
- Turns: a `move_relative` with dyaw != 0 (or a goal that needs an in-place turn) is refused before any command when the
  latest scan shows a return inside the rotation sweep (footprint circumscribed radius + the look_around clearance
  margin, the check `look_around` already uses; reuse it, do not copy it), with a message naming the nearest return
  (bearing, distance) and suggesting a translation away first. No placeholder pose: no scan -> refused as today.
- StopBox polygons change only through `scripts/gen_nav_footprint.py` (with its tests) if the diagnosis shows a polygon is
  wrong; otherwise unchanged.
- Tests first (seen failing): `nodes/mcp_server/tests/test_move_relative_clearance.py`
  (`test_backing_move_does_not_turn_first`, `test_turn_refused_when_obstacle_in_sweep`,
  `test_turn_allowed_when_sweep_clear`, and `test_back_polygon_clear_of_front_left_obstacle`, which reads the shipped
  `nodes/nav2_bringup/config/nav2_params.yaml` by its repo path: with the shipped StopBox polygons, a 4-point obstacle cluster
  0.34 m from base_link at any bearing from 0 to +60 deg (front-left, as in the retrospective) lies outside the `back`
  and `back_left` sub-polygons).
- `nodes/mcp_server/README.md` move_relative entry updated; tool description states the backing and refusal behaviour.
- Type hints, docstrings with types, constants on top (CLAUDE.md).

## Acceptance conditions

- a1: the StopBox stops of the 16:28 job are explained per stop in the runbook [evidence: doc-section:docs/runbooks/agent-retro-fixes.md#StopBox on backing and turning]
- a2: a short backing move runs without an in-place turn first [evidence: test:nodes/mcp_server/tests/test_move_relative_clearance.py::test_backing_move_does_not_turn_first]
- a3: a turn with a scan return inside the rotation sweep is refused before moving, naming the return [evidence: test:nodes/mcp_server/tests/test_move_relative_clearance.py::test_turn_refused_when_obstacle_in_sweep]
- a4: a turn with a clear sweep is still executed [evidence: test:nodes/mcp_server/tests/test_move_relative_clearance.py::test_turn_allowed_when_sweep_clear]
- a5: with the shipped polygons the back sub-polygons are clear of a front-left obstacle 0.34 m from base_link [evidence: test:nodes/mcp_server/tests/test_move_relative_clearance.py::test_back_polygon_clear_of_front_left_obstacle]
- a6: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/look_around.py`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `nodes/mcp_server/tests/test_move_relative_clearance.py`
- `docs/runbooks/agent-retro-fixes.md`
- `nodes/mcp_server/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
