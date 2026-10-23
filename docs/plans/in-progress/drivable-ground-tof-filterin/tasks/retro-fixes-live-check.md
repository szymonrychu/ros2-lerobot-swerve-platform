---
title: Retro fixes live check
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Proof on the robot that the final turns, backing, mirror keep-out and front-camera positions work"
files: [docs/runbooks/agent-retro-fixes.md, tests/test_agent_retro_runbook.py, tests/README.md]
verify: "uv run pytest tests/test_agent_retro_runbook.py -q && uv run poe lint"
depends_on: [nav2-goal-across-the-scene, collision-source-stall-fix, stopbox-backing-and-turns, frontiers-honour-keep-outs, front-camera-ground-points, agent-retro-skill-lessons, tilt-event-evidence, map-epoch-reset-event]
owns: []
uses: [collision-source-fix, move-relative-clearance, frontier-keepout, front-ground-points]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--retro-fixes-live-check
branch: drivable-ground-tof-filterin--retro-fixes-live-check
verified_head: ""
run: ""
provides: []
---

# Retro fixes live check

## Requirement

Prove the retrospective fixes on the robot (design D62; approvals R1, R2, R4, R5). Live step 4 of design D29, after
nav2-goal-across-the-scene, with the user at the robot (undocked by the user), Nav2's 0.25 m/s cap unchanged.

Preconditions (each missing one: stop with `blocked`, `whoCanClear: user`): the user at the robot; the orchestrator has
deployed laser_filter (if collision-source-stall-fix changed it), nav2_bringup, mcp_server, web_ui and claude_agent from
main (no agent session running: deploying mcp_server or claude_agent kills it, MEMORY); the map holds the hallway with
the mirror wardrobe. Live data only through mcp_server tools, the web UI, Loki and Mimir; no rclpy, `ros2` CLI or SSH on
the client.

- `docs/runbooks/agent-retro-fixes.md` gets `## Procedure` and `## Results` (date, numbers, PASS/FAIL per row), logs in
  `.logs/`:
  1. Final turns: four `navigate_to_pose` goals of at most 1.5 m, each ending with an in-place turn of >= 90 deg: every
     final heading reached within the goal tolerance, and Loki shows no collision_monitor "invalid source" line in the
     window (R1).
  2. Backing: the user places the box about 0.34 m front-left of the robot; `move_relative` dx -0.35 completes without
     a StopBox stop; `move_relative` dyaw -0.6 is refused before moving, naming the box (R4).
  3. Mirror: the user draws a keep-out over the ghost area behind the mirror wardrobe in the web UI;
     `get_exploration_status` lists no frontier inside it and reports them in `excluded_keepout` (R5).
  4. Front camera: an object at tape-measured spots 1.0 m and 2.0 m ahead of base_link; `pixel_to_ground` on a fresh
     default front picture gives the spot within 0.05 m at 1.0 m and 0.10 m at 2.0 m (my call) (R2).
- A FAIL row escalates with the measured numbers; it never closes the task and no threshold is loosened.
- `tests/test_agent_retro_runbook.py` (`test_runbook_has_results_rows`): the Results table has the four rows with a
  date and PASS or FAIL each; `tests/README.md` lists it.

## Acceptance conditions

- a1: the runbook Results hold the four rows with date, numbers and PASS or FAIL [evidence: doc-section:docs/runbooks/agent-retro-fixes.md#Results]
- a2: the Results table is complete [evidence: test:tests/test_agent_retro_runbook.py::test_runbook_has_results_rows]
- a3: the final turns complete with no invalid-source stop, backing works and the turn is refused near the box [evidence: judgement]
- a4: no frontier is listed inside the drawn keep-out and front-camera floor points are within 0.05 m at 1 m and 0.10 m at 2 m [evidence: judgement]
- a5: root tests and lint pass [evidence: command:uv run pytest tests/test_agent_retro_runbook.py -q && uv run poe lint]

## Files

- `docs/runbooks/agent-retro-fixes.md`
- `tests/test_agent_retro_runbook.py`
- `tests/README.md`

## Interfaces

Uses parts collision-source-fix, move-relative-clearance, frontier-keepout and front-ground-points.
`docs/runbooks/agent-retro-fixes.md` `## Procedure` and `## Results`.

## Expected outcome

The Results table has four dated rows with numbers and PASS or FAIL; `uv run pytest tests/test_agent_retro_runbook.py -q`
passes.

## Steps

- Check every precondition; stop `blocked` (whoCanClear: user) when one is missing.
- Write the Procedure; run rows 1-4 with the user; fill Results with numbers; logs in `.logs/`.
- Write `tests/test_agent_retro_runbook.py`; run the verify command; commit `docs: agent retro fixes live check results`.

## Notes

- LIVE step 4 of D29 (user at the robot, undocked by the user; short slow goals only), after nav2-goal-across-the-scene.
- Driving only through mcp_server's guarded tools under the base lease; no rclpy, `ros2` CLI or SSH on the client.

## Added 2026-10-23

Also record the head-from-arm step 5 (docs/runbooks/head-arm-calibration.md): arm at the base pose without the board, `head_look_at` on the tool point, the tool point pixel offset from the head camera image centre in deg, after the applied chain r-20261023-112701 (and the CAD chain for comparison if it is quick to restore). Fill the runbook row.
