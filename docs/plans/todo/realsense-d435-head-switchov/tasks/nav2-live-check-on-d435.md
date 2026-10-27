---
title: Nav2 live check on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "Nav2's use of the D435 is proven on the real floor before anyone relies on it."
files: [docs/runbooks/d435-switchover.md, tests/test_d435_runbook.py, tests/README.md]
verify: "uv run pytest tests/test_d435_runbook.py -q && uv run poe lint"
depends_on: [nav2-ground-layer-and-keepou, ground-live-acceptance-on-d4]
model: sonnet
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

# Nav2 live check on D435

## Requirement

LIVE task. Goal G4. Design D6. Supersedes tof-3d-obstacle-detection-fo final-suite-and-runbook (D15), and drivable-ground-tof-filterin nav2-goal-across-the-scene and retro-fixes-live-check (amended 2026-10-27: both needed the retired ToF ground layer / front camera; one robot session instead of two). Motion rule (CLAUDE.md least-dramatic): short, slow goals only (<= 1 m, nav speed limits as configured), user watching; the arm stays parked.

1. `./scripts/deploy-nodes.sh client nav2_bringup 2>&1 | tee .logs/deploy-client-nav2.log`.
2. Empty floor in front of the robot: the local costmap (web_ui Map tab costmap layer) shows no D435 phantom cost within 2 m over 2 min with the head at home and at pan +-30 deg.
3. A 5 cm box 1 m ahead: it appears in the local costmap; a 1 m goal through it is planned around it.
4. A table-edge or step-down drop: CliffStop stops motion toward it (collision monitor state via Mimir/Loki, no rclpy).
5. `docs/runbooks/d435-switchover.md` section "Nav2": each step with its observation and the evidence source (screenshot path under docs/runbooks/img/ or log line); `tests/test_d435_runbook.py::test_nav2_section_is_recorded`.

Retro-fix checks carried from drivable-ground-tof-filterin retro-fixes-live-check (same session, after step 4). Before step 1 also deploy laser_filter, mcp_server and web_ui from main (check first that no agent session is running: deploying mcp_server kills it).
6. Final turns: three navigate_to_pose goals that end with an in-place turn complete with no collision_monitor "invalid source" stop. If one stops, read the laser_filter stall metrics (scan gap and lag, input and output, executor tick gap) in Mimir for that second, name the cause with the numbers in nodes/nav2_bringup/README.md section `### Invalid source stop during the final turn`, and record it.
7. Backing and turns (mcp_server move_relative): with the box 0.34 m front-left, dx -0.35 backs straight and is not stopped by StopBox; a dyaw -0.6 is refused before any command with a message naming the return.
8. Rotate sweep: a return placed in a corner of the rotation circle (outside the old |y| 0.2158 box) stops a Nav2 in-place rotation.
9. Keep-out: with a keep-out drawn over the mirror wardrobe, no frontier is listed inside it.
10. Wi-Fi bridges: map3d_uplink, leader_bridge and master2master load /etc/ros2/fastdds_wifi_bridge.xml (journal shows no profile error) and keep publishing; note whether FASTDDS_BUILTIN_TRANSPORTS conflicts with useBuiltinTransports=false.
11. docs/runbooks/d435-switchover.md section "Retro fixes": one row per step 6-10 with date, numbers and PASS or FAIL; tests/test_d435_runbook.py::test_retro_fixes_section_is_recorded.

## Acceptance conditions

- AC1: nav2_bringup is deployed with the D435 inputs [evidence: command:./scripts/deploy-nodes.sh client nav2_bringup]
- AC2: no phantom costs on an empty floor, the box is costed, CliffStop stops at the drop, all recorded [evidence: test:tests/test_d435_runbook.py::test_nav2_section_is_recorded]
- AC3: final turns complete with no invalid-source stop (or the cause is named with Mimir numbers in the nav2_bringup README), backing works, the turn is refused near the box, the rotate sweep stops a corner return, no frontier inside the keep-out, bridges load the non-blocking profile; all recorded [evidence: test:tests/test_d435_runbook.py::test_retro_fixes_section_is_recorded]

## Files

- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add `test_nav2_section_is_recorded`; run `uv run pytest tests/test_d435_runbook.py -q`; expect a failure.
- Run the live steps with the user present; record observations and evidence paths.
- Rerun; commit `docs(runbooks): Nav2 on the D435, live check`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: absorbs drivable-ground nav2-goal-across-the-scene and retro-fixes-live-check (ToF ground layer / front camera retired)
