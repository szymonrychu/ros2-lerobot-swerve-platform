---
title: Collision source stall fix
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Nav2 goals finish their final turn instead of being cut by a spurious invalid-source stop
files: [nodes/laser_filter/laser_filter/node.py, nodes/nav2_bringup/config/nav2_params.yaml, tests/test_collision_source_stall.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_collision_source_stall.py -q && uv run poe lint && (cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run poe lint)"
depends_on: []
owns: [collision-source-fix]
uses: []
model: opus
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--collision-source-stall-fix
branch: drivable-ground-tof-filterin--collision-source-stall-fix
verified_head: ""
run: ""
provides: []
---

# Collision source stall fix

## Requirement

Find and fix why Nav2's collision_monitor stopped the robot with polygon 'invalid source' during the final in-place
turn of a navigate_to_pose goal (robot agent retrospective 2026-10-22 16:32, "explore other end of the room": "the final
in-place turn of the navigate goal was cut by a collision_stop 'invalid source' (spurious); the position was reached but
the heading was not"; nearest lidar return 0.7 m). Design D55; approval R1.

Evidence already pulled (Loki, saved under `/Users/szymonri/Documents/ros2-lerobot-sverve-platform/.logs/agent-retro-2026-10-22/`,
unit logs 16:20-16:34 CEST): at 1792679435.16 (16:30:35) controller_server and the local costmap logged TF
"Lookup would require extrapolation into the future" (map/odom/base_link), the rotation shim aborted, then
collision_monitor logged `[tof_obstacles]` and `[scan]` "Latest source and current collision monitor node timestamps
differ on 1.01-1.91 seconds. Ignoring the source." for about 1 s and "Robot to stop due to invalid source"; the
controller loop had missed its 10 Hz rate in the seconds before. Both sources going stale together points at a common
cause upstream (TF / odometry / EKF publishing late, the client CPU, DDS, or the nav2 container's executors), not at one
sensor.

- Diagnose first, before any change: from Loki (`http://server.ros2.lan:3100`) and Mimir
  (`http://server.ros2.lan:9009/prometheus`, the API `scripts/node_cpu_compare.py` uses) for 16:30:25-16:30:40 CEST,
  read the client CPU per service, the publish rates or ages of `/scan_filtered`, `/tof/points_filtered`,
  `/odometry/filtered` and TF that exist as metrics, and the logs of robot_localization_ekf, rf2o, slam_toolbox,
  laser_filter and nav2. Write the named cause with the numbers into `nodes/nav2_bringup/README.md` section
  `### Invalid source stop during the final turn` (under its collision monitor part).
- Reproduce the cause in a test first (seen failing): `tests/test_collision_source_stall.py` (it may import the node
  module or read the shipped config, whichever layer holds the cause).
- Fix the cause where it is. Keep the fail-safe (user decision 2026-10-18, comment in nav2_params.yaml): an enabled
  source with no fresh data for `source_timeout` 1.0 s still stops the robot, `source_timeout` is not raised and no source
  is disabled; contract nav2-tof-voxel-bands (collision_monitor tof_obstacles/tof_cliff bands) unchanged. If the
  evidence shows the only remedy is a timeout or fail-safe change, stop and escalate with the numbers instead.
- `tests/test_collision_source_stall.py` also pins the fail-safe: every collision_monitor observation source is enabled
  and the node `source_timeout` is 1.0 s in the shipped nav2_params.yaml.
- `nodes/nav2_bringup/README.md` (collision monitor part) names the cause and the fix in two lines; `tests/README.md`
  lists the new test file.
- A cause in a file outside this task's list (for example `ansible/group_vars/client.yml`, ordered among many tasks of
  this epic) is reported as a `discovered` entry with the exact change, not edited here.
- Type hints, docstrings with types, constants on top (CLAUDE.md).

## Acceptance conditions

- a1: the cause of the 16:30:35 invalid-source stop is named in the nav2_bringup README with Loki and Mimir numbers [evidence: doc-section:nodes/nav2_bringup/README.md#Invalid source stop during the final turn]
- a2: a regression test reproduces the cause and passes with the fix [evidence: command:uv run pytest tests/test_collision_source_stall.py -q]
- a3: the stale-source fail-safe is kept: every source enabled and source_timeout 1.0 s [evidence: test:tests/test_collision_source_stall.py::test_fail_safe_stale_source_stop_kept]
- a4: root lint and the laser_filter suite pass [evidence: command:uv run poe lint && cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run poe lint]
- a5: the fix acts on the named cause, not on the timeout or the fail-safe [evidence: judgement]

## Files

- `nodes/laser_filter/laser_filter/node.py`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_collision_source_stall.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

Builds part collision-source-fix (the named cause fixed at its layer). Keeps contract nav2-tof-voxel-bands
(collision_monitor sources and bands) and the stale-source fail-safe (user decision 2026-10-18).

## Expected outcome

the README section names the cause of the 16:30:35 stop with numbers; `uv run pytest tests/test_collision_source_stall.py -q`
passes with a test that failed before the fix; the fail-safe pin passes; no timeout raised.

## Steps

- Read the saved logs and query Loki/Mimir for 16:30:25-16:30:40 CEST; write the cause and numbers to the README section.
- Write the reproducing test and the fail-safe pin; run `uv run pytest tests/test_collision_source_stall.py -q`: the
  reproducing test FAILS.
- Fix the cause; the tests PASS; run the verify command; commit `fix: <layer> stall behind the invalid-source stop`.

## Notes

- Live step: none (code, run 1 of D29). Proven live by retro-fixes-live-check at live step 4.
- Model opus: diagnosis across TF, executors and CPU; review full.
- Saved evidence: `/Users/szymonri/Documents/ros2-lerobot-sverve-platform/.logs/agent-retro-2026-10-22/` (nav2_bringup,
  mcp_server, web_ui, slam_toolbox, map_guard_cpp, bno055_imu, claude_agent logs 16:20-16:34 CEST and the
  retrospectives JSON); Loki keeps more units (robot_localization_ekf and laser_filter had no lines in the window).
