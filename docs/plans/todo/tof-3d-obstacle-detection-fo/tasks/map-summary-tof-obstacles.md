---
title: Map summary ToF obstacles
status: todo
repo: ros2-lerobot-sverve-platform
value: The agent can ask get_map_summary what 3D obstacles and cliffs are near the robot.
files: [nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/tool_context.py, nodes/mcp_server/mcp_server/perception_tools.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/tests/fakes.py, nodes/mcp_server/tests/test_map_summary_tof.py, nodes/mcp_server/README.md, ansible/group_vars/client.yml]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q"
depends_on: [tof-obstacle-sector-summary, web-ui-obstacle-points-api]
owns: []
uses: [tof-sector-summary, tof-points-filtered-topic]
model: sonnet
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map summary ToF obstacles

## Requirement

Test first: `tests/test_map_summary_tof.py` fails before the wiring exists (D11).

- `config.py` `TopicSettings.tof_points: str = "/tof/points_filtered"` (comment: subscribed lazily by
  get_map_summary, latest message only); a staleness bound for it (reuse an existing timeouts field or add
  `timeouts.tof_points_stale_s`, default 1.0).
- `ros_iface.py`: lazy, latest-only subscription to `tof_points` (same on-demand pattern as `local_costmap`), returning
  the points transformed to `base_frame` with the TF at the cloud stamp, plus stamp and frame; None when no cloud, a
  stale cloud or no TF.
- `tool_context.py` `RobotApi` protocol method for it; `tests/fakes.py` FakeRobot implements it.
- `perception_tools.py` `get_map_summary`: adds `obstacles_3d` (`Obstacles3d` from `tof_obstacle_sectors` with the
  STVL bands 0.06-0.6 m and cliff below -0.10 m, range 2.0 m) or null with a short reason when unavailable; the tool
  description mentions it (ToF on the head, only what the head currently sees).
- `ansible/group_vars/client.yml` mcp_server `topics:` sets `tof_points: /tof/points_filtered` with a comment.
- `tests/test_map_summary_tof.py`: summary with a fake cloud (obstacle ahead, cliff left), null when stale / missing,
  description text; root `tests/test_mcp_server_config.py` stays green (client.yml/README/source coupling).
- `nodes/mcp_server/README.md` tool table row and topics row updated.

## Acceptance conditions

- a1: get_map_summary returns obstacles_3d from a fresh ToF cloud in base_link [evidence: test: nodes/mcp_server/tests/test_map_summary_tof.py]
- a2: obstacles_3d is null when the cloud is missing, stale or has no TF [evidence: test: nodes/mcp_server/tests/test_map_summary_tof.py]
- a3: the ToF topic is subscribed lazily, latest only [evidence: judgement]
- a4: client.yml mcp_server topics names tof_points and the root coupling test passes [evidence: command: uv run pytest tests/test_mcp_server_config.py -q]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/fakes.py`
- `nodes/mcp_server/tests/test_map_summary_tof.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
