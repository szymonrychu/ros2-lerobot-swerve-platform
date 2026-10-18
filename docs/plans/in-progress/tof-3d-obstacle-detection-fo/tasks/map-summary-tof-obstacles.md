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

- a1: get_map_summary returns obstacles_3d from a fresh ToF cloud in base_link [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_summary_tof.py -q]
- a2: obstacles_3d is null when the cloud is missing, stale or has no TF [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_summary_tof.py -q]
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

- Consumes `tof-sector-summary` and `tof-points-filtered-topic`.
- New `RobotApi` method (name chosen by the builder) returning base_link points, stamp and frame or None.

## Expected outcome

`get_map_summary` gives the agent a per-sector 3D obstacle and cliff view from the head ToF, or an explicit null.

## Steps

- Write `tests/test_map_summary_tof.py`; run it; it fails.
- Add the topic setting, RobotApi method, ros_iface subscription, fake, tool output; rerun; run the verify command.
- README and client.yml; commit `feat(mcp_server): 3D obstacles in get_map_summary`.

## Notes

- Deploying mcp_server kills a running agent session (auto-memory): the runbook checks first. mcp_server stays Python.
- No new tool, so the claude_agent tool lists stay unchanged.
