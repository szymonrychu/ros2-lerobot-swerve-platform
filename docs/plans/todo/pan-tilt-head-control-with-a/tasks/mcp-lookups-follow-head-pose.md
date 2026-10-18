---
title: MCP lookups follow head pose
status: todo
repo: ros2-lerobot-sverve-platform
value: "Photos, frontier facing and approach view checks record where the head camera really looked."
files: [nodes/mcp_server/mcp_server/lookup_tools.py, nodes/mcp_server/mcp_server/frontiers.py, nodes/mcp_server/mcp_server/approach_tools.py, nodes/mcp_server/tests/test_lookup_head_pose.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-front-camera-on-head, mcp-lease-release-homes-head]
owns: []
uses: [head-controller, front-camera-head-scene]
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

# MCP lookups follow head pose

## Requirement

- `lookup_tools.py` (D11): `camera_pose_base('front', ...)` composes the head parent pose (measured head angles) with the mount instead of identity; when head joint states are stale it returns None (the view then has no camera pose, never a base_link guess).
- `frontiers.py`: the facing of a front lookup comes from its recorded `camera_pose_base` (already supported); the base-heading fallback stays only for records without one, and its docstring says so (no "the front camera looks along the base heading" claim for new records).
- `approach_tools.py`: `camera_scene` / `camera_view_problem` build the front scene with the current measured head pose (the check assumes the head keeps its pose while the base moves; the docstring says so); stale head joints give the reason "head joint states stale".
- `nodes/mcp_server/README.md`: the Frontiers and camera geometry paragraphs describe the head-mounted front camera.
- Tests first in new `nodes/mcp_server/tests/test_lookup_head_pose.py`, seen failing: a front lookup records the head-composed camera pose (pan +45 deg yields a view yaw of base yaw + 45 deg); stale head joints record no camera pose; frontier facing uses the recorded pose; approach view problem names stale head joints.

## Acceptance conditions

- a1: front lookups record the head-composed camera pose and none when head joints are stale [evidence: test: nodes/mcp_server/tests/test_lookup_head_pose.py]
- a2: frontier facing and approach view checks use the head pose [evidence: test: nodes/mcp_server/tests/test_lookup_head_pose.py]
- a3: the README describes the head-mounted front camera in the Frontiers section [evidence: judgement]
- a4: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/lookup_tools.py`
- `nodes/mcp_server/mcp_server/frontiers.py`
- `nodes/mcp_server/mcp_server/approach_tools.py`
- `nodes/mcp_server/tests/test_lookup_head_pose.py`
- `nodes/mcp_server/README.md`

## Interfaces

- `lookup_tools.py:LookupRecorder.camera_pose_base` (or its current owner), `frontiers.py` facing helper, `approach_tools.py:camera_view_problem`.

## Expected outcome

Recorded views, frontier facing and approach checks use the real head camera pose.

## Steps

- Write `tests/test_lookup_head_pose.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_lookup_head_pose.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): lookups use the head camera pose`.

## Notes

- Persisted lookup records keep their schema (`camera_pose_base`); only the values change.
