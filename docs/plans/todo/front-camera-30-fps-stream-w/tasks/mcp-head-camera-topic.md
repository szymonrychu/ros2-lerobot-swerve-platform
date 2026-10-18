---
title: MCP head camera topic
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent's camera tool reads the head camera stream, described as the head camera."
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/tests/test_config.py, nodes/mcp_server/tests/test_on_demand_subscriptions.py, nodes/mcp_server/README.md, ansible/group_vars/client.yml, tests/test_mcp_server_config.py, tests/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py tests/test_mcp_camera_config.py -q && uv run poe lint"
depends_on: []
owns: [mcp-head-camera-topic]
uses: []
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

# MCP head camera topic

## Requirement

- mcp_server config `topics.front_camera` (Jetson end state `/front_camera/image_raw/compressed`) becomes `topics.head_camera` with default `/head_camera/image_raw/compressed` (D10, U9); `ros_iface.camera_topics` maps the tool's camera value `front` to `t.head_camera`. No `front_camera` key or `/front_camera` topic remains in mcp_server code, tests or README. The tool's camera value stays `front` (D10).
- `ansible/group_vars/client.yml` mcp_server `topics:` block: `head_camera: /head_camera/image_raw/compressed` replaces `front_camera`, its comment says head RGB camera (Camera Module 3 on the pan/tilt head with the ToF, head_camera_cpp, 1152x648 at 30 fps, 16:9).
- `nodes/mcp_server/README.md`: the `get_camera_image` row and the on-demand topics paragraph name `topics.head_camera`, describe `front` as the head RGB camera (1152x648 at 30 fps, 16:9, autofocus, default picture 384x216) and drop the 640x480 / 10 fps / overhead wording.
- Tests first, seen failing: `tests/test_config.py` (default `topics.head_camera`, no `front_camera` field accepted: extra keys forbidden); `tests/test_on_demand_subscriptions.py` uses `/head_camera/image_raw/compressed` for the front camera; root `tests/test_mcp_server_config.py`: the client.yml front-camera pin becomes `test_mcp_server_head_camera_is_the_head_camera_stream` (`topics.head_camera == /head_camera/image_raw/compressed`, no `front_camera` key) and the README size pin checks the 1152x648 / 384x216 wording and no 640x480 front size. `tests/README.md` rows follow the renamed tests.

## Acceptance conditions

- a1: the front camera value reads topics.head_camera, default /head_camera/image_raw/compressed [evidence: test: nodes/mcp_server/tests/test_config.py]
- a2: client.yml sets topics.head_camera and no front_camera key [evidence: test: tests/test_mcp_server_config.py::test_mcp_server_head_camera_is_the_head_camera_stream]
- a3: no mcp_server code, test or README names /front_camera or topics.front_camera [evidence: command: ! git grep -n -E '/front_camera|front_camera:|topics.front_camera|t.front_camera' -- nodes/mcp_server]
- a4: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/tests/test_on_demand_subscriptions.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
