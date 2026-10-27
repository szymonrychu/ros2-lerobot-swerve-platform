---
title: MCP zoom retired
status: todo
repo: ros2-lerobot-sverve-platform
value: The agent no longer has a zoom option that no camera can serve.
files: [nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/zoom_client.py, nodes/mcp_server/mcp_server/tool_context.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/tests/test_camera_zoom_tool.py, nodes/mcp_server/tests/test_zoom_client.py, nodes/mcp_server/tests/fakes.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: [mcp-depth-summary-on-d435-cl]
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

# MCP zoom retired

## Requirement

Goal G9: "The ... head RGB (head_camera_cpp, Camera Module 3) ... retired ... with their consumers". Design D11, open question Q1 (recommended: retire zoom everywhere). If the gate answers Q1 otherwise, this task is amended before it runs.

- `mcp_server/tools.py` `get_camera_image`: the `zoom` parameter and its metadata are removed.
- `mcp_server/zoom_client.py` deleted. The `camera_zoom` method is removed from `mcp_server/tool_context.py` and `mcp_server/ros_iface.py`. The `zoom_*` topics are removed from `mcp_server/config.py` and from the client.yml mcp_server block.
- `tests/test_camera_zoom_tool.py` and `tests/test_zoom_client.py` deleted (the feature they test is retired). `tests/fakes.py` drops camera_zoom. New `tests/test_tools.py::test_get_camera_image_has_no_zoom` (schema has no zoom; a call with zoom fails validation).
- Retirement pin: `tests/test_tools.py::test_zoom_client_is_gone` asserts on `git ls-files nodes/mcp_server/mcp_server/zoom_client.py` being empty.
TDD exception for the deletions: pinned by the retirement test.

## Acceptance conditions

- AC1: get_camera_image has no zoom parameter [evidence: test:nodes/mcp_server/tests/test_tools.py::test_get_camera_image_has_no_zoom]
- AC2: the zoom client is no longer tracked [evidence: test:nodes/mcp_server/tests/test_tools.py::test_zoom_client_is_gone]
- AC3: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/zoom_client.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_camera_zoom_tool.py`
- `nodes/mcp_server/tests/test_zoom_client.py`
- `nodes/mcp_server/tests/fakes.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the two tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py -q`; expect failures.
- Remove the parameter, client, protocol method, config topics and the two test files; rerun the full mcp_server suite; expect pass.
- Run the verify command; commit `refactor(mcp_server): retire head camera zoom with head_camera_cpp`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
