---
title: MCP front camera topic
status: todo
repo: ros2-lerobot-sverve-platform
value: "mcp_server's `front` camera reads the live front Camera Module 3 instead of the Jetson's overview stream."
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/tests/test_config.py, nodes/mcp_server/tests/test_on_demand_subscriptions.py, nodes/mcp_server/tests/test_tools.py, nodes/mcp_server/README.md, ansible/group_vars/client.yml, tests/test_mcp_server_config.py, tests/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint"
depends_on: [calibrate-page-imu-only]
owns: []
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

# MCP front camera topic

## Requirement

- mcp_server `topics.front_camera` defaults to `/front_camera/image_raw/compressed` (`mcp_server/config.py`); `client.yml` mcp_server `topics.front_camera` is the same, with a comment naming the pi_cameras front Camera Module 3 (640x480 at 10 fps) instead of the left IMX219.
- Tests first: `nodes/mcp_server/tests/test_config.py` default assertion and `test_on_demand_subscriptions.py` `CAMERA_TOPICS` use the new topic; `test_tools.py`'s no-hardcoded-size assertion keeps its check with a message that names no IMX219.
- `nodes/mcp_server/README.md`: the `get_camera_image` row (~27) and the camera mount/intrinsics note (~123) describe the front camera as the pi_cameras front Camera Module 3 (640x480 at 10 fps); the `front` role wording ("fixed overhead camera") stays (D6).
- Root `tests/test_mcp_server_config.py`: `test_mcp_server_front_camera_is_the_compressed_overview_camera` becomes `test_mcp_server_front_camera_is_the_compressed_front_camera` (asserts `/front_camera/image_raw/compressed`); `test_mcp_server_states_no_hardcoded_front_camera_size` keeps its tools.py checks, asserts the mcp README gives 640x480 for the front camera, and drops its web_ui README assertions (the web_ui tasks own that README); `test_mcp_server_get_camera_image_docs_name_the_imx219_left_camera` becomes `test_mcp_server_get_camera_image_docs_name_the_front_camera_module_3` (the row names Camera Module 3 and 640x480; no mcp_server text names IMX219 or 480x360).
- claude_agent needs no code change (its `camera_note` already describes a 640x480 autofocus front camera; it reaches cameras through mcp_server): verified by the grep AC below.
- `tests/README.md`: the renamed root tests and the mcp_server test rows are updated.

## Acceptance conditions

- a1: the mcp_server default front camera topic is /front_camera/image_raw/compressed [evidence: test: nodes/mcp_server/tests/test_config.py]
- a2: client.yml mcp_server front_camera is the compressed front camera [evidence: test: tests/test_mcp_server_config.py::test_mcp_server_front_camera_is_the_compressed_front_camera]
- a3: mcp_server docs name the front Camera Module 3 at 640x480 and no IMX219 [evidence: test: tests/test_mcp_server_config.py::test_mcp_server_get_camera_image_docs_name_the_front_camera_module_3]
- a4: neither mcp_server nor claude_agent references overview_camera, IMX219 or 480x360 [evidence: command: ! git grep -n -E 'overview_camera|IMX219|480x360' -- nodes/mcp_server nodes/claude_agent]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/tests/test_on_demand_subscriptions.py`
- `nodes/mcp_server/tests/test_tools.py`
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
