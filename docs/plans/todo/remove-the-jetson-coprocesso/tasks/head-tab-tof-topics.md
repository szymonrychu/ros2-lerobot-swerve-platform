---
title: Head tab ToF topics
status: todo
repo: ros2-lerobot-sverve-platform
value: The Head tab shows the Arducam ToF depth and amplitude the robot actually publishes.
files: [nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_config.py, nodes/web_ui/tests/test_map3d_backend.py, nodes/web_ui/tests/test_msg_serializer.py, tests/test_mcp_server_config.py, nodes/web_ui/README.md, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q"
depends_on: [drop-fused-depth-topic]
owns: []
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head tab ToF topics

## Requirement

- `default.yaml` and `client.yml` head tabs: `tof_depth_topic: /tof/depth/image_raw`, `tof_amplitude_topic: /tof/amplitude/image_raw`, `tof_range_m: 4.0`, no `rgb_camera_topic` and no `camera_info_topic` (D5). Their comments describe the Arducam ToF (arducam_tof on CSI cam1: 16UC1 mm depth, mono16 amplitude) and say `/tof/camera_info` is published but not consumed by the Head tab.
- Tests first, seen failing: `test_map3d_backend.py` (~1183-1186) asserts the client and default head tabs agree on both ToF topics and both have `rgb_camera_topic` None; root `tests/test_mcp_server_config.py` head pins assert the same for client.yml; new `test_default_head_tab_subscribes_the_tof_topics` in `tests/test_config.py` asserts the shipped default head tab subscribes exactly the two ToF topics with roles `head_depth` and `head_amplitude`.
- `test_msg_serializer.py` sample topics `/stereo/depth/image_raw` (~168, ~216) become `/tof/depth/image_raw`.
- `nodes/web_ui/README.md` Head row (~14) and Head section: ToF depth + amplitude switch, no RGB, the Arducam ToF, `/tof/camera_info` not consumed. `tests/README.md` head rows updated.

## Acceptance conditions

- a1: the default head tab subscribes exactly /tof/depth/image_raw and /tof/amplitude/image_raw with the head roles [evidence: test: nodes/web_ui/tests/test_config.py::test_default_head_tab_subscribes_the_tof_topics]
- a2: client and default head tabs agree on the ToF topics and have no RGB [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_backend.py -q]
- a3: client.yml head tab pins pass [evidence: command: uv run pytest tests/test_mcp_server_config.py -q]
- a4: no web_ui config, test or README names a /stereo or /overview_camera head topic [evidence: command: ! git grep -n -E '/stereo/depth|rgb_camera_topic: /overview' -- nodes/web_ui/config nodes/web_ui/tests/test_map3d_backend.py nodes/web_ui/tests/test_msg_serializer.py nodes/web_ui/README.md ansible/group_vars/client.yml]
- a5: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_map3d_backend.py`
- `nodes/web_ui/tests/test_msg_serializer.py`
- `tests/test_mcp_server_config.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
