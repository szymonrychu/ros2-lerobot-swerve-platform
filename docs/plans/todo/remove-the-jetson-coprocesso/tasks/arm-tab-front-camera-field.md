---
title: Arm tab front camera field
status: todo
repo: ros2-lerobot-sverve-platform
value: "The Arm tab's second stream is the front camera under a field name that says so."
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_config.py, tests/test_mcp_server_config.py, nodes/web_ui/README.md, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q"
depends_on: [remove-overview-tab]
owns: [arm-front-camera-field]
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Arm tab front camera field

## Requirement

- `TabConfig.overview_camera_topic` is renamed `front_camera_topic` (comment: arm: sensor_msgs/CompressedImage of the front camera); `ARM_TAB_DEFAULTS` gets `front_camera_topic: /front_camera/image_raw/compressed`; the topic field list (~137) uses the new name.
- `default.yaml` and `client.yml` arm tabs set `front_camera_topic: /front_camera/image_raw/compressed`; the arm comments say gripper and front cameras.
- Tests first, seen failing (`tests/test_config.py` ~261, ~281-285, ~300): the default arm tab's `front_camera_topic` value; an explicit `front_camera_topic` round-trips; subscribed topics include `/front_camera/image_raw/compressed` and no `/overview_camera/*`. Root `tests/test_mcp_server_config.py` arm tab checks (if any pin the topic) use the new field.
- `nodes/web_ui/README.md`: the Arm row (~13) and Arm layout (~535) say front camera / `front_camera_topic`. `tests/README.md` rows updated.
- The frontend rename is delivered by head-and-arm-tabs-frontend.

## Acceptance conditions

- a1: the default arm tab streams /front_camera/image_raw/compressed through front_camera_topic [evidence: test: nodes/web_ui/tests/test_config.py::test_default_arm_tab_uses_the_front_camera]
- a2: no web_ui backend, config, test or README and no client.yml line names overview_camera_topic [evidence: command: ! git grep -n overview_camera_topic -- nodes/web_ui/web_ui nodes/web_ui/config nodes/web_ui/tests nodes/web_ui/README.md ansible/group_vars/client.yml tests]
- a3: root client.yml web_ui checks pass [evidence: command: uv run pytest tests/test_mcp_server_config.py -q]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
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
