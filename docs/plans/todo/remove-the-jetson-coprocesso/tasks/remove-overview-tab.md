---
title: Remove Overview tab
status: todo
repo: ros2-lerobot-sverve-platform
value: The web UI no longer offers a tab for a camera that no longer exists.
files: [nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/tests/test_config.py, nodes/web_ui/tests/test_bridge.py, tests/test_mcp_server_config.py, nodes/web_ui/README.md, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q"
depends_on: [head-tab-tof-topics]
owns: []
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

# Remove Overview tab

## Requirement

- `default.yaml` and `client.yml` have no `overview_camera` camera tab; the Front (pi_cameras) tab stays.
- `bridge.py` `TOPIC_TYPE_HINTS` has no `/overview_camera/image_raw/compressed` entry.
- Tests first, seen failing: `tests/test_config.py` asserts the default config has no `overview_camera` tab and subscribes no `/overview_camera/*` topic (replacing ~133-140), and the tab order assertion (~312) matches the remaining tabs; `tests/test_bridge.py` replaces the overview tests (~63-69, ~92) with `/front_camera/image_raw/compressed` resolving to CompressedImage and `/overview_camera/image_raw/compressed` resolving to None, and the stale-stereo test's live head assertion (~93) uses `/tof/depth/image_raw`; root `test_web_ui_tab_set` (~470) expects no overview tab.
- `nodes/web_ui/README.md`: the Overview row (~15) is removed. `tests/README.md` (~578-579) updated.

## Acceptance conditions

- a1: the default config has no overview tab and subscribes no /overview_camera topic [evidence: test: nodes/web_ui/tests/test_config.py::test_default_config_has_no_overview_tab]
- a2: the bridge resolves /front_camera and no longer knows /overview_camera [evidence: test: nodes/web_ui/tests/test_bridge.py::test_front_camera_resolves_and_overview_camera_is_gone]
- a3: the client web_ui tab set has no overview tab [evidence: test: tests/test_mcp_server_config.py::test_web_ui_tab_set]
- a4: no web_ui backend, config or README mentions the Overview tab or /overview_camera, except the bridge test asserting it is gone [evidence: command: ! (git grep -n -E 'overview_camera|"Overview"' -- nodes/web_ui/web_ui nodes/web_ui/config nodes/web_ui/README.md | grep -v overview_camera_topic)]
- a5: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_bridge.py`
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
