---
title: Drop fused depth topic
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The web_ui backend and configs carry no Jetson AI-filled depth topic.
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_config.py, nodes/web_ui/tests/test_map3d_backend.py, tests/test_mcp_server_config.py, nodes/web_ui/README.md, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q"
depends_on: [mcp-front-camera-topic]
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

# Drop fused depth topic

## Requirement

- `TabConfig` has no `fused_depth_topic`; the topic field list (~143), `tof_ranges` (~527) and `topic_roles` (~629) no longer mention it. A YAML head tab still carrying `fused_depth_topic` loads (unknown keys are ignored as elsewhere) and that topic is neither subscribed nor given a role.
- `default.yaml` and `client.yml` head tabs drop `fused_depth_topic`, and their comments drop the fused/raw toggle wording.
- Tests first, seen failing: in `tests/test_config.py`, `test_head_fused_depth_topic_is_optional_and_subscribed` is replaced by `test_head_tab_has_no_fused_depth_topic` (not a model field; a YAML key of that name is not subscribed and has no role); the `tof_ranges` test (~375-377) and the sample YAML (~324) no longer use it. `test_map3d_backend.py` (~1184) drops the fused assertion. Root `tests/test_mcp_server_config.py` head pins (~485-488) drop the fused topic.
- `nodes/web_ui/README.md` head tab fields (~525) name no `fused_depth_topic` and no raw/fused toggle. `tests/README.md` head rows updated.
- The frontend part (types.ts, HeadTab toggle) is delivered by head-and-arm-tabs-frontend.

## Acceptance conditions

- a1: TabConfig has no fused_depth_topic and a stray YAML key of that name is not subscribed [evidence: test: nodes/web_ui/tests/test_config.py::test_head_tab_has_no_fused_depth_topic]
- a2: no backend file, config or README names fused_depth_topic [evidence: command: ! git grep -n fused_depth_topic -- nodes/web_ui/web_ui nodes/web_ui/config nodes/web_ui/tests nodes/web_ui/README.md ansible/group_vars/client.yml tests/test_mcp_server_config.py]
- a3: client and default head tabs still match [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_backend.py -q]
- a4: root mcp/web_ui config tests pass [evidence: command: uv run pytest tests/test_mcp_server_config.py -q]
- a5: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_map3d_backend.py`
- `tests/test_mcp_server_config.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

## Interfaces


- `TabConfig` head fields after the change: `rgb_camera_topic`, `tof_depth_topic`, `tof_amplitude_topic`, `tof_range_m`.

## Expected outcome


No fused depth topic in the web_ui backend, configs, README or root tests.

## Steps


- Replace the fused test and adjust the others; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q` and see the new test fail.
- Edit config.py, default.yaml, client.yml, test_map3d_backend.py and the root test.
- Run the verify command; update READMEs; commit `refactor(web_ui): drop fused_depth_topic`.

## Notes


- Only the Jetson produced `/stereo/depth/fused` (reuse brief, dead once Jetson goes).
