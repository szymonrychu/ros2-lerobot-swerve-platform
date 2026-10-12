# Web UI Head Tab Implementation Plan

> For agentic workers: execute through `workbench:plan-execute` with
> `docs/plans/2026-10-13-web-ui-head-tab-tasks.json` as `args.tasks`. Each task is test-first: the test is written and
> seen failing before the code that makes it pass. Steps use checkbox syntax.

**Goal:** A `head` web UI tab (overview camera RGB, greyscale ToF depth with optional amplitude, disabled pan/tilt placeholder) and an in-repo `arducam_tof` driver node deployed disabled by Ansible.

**Architecture:** `arducam_tof` publishes `/tof/depth/image_raw`, `/tof/amplitude/image_raw` and `/tof/camera_info`. The web_ui backend gets a `head` tab type whose topics are subscribed and serialised (greyscale depth), and the frontend `HeadTab` renders them. Existing tabs, the rgbd depth path and the declutter design rules stay unchanged. The driver, Ansible, backend and frontend tasks touch disjoint files and run in parallel.

**Tech stack:** Python 3.12 with uv and pydantic, ROS2 Jazzy, Ansible, React with vitest. Tests: `uv run pytest`, `uv run poe lint`, `npm test`, `uv run poe test-ansible`.

**Spec:** `docs/plans/2026-10-13-web-ui-head-tab-design.md` (executors read both: the design doc for the why and the decisions, this plan for the what).

## Project brief

What: a ROS2 lerobot swerve robot platform with a browser dashboard (web_ui).
How: uv Python nodes on Raspberry Pis deployed by Ansible; web_ui bridges topics to a React UI.
Why: the user is about to plug in the ToF camera and needs a head preview with room for pan/tilt.

## Global constraints

- Tests first; no placeholder data on topics; type hints, constants on top, imports at the top, pydantic config.
- Plain hyphens and straight quotes only; no deploys in tasks (deploy only through scripts/deploy-nodes.sh, later).
- Model: sonnet, except T1 on opus.
- Node-registration mirrors leader_bridge (60d7421).

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| head-brief.md plan input 1 | ToF driver node | T1 |
| head-brief.md plan input 2 | Ansible node type, apt repo and SDK | T3 |
| head-brief.md plan input 3 | head UVC camera | rejected: the user chose the overview camera as head RGB |
| head-brief.md plan input 4 | backend tab type, fields, hints | T4 |
| head-brief.md plan input 5 | depth serializer reuse, greyscale and range | T4 |
| head-brief.md plan input 6 | frontend HeadTab and plumbing, design guard FILES | T5 |
| head-brief.md plan input 7 | configs and pinned tests | T6 |
| head-brief.md plan input 8 | docs | T7 |
| leader_bridge registration 60d7421 | lint-all-nodes, poe lint-nodes, CI matrix, cgroups test, CLAUDE.md | T2, T3 |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 In-repo arducam_tof node on the Arducam SDK | T1, T2, T3, T7 |
| D2 Head tab: overview camera as head RGB, greyscale depth, disabled pan/tilt placeholder | T4, T5, T6, T7 |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `arducam-tof-topics` | T1 | T2, T7 | `nodes/bridges/arducam_tof/arducam_tof/bridge.py` |
| `head-tab-config` | T4 | T6, T7 | `nodes/web_ui/web_ui/config.py` |
| `head-tab-ui` | T5 | T7 | `nodes/web_ui/frontend/src/tabs/HeadTab.tsx` |

## Review focus

- T1: no publish without a fresh frame, invalid pixels are 0, numpy dependency, SDK import does not break macOS tests.
- T3: the apt task is idempotent and never fails the deploy, `enabled` stays false.
- T4: unknown encodings are never decoded as colour; the rgbd tab output is unchanged.
- T5: disabled pan/tilt publishes nothing; the design guard covers HeadTab.

### Task 1: arducam_tof driver node

**Files:**
- Create: `nodes/bridges/arducam_tof/` (pyproject.toml, uv.lock, arducam_tof/__init__.py, __main__.py, config.py, bridge.py)
- Test: `nodes/bridges/arducam_tof/tests/test_config.py`, `tests/test_bridge.py`

**Interfaces:**
- Consumes: the `ArducamDepthCamera` SDK (mocked in tests), `shared/ros2_metrics`.
- Produces: topics `/tof/depth/image_raw` (16UC1 mm, invalid 0), `/tof/amplitude/image_raw`, `/tof/camera_info`; config keys camera_index, range_mode, confidence_threshold, frame_id, publish_rate_hz, metrics_port.

**Ownership:**
- owns: arducam-tof-topics
- uses: none
- dependsOn: none

**Requirement:** see T1 in the tasks JSON.

**Value:** a real ToF source for the web UI.

**Steps:**
- [ ] Write the tests.
- [ ] Run them. Expected: FAIL (module missing).
- [ ] Implement.
- [ ] Run them. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(arducam_tof): ToF depth and amplitude driver on the Arducam SDK`

**Verify:**
`cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint`

### Task 2: Node registration

**Files:**
- Modify: `scripts/lint-all-nodes.sh`, `pyproject.toml`, `.github/workflows/ci.yml`, `CLAUDE.md`

**Interfaces:**
- Consumes: the node directory from T1.
- Produces: lint, CI and doc coverage of arducam_tof.

**Ownership:**
- owns: none
- uses: arducam-tof-topics
- dependsOn: T1

**Requirement:** see T2 in the tasks JSON.

**Value:** CI covers the driver.

**Steps:**
- [ ] Extend the lint-coverage test expectation or add a check; run it. Expected: FAIL.
- [ ] Register the node.
- [ ] Run it. Expected: PASS.
- [ ] Commit: `chore: register arducam_tof with lint-nodes and CI`

**Verify:**
`bash -o pipefail -c 'uv run poe lint-nodes </dev/null 2>&1 | tail -3'`

### Task 3: Ansible for arducam_tof

**Files:**
- Create: `ansible/playbooks/tasks/arducam_tof_sdk.yml`, `tests/test_arducam_tof_config.py`
- Modify: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_node_metrics_wiring.py`, `tests/test_unit_cgroups.py`, `tests/test_dds_discovery_config.py`

**Interfaces:**
- Consumes: node path and topic names from T1 (by name).
- Produces: `arducam_tof` node type and a disabled `ros2_nodes` entry.

**Ownership:**
- owns: none
- uses: none (topic names are fixed in the design)
- dependsOn: none

**Requirement:** see T3 in the tasks JSON.

**Value:** the driver deploys switched off.

**Steps:**
- [ ] Write tests/test_arducam_tof_config.py.
- [ ] Run it. Expected: FAIL (no node type).
- [ ] Implement Ansible changes.
- [ ] Run it plus `uv run poe test-ansible`. Expected: PASS.
- [ ] Commit: `feat(ansible): deploy arducam_tof disabled with the Arducam SDK install`

**Verify:**
`uv run pytest tests/test_arducam_tof_config.py tests/test_node_metrics_wiring.py tests/test_unit_cgroups.py tests/test_dds_discovery_config.py -q && bash -o pipefail -c 'uv run poe test-ansible </dev/null 2>&1 | tail -2'`

### Task 4: web_ui backend

**Files:**
- Modify: `nodes/web_ui/web_ui/config.py`, `bridge.py`, `msg_serializer.py`
- Test: `nodes/web_ui/tests/test_config.py`, `test_bridge.py`, `test_msg_serializer.py`

**Interfaces:**
- Consumes: topic names and encodings of T1.
- Produces: tab type `head` with fields rgb_camera_topic, tof_depth_topic, tof_amplitude_topic, tof_range_m; greyscale depth preview.

**Ownership:**
- owns: head-tab-config
- uses: none (topic names are fixed in the design)
- dependsOn: none

**Requirement:** see T4 in the tasks JSON.

**Value:** real ToF previews reach the browser.

**Steps:**
- [ ] Write the tests.
- [ ] Run them. Expected: FAIL (type and fields unknown).
- [ ] Implement.
- [ ] Run them. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `feat(web_ui): head tab type, ToF topics and greyscale depth preview`

**Verify:**
`cd nodes/web_ui && uv run pytest tests/test_config.py tests/test_bridge.py tests/test_msg_serializer.py -q && uv run poe lint`

### Task 5: Frontend HeadTab

**Files:**
- Create: `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- Modify: `App.tsx`, `tabSelection.ts`, `types.ts`, `tabs/tabsDesign.test.ts`, `tabSelection.test.ts`
- Test: `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`

**Interfaces:**
- Consumes: tab fields from the design (by name), `depth_preview_b64`, `jpeg_b64`.
- Produces: the Head tab component.

**Ownership:**
- owns: head-tab-ui
- uses: none
- dependsOn: none (fields are fixed by name in the design; the backend task runs in parallel)

**Requirement:** see T5 in the tasks JSON.

**Value:** the visible Head tab.

**Steps:**
- [ ] Write HeadTab.test.tsx and the pinned list changes.
- [ ] Run them. Expected: FAIL.
- [ ] Implement.
- [ ] Run them. Expected: PASS.
- [ ] Lint and build.
- [ ] Commit: `feat(web_ui): head tab with RGB, ToF depth and a disabled pan/tilt block`

**Verify:**
`cd nodes/web_ui/frontend && npm test -- --run && npm run lint && npm run build`

### Task 6: Configs and pinned tests

**Files:**
- Modify: `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_map3d_backend.py`, `tests/test_mcp_server_config.py`

**Interfaces:**
- Consumes: head tab fields (T4), topics (T1).
- Produces: deployed head tab config.

**Ownership:**
- owns: none
- uses: head-tab-config
- dependsOn: T3, T4

**Requirement:** see T6 in the tasks JSON.

**Value:** the deployed UI lists the Head tab.

**Steps:**
- [ ] Update the pinned lists first.
- [ ] Run them. Expected: FAIL.
- [ ] Add the config entries.
- [ ] Run them. Expected: PASS.
- [ ] Commit: `feat(config): head tab in default and client web_ui tabs`

**Verify:**
`cd nodes/web_ui && uv run pytest tests -q && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && bash -o pipefail -c 'uv run poe lint-ansible </dev/null 2>&1 | tail -2'`

### Task 7: Docs

**Files:**
- Create: `nodes/bridges/arducam_tof/README.md`
- Modify: `nodes/README.md`, `nodes/web_ui/README.md`, `tests/README.md`, `ansible/README.md`

**Interfaces:**
- Consumes: the merged code of T1-T6.
- Produces: docs.

**Ownership:**
- owns: none
- uses: arducam-tof-topics, head-tab-config, head-tab-ui
- dependsOn: T1, T2, T3, T4, T5, T6

**Requirement:** see T7 in the tasks JSON.

**Value:** the camera can be enabled from the docs.

**Steps:**
- [ ] Write the docs from the merged code.
- [ ] Run the docs-related tests.
- [ ] Commit: `docs: arducam_tof node and head tab`

**Verify:**
`uv run pytest tests -q -k 'readme or docs' && git diff --stat`

## Execution

- Run name: `web-ui-head-tab`.
- Model per task: sonnet; T1 on opus (hardware SDK wrapper with the no-fake-frame invariant).
- Lenses run: research-code-internal and research-docs ran (head-brief.md); research-code-external and research-papers skipped (thin SDK wrapper).
- Lint: `node /Users/szymonri/Documents/claude-infra-plugin/workflows/validate-tasks.mjs docs/plans/2026-10-13-web-ui-head-tab-tasks.json --plan docs/plans/2026-10-13-web-ui-head-tab-plan.md --approvals docs/plans/2026-10-13-web-ui-head-tab-approvals.json --design docs/plans/2026-10-13-web-ui-head-tab-design.md`
- Tasks JSON: `docs/plans/2026-10-13-web-ui-head-tab-tasks.json`.
- Approvals: `docs/plans/2026-10-13-web-ui-head-tab-approvals.json`.
