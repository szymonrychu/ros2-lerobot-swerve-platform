---
title: Web UI head camera topics
status: done
repo: ros2-lerobot-sverve-platform
value: "The web UI Arm and Head tabs show the head camera under head names; the stale Front tab is gone."
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_config.py, nodes/web_ui/tests/test_map3d_backend.py, tests/test_mcp_server_config.py, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint"
depends_on: [retire-pi-cameras]
owns: [web-head-camera-fields]
uses: []
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: ""
branch: front-camera-30-fps-stream-w--web-ui-head-camera-topics
verified_head: e92b02a7910cae9664178628bb2adf89448575fa
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: ["Task changes are uncommitted in the worktree (git status shows 7 modified files, nothing committed); the specified git diff merge-base..HEAD shows no config.py or default.yaml change.", "No recorded red/green evidence (tests.red) for the config.py, default.yaml and client.yml changes. The checklist requires reverting each non-test file to the merge-base, showing the failing tail, then restoring it and showing the passing tail."]
---

# Web UI head camera topics

## Requirement

- `web_ui/config.py`: `TabConfig.front_camera_topic` (Jetson end state) is renamed `head_camera_topic` (comment: arm: sensor_msgs/CompressedImage of the head camera); `ARM_TAB_DEFAULTS["head_camera_topic"] = "/head_camera/image_raw/compressed"`; the topic field list uses the new name (D12, U9).
- `config/default.yaml` and `ansible/group_vars/client.yml` web_ui tabs: arm `head_camera_topic: /head_camera/image_raw/compressed`; head `rgb_camera_topic: /head_camera/image_raw/compressed` (comment: head RGB camera, Camera Module 3 on the pan/tilt head, head_camera_cpp); the `pi_cameras` "Front" camera tab is removed from both (the Head tab shows the same camera).
- Tests first, seen failing: `tests/test_config.py` (default arm tab `head_camera_topic`, explicit value round-trips, subscribed topics contain `/head_camera/image_raw/compressed` and no `/front_camera/*`, no `pi_cameras` tab id in the default config); `tests/test_map3d_backend.py` head tab agreement on `/head_camera/image_raw/compressed`; root `tests/test_mcp_server_config.py` client.yml web_ui pins (arm, head, no Front tab). `tests/README.md` rows follow.

## Acceptance conditions

- a1: the default arm tab streams /head_camera/image_raw/compressed through head_camera_topic [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q]
- a2: client and default head tabs use /head_camera/image_raw/compressed as RGB [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_backend.py -q]
- a3: no web_ui backend, config or client.yml line names front_camera_topic or /front_camera [evidence: command: ! git grep -n -E 'front_camera_topic|/front_camera/' -- nodes/web_ui/web_ui nodes/web_ui/config nodes/web_ui/tests ansible/group_vars/client.yml]
- a4: web_ui suite, lint and root pins pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_map3d_backend.py`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

## Interfaces

- `web-head-camera-fields`: `TabConfig.head_camera_topic`, `ARM_TAB_DEFAULTS["head_camera_topic"]` and the head tab `rgb_camera_topic` value in `nodes/web_ui/web_ui/config.py` (the frontend reads `head_camera_topic`).

## Expected outcome

web_ui backend and configs use /head_camera for the Arm and Head tabs and no Front tab remains.

## Steps

- Update the tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py tests/test_map3d_backend.py -q`; see them fail.
- Rename in config.py; edit default.yaml and client.yml; run the verify command; commit `refactor(web_ui): head camera topics, drop the Front tab`.

## Notes

- Builds on the Jetson epic's `arm-tab-front-camera-field` and `head-tab-tof-topics` end state; re-read first.
- The bridge type hint for /front_camera moves in web-ui-zoom-backend (the head role already types the topic).

## Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.
