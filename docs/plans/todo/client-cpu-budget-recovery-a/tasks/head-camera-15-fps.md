---
title: Head camera 15 fps
status: todo
repo: ros2-lerobot-sverve-platform
value: "Halves the head camera's encode load (about 0.12 core) so its subscribers stop dropping frames."
files: [ansible/group_vars/client.yml, tests/test_client_cpu_load.py, tests/README.md, nodes/head_camera_cpp/README.md]
verify: "uv run pytest tests/test_client_cpu_load.py tests/test_head_camera_live_refs.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [imu-publish-rate-25-hz]
owns: [head-camera-stream-rate]
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head camera 15 fps

## Requirement

Item 3 of the brief (design D3; approval A3): head_camera_cpp captures and encodes 15 fps instead of ~19-30 fps.

- `ansible/group_vars/client.yml` head_camera_cpp `config: |`: `frame_duration_us: 66666` with a comment (15 fps; encode ~30 ms per frame at nice 10 saturated the client and subscribers got 8.5 fps). The code default (33333) stays. The comments that say 30 fps for the deployed stream (client.yml node type comment near line 338, mcp_server topics comment near line 2350) are corrected.
- Tests first (red seen) in `tests/test_client_cpu_load.py`: `test_head_camera_runs_at_15_fps` (frame_duration_us == 66666) and `test_head_camera_mapping_rate_fits_15_fps` (mapping.rate_hz and marker_detector process_hz are both <= 15).
- Check every consumer of `/head_camera/image_raw/compressed`, `/head_camera/image_raw` and `/head_camera/camera_info` for rate assumptions and state the result in `nodes/head_camera_cpp/README.md` section `## Deployed frame rate`: marker_detector (process_hz 2), the mapping stream (rate_hz 2, head_rgbd_cpp and RTAB-Map), web_ui preview and its stale threshold (TOPIC_STALE_S 10 s), mcp_server `front` camera tool (image_timeout_s), autofocus and the zoom-crop mode switch (does FrameDurationLimits also apply to the zoom capture, and does the stream resume at 15 fps after a zoom). Any consumer that assumes 30 fps is fixed in its config in this task, with a test.
- `nodes/head_camera_cpp/README.md` text that says the deployed stream is 30 fps is corrected; `tests/README.md` is updated. `tests/test_head_camera_live_refs.py` keeps passing (it forbids "10 fps" near head camera words; 15 fps is fine).

## Acceptance conditions

- a1: the deployed head camera frame duration is 66666 us [evidence: test:tests/test_client_cpu_load.py::test_head_camera_runs_at_15_fps]
- a2: the mapping stream and marker processing rates stay at or below 15 Hz [evidence: test:tests/test_client_cpu_load.py::test_head_camera_mapping_rate_fits_15_fps]
- a3: the head_camera_cpp README states the deployed rate and the consumer check [evidence: doc-section:nodes/head_camera_cpp/README.md#Deployed frame rate]
- a4: the live-reference pin still passes [evidence: test:tests/test_head_camera_live_refs.py]
- a5: every listed consumer was checked and none assumes 30 fps [evidence: judgement:]
- a6: root tests, lint and ansible lint pass [evidence: command:uv run pytest tests/test_client_cpu_load.py tests/test_head_camera_live_refs.py -q && uv run poe lint && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_client_cpu_load.py`
- `tests/README.md`
- `nodes/head_camera_cpp/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
