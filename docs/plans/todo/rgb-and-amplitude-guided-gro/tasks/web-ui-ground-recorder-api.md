---
title: Web UI ground recorder API
status: todo
repo: ros2-lerobot-sverve-platform
value: Replay recordings captured from the robot without rclpy probes or SSH
files: [nodes/web_ui/web_ui/bridge.py, nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/tests/test_ground_recorder_api.py, nodes/web_ui/tests/test_config.py, nodes/web_ui/config/default.yaml, nodes/web_ui/README.md, ansible/group_vars/client.yml]
verify: "(cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder_api.py tests/test_ground_recorder.py tests/test_config.py -q && uv run --frozen poe lint) && uv run poe test-ansible"
depends_on: [ground-recording-layout, guidance-config-keys]
owns: [ground-recorder-api]
uses: [ground-recording-layout]
model: ""
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI ground recorder API

## Requirement

A recorder API in web_ui (design D8, part ground-recorder-api). An operator captures replay recordings with RGB + amplitude from the existing, long-lived web_ui node: no rclpy probe, no SSH (user decision 2026-10-22). It uses part ground-recording-layout.

- Config (`web_ui/config.py`, pydantic, unknown keys rejected like the rest): block `ground_recorder` with:
  - `enabled: bool = False` and `dir: str = "/tmp/web_ui_ground_recordings"`;
  - `max_duration_s: float = 30` (1..60) and `max_files: int = 4` (1..20);
  - `tf_timeout_s: float = 0.5` (0..2);
  - topics: `cloud_topic /tof/points_filtered`, `tof_depth_topic /tof/depth/image_raw`, `tof_amplitude_topic /tof/amplitude/image_raw`, `tof_info_topic /tof/camera_info`, `pair_depth_topic /head_rgbd/depth_upsampled/image_raw`, `pair_info_topic /head_rgbd/depth_upsampled/camera_info`, `pair_rgb_topic /head_rgbd/rgb/image_raw/compressed`, `imu_topic /imu/data`, `head_js_topic /head/joint_states`, `state_topic /ground/state`;
  - frames: `base_frame base_link`, `odom_frame odom`.

  Documented in `config/default.yaml`, and enabled in the web_ui block of `ansible/group_vars/client.yml` with a comment (operator tool, idle when not recording).
- Routes (`web_ui/server.py`):
  - `POST /api/ground/recordings` with `{name: [a-z0-9_-]{1,32}, duration_s}` answers 202 `{name, duration_s}`. It answers 409 while a recording runs, 404 when the recorder is disabled, and 400 on a bad name or a duration outside 1..max_duration_s.
  - `GET /api/ground/recordings` lists `[{name, bytes, done, counts, dropped}]`.
  - `GET /api/ground/recordings/{name}` downloads the npz (application/octet-stream) once done. Unknown is 404, in progress is 409.
  - `DELETE /api/ground/recordings/{name}` removes the file (404 unknown).
- Bridge (`web_ui/bridge.py`):
  - Starting creates the subscriptions (sensor-data QoS for images and IMU, reliable for the cloud and camera_info, as their publishers).
  - Every message gets the TF at its stamp from the existing tf buffer: base_link <- message frame, plus odom <- base_link for clouds and pairs. A message whose TF does not arrive within `tf_timeout_s` is dropped and counted. It is never recorded with a guessed or latest pose.
  - Messages are converted to arrays: depth and upsampled depth 16UC1, amplitude mono16, the cloud's float32 x y z, the CompressedImage bytes as-is.
  - At the end of the window the subscriptions are destroyed and the file is written; files beyond `max_files` are deleted oldest first.
  - The recorder publishes nothing and commands nothing.
- README `nodes/web_ui/README.md` section `## Ground recordings`: the routes, the config keys, the layout (pointer to ground_recorder.py), the load while recording, and why it exists (no rclpy probes on the client).
- Tests first (seen failing):
  - `nodes/web_ui/tests/test_ground_recorder_api.py` with the FastAPI TestClient and a fake recorder: `test_start_list_download_delete`, `test_second_start_conflicts`, `test_disabled_recorder_is_404`, `test_bad_name_or_duration_is_400`;
  - with ROS doubles, following the existing bridge tests (`tests/test_bridge_on_demand.py`): `test_message_without_tf_dropped_and_counted`, `test_message_with_tf_recorded_with_its_pose`, `test_subscriptions_destroyed_after_window`;
  - `tests/test_config.py` gains `test_ground_recorder_defaults_and_ranges`.

## Acceptance conditions

- a1: start, list, download and delete work through the API [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_start_list_download_delete]
- a2: a second start while recording answers 409 [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_second_start_conflicts]
- a3: the disabled recorder answers 404 [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_disabled_recorder_is_404]
- a4: bad names and durations answer 400 [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_bad_name_or_duration_is_400]
- a5: a message without TF at its stamp is dropped and counted [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_message_without_tf_dropped_and_counted]
- a6: a message with TF is recorded with that pose [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_message_with_tf_recorded_with_its_pose]
- a7: the subscriptions are destroyed after the window [evidence: test: nodes/web_ui/tests/test_ground_recorder_api.py::test_subscriptions_destroyed_after_window]
- a8: the config block validates its defaults and ranges [evidence: test: nodes/web_ui/tests/test_config.py::test_ground_recorder_defaults_and_ranges]
- a9: client.yml enables the recorder and Ansible checks pass [evidence: command: uv run poe test-ansible]
- a10: the README documents the recorder [evidence: doc-section: nodes/web_ui/README.md#Ground recordings]

## Files

- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/tests/test_ground_recorder_api.py`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/README.md`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
