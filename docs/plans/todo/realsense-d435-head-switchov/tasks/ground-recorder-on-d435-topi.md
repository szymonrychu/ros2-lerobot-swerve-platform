---
title: Ground recorder on D435 topics
status: todo
repo: ros2-lerobot-sverve-platform
value: Ground scenes can be recorded from the D435 pipeline to rebuild the replay fixtures.
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/ground_recorder.py, nodes/web_ui/web_ui/ground_session.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/config/default.yaml, nodes/web_ui/tests/test_ground_recorder.py, nodes/web_ui/tests/test_ground_recorder_api.py]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py tests/test_ground_recorder_api.py tests/test_config.py -q && uv run poe lint"
depends_on: [laser-filter-on-the-d435-clo, head-rgb-d-node-on-d435-alig]
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

# Ground recorder on D435 topics

## Requirement

Goal G3: "... replay fixtures re-recorded." Design D7, D12 ("The ground recorder records the D435 cloud, the /head_rgbd pair, IMU and the ground state").

- `nodes/web_ui/web_ui/config.py` `GroundRecorderConfig`: `cloud_topic` `/head/d435/points_filtered`; `depth_info_topic` `/head/d435/depth/camera_info` (replaces `tof_info_topic`); `pair_depth_topic` `/head_rgbd/depth/image_raw`, `pair_info_topic` `/head_rgbd/depth/camera_info`, `pair_rgb_topic` unchanged; `tof_depth_topic` and `tof_amplitude_topic` removed (no amplitude; full D435 depth frames are not recorded: 640x360 at 15 Hz would be ~200 MB per 30 s, and only the depth camera model is needed). Docstring says D435.
- `nodes/web_ui/web_ui/ground_recorder.py` and `ground_session.py`: the `tof` frame stream and its `tof_*` npz keys are removed. The depth camera model is stored as `depth_K`, `depth_D`, `depth_size`, `depth_frame`. `layout_version` becomes 3. Cloud, pairs, IMU, head joint states and ground state are unchanged.
- `nodes/web_ui/web_ui/bridge.py` subscribes the recorder to the renamed topics. `nodes/web_ui/config/default.yaml` matches.
- Tests: `nodes/web_ui/tests/test_ground_recorder.py` (layout 3 keys, no tof keys, the depth model recorded from a CameraInfo) and `test_ground_recorder_api.py` (topics in the API response). The ToF-frame tests are removed as no longer accurate.
- No placeholder data: a recording without a depth camera_info still writes the cloud/IMU streams and records `depth_size` (0, 0), as today's ToF behaviour; the export refuses it.

## Acceptance conditions

- AC1: the recorder config defaults name the D435 cloud, the D435 depth camera_info and the /head_rgbd depth pair, with no ToF field [evidence: test:nodes/web_ui/tests/test_ground_recorder.py::test_config_defaults_are_the_d435_topics]
- AC2: a recording writes layout 3 with the depth camera model and no tof keys [evidence: test:nodes/web_ui/tests/test_ground_recorder.py::test_layout3_records_the_depth_model]
- AC3: the recordings API reports the D435 topics [evidence: test:nodes/web_ui/tests/test_ground_recorder_api.py::test_api_reports_the_d435_topics]
- AC4: web_ui lint passes [evidence: command:cd nodes/web_ui && uv run poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/ground_recorder.py`
- `nodes/web_ui/web_ui/ground_session.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_ground_recorder.py`
- `nodes/web_ui/tests/test_ground_recorder_api.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the new tests and drop the ToF-frame ones; run `cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py tests/test_ground_recorder_api.py -q`; expect failures.
- Change config, recorder, session, bridge wiring and default.yaml; rerun; expect pass.
- Run the verify command; commit `feat(web_ui): ground recorder on the D435 topics (layout 3)`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
