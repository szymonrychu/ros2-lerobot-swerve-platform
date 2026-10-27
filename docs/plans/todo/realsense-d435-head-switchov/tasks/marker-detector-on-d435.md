---
title: Marker detector on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "Printed markers are still found and posed, now from the D435."
files: [nodes/marker_detector/marker_detector/config.py, nodes/marker_detector/marker_detector/tof_plane.py, nodes/marker_detector/marker_detector/depth_plane.py, nodes/marker_detector/marker_detector/pipeline.py, nodes/marker_detector/tests/test_tof_plane.py, nodes/marker_detector/tests/test_depth_plane.py, nodes/marker_detector/tests/test_config.py, ansible/group_vars/client.yml]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_marker_detector_config.py -q && uv run poe test-ansible"
depends_on: [shared-d435-camera-geometry, head-rgb-d-node-on-d435-alig]
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

# Marker detector on D435

## Requirement

Goal G9: "... marker_detector moves to D435 colour+depth." Reuse-brief fix-as-ac: "Inline depth deprojection duplicated - marker_detector/tof_plane.py:132 ... these use it". Reuse: pose.estimate_pose, tof_plane.fit_plane/plane_chooser.

- `marker_detector/config.py` `TopicsConfig`: `image: /head/d435/color/image_raw/compressed`, `camera_info: /head/d435/color/camera_info`, `depth: /head_rgbd/depth/image_raw`, `depth_info: /head_rgbd/depth/camera_info` (the D435 aligned depth, decimated, same stamp as the RGB set). The zoom topics stay until marker-detector-zoom-retired.
- `git mv marker_detector/tof_plane.py marker_detector/depth_plane.py`. The deprojection uses `ros2_common.camera_geometry.depth_to_points` with `CameraIntrinsics.from_camera_info` (no inline copy). `fit_plane` and `plane_chooser` are kept. Names and docstrings say depth, not ToF. `pipeline.py` imports the new module.
- `git mv tests/test_tof_plane.py tests/test_depth_plane.py` with the D435 frame `d435_color_optical_frame` (ToF frame names are no longer accurate); assertions kept. New `test_depth_plane.py::test_uses_the_shared_deprojection`.
- The client.yml marker_detector `topics` block matches.

## Acceptance conditions

- AC1: marker_detector reads the D435 colour and the /head_rgbd depth [evidence: test:nodes/marker_detector/tests/test_config.py::test_topics_are_the_d435]
- AC2: the planar ambiguity is settled with the shared deprojection on D435 depth [evidence: test:nodes/marker_detector/tests/test_depth_plane.py::test_uses_the_shared_deprojection]
- AC3: no ToF module or name remains in marker_detector code [evidence: command:! grep -rni "tof" nodes/marker_detector/marker_detector]
- AC4: marker_detector suite and lint pass, deployed config pins pass [evidence: command:cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_marker_detector_config.py -q]

## Files

- `nodes/marker_detector/marker_detector/config.py`
- `nodes/marker_detector/marker_detector/tof_plane.py`
- `nodes/marker_detector/marker_detector/depth_plane.py`
- `nodes/marker_detector/marker_detector/pipeline.py`
- `nodes/marker_detector/tests/test_tof_plane.py`
- `nodes/marker_detector/tests/test_depth_plane.py`
- `nodes/marker_detector/tests/test_config.py`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
