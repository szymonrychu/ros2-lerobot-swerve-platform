---
title: Marker detector raw image
status: todo
repo: ros2-lerobot-sverve-platform
value: "marker_detector stops decoding every camera frame to process two per second (about 0.07 core)."
files: [nodes/marker_detector/marker_detector/node.py, nodes/marker_detector/tests/test_node_raw.py, nodes/marker_detector/tests/ros_stubs.py, nodes/marker_detector/README.md]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: [marker-events-executor]
owns: []
uses: []
model: ""
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Marker detector raw image

## Requirement

Item 4 of the brief (design D4; approval A4): marker_detector deserializes the head camera image and camera_info only in its process timer, not on every ~15-19 fps frame.

- `marker_detector/node.py` MarkerDetectorRos: the `topics.image` (CompressedImage) and `topics.camera_info` (CameraInfo) subscriptions are created with `raw=True`. `on_image` keeps only the newest serialized bytes and its receive time; `on_camera_info` keeps only the newest serialized bytes. `on_timer` deserializes (`rclpy.serialization.deserialize_message`) the newest camera_info when new bytes arrived since the last decode and passes it to `pipeline.on_camera_info` before the image, then deserializes the newest image once and processes it exactly as today (stamp from the decoded header, `bytes(msg.data)` to the pipeline). A frame is decoded at most once; a timer tick with no new frame decodes nothing.
- Depth, depth_info, zoom image and zoom result subscriptions are unchanged.
- No placeholder data: nothing is published for a tick without a new frame (unchanged behaviour).
- Tests first (red seen) in `tests/test_node_raw.py` with the package's ros stubs (extended with `deserialize_message` and the `raw` argument): `test_image_is_deserialized_only_in_the_timer`, `test_camera_info_is_deserialized_only_when_new`, `test_timer_processes_the_newest_frame_once`.
- `nodes/marker_detector/README.md` describes the raw subscriptions.

## Acceptance conditions

- a1: image callbacks store bytes only; deserialization happens in the timer [evidence: test:nodes/marker_detector/tests/test_node_raw.py::test_image_is_deserialized_only_in_the_timer]
- a2: camera_info is decoded only when new bytes arrived [evidence: test:nodes/marker_detector/tests/test_node_raw.py::test_camera_info_is_deserialized_only_when_new]
- a3: the newest frame is processed once and an idle tick decodes nothing [evidence: test:nodes/marker_detector/tests/test_node_raw.py::test_timer_processes_the_newest_frame_once]
- a4: the marker_detector tests and lint pass [evidence: command:cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/marker_detector/marker_detector/node.py`
- `nodes/marker_detector/tests/test_node_raw.py`
- `nodes/marker_detector/tests/ros_stubs.py`
- `nodes/marker_detector/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
