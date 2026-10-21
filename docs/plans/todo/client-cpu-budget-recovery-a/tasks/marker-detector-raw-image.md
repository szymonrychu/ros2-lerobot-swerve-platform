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

`MarkerDetectorRos.on_image(raw: bytes)`, `on_camera_info(raw: bytes)` and `on_timer()` in nodes/marker_detector/marker_detector/node.py. Keep `MarkerPipeline.on_camera_info(K, D, size, frame_id)` and the pipeline's frame input exactly as today, and every published topic and QoS.

## Expected outcome

marker_detector decodes at most process_hz frames per second; `cd nodes/marker_detector && uv run --frozen pytest tests -q` passes.

## Steps

- Extend tests/ros_stubs.py (raw subscriptions, `deserialize_message`) and write tests/test_node_raw.py; `uv run --frozen pytest tests/test_node_raw.py -q` FAILS (callbacks decode messages).
- Switch the two subscriptions to raw and move the decode into on_timer; rerun, PASS; run the whole suite.
- `uv run poe lint`; update the README.
- Commit `perf(marker_detector): decode the head image only in the process timer`.

## Notes

Evidence (brief item 4): the node deserialized every ~19 fps frame to process 2.2 Hz at 25 ms each; expected -0.07 core. Prior art for raw subscriptions with lazy decode: nodes/mcp_server/mcp_server/ros_iface.py `cache_raw`/`raw_store` (rclpy `create_subscription(..., raw=True)` and `rclpy.serialization.deserialize_message`). Depends on marker-events-executor (same node.py).
