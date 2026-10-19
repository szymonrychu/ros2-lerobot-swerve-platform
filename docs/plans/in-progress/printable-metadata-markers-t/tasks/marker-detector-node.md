---
title: Marker detector node
status: todo
repo: ros2-lerobot-sverve-platform
value: "Markers the head camera sees turn into POIs automatically, and nothing is sent without a real, unambiguous detection."
files: [nodes/marker_detector/marker_detector/pipeline.py, nodes/marker_detector/marker_detector/node.py, nodes/marker_detector/marker_detector/__main__.py, nodes/marker_detector/marker_detector/metrics.py, nodes/marker_detector/tests/ros_stubs.py, nodes/marker_detector/tests/test_pipeline.py, nodes/marker_detector/README.md]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [marker-tof-plane, marker-map-pose-poi, marker-zoom-planner]
owns: [marker-detector-node-entry]
uses: [marker-detector-config, marker-synthetic-scenes, marker-decode, marker-pose, marker-tof-plane, marker-geometry, marker-zoom]
model: sonnet
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-detector-node
branch: printable-metadata-markers-t--marker-detector-node
verified_head: ""
run: printable-metadata-markers-t
provides: []
acs_for_audit: []
process_gaps: []
---

# Marker detector node

## Requirement

- `nodes/marker_detector/marker_detector/pipeline.py` (no ROS): `MarkerPipeline(cfg, clock)` with `on_camera_info(K, D, size, frame_id)`, `on_depth(stamp_ns, depth_mm, depth_K)`, `process_frame(stamp_ns, jpeg: bytes, source: str, lookup_tf: Callable[[str, int], tuple | None], K_override=None) -> FrameOutput` (decode -> pose (with the ToF chooser when an ambiguous pose has a depth frame within `tof.max_dt_s` and `tof.enabled`) -> TF (map <- camera frame at the image stamp) -> `min_confidence` gate -> commands, report and zoom proposals) and `process_zoom(frame: ZoomFrame, result: dict, lookup_tf)` (zoom intrinsics from the stored stream camera_info). `FrameOutput`: `commands` (upsert dicts), `report` (str or None), `zoom_request` (dict or None), `rejections` (reason counts). Placeholder rules: no camera_info -> nothing processed (one log line per state change); undecodable or invalid payload, ambiguous or failed pose, TF lookup failure (None) -> no command; a report only when at least one detection was accepted.
- `nodes/marker_detector/marker_detector/node.py` (rclpy, thin): subscriptions (image CompressedImage keep_last 1 best effort, camera_info, depth Image + camera_info when `tof.enabled`, zoom image/result when `zoom.process_stills` or `zoom.auto_zoom`), a timer at `process_hz` that processes only the newest unprocessed frame, `tf2_ros.Buffer`/`TransformListener` lookup at the image stamp with `tf_timeout_s`, publishers `/poi/command` (String JSON, reliable keep_last 20) and `/markers/detections` (String), zoom command publisher; metrics server via `ros2_metrics`. `__main__.py` loads `config_path()`. `metrics.py`: counters for frames processed, detections accepted, rejections by reason, zoom requests by status, and a processing-time histogram.
- Tests first in `nodes/marker_detector/tests/test_pipeline.py` (synthetic scenes; `tests/ros_stubs.py` stubs rclpy, tf2_ros, sensor_msgs, std_msgs for a node import smoke test), seen failing: a frame with one 150 mm marker at 1 m and identity TF gives exactly one `upsert_marker` command with the right key, map pose within 1 % and a report; without camera_info nothing is produced; with a TF lookup returning None nothing is produced; an invalid payload QR produces no command and a `payload` rejection; an ambiguous far marker produces no command without depth and one command with a matching synthetic depth plane (chooser `tof`); a 60 mm marker at 2.5 m (module < 6 px) yields no zoom request with the default `auto_zoom: false`, and an externally requested synthetic zoom still of it (result plus image, rendered with the crop intrinsics) yields one command whose pose matches within 2 %; with `auto_zoom: true` the same frame yields a zoom request and its still yields the command; confidence below `min_confidence` sends nothing; the node module imports with the stubs and wires the configured topics.
- `nodes/marker_detector/README.md`: pipeline, placeholder rules, topics and QoS, the report format, metrics, and the dependency on calibrated camera_info, head TF and (optional) `/head_rgbd/depth`.

## Acceptance conditions

- a1: a synthetic detection produces exactly one upsert_marker command and a report [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q]
- a2: no camera_info, no TF, an invalid payload, an ambiguous pose or low confidence produce nothing [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q]
- a3: an ambiguous pose with matching ToF depth is accepted via the plane chooser [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q]
- a4: zoom stills (external by default, own requests only with auto_zoom on) produce the POI of a small distant marker [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q]
- a5: the node entry wires the configured topics [evidence: symbol: nodes/marker_detector/marker_detector/node.py:run_node]
- a6: the README documents topics, placeholder rules and dependencies [evidence: command: grep -q '/markers/detections' nodes/marker_detector/README.md && grep -q 'camera_info' nodes/marker_detector/README.md]

## Files

- `nodes/marker_detector/marker_detector/pipeline.py`
- `nodes/marker_detector/marker_detector/node.py`
- `nodes/marker_detector/marker_detector/__main__.py`
- `nodes/marker_detector/marker_detector/metrics.py`
- `nodes/marker_detector/tests/ros_stubs.py`
- `nodes/marker_detector/tests/test_pipeline.py`
- `nodes/marker_detector/README.md`

## Interfaces

- `nodes/marker_detector/marker_detector/pipeline.py:MarkerPipeline`, `nodes/marker_detector/marker_detector/node.py:run_node`, `python3 -m marker_detector` (part `marker-detector-node-entry`); topics `/poi/command` (op `upsert_marker`), `/markers/detections`, `/head_camera/zoom/command`.

## Expected outcome

`python3 -m marker_detector` turns head camera sightings into POI upserts.

## Steps

- Write `tests/test_pipeline.py` and the stubs; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q`; see it fail.
- Implement pipeline.py, node.py, __main__.py, metrics.py; rerun the whole node suite; `uv run --frozen poe lint`.
- Update the README; commit `feat(marker_detector): detection node publishing marker POI sightings`.

## Notes

- numpy dependency guard (CLAUDE.md): numpy is a direct dependency already; the deploy check in the runbook confirms no ModuleNotFoundError.
- Pattern for a ROS-free core plus a thin node: nodes/poi_store (store.py vs node.py) and its tests/ros_stubs.py.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-tof-plane, marker-map-pose-poi
