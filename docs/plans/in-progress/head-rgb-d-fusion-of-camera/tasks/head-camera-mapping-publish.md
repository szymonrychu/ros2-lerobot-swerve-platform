---
title: Head camera mapping publish
status: done
repo: ros2-lerobot-sverve-platform
value: "The head camera publishes a calibrated 576x324 JPEG at 2 Hz for the RGB-D fusion node (and through it RTAB-Map) only while someone subscribes, and its focus is locked where the calibration was taken only while fusion or 3D mapping runs."
files: [nodes/head_camera_cpp/include/head_camera_cpp/head_camera_node.hpp, nodes/head_camera_cpp/src/head_camera_node.cpp, nodes/head_camera_cpp/test/test_node.cpp, nodes/head_camera_cpp/src/core/zoom_controller.cpp, nodes/head_camera_cpp/test/core/test_zoom_controller.cpp, nodes/head_camera_cpp/src/libcamera_controls.cpp, nodes/head_camera_cpp/test/test_libcamera_controls.cpp, nodes/head_camera_cpp/README.md]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: [head-camera-mapping-config]
owns: [head-camera-mapping-topics]
uses: [head-camera-mapping-core]
model: sonnet
estimate: 75
actual: ""
review: ""
branch: head-rgb-d-fusion-of-camera--head-camera-mapping-publish
verified_head: 39c9bcd379da707c0c2a23c6a0d6bea85ffd982a
run: head-rgb-d-fusion-of-camera
provides: [head-camera-mapping-topics, camera-backend-focus-lock]
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--head-camera-mapping-publish
acs_for_audit: []
process_gaps: []
---

# Head camera mapping publish

## Requirement

- Node wiring (D2; design D4 of the RTAB-Map epic): when `mapping.enabled`, the node creates `mapping_image_topic()` (sensor_msgs/CompressedImage, JPEG at `mapping.jpeg_quality`) and `mapping_camera_info_topic()` (sensor_msgs/CameraInfo) publishers, best_effort depth 1 (head_rgbd_cpp subscribes best_effort). For each captured stream frame, if `MappingDecimator::due(stamp)` and the image publisher has subscribers, it resizes with `resize_for_mapping`, encodes, and publishes the image and, only when a calibration file is loaded, the camera_info from `scale_intrinsics` with the same header (stamp = capture stamp, frame_id = `frame_id`). Without a calibration the image is still published and camera_info is not (no placeholder intrinsics; head_rgbd_cpp then publishes nothing, which the runbook checks). No mapping publish while a zoom holds the camera in still mode.
- Focus lock (decided by the user on 2026-10-18): the camera keeps the configured `af_mode` (default continuous) normally. While `mapping.enabled` and `mapping.lock_focus` (runtime-settable node parameter) are true and the topic `/head_camera/mapping/image_raw/compressed` has at least one subscriber, the node applies `AfMode` Manual and `LensPosition` = `lens_position` (libcamera_controls); when the last subscriber leaves (or `mapping.lock_focus` is set false) it restores the configured af_mode. A zoom still autofocuses on its point for its single still; the zoom controller's RESTORE phase then returns to the mapping lock (Manual at `lens_position`) when the lock is active, else to the configured AF mode, so a zoom never leaves the lens elsewhere. The intrinsics used for mapping are those calibrated at the locked focus.
- The mapping encode runs only on due frames with subscribers: at 30 fps input and 2 Hz output at most 2 encodes per second (asserted with the fake backend).
- Tests first, seen failing: `test/test_node.cpp` with the fake backend (publishers exist only when enabled; 2 Hz output from 30 fps input with a subscriber, none without; camera_info only with the test calibration `test/data/calibration.yaml`, scaled K; same stamp on image and info; nothing published during a zoom), `test/core/test_zoom_controller.cpp` (restore after ok, after af_failed and after timeout returns to the mapping lock at the configured lens position when the lock is active; continuous stays continuous when it is not), and `test_node.cpp` also covers the lock engaging on the first mapping subscriber and releasing on the last, and `mapping.lock_focus=false` disabling it, `test/test_libcamera_controls.cpp` (manual control list carries AfMode Manual and LensPosition).
- README: `## Mapping stream` section (topics, rate, size, the calibration requirement, the focus-lock rule: continuous AF normally, locked at `lens_position` only while the mapping topic has subscribers, `mapping.lock_focus` to disable it) and the configuration table rows for `lens_position` and `mapping.*`.

## Acceptance conditions

- a1: the mapping stream is published at the configured rate only with a subscriber and only when enabled [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: mapping camera_info is published only from a loaded calibration, scaled, with the image's stamp [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: every zoom outcome returns to the mapping focus lock at the configured lens position while mapping is active, and to the configured af_mode otherwise [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: the lock controls carry AfMode Manual and LensPosition [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a5: the README documents the mapping stream [evidence: doc-section: nodes/head_camera_cpp/README.md#Mapping stream]
- a6: focus is locked at lens_position only while the mapping image topic has subscribers and mapping.lock_focus is true, continuous autofocus otherwise, with a zoom autofocusing then returning to the lock [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a7: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

## Files

- `nodes/head_camera_cpp/include/head_camera_cpp/head_camera_node.hpp`
- `nodes/head_camera_cpp/src/head_camera_node.cpp`
- `nodes/head_camera_cpp/test/test_node.cpp`
- `nodes/head_camera_cpp/src/core/zoom_controller.cpp`
- `nodes/head_camera_cpp/test/core/test_zoom_controller.cpp`
- `nodes/head_camera_cpp/src/libcamera_controls.cpp`
- `nodes/head_camera_cpp/test/test_libcamera_controls.cpp`
- `nodes/head_camera_cpp/README.md`

## Interfaces

- Topics `/head_camera/mapping/image_raw/compressed` (CompressedImage, JPEG) and `/head_camera/mapping/camera_info` (CameraInfo, frame `head_camera_optical_frame`): the `head-camera-mapping-topics` part, consumed by head_rgbd_cpp (head-rgbd-node-wiring), which passes the pair on to RTAB-Map.

## Expected outcome

With `mapping.enabled: true` and a calibration file, a subscriber receives 576x324 JPEG + matching CameraInfo at 2 Hz; focus is locked while subscribed (continuous otherwise) and returns to the lock after zooms.

## Steps

- Write the node, zoom controller and controls tests; run `scripts/cpp-node-test.sh head_camera_cpp`; see `FAIL`.
- Implement the wiring, restore change and controls; rerun until `cpp-node-test: head_camera_cpp PASS`.
- Update the README; commit `feat(head_camera_cpp): 2 Hz mapping stream and focus lock`.

## Notes

- File names are the front-camera epic's end state (`src/head_camera_node.cpp`, `src/core/zoom_controller.cpp`, `src/libcamera_controls.cpp`).
- The Ansible config (`mapping.enabled`, `mapping.lock_focus`, `lens_position`, `camera_info_file`; `af_mode` stays continuous) is set by head-rgbd-node-type and rgbd-final-suite-and-runbook, not here.
- CPU: the mapping encode at 576x324 / 2 Hz is a few ms per second; the runbook measures it.
- Moved unchanged in substance from epic rtab-map-rgb-d-3d-mapping-on (which now depends on this epic, U3); the RGB-D fusion node is its first consumer and engages the focus lock by subscribing (U4).
