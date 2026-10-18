---
title: Head camera mapping publish
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head camera mapping publish

## Requirement

- Node wiring (D2; design D4 of the RTAB-Map epic): when `mapping.enabled`, the node creates `mapping_image_topic()` (sensor_msgs/CompressedImage, JPEG at `mapping.jpeg_quality`) and `mapping_camera_info_topic()` (sensor_msgs/CameraInfo) publishers, best_effort depth 1 (head_rgbd_cpp subscribes best_effort). For each captured stream frame, if `MappingDecimator::due(stamp)` and the image publisher has subscribers, it resizes with `resize_for_mapping`, encodes, and publishes the image and, only when a calibration file is loaded, the camera_info from `scale_intrinsics` with the same header (stamp = capture stamp, frame_id = `frame_id`). Without a calibration the image is still published and camera_info is not (no placeholder intrinsics; head_rgbd_cpp then publishes nothing, which the runbook checks). No mapping publish while a zoom holds the camera in still mode.
- Focus lock (decided by the user on 2026-10-18): the camera keeps the configured `af_mode` (default continuous) normally. While `mapping.enabled` and `mapping.lock_focus` (runtime-settable node parameter) are true and the topic `/head_camera/mapping/image_raw/compressed` has at least one subscriber, the node applies `AfMode` Manual and `LensPosition` = `lens_position` (libcamera_controls); when the last subscriber leaves (or `mapping.lock_focus` is set false) it restores the configured af_mode. A zoom still autofocuses on its point for its single still; the zoom controller's RESTORE phase then returns to the mapping lock (Manual at `lens_position`) when the lock is active, else to the configured AF mode, so a zoom never leaves the lens elsewhere. The intrinsics used for mapping are those calibrated at the locked focus.
- The mapping encode runs only on due frames with subscribers: at 30 fps input and 2 Hz output at most 2 encodes per second (asserted with the fake backend).
- Tests first, seen failing: `test/test_node.cpp` with the fake backend (publishers exist only when enabled; 2 Hz output from 30 fps input with a subscriber, none without; camera_info only with the test calibration `test/data/calibration.yaml`, scaled K; same stamp on image and info; nothing published during a zoom), `test/core/test_zoom_controller.cpp` (restore after ok, after af_failed and after timeout returns to the mapping lock at the configured lens position when the lock is active; continuous stays continuous when it is not), and `test_node.cpp` also covers the lock engaging on the first mapping subscriber and releasing on the last, and `mapping.lock_focus=false` disabling it, `test/test_libcamera_controls.cpp` (manual control list carries AfMode Manual and LensPosition).
- README: `## Mapping stream` section (topics, rate, size, the calibration requirement, the focus-lock rule: continuous AF normally, locked at `lens_position` only while the mapping topic has subscribers, `mapping.lock_focus` to disable it) and the configuration table rows for `lens_position` and `mapping.*`.

## Acceptance conditions

- a1: the mapping stream is published at the configured rate only with a subscriber and only when enabled [evidence: test: nodes/head_camera_cpp/test/test_node.cpp]
- a2: mapping camera_info is published only from a loaded calibration, scaled, with the image's stamp [evidence: test: nodes/head_camera_cpp/test/test_node.cpp]
- a3: every zoom outcome returns to the mapping focus lock at the configured lens position while mapping is active, and to the configured af_mode otherwise [evidence: test: nodes/head_camera_cpp/test/core/test_zoom_controller.cpp]
- a4: the lock controls carry AfMode Manual and LensPosition [evidence: test: nodes/head_camera_cpp/test/test_libcamera_controls.cpp]
- a5: the README documents the mapping stream [evidence: doc-section: nodes/head_camera_cpp/README.md#Mapping stream]
- a6: focus is locked at lens_position only while the mapping image topic has subscribers and mapping.lock_focus is true, continuous autofocus otherwise, with a zoom autofocusing then returning to the lock [evidence: test: nodes/head_camera_cpp/test/test_node.cpp]
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
