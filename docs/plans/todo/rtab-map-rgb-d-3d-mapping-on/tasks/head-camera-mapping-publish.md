---
title: Head camera mapping publish
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head camera publishes a calibrated 576x324 JPEG at 2 Hz for RTAB-Map only while someone subscribes, and its focus stays where the calibration was taken."
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

- Node wiring (D4): when `mapping.enabled`, the node creates `mapping_image_topic()` (sensor_msgs/CompressedImage, JPEG at `mapping.jpeg_quality`) and `mapping_camera_info_topic()` (sensor_msgs/CameraInfo) publishers, best_effort depth 1 (the uplink relay subscribes best_effort). For each captured stream frame, if `MappingDecimator::due(stamp)` and the image publisher has subscribers, it resizes with `resize_for_mapping`, encodes, and publishes the image and, only when a calibration file is loaded, the camera_info from `scale_intrinsics` with the same header (stamp = capture stamp, frame_id = `frame_id`). Without a calibration the image is still published and camera_info is not (no placeholder intrinsics; RTAB-Map then waits, which the runbook checks). No mapping publish while a zoom holds the camera in still mode.
- Focus: with `af_mode: manual`, startup applies `AfMode` Manual and `LensPosition` = `lens_position` (libcamera_controls); the zoom controller's RESTORE phase returns to the configured AF mode and, for manual, re-applies the configured lens position (today it restores Continuous unconditionally), so a zoom never leaves the lens elsewhere.
- The mapping encode runs only on due frames with subscribers: at 30 fps input and 2 Hz output at most 2 encodes per second (asserted with the fake backend).
- Tests first, seen failing: `test/test_node.cpp` with the fake backend (publishers exist only when enabled; 2 Hz output from 30 fps input with a subscriber, none without; camera_info only with the test calibration `test/data/calibration.yaml`, scaled K; same stamp on image and info; nothing published during a zoom), `test/core/test_zoom_controller.cpp` (restore after ok, after af_failed and after timeout restores manual focus with the configured lens position; continuous stays continuous), `test/test_libcamera_controls.cpp` (manual control list carries AfMode Manual and LensPosition).
- README: `## Mapping stream` section (topics, rate, size, the calibration requirement, the fixed-focus rule and its cost: close-up shots rely on zoom AF) and the configuration table rows for `lens_position` and `mapping.*`.

## Acceptance conditions

- a1: the mapping stream is published at the configured rate only with a subscriber and only when enabled [evidence: test: nodes/head_camera_cpp/test/test_node.cpp]
- a2: mapping camera_info is published only from a loaded calibration, scaled, with the image's stamp [evidence: test: nodes/head_camera_cpp/test/test_node.cpp]
- a3: every zoom outcome restores manual focus to the configured lens position [evidence: test: nodes/head_camera_cpp/test/core/test_zoom_controller.cpp]
- a4: manual focus controls carry AfMode Manual and LensPosition [evidence: test: nodes/head_camera_cpp/test/test_libcamera_controls.cpp]
- a5: the README documents the mapping stream [evidence: doc-section: nodes/head_camera_cpp/README.md#Mapping stream]
- a6: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

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

- Topics `/head_camera/mapping/image_raw/compressed` (CompressedImage, JPEG) and `/head_camera/mapping/camera_info` (CameraInfo, frame `head_camera_optical_frame`): the `head-camera-mapping-topics` part, consumed through the map3d topic contract.

## Expected outcome

With `mapping.enabled: true` and a calibration file, a subscriber receives 576x324 JPEG + matching CameraInfo at 2 Hz; focus is fixed and survives zooms.

## Steps

- Write the node, zoom controller and controls tests; run `scripts/cpp-node-test.sh head_camera_cpp`; see `FAIL`.
- Implement the wiring, restore change and controls; rerun until `cpp-node-test: head_camera_cpp PASS`.
- Update the README; commit `feat(head_camera_cpp): 2 Hz mapping stream and fixed focus`.

## Notes

- File names are the front-camera epic's end state (`src/head_camera_node.cpp`, `src/core/zoom_controller.cpp`, `src/libcamera_controls.cpp`).
- The Ansible config (`mapping.enabled`, `af_mode: manual`, `lens_position`, `camera_info_file`) is set by map3d-uplink-node-type and final-suite-and-runbook, not here.
- CPU: the mapping encode at 576x324 / 2 Hz is a few ms per second; the runbook measures it.
