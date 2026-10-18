---
title: Libcamera backend
status: todo
repo: ros2-lerobot-sverve-platform
value: "The node drives the real IMX708: 30 fps video, a full-resolution still on request and windowed autofocus."
files: [nodes/head_camera_cpp/include/head_camera_cpp/libcamera_backend.hpp, nodes/head_camera_cpp/src/libcamera_backend.cpp, nodes/head_camera_cpp/src/libcamera_controls.cpp, nodes/head_camera_cpp/test/test_libcamera_controls.cpp, nodes/head_camera_cpp/CMakeLists.txt]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: [zoom-state-machine]
owns: []
uses: [camera-backend-interface, crop-geometry, head-camera-config]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Libcamera backend

## Requirement

- `libcamera_controls.cpp` (declared in `libcamera_backend.hpp`), pure helpers on libcamera types only (no camera needed): `libcamera::ControlList video_controls(const Config &)` (FrameDurationLimits [frame_duration_us, frame_duration_us], AfMode Continuous or as configured); `libcamera::ControlList af_controls(const AfRequest &)` (AfMode Auto, AfMetering Windows, AfWindows = the window as `libcamera::Rectangle`, AfTrigger Start when trigger); `libcamera::ControlList still_controls()` (AfMode Auto, no trigger); `AfState to_af_state(int32_t libcamera_af_state)`; `libcamera::Rectangle to_rectangle(Rect)` / `Rect from_rectangle(const libcamera::Rectangle &)`.
- `LibcameraBackend : CameraBackend` in `libcamera_backend.cpp`, the only file that touches a camera: owns a `libcamera::CameraManager`, gets the camera by `camera_id`, acquires it; video mode = VideoRecording role 1152x648 RGB888 plus the 2304:1296 sensor mode (Raw stream or sensor configuration), 4 buffers, requests re-queued on completion; still mode = StillCapture 4608x2592 YUV420 plus the 4608:2592 sensor mode, `still_buffers` buffers allocated on configure and freed when leaving the mode; `requestCompleted` runs on libcamera's thread and only hands frames off (video: the registered callback through a non-blocking handoff, still: a queue `wait_still_frame` pops with a timeout); buffers mmapped once per configuration; frame metadata gives SensorTimestamp, AfState and LensPosition; `af_state()` returns the latest metadata value; `reacquire()` releases and re-acquires the camera; `scaler_crop_maximum()` from camera properties. Every libcamera call's error is logged and returned as false (no exceptions across the libcamera thread).
- `CMakeLists.txt`: `pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)`; library `head_camera_libcamera` (the two .cpp files) linked to `head_camera_core` and `PkgConfig::LIBCAMERA`; gtest `test_libcamera_controls` (explicit target, `test/test_libcamera_controls.cpp`).
- Tests first in `test/test_libcamera_controls.cpp`, seen failing: each control list holds exactly the expected controls and values (AfWindows rectangle equals the given window; FrameDurationLimits 33333 for the default config); AfState mapping of every libcamera value; Rect/Rectangle round trip.
- The adapter compiles in docker against the rosdep libcamera and must use only API present in both it and the fork `v0.7.2+rpt20260817`; any version difference handled with `LIBCAMERA_VERSION_*` guards, stated in a comment.

## Acceptance conditions

- a1: video, AF and still control lists hold exactly the D2/D6 controls [evidence: test: nodes/head_camera_cpp/test/test_libcamera_controls.cpp]
- a2: libcamera AfState values map onto the backend AfState [evidence: test: nodes/head_camera_cpp/test/test_libcamera_controls.cpp]
- a3: LibcameraBackend implements every CameraBackend method [evidence: symbol: head_camera_cpp::LibcameraBackend]
- a4: requestCompleted never blocks: it only hands frames to the callback or the still queue [evidence: judgement]
- a5: the adapter compiles against the docker libcamera and all package tests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

## Files

- `nodes/head_camera_cpp/include/head_camera_cpp/libcamera_backend.hpp`
- `nodes/head_camera_cpp/src/libcamera_backend.cpp`
- `nodes/head_camera_cpp/src/libcamera_controls.cpp`
- `nodes/head_camera_cpp/test/test_libcamera_controls.cpp`
- `nodes/head_camera_cpp/CMakeLists.txt`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
