---
title: Zoom state machine
status: escalated
repo: ros2-lerobot-sverve-platform
value: A zoom always returns a sharp crop or a clear error and always gives the 30 fps stream back.
files: [nodes/head_camera_cpp/include/head_camera_cpp/camera_backend.hpp, nodes/head_camera_cpp/include/head_camera_cpp/zoom_controller.hpp, nodes/head_camera_cpp/src/core/zoom_controller.cpp, nodes/head_camera_cpp/test/core/fake_camera_backend.hpp, nodes/head_camera_cpp/test/core/test_zoom_controller.cpp]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: [zoom-crop-geometry, jpeg-encode-and-yuv-crop, zoom-request-json]
owns: [camera-backend-interface]
uses: [head-camera-config, crop-geometry, jpeg-encoding, zoom-json-contract]
model: opus
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--zoom-state-machine
branch: front-camera-30-fps-stream-w--zoom-state-machine
verified_head: ""
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# Zoom state machine

## Requirement

- `camera_backend.hpp` (the `camera-backend-interface` contract, no libcamera types): `enum class Mode {Video, Still}`; `enum class AfState {Idle, Scanning, Focused, Failed}`; `struct AfRequest {bool windowed; Rect window; bool trigger;}`; `struct StillFrame {Yuv420View view; int64_t stamp_ns; AfState af_state; std::optional<double> lens_position;}` (valid until the next backend call); `struct VideoFrame {const uint8_t * bgr; int stride; Size size; int64_t stamp_ns;}`; `class CameraBackend` with virtual `bool start_video()` (video mode, continuous AF), `void stop()`, `bool configure(Mode)`, `bool start_still()` (AF auto, lens held), `bool set_af(const AfRequest &)`, `AfState af_state()`, `std::optional<StillFrame> wait_still_frame(std::chrono::milliseconds)`, `void set_video_callback(std::function<void(const VideoFrame &)>)`, `bool reacquire()`, `Rect scaler_crop_maximum()`. `class Clock` with `now()` and `sleep_for()`, plus `SteadyClock`.
- `zoom_controller.hpp` / `src/core/zoom_controller.cpp` (D6): `ZoomController(CameraBackend &, Clock &, const ZoomConfig &, std::function<void()> fatal = [] { std::exit(1); })`; `ZoomOutcome run(const ZoomRequest &)` returning `{ZoomResult result; std::vector<uint8_t> jpeg}` (jpeg empty unless `carries_image`); `bool try_begin()` / busy handling so a second concurrent `run` returns `busy` at once without touching the backend.
- Phases: AF (skipped when `af` false): `set_af` with the window from `af_window(u, v, af_window_fraction, scaler_crop_maximum())` and trigger, poll `af_state()` until Focused/Failed or `af_timeout_s`; Failed or timed out continues as `af_failed`. SWITCH: `stop`, `configure(Still)`, `start_still`, take frames until the (`settle_frames`+1)-th within `switch_timeout_s` (else `timeout`); `configure` or `start_still` false -> `camera_error`. CROP: `zoom_region` + `output_size` + `crop_yuv420_jpeg` at `zoom.jpeg_quality`; encode error -> `camera_error`. RESTORE always (after success and after every failure): `stop`, `configure(Video)`, `start_video`; on failure sleep `restore_retry_delay_s` and retry once, then `reacquire()` and try again, then call `fatal`. Watchdog `watchdog_s` over the whole run -> `timeout` (restore still runs). `phase_ms` measured with the Clock; result carries crop, full, out, af_state, lens_position and the still frame's stamp_ns.
- `test/core/fake_camera_backend.hpp`: scripted fake (AfState sequence, per-call success flags, frames from a synthetic YUV420 buffer, call log) reusable by head-camera-ros-wiring; a fake Clock advancing on `sleep_for`.
- Tests first in `test/core/test_zoom_controller.cpp`, seen failing: happy path call order (set_af, AF poll, stop, configure Still, start_still, 3 frames, stop, configure Video, start_video) and an ok result with a JPEG of `output_size`; af false sets no AF; AF Failed and AF timeout give `af_failed` with an image; switch timeout gives `timeout`, no image, restore ran; configure(Still) failure gives `camera_error`, restore ran; restore fails once then succeeds -> no fatal; restore fails twice -> `reacquire` then success; reacquire fails -> `fatal` called once; watchdog -> `timeout` and restore ran; concurrent second run -> `busy` with an untouched backend; settle_frames honoured; phase_ms non-negative and summing to the elapsed fake time.

## Acceptance conditions

- a1: a successful zoom runs AF, switch, crop and restore in order and returns an ok result with a JPEG [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: every failure branch (AF failed or timed out, switch timeout, configure failure, watchdog) restores the video mode and only af_failed carries an image [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: restore retries once, then re-acquires, then calls fatal [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: a concurrent request gets busy without touching the camera [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a5: package gtests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

## Files

- `nodes/head_camera_cpp/include/head_camera_cpp/camera_backend.hpp`
- `nodes/head_camera_cpp/include/head_camera_cpp/zoom_controller.hpp`
- `nodes/head_camera_cpp/src/core/zoom_controller.cpp`
- `nodes/head_camera_cpp/test/core/fake_camera_backend.hpp`
- `nodes/head_camera_cpp/test/core/test_zoom_controller.cpp`

## Interfaces

- `camera-backend-interface`: `include/head_camera_cpp/camera_backend.hpp` (`CameraBackend`, `Mode`, `AfState`, `AfRequest`, `StillFrame`, `VideoFrame`, `Clock`, `SteadyClock`).
- `head_camera_cpp::ZoomController::run`, `ZoomOutcome` in `zoom_controller.hpp`.
- `test/core/fake_camera_backend.hpp` (`FakeCameraBackend`, `FakeClock`), reused by test_node.

## Expected outcome

The four-phase zoom state machine is fully tested against a fake camera; the interface the libcamera adapter implements is fixed.

## Steps

- Write `camera_backend.hpp`, the fake and `test/core/test_zoom_controller.cpp` against a declared-only controller; run `scripts/cpp-node-test.sh head_camera_cpp`, see it fail.
- Implement the controller; rerun until PASS; commit `feat(head_camera_cpp): zoom state machine behind a camera backend interface`.

## Notes

- Timeouts and phase order are D6 (architecture decision); AfTrigger only works in AfModeAuto (`ipa_base.cpp:1399-1406`), so the AF request switches to Auto.
- `fatal` is injectable so tests never exit; production passes the default.
- The fake header lives under `test/core/` and is not a test (the glob matches only `test_*.cpp`).
