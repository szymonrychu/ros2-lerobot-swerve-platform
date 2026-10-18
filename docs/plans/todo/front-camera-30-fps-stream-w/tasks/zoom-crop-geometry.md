---
title: Zoom crop geometry
status: todo
repo: ros2-lerobot-sverve-platform
value: Zoom requests map to the exact full-resolution crop and AF window the user pointed at.
files: [nodes/head_camera_cpp/include/head_camera_cpp/crop.hpp, nodes/head_camera_cpp/src/core/crop.cpp, nodes/head_camera_cpp/test/core/test_crop.cpp]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: [head-camera-node-config]
owns: [crop-geometry]
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Zoom crop geometry

## Requirement

- `crop.hpp` / `src/core/crop.cpp` (D5), pure functions, no OpenCV or libcamera types: `struct Size {int width; int height;}`, `struct Rect {int x; int y; int width; int height;}`.
- `Rect zoom_region(double u, double v, int zoom, Size full)`: region (full.width/zoom) x (full.height/zoom) centred on (u*full.width, v*full.height), shifted (never shrunk) to lie inside the frame, x, y, width and height even. Throws `std::invalid_argument` for u or v outside [0, 1] or zoom < 1.
- `Size output_size(Rect region, int max_px)`: longest side min(region longest side, max_px), aspect kept (rounded to even), never larger than the region.
- `Rect af_window(double u, double v, double fraction, Rect scaler_crop_max)`: rectangle of `fraction` of the scaler crop's width and height centred on (u, v) mapped into `scaler_crop_max` (offset included), clamped inside it.
- Tests first in `test/core/test_crop.cpp`, seen failing: centre point for zoom 2/4/8 on 4608x2592 gives 2304x1296 / 1152x648 / 576x324 centred; the four corners shift the region inside the frame without shrinking it; all coordinates even; output_size: zoom 2 with max_px 1024 -> 1024x576, zoom 8 -> 576x324 (native, no upscale), max_px 256 -> 256x144; af_window at the centre, at each corner (clamped), and with a non-zero scaler crop offset; invalid u, v and zoom throw.

## Acceptance conditions

- a1: zoom 2/4/8 regions are full/zoom, centred and shifted inside the 4608x2592 frame with even coordinates [evidence: test: nodes/head_camera_cpp/test/core/test_crop.cpp]
- a2: output size caps the longest side at max_px and never upscales [evidence: test: nodes/head_camera_cpp/test/core/test_crop.cpp]
- a3: the AF window is a clamped fraction of ScalerCropMaximum around the point [evidence: test: nodes/head_camera_cpp/test/core/test_crop.cpp]
- a4: package gtests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

## Files

- `nodes/head_camera_cpp/include/head_camera_cpp/crop.hpp`
- `nodes/head_camera_cpp/src/core/crop.cpp`
- `nodes/head_camera_cpp/test/core/test_crop.cpp`

## Interfaces

- `head_camera_cpp::Size`, `Rect`, `zoom_region`, `output_size`, `af_window` in `include/head_camera_cpp/crop.hpp` (the `crop-geometry` contract).

## Expected outcome

Crop and AF-window maths exist as pure, tested functions used by the encoder, the state machine and the libcamera adapter.

## Steps

- Write `test/core/test_crop.cpp` and an empty `crop.cpp` with the declarations; run `scripts/cpp-node-test.sh head_camera_cpp`, see test_crop fail.
- Implement; rerun until PASS; commit `feat(head_camera_cpp): zoom crop geometry and AF window`.

## Notes

- The reading of the decision's "side = clamp(4608/zoom, max_px)" is D5: region = full/zoom, output <= max_px.
- No CMakeLists edit: the core glob picks the files up.
