---
title: JPEG encode and YUV crop
status: done
repo: ros2-lerobot-sverve-platform
value: Preview frames are compressed at the configured quality and zoom crops are converted only where needed.
files: [nodes/head_camera_cpp/include/head_camera_cpp/encode.hpp, nodes/head_camera_cpp/src/core/encode.cpp, nodes/head_camera_cpp/test/core/test_encode.cpp]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: [zoom-crop-geometry]
owns: [jpeg-encoding]
uses: [crop-geometry]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--jpeg-encode-and-yuv-crop
branch: front-camera-30-fps-stream-w--jpeg-encode-and-yuv-crop
verified_head: 5c2b6ea405445603d10e3fa6661c615f369af806
run: front-camera-30-fps-stream-w
provides: [jpeg-encoding]
acs_for_audit: []
process_gaps: []
---

# JPEG encode and YUV crop

## Requirement

- `encode.hpp` / `src/core/encode.cpp` with OpenCV (D2, D5):
  - `std::vector<uint8_t> encode_bgr_jpeg(const uint8_t * data, int width, int height, int stride, int quality)`: JPEG of a packed BGR888 frame with row stride, `cv::IMWRITE_JPEG_QUALITY` = quality; throws `std::invalid_argument` on null data, non-positive size, stride < 3*width or quality outside 1..100.
  - `struct Yuv420View {const uint8_t * y; const uint8_t * u; const uint8_t * v; int y_stride; int uv_stride; Size size;}` and `std::vector<uint8_t> crop_yuv420_jpeg(const Yuv420View & frame, Rect region, Size out, int quality)`: converts only the region to BGR (sub-plane I420 copy, `cv::COLOR_YUV2BGR_I420`), resizes with `cv::INTER_AREA` when `out` is smaller, encodes at quality. Throws for a region outside the frame, odd region coordinates, an `out` larger than the region (no upscaling) or a bad quality.
- Tests first in `test/core/test_encode.cpp`, seen failing: the bytes of `encode_bgr_jpeg` at quality q equal `cv::imencode` at q on the same synthetic frame; a textured frame at quality 30 is smaller than at 90; a padded stride gives the same image as the packed frame; a synthetic YUV420 frame with four coloured quadrants cropped around one quadrant decodes (`cv::imdecode`) to the expected size and dominant colour; a downscaled crop has the requested `out` size; every invalid input throws and produces no bytes.

## Acceptance conditions

- a1: encode_bgr_jpeg honours the requested quality (fixes the ignored quality, U8) [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: crop_yuv420_jpeg converts only the region, downscales with INTER_AREA and never upscales [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: invalid frames, regions and qualities throw instead of producing an image [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: package gtests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

## Files

- `nodes/head_camera_cpp/include/head_camera_cpp/encode.hpp`
- `nodes/head_camera_cpp/src/core/encode.cpp`
- `nodes/head_camera_cpp/test/core/test_encode.cpp`

## Interfaces

- `head_camera_cpp::encode_bgr_jpeg`, `Yuv420View`, `crop_yuv420_jpeg` in `include/head_camera_cpp/encode.hpp` (the `jpeg-encoding` contract), using `Size`/`Rect` from crop.hpp.

## Expected outcome

Tested JPEG encoding for the preview and the zoom crop, with the configured quality honoured.

## Steps

- Write `test/core/test_encode.cpp` against declared-only functions; run `scripts/cpp-node-test.sh head_camera_cpp`, see test_encode fail.
- Implement; rerun until PASS; commit `feat(head_camera_cpp): JPEG encoding and YUV420 crop`.

## Notes

- The full-resolution frame is never converted as a whole: only the crop (36 MB RGB per full frame).
- The libcamera RGB888 format is B, G, R in memory, so the preview path is BGR.
