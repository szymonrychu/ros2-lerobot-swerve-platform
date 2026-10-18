#pragma once

#include <cstdint>
#include <vector>

#include "head_camera_cpp/crop.hpp"

namespace head_camera_cpp
{

/// Read-only view of a planar YUV 4:2:0 (I420) frame: full-size Y plane, half-size U and V planes.
struct Yuv420View
{
  const uint8_t * y;
  const uint8_t * u;
  const uint8_t * v;
  int y_stride;   ///< Bytes per Y row.
  int uv_stride;  ///< Bytes per U or V row.
  Size size;      ///< Luma size in pixels (even width and height).
};

/**
 * JPEG of a packed BGR888 frame (libcamera RGB888 is B, G, R in memory).
 *
 * @param data First byte of the frame.
 * @param width Width in pixels, > 0.
 * @param height Height in pixels, > 0.
 * @param stride Bytes per row, >= 3 * width.
 * @param quality JPEG quality in 1..100.
 * @return The encoded JPEG bytes.
 * @throws std::invalid_argument for null data, a non-positive size, stride < 3 * width or quality outside 1..100.
 */
std::vector<uint8_t> encode_bgr_jpeg(const uint8_t * data, int width, int height, int stride, int quality);

/**
 * JPEG of a region of a YUV 4:2:0 frame. Only the region is converted to BGR, never the whole frame; it is resized
 * with area interpolation when `out` is smaller than the region.
 *
 * @param frame The source frame.
 * @param region Crop region in luma pixels: even x, y, width and height, inside the frame.
 * @param out Output size in pixels, positive and not larger than the region (no upscaling).
 * @param quality JPEG quality in 1..100.
 * @return The encoded JPEG bytes.
 * @throws std::invalid_argument for a bad frame, a region outside the frame or with odd values, an `out` larger
 *   than the region or non-positive, or a quality outside 1..100.
 */
std::vector<uint8_t> crop_yuv420_jpeg(const Yuv420View & frame, Rect region, Size out, int quality);

}  // namespace head_camera_cpp
