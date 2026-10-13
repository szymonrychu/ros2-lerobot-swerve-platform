// Lidar overlay kernel of camera_overlay.draw_lidar: which lidar returns land in the image, where, and in what colour.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mcp_core::camera
{

// camera_overlay.MIN_DEPTH: a point at or behind this optical depth (m) is not drawn.
constexpr double MIN_DEPTH = 1e-9;

struct LidarDots
{
  std::vector<int32_t> centres;  // (M, 2) rounded pixel centres (u, v), round half to even as Python's round
  std::vector<uint8_t> values;   // (M,) JET colormap index of each dot's range (255 near .. 0 far)
};

// Projects (N, 4) rows (x, y, z, range) in base_link through t_base_ref (4x4, reference frame in base_link) and
// t_ref_optical (4x4, optical frame in the reference frame) with the OpenCV camera model (camera_matrix 3x3 row-major,
// distortion of 0, 4, 5, 8, 12 or 14 coefficients, the tilt ones zero) and keeps, in input order, the points in front
// of the camera and inside width x height. Throws std::invalid_argument for an unsupported distortion or a singular
// transform, std::domain_error for a NaN range of a kept point (Python's round(nan) fails the same way).
LidarDots lidar_dots(
  const double * points, size_t n, const double * t_base_ref, const double * t_ref_optical,
  const double * camera_matrix, const std::vector<double> & distortion, int64_t width, int64_t height,
  double max_range_m);

}  // namespace mcp_core::camera
