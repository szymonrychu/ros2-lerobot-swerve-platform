// Lidar overlay kernel (see lidar.hpp): the per-point loop of camera_overlay.draw_lidar (Scene.base_to_ref,
// project_points = cv2.projectPoints with zero rvec / tvec, the in-image test, ipt and range_color's colormap index).
#include "camera/lidar.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace mcp_core::camera
{
namespace
{

using Mat4 = std::array<double, 16>;

constexpr size_t OPENCV_DIST_MAX = 14;
constexpr double COLORMAP_TOP = 255.0;

// Inverse of a 4x4 row-major matrix (Gauss-Jordan, partial pivoting), as np.linalg.inv.
Mat4 inverse(const double * m)
{
  std::array<std::array<double, 8>, 4> a{};
  for (size_t r = 0; r < 4; ++r) {
    for (size_t c = 0; c < 4; ++c) {
      a[r][c] = m[r * 4 + c];
    }
    a[r][4 + r] = 1.0;
  }
  for (size_t col = 0; col < 4; ++col) {
    size_t pivot = col;
    for (size_t r = col + 1; r < 4; ++r) {
      if (std::fabs(a[r][col]) > std::fabs(a[pivot][col])) {
        pivot = r;
      }
    }
    if (a[pivot][col] == 0.0) {
      throw std::invalid_argument("Singular matrix");
    }
    std::swap(a[col], a[pivot]);
    const double inv_pivot = 1.0 / a[col][col];
    for (double & v : a[col]) {
      v *= inv_pivot;
    }
    for (size_t r = 0; r < 4; ++r) {
      if (r == col || a[r][col] == 0.0) {
        continue;
      }
      const double f = a[r][col];
      for (size_t c = 0; c < 8; ++c) {
        a[r][c] -= f * a[col][c];
      }
    }
  }
  Mat4 out{};
  for (size_t r = 0; r < 4; ++r) {
    for (size_t c = 0; c < 4; ++c) {
      out[r * 4 + c] = a[r][4 + c];
    }
  }
  return out;
}

// First three rows of t @ (p, 1).
std::array<double, 3> transform(const Mat4 & t, const std::array<double, 3> & p)
{
  std::array<double, 3> out{};
  for (size_t r = 0; r < 3; ++r) {
    out[r] = t[r * 4] * p[0] + t[r * 4 + 1] * p[1] + t[r * 4 + 2] * p[2] + t[r * 4 + 3];
  }
  return out;
}

// OpenCV's 14-slot distortion vector (k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4 taux tauy) of the given coefficients.
std::array<double, OPENCV_DIST_MAX> opencv_distortion(const std::vector<double> & distortion)
{
  const size_t n = distortion.size();
  if (n != 0 && n != 4 && n != 5 && n != 8 && n != 12 && n != 14) {
    throw std::invalid_argument("distortion must have 0, 4, 5, 8, 12 or 14 coefficients (cv2.projectPoints)");
  }
  std::array<double, OPENCV_DIST_MAX> k{};
  for (size_t i = 0; i < n; ++i) {
    k[i] = distortion[i];
  }
  if (k[12] != 0.0 || k[13] != 0.0) {
    throw std::invalid_argument("tilted sensor distortion (taux, tauy) is not supported");
  }
  return k;
}

// Python's round() of a float to an int (half to even), then int().
int64_t round_half_even(double v)
{
  return static_cast<int64_t>(std::nearbyint(v));
}

// range_color's colormap index: round(255 * (1 - min(max(range / max_range, 0), 1))) with Python's max / min
// semantics (max(nan, 0.0) is nan, which round() rejects).
uint8_t colormap_index(double range_m, double max_range_m)
{
  double v = range_m / max_range_m;
  if (0.0 > v) {
    v = 0.0;
  }
  if (1.0 < v) {
    v = 1.0;
  }
  if (std::isnan(v)) {
    throw std::domain_error("cannot convert float NaN to integer");
  }
  return static_cast<uint8_t>(round_half_even(COLORMAP_TOP * (1.0 - v)));
}

}  // namespace

LidarDots lidar_dots(
  const double * points, size_t n, const double * t_base_ref, const double * t_ref_optical,
  const double * camera_matrix, const std::vector<double> & distortion, int64_t width, int64_t height,
  double max_range_m)
{
  const std::array<double, OPENCV_DIST_MAX> k = opencv_distortion(distortion);
  const Mat4 ref_from_base = inverse(t_base_ref);
  const Mat4 optical_from_ref = inverse(t_ref_optical);
  const double fx = camera_matrix[0], cx = camera_matrix[2], fy = camera_matrix[4], cy = camera_matrix[5];
  const double w = static_cast<double>(width), h = static_cast<double>(height);
  LidarDots dots;
  for (size_t i = 0; i < n; ++i) {
    const double * p = points + i * 4;
    const std::array<double, 3> cam = transform(optical_from_ref, transform(ref_from_base, {p[0], p[1], p[2]}));
    const double depth = cam[2];
    // project_points: depth replaced by 1 for points behind the camera (they are dropped below anyway).
    const double z_safe = depth > MIN_DEPTH ? depth : 1.0;
    // cv2.projectPoints (cvProjectPoints2Internal) with identity rotation, zero translation and no tilt.
    const double iz = z_safe != 0.0 ? 1.0 / z_safe : 1.0;
    const double x = cam[0] * iz, y = cam[1] * iz;
    const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    const double a1 = 2 * x * y, a2 = r2 + 2 * x * x, a3 = r2 + 2 * y * y;
    const double cdist = 1 + k[0] * r2 + k[1] * r4 + k[4] * r6;
    const double icdist2 = 1. / (1 + k[5] * r2 + k[6] * r4 + k[7] * r6);
    const double xd = x * cdist * icdist2 + k[2] * a1 + k[3] * a2 + k[8] * r2 + k[9] * r4;
    const double yd = y * cdist * icdist2 + k[2] * a3 + k[3] * a1 + k[10] * r2 + k[11] * r4;
    const double u = xd * fx + cx;
    const double v = yd * fy + cy;
    if (depth > MIN_DEPTH && 0 <= u && u < w && 0 <= v && v < h) {
      dots.values.push_back(colormap_index(p[3], max_range_m));
      dots.centres.push_back(static_cast<int32_t>(round_half_even(u)));
      dots.centres.push_back(static_cast<int32_t>(round_half_even(v)));
    }
  }
  return dots;
}

}  // namespace mcp_core::camera
