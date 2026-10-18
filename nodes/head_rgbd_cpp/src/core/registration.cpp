#include "head_rgbd_cpp/registration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace head_rgbd_cpp
{

// The fold radius search: coarse steps up to FOLD_SEARCH_MAX_RADIUS, then bisection of the first sign change.
constexpr double FOLD_SEARCH_MAX_RADIUS = 10.0;
constexpr int FOLD_SEARCH_STEPS = 1000;
constexpr int FOLD_BISECTION_STEPS = 60;
// cv::undistortPoints iterations: enough for the round trip to be exact to well below a micro-pixel.
constexpr int UNDISTORT_MAX_ITERATIONS = 100;
constexpr double UNDISTORT_EPS = 1e-14;
constexpr double MM_PER_M = 1000.0;

PinholeModel scale_model(const PinholeModel & m, int width, int height)
{
  if (m.width <= 0 || m.height <= 0 || width <= 0 || height <= 0) {
    throw std::invalid_argument(
      "scale_model: sizes must be positive, got " + std::to_string(m.width) + "x" + std::to_string(m.height) + " -> " +
      std::to_string(width) + "x" + std::to_string(height));
  }
  const double sx = static_cast<double>(width) / m.width;
  const double sy = static_cast<double>(height) / m.height;
  // The smallest change of one side, in pixels, that would keep the calibrated aspect ratio.
  const double aspect_error_px = std::min(std::abs(height - m.height * sx), std::abs(width - m.width * sy));
  if (aspect_error_px > MAX_SCALE_ASPECT_ERROR_PX) {
    throw std::invalid_argument(
      "scale_model: " + std::to_string(m.width) + "x" + std::to_string(m.height) + " -> " + std::to_string(width) +
      "x" + std::to_string(height) + " changes the aspect ratio by " + std::to_string(aspect_error_px) + " px");
  }
  PinholeModel s = m;
  s.K(0, 0) = m.K(0, 0) * sx;
  s.K(0, 1) = m.K(0, 1) * sx;
  s.K(0, 2) = (m.K(0, 2) + 0.5) * sx - 0.5;
  s.K(1, 1) = m.K(1, 1) * sy;
  s.K(1, 2) = (m.K(1, 2) + 0.5) * sy - 0.5;
  s.width = width;
  s.height = height;
  return s;
}

cv::Point2d project_normalised(const PinholeModel & m, double x, double y)
{
  const double k1 = m.D[0];
  const double k2 = m.D[1];
  const double p1 = m.D[2];
  const double p2 = m.D[3];
  const double k3 = m.D[4];
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
  const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
  const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
  return {m.K(0, 0) * xd + m.K(0, 2), m.K(1, 1) * yd + m.K(1, 2)};
}

cv::Point2d project_point(const PinholeModel & m, const Eigen::Vector3d & p)
{
  return project_normalised(m, p.x() / p.z(), p.y() / p.z());
}

double fold_radius_squared(const PinholeModel & m)
{
  const double k1 = m.D[0];
  const double k2 = m.D[1];
  const double k3 = m.D[4];
  // d/dr [r (1 + k1 r^2 + k2 r^4 + k3 r^6)] as a function of r^2.
  const auto slope = [k1, k2, k3](double r2) { return 1.0 + r2 * (3.0 * k1 + r2 * (5.0 * k2 + r2 * 7.0 * k3)); };
  double previous = 0.0;
  for (int i = 1; i <= FOLD_SEARCH_STEPS; ++i) {
    const double r = FOLD_SEARCH_MAX_RADIUS * i / FOLD_SEARCH_STEPS;
    if (slope(r * r) > 0.0) {
      previous = r;
      continue;
    }
    double lo = previous;
    double hi = r;
    for (int j = 0; j < FOLD_BISECTION_STEPS; ++j) {
      const double mid = 0.5 * (lo + hi);
      (slope(mid * mid) > 0.0 ? lo : hi) = mid;
    }
    return lo * lo;
  }
  return std::numeric_limits<double>::infinity();
}

RayTable::RayTable(const PinholeModel & m) : model_(m)
{
  if (m.width <= 0 || m.height <= 0) {
    throw std::invalid_argument(
      "RayTable: the model size must be positive, got " + std::to_string(m.width) + "x" + std::to_string(m.height));
  }
  // Every pixel centre, then every pixel corner, undistorted in one call.
  std::vector<cv::Point2d> pixels;
  pixels.reserve(static_cast<size_t>(m.width * m.height + (m.width + 1) * (m.height + 1)));
  for (int v = 0; v < m.height; ++v) {
    for (int u = 0; u < m.width; ++u) {
      pixels.emplace_back(u, v);
    }
  }
  for (int y = 0; y <= m.height; ++y) {
    for (int x = 0; x <= m.width; ++x) {
      pixels.emplace_back(x - 0.5, y - 0.5);
    }
  }
  std::vector<cv::Point2d> rays;
  cv::undistortPoints(
    pixels, rays, cv::Mat(m.K), cv::Mat(m.D), cv::noArray(), cv::noArray(),
    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, UNDISTORT_MAX_ITERATIONS, UNDISTORT_EPS));
  centres_.create(m.height, m.width);
  corners_.create(m.height + 1, m.width + 1);
  size_t i = 0;
  for (int v = 0; v < m.height; ++v) {
    for (int u = 0; u < m.width; ++u, ++i) {
      centres_(v, u) = cv::Vec2d(rays[i].x, rays[i].y);
    }
  }
  for (int y = 0; y <= m.height; ++y) {
    for (int x = 0; x <= m.width; ++x, ++i) {
      corners_(y, x) = cv::Vec2d(rays[i].x, rays[i].y);
    }
  }
}

cv::Rect clip_footprint(double u_min, double v_min, double u_max, double v_max, int width, int height)
{
  // NaN bounds fail these comparisons too.
  if (width <= 0 || height <= 0 || !(u_min <= u_max) || !(v_min <= v_max)) {
    return {};
  }
  // Clamp in double before converting, so bounds far outside the image cannot overflow an int.
  const double x0 = std::max(0.0, std::ceil(u_min));
  const double x1 = std::min(width - 1.0, std::floor(u_max));
  const double y0 = std::max(0.0, std::ceil(v_min));
  const double y1 = std::min(height - 1.0, std::floor(v_max));
  if (x0 > x1 || y0 > y1) {
    return {};
  }
  const auto left = static_cast<int>(x0);
  const auto top = static_cast<int>(y0);
  return {left, top, static_cast<int>(x1) - left + 1, static_cast<int>(y1) - top + 1};
}

void register_depth_into(
  const cv::Mat & tof_depth_mm, const RayTable & rays, const Eigen::Isometry3d & T_rgb_tof,
  const PinholeModel & rgb_out, const DepthConfig & range, cv::Mat & out, RegistrationStats * stats)
{
  if (tof_depth_mm.type() != CV_16UC1 || tof_depth_mm.cols != rays.width() || tof_depth_mm.rows != rays.height()) {
    throw std::invalid_argument(
      "register_depth: the ToF depth must be CV_16UC1 " + std::to_string(rays.width()) + "x" +
      std::to_string(rays.height()) + ", got type " + std::to_string(tof_depth_mm.type()) + " " +
      std::to_string(tof_depth_mm.cols) + "x" + std::to_string(tof_depth_mm.rows));
  }
  if (out.type() != CV_16UC1 || out.cols != rgb_out.width || out.rows != rgb_out.height) {
    throw std::invalid_argument(
      "register_depth: the output must be CV_16UC1 " + std::to_string(rgb_out.width) + "x" +
      std::to_string(rgb_out.height) + ", got type " + std::to_string(out.type()) + " " + std::to_string(out.cols) +
      "x" + std::to_string(out.rows));
  }
  out.setTo(cv::Scalar(0));
  RegistrationStats counts;
  const double fold_r2 = fold_radius_squared(rgb_out);
  const Eigen::Matrix3d R = T_rgb_tof.linear();
  const Eigen::Vector3d t = T_rgb_tof.translation();
  // Corner offsets of a pixel in the corner grid.
  const int corner_du[4] = {0, 1, 0, 1};
  const int corner_dv[4] = {0, 0, 1, 1};

  for (int v = 0; v < tof_depth_mm.rows; ++v) {
    const auto * depth_row = tof_depth_mm.ptr<uint16_t>(v);
    for (int u = 0; u < tof_depth_mm.cols; ++u) {
      if (depth_row[u] == 0) {
        continue;
      }
      const double z = depth_row[u] / MM_PER_M;
      if (z < range.min_m || z > range.max_m) {
        continue;
      }
      ++counts.valid_points;
      const cv::Vec2d & ray = rays.ray(u, v);
      const Eigen::Vector3d point = R * Eigen::Vector3d(ray[0] * z, ray[1] * z, z) + t;
      if (point.z() <= 0.0) {
        ++counts.behind_points;
        continue;
      }
      // The footprint: the bounding rectangle of the pixel's four corners at the same depth, projected.
      double u_min = std::numeric_limits<double>::infinity();
      double v_min = std::numeric_limits<double>::infinity();
      double u_max = -std::numeric_limits<double>::infinity();
      double v_max = -std::numeric_limits<double>::infinity();
      bool behind = false;
      bool folded = false;
      for (int k = 0; k < 4; ++k) {
        const cv::Vec2d & c = rays.corner(u + corner_du[k], v + corner_dv[k]);
        const Eigen::Vector3d corner = R * Eigen::Vector3d(c[0] * z, c[1] * z, z) + t;
        if (corner.z() <= 0.0) {
          behind = true;
          break;
        }
        const double xn = corner.x() / corner.z();
        const double yn = corner.y() / corner.z();
        if (xn * xn + yn * yn >= fold_r2) {
          folded = true;
          break;
        }
        const cv::Point2d px = project_normalised(rgb_out, xn, yn);
        u_min = std::min(u_min, px.x);
        v_min = std::min(v_min, px.y);
        u_max = std::max(u_max, px.x);
        v_max = std::max(v_max, px.y);
      }
      if (behind) {
        ++counts.behind_points;
        continue;
      }
      if (folded) {
        ++counts.folded_points;
        continue;
      }
      const cv::Rect footprint = clip_footprint(u_min, v_min, u_max, v_max, rgb_out.width, rgb_out.height);
      if (footprint.empty()) {
        ++counts.outside_points;
        continue;
      }
      // z in the RGB frame, mm; at least 1 so a point never reads as "no depth".
      const auto depth = static_cast<uint16_t>(std::clamp(std::round(point.z() * MM_PER_M), 1.0, MAX_DEPTH_MM));
      for (int row = footprint.y; row < footprint.y + footprint.height; ++row) {
        auto * out_row = out.ptr<uint16_t>(row);
        for (int col = footprint.x; col < footprint.x + footprint.width; ++col) {
          if (out_row[col] == 0 || depth < out_row[col]) {
            out_row[col] = depth;
          }
        }
      }
      ++counts.splatted_points;
    }
  }
  counts.filled_pixels = static_cast<std::size_t>(cv::countNonZero(out));
  if (stats != nullptr) {
    *stats = counts;
  }
}

cv::Mat register_depth(
  const cv::Mat & tof_depth_mm, const RayTable & rays, const Eigen::Isometry3d & T_rgb_tof,
  const PinholeModel & rgb_out, const DepthConfig & range, RegistrationStats * stats)
{
  cv::Mat out(std::max(rgb_out.height, 0), std::max(rgb_out.width, 0), CV_16UC1, cv::Scalar(0));
  register_depth_into(tof_depth_mm, rays, T_rgb_tof, rgb_out, range, out, stats);
  return out;
}

}  // namespace head_rgbd_cpp
