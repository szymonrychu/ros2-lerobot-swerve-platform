#include "ground_estimator_cpp/guidance.hpp"

#include <cmath>
#include <cstring>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace ground_estimator_cpp
{

namespace
{

constexpr const char * PLUMB_BOB = "plumb_bob";
constexpr const char * DEPTH_ENCODING = "16UC1";
constexpr std::size_t D_SIZE = 5;
constexpr double MM_PER_M = 1000.0;
constexpr double MIN_DENOM = 1e-9;
constexpr int UNDISTORT_ITERATIONS = 100;
constexpr double UNDISTORT_EPS = 1e-14;

cv::Vec3b rgb_of(const cv::Mat & bgr, int u, int v)
{
  const cv::Vec3b p = bgr.at<cv::Vec3b>(v, u);
  return cv::Vec3b(p[2], p[1], p[0]);
}

pcl::PointXYZ to_pcl(const Eigen::Vector3d & p)
{
  return pcl::PointXYZ(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
}

// The pixel of a point in the depth camera optical frame, plumb_bob distortion applied.
cv::Point2d project(const CameraModel & m, const Eigen::Vector3d & p)
{
  const double x = p.x() / p.z();
  const double y = p.y() / p.z();
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (m.D[0] + r2 * (m.D[1] + r2 * m.D[4]));
  const double xd = x * radial + 2.0 * m.D[2] * x * y + m.D[3] * (r2 + 2.0 * x * x);
  const double yd = y * radial + m.D[2] * (r2 + 2.0 * y * y) + 2.0 * m.D[3] * x * y;
  return cv::Point2d(m.K(0, 0) * xd + m.K(0, 2), m.K(1, 1) * yd + m.K(1, 2));
}

}  // namespace

bool CameraModel::operator==(const CameraModel & o) const
{
  return width == o.width && height == o.height && cv::norm(K - o.K, cv::NORM_INF) == 0.0 &&
         cv::norm(D - o.D, cv::NORM_INF) == 0.0;
}

std::optional<CameraModel> camera_model_of(const sensor_msgs::msg::CameraInfo & info)
{
  if (info.distortion_model != PLUMB_BOB || info.width == 0 || info.height == 0) {
    return std::nullopt;
  }
  if (info.d.size() != 0 && info.d.size() != D_SIZE) {
    return std::nullopt;
  }
  bool all_zero = true;
  for (const double k : info.k) {
    all_zero = all_zero && k == 0.0;
  }
  if (all_zero || info.k[0] <= 0.0 || info.k[4] <= 0.0) {
    return std::nullopt;
  }
  CameraModel m;
  for (int i = 0; i < 9; ++i) {
    m.K(i / 3, i % 3) = info.k[static_cast<std::size_t>(i)];
  }
  for (std::size_t i = 0; i < info.d.size(); ++i) {
    m.D[static_cast<int>(i)] = info.d[i];
  }
  m.width = static_cast<int>(info.width);
  m.height = static_cast<int>(info.height);
  return m;
}

RayGrid::RayGrid(const CameraModel & model, int step_px) : model_(model), step_(step_px)
{
  if (model.width <= 0 || model.height <= 0 || step_px < 1) {
    throw std::invalid_argument("RayGrid: non-positive size or step");
  }
  cols_ = (model.width + step_px - 1) / step_px;
  rows_ = (model.height + step_px - 1) / step_px;
  std::vector<cv::Point2d> pixels;
  pixels.reserve(static_cast<std::size_t>(cols_) * rows_);
  for (int j = 0; j < rows_; ++j) {
    for (int i = 0; i < cols_; ++i) {
      pixels.emplace_back(static_cast<double>(i * step_px), static_cast<double>(j * step_px));
    }
  }
  std::vector<cv::Point2d> normalised;
  const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, UNDISTORT_ITERATIONS, UNDISTORT_EPS);
  cv::undistortPoints(pixels, normalised, cv::Mat(model.K), cv::Mat(model.D), cv::noArray(), cv::noArray(), criteria);
  rays_.reserve(normalised.size());
  for (const auto & p : normalised) {
    rays_.emplace_back(p.x, p.y);
  }
}

bool RayGrid::matches(const CameraModel & model, int step_px) const
{
  return !rays_.empty() && step_ == step_px && model_ == model;
}

bool RayGrid::ensure(const CameraModel & model, int step_px)
{
  if (matches(model, step_px)) {
    return false;
  }
  *this = RayGrid(model, step_px);
  return true;
}

std::optional<GuidanceFrame> decode_guidance(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & info,
  const sensor_msgs::msg::CompressedImage & rgb, std::string * why)
{
  const auto reject = [why](const std::string & reason) -> std::optional<GuidanceFrame> {
    if (why != nullptr) {
      *why = reason;
    }
    return std::nullopt;
  };
  if (depth.encoding != DEPTH_ENCODING) {
    return reject("depth encoding is '" + depth.encoding + "', not 16UC1");
  }
  if (depth.width == 0 || depth.height == 0) {
    return reject("depth has a zero size");
  }
  const std::size_t row_bytes = static_cast<std::size_t>(depth.width) * sizeof(uint16_t);
  if (depth.step < row_bytes || depth.data.size() < static_cast<std::size_t>(depth.step) * depth.height) {
    return reject("depth data is shorter than its size");
  }
  if (info.width != depth.width || info.height != depth.height) {
    return reject("camera_info size differs from the depth size");
  }
  const auto model = camera_model_of(info);
  if (!model) {
    return reject("camera_info has no usable plumb_bob calibration");
  }
  if (rgb.data.empty()) {
    return reject("JPEG is empty");
  }
  const cv::Mat encoded(1, static_cast<int>(rgb.data.size()), CV_8UC1, const_cast<uint8_t *>(rgb.data.data()));
  const cv::Mat bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return reject("JPEG does not decode");
  }
  if (bgr.cols != static_cast<int>(depth.width) || bgr.rows != static_cast<int>(depth.height)) {
    return reject("JPEG size differs from the depth size");
  }
  GuidanceFrame frame;
  frame.stamp_ns = static_cast<int64_t>(depth.header.stamp.sec) * 1000000000LL + depth.header.stamp.nanosec;
  frame.frame_id = depth.header.frame_id;
  frame.depth_mm = cv::Mat(static_cast<int>(depth.height), static_cast<int>(depth.width), CV_16UC1);
  for (int v = 0; v < frame.depth_mm.rows; ++v) {
    std::memcpy(frame.depth_mm.ptr(v), depth.data.data() + static_cast<std::size_t>(v) * depth.step, row_bytes);
  }
  frame.bgr = bgr;
  frame.model = *model;
  return frame;
}

GuidanceSamples sample_guidance(
  const GuidanceFrame & frame, const RayGrid & grid, const Eigen::Isometry3d & base_T_cam, const Plane & plane,
  const CameraModel & tof, const Eigen::Isometry3d & tof_T_base, const GuidanceConfig & config)
{
  GuidanceSamples out;
  const bool consistent = frame.depth_mm.type() == CV_16UC1 && frame.bgr.type() == CV_8UC3 &&
                          frame.depth_mm.size() == frame.bgr.size() && frame.model.width == frame.depth_mm.cols &&
                          frame.model.height == frame.depth_mm.rows && grid.matches(frame.model, grid.step());
  if (!consistent) {
    return out;
  }
  const double max_range = config.max_range_m;
  const double min_range = config.no_return.min_range_m;
  const double margin = static_cast<double>(config.no_return.margin_px);
  const Eigen::Vector3d origin = base_T_cam.translation();
  const Eigen::Matrix3d rot = base_T_cam.linear();
  for (int j = 0; j < grid.rows(); ++j) {
    for (int i = 0; i < grid.cols(); ++i) {
      const int u = i * grid.step();
      const int v = j * grid.step();
      const cv::Vec2d & r = grid.ray(i, j);
      const Eigen::Vector3d ray(r[0], r[1], 1.0);
      const uint16_t mm = frame.depth_mm.at<uint16_t>(v, u);
      if (mm > 0) {
        const double z = mm / MM_PER_M;
        if ((ray * z).norm() > max_range) {
          continue;
        }
        const Eigen::Vector3d p = base_T_cam * (ray * z);
        out.points.push_back(to_pcl(p));
        out.heights.push_back(static_cast<float>(height_above(plane, p)));
        out.colours.push_back(rgb_of(frame.bgr, u, v));
        continue;
      }
      const Eigen::Vector3d dir = rot * ray;
      const double denom = plane.normal.dot(dir);
      if (std::fabs(denom) < MIN_DENOM) {
        continue;
      }
      const double t = -(plane.normal.dot(origin) + plane.d) / denom;
      if (t <= 0.0) {
        continue;
      }
      const Eigen::Vector3d hit = origin + t * dir;
      const double horizontal = std::hypot(hit.x() - origin.x(), hit.y() - origin.y());
      if (horizontal < min_range || horizontal > max_range) {
        continue;
      }
      const Eigen::Vector3d in_tof = tof_T_base * hit;
      if (in_tof.z() <= 0.0) {
        continue;
      }
      const cv::Point2d px = project(tof, in_tof);
      if (px.x < margin || px.y < margin || px.x > tof.width - 1 - margin || px.y > tof.height - 1 - margin) {
        continue;
      }
      out.no_return.push_back(to_pcl(hit));
      out.no_return_colours.push_back(rgb_of(frame.bgr, u, v));
    }
  }
  return out;
}

}  // namespace ground_estimator_cpp
