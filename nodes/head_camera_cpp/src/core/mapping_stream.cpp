#include "head_camera_cpp/mapping_stream.hpp"

#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

namespace head_camera_cpp
{

namespace
{

constexpr double NS_PER_S = 1e9;
constexpr double ASPECT_TOLERANCE_PX = 1.0;

}  // namespace

MappingDecimator::MappingDecimator(double rate_hz) : period_ns_(static_cast<int64_t>(std::llround(NS_PER_S / rate_hz)))
{
  if (!(rate_hz > 0.0)) {
    throw std::invalid_argument("rate_hz must be > 0");
  }
}

bool MappingDecimator::due(int64_t stamp_ns)
{
  if (last_ns_ && stamp_ns >= *last_ns_ && stamp_ns - *last_ns_ < period_ns_) {
    return false;
  }
  last_ns_ = stamp_ns;
  return true;
}

cv::Mat resize_for_mapping(const cv::Mat & bgr, int width, int height)
{
  cv::Mat out;
  cv::resize(bgr, out, cv::Size(width, height), 0, 0, cv::INTER_AREA);
  return out;
}

CameraIntrinsics scale_intrinsics(const CameraIntrinsics & calibrated, int width, int height)
{
  if (width <= 0 || height <= 0 || calibrated.width <= 0 || calibrated.height <= 0) {
    throw std::invalid_argument("sizes must be positive");
  }
  const double expected = static_cast<double>(width) * calibrated.height / calibrated.width;
  if (std::abs(expected - height) > ASPECT_TOLERANCE_PX) {
    throw std::invalid_argument(
      "aspect of " + std::to_string(width) + "x" + std::to_string(height) + " differs from the calibration at " +
      std::to_string(calibrated.width) + "x" + std::to_string(calibrated.height));
  }
  const double sx = static_cast<double>(width) / calibrated.width;
  const double sy = static_cast<double>(height) / calibrated.height;
  CameraIntrinsics out = calibrated;
  out.width = width;
  out.height = height;
  for (const size_t i : {0, 2}) {
    out.k[i] *= sx;
  }
  for (const size_t i : {4, 5}) {
    out.k[i] *= sy;
  }
  for (const size_t i : {0, 2, 3}) {
    out.p[i] *= sx;
  }
  for (const size_t i : {5, 6, 7}) {
    out.p[i] *= sy;
  }
  return out;
}

}  // namespace head_camera_cpp
