#include "ros2_common_cpp/depth_mask.hpp"

#include <limits>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

namespace ros2_common_cpp
{

namespace
{
constexpr double M_PER_MM = 0.001;
constexpr int MIN_WINDOW_PX = 3;
}  // namespace

void check_depth_edge_mask_params(const DepthEdgeMaskParams & params)
{
  if (params.window_px < MIN_WINDOW_PX || params.window_px % 2 == 0) {
    throw std::invalid_argument("window_px must be odd and >= 3, got " + std::to_string(params.window_px));
  }
  if (params.abs_m < 0.0 || params.rel < 0.0) {
    throw std::invalid_argument("abs_m and rel must be >= 0");
  }
  if (params.abs_m == 0.0 && params.rel == 0.0) {
    throw std::invalid_argument("abs_m or rel must be > 0: a zero threshold removes every pixel behind any other");
  }
  if (params.grow_px < 0) {
    throw std::invalid_argument("grow_px must be >= 0, got " + std::to_string(params.grow_px));
  }
}

cv::Mat depth_edge_mask(const cv::Mat & depth_mm, const DepthEdgeMaskParams & params)
{
  if (depth_mm.type() != CV_16UC1) {
    throw std::invalid_argument("depth_edge_mask needs a CV_16UC1 depth in millimetres");
  }
  check_depth_edge_mask_params(params);

  cv::Mat depth_m;
  depth_mm.convertTo(depth_m, CV_32F, M_PER_MM);
  const cv::Mat missing = depth_mm == 0;
  // Missing depth is never the nearer neighbour: it takes +max before the window minimum.
  cv::Mat for_min = depth_m.clone();
  for_min.setTo(std::numeric_limits<float>::max(), missing);
  cv::Mat nearest;
  const cv::Mat window = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.window_px, params.window_px));
  cv::erode(for_min, nearest, window, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);

  // Per-pixel threshold max(abs_m, rel x z) and the depth behind the nearest neighbour.
  cv::Mat threshold = depth_m * params.rel;
  threshold = cv::max(threshold, params.abs_m);
  const cv::Mat behind = depth_m - nearest;
  cv::Mat mask = behind > threshold;
  mask.setTo(0, missing);

  if (params.grow_px > 0) {
    const int side = 2 * params.grow_px + 1;
    cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(side, side)));
    mask.setTo(0, missing);
  }
  return mask;
}

int apply_depth_edge_mask(cv::Mat & depth_mm, const DepthEdgeMaskParams & params)
{
  const cv::Mat mask = depth_edge_mask(depth_mm, params);
  depth_mm.setTo(0, mask);
  return cv::countNonZero(mask);
}

}  // namespace ros2_common_cpp
