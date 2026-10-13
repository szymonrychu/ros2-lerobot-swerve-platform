#include "stereo_depth_cpp/depth.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace stereo_depth_cpp
{

cv::Mat disparity_to_depth_mm(
  const cv::Mat & disparity, double f, double t, double min_disparity, double min_depth_m, double max_depth_m)
{
  cv::Mat d;
  disparity.convertTo(d, CV_64F);
  cv::Mat depth(d.size(), CV_16UC1);
  const double ft = f * t;
  for (int y = 0; y < d.rows; ++y) {
    const double * in = d.ptr<double>(y);
    std::uint16_t * out = depth.ptr<std::uint16_t>(y);
    for (int x = 0; x < d.cols; ++x) {
      const double v = in[x];
      std::uint16_t mm = NO_READING;
      if (std::isfinite(v) && v > 0.0 && v >= min_disparity) {
        const double depth_m = ft / v;
        if (depth_m >= min_depth_m && depth_m <= max_depth_m) {
          // numpy rint: round half to even (the default floating-point rounding mode).
          mm = static_cast<std::uint16_t>(std::clamp(std::nearbyint(depth_m * MM_PER_M), 0.0, UINT16_MAX_MM));
        }
      }
      out[x] = mm;
    }
  }
  return depth;
}

cv::Ptr<cv::StereoSGBM> create_matcher(const DepthConfig & config)
{
  return cv::StereoSGBM::create(
    MIN_DISPARITY, config.num_disparities, config.block_size, config.p1, config.p2, config.disp12_max_diff,
    config.pre_filter_cap, config.uniqueness_ratio, config.speckle_window_size, config.speckle_range,
    cv::StereoSGBM::MODE_SGBM_3WAY);
}

cv::Mat compute_disparity(const cv::Ptr<cv::StereoSGBM> & matcher, const cv::Mat & left, const cv::Mat & right)
{
  cv::Mat fixed_point;
  matcher->compute(left, right, fixed_point);
  cv::Mat disparity;
  fixed_point.convertTo(disparity, CV_32F, 1.0 / DISPARITY_SCALE);
  return disparity;
}

}  // namespace stereo_depth_cpp
