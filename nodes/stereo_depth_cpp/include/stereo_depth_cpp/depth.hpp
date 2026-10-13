// Stereo matching (OpenCV StereoSGBM, 3-way) and the disparity to depth (mm) conversion: the C++ mirror of
// nodes/stereo_depth/stereo_depth/depth.py. Depth is uint16 millimetres with 0 = no reading (REP 118).
#ifndef STEREO_DEPTH_CPP__DEPTH_HPP_
#define STEREO_DEPTH_CPP__DEPTH_HPP_

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "stereo_depth_cpp/config.hpp"

namespace stereo_depth_cpp
{

constexpr double MM_PER_M = 1000.0;
constexpr double DISPARITY_SCALE = 16.0;  // StereoSGBM returns fixed-point disparity with 4 fractional bits
constexpr int MIN_DISPARITY = 0;
constexpr double UINT16_MAX_MM = 65535.0;
constexpr int NO_READING = 0;

// Convert a CV_32F (or CV_64F) disparity image in pixels to CV_16U depth in mm, Z = f * T / d. Pixels with a
// non-finite disparity, d <= 0, d < min_disparity or a depth outside [min_depth_m, max_depth_m] become 0; depth is
// rounded half to even (numpy rint) and clamped to the uint16 range.
cv::Mat disparity_to_depth_mm(
  const cv::Mat & disparity, double f, double t, double min_disparity, double min_depth_m, double max_depth_m);

// The StereoSGBM matcher (MODE_SGBM_3WAY, minDisparity 0) with the config's parameters, reused for every pair.
cv::Ptr<cv::StereoSGBM> create_matcher(const DepthConfig & config);

// Match a rectified CV_8U grey pair; CV_32F disparity in pixels (fixed point / 16), unmatched pixels negative.
cv::Mat compute_disparity(const cv::Ptr<cv::StereoSGBM> & matcher, const cv::Mat & left, const cv::Mat & right);

}  // namespace stereo_depth_cpp

#endif  // STEREO_DEPTH_CPP__DEPTH_HPP_
