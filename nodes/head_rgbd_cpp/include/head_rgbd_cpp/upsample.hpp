// Optional masked guided-filter upsampling of the registered depth to the RGB mapping size (design D8): the depth is
// nearest-upsampled by 2, then smoothed along the edges of the grey RGB guide in normalised-convolution form
// D = GF(I, D0 * M) / GF(I, M), so holes never bleed zeros into their neighbours and no depth is invented across them.
#ifndef HEAD_RGBD_CPP__UPSAMPLE_HPP_
#define HEAD_RGBD_CPP__UPSAMPLE_HPP_

#include <opencv2/core.hpp>

#include "head_rgbd_cpp/config.hpp"

namespace head_rgbd_cpp
{

// Divisor between the guide size and the depth size.
constexpr int UPSAMPLE_FACTOR = 2;

// What one upsampling call did.
struct UpsampleStats
{
  int valid_in = 0;           // valid pixels of the input depth
  int valid_out = 0;          // valid pixels of the output
  int rejected_support = 0;   // candidate pixels dropped because GF(I, M) < min_support
  int rejected_distance = 0;  // pixels dropped for being farther than radius from every valid pixel
};

// Upsample a registered depth by 2 along the edges of an RGB guide.
// @param depth_mm CV_16UC1 depth in mm, 0 invalid.
// @param guide_bgr CV_8UC3 guide, exactly twice the size of depth_mm.
// @param cfg radius (guided-filter window and maximum distance to a valid pixel), eps, min_support.
// @param stats optional counters, overwritten when given.
// @return CV_16UC1 depth in mm at the guide size; 0 where the support is too low or no valid input pixel is within
//   cfg.radius.
// @throws std::invalid_argument on a wrong type or size.
cv::Mat guided_upsample(
  const cv::Mat & depth_mm, const cv::Mat & guide_bgr, const UpsamplingConfig & cfg, UpsampleStats * stats = nullptr);

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__UPSAMPLE_HPP_
