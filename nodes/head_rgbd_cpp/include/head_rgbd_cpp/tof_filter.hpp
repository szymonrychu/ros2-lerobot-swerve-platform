// Flying-pixel (jump edge) filter of the ToF depth image. Runs on every frame before reprojection.
#ifndef HEAD_RGBD_CPP__TOF_FILTER_HPP_
#define HEAD_RGBD_CPP__TOF_FILTER_HPP_

#include <opencv2/core.hpp>

namespace head_rgbd_cpp
{

// Flying-pixel filter of the ToF depth.
struct FilterConfig
{
  double jump_rel = 0.05;        // 0 < v < 1
  double jump_abs_mm = 30.0;     // >= 0
  int min_valid_neighbours = 3;  // 0..8
};

// Pixels removed by one filter pass.
struct FilterStats
{
  int removed_jump = 0;      // a valid 8-neighbour differs by more than the jump threshold
  int removed_isolated = 0;  // fewer than min_valid_neighbours valid neighbours (and no jump)
};

// Removes flying pixels from a depth image.
// depth_mm: CV_16UC1, millimetres, 0 = invalid. Not modified.
// cfg: a valid pixel at depth z becomes 0 when any valid 8-neighbour differs by more than
//   max(cfg.jump_abs_mm, cfg.jump_rel * z), or when it has fewer than cfg.min_valid_neighbours valid neighbours.
//   Neighbours outside the image count as invalid.
// stats: optional, set to the counts of removed pixels.
// Returns a new CV_16UC1 image of the same size.
cv::Mat filter_jump_edges(const cv::Mat & depth_mm, const FilterConfig & cfg, FilterStats * stats = nullptr);

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__TOF_FILTER_HPP_
