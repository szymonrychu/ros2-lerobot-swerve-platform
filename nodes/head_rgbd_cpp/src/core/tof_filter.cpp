#include "head_rgbd_cpp/tof_filter.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>

namespace head_rgbd_cpp
{

cv::Mat filter_jump_edges(const cv::Mat & depth_mm, const FilterConfig & cfg, FilterStats * stats)
{
  if (depth_mm.type() != CV_16UC1) {
    throw std::invalid_argument("filter_jump_edges: depth must be CV_16UC1");
  }
  FilterStats counts;
  cv::Mat out = depth_mm.clone();
  const int rows = depth_mm.rows;
  const int cols = depth_mm.cols;
  for (int v = 0; v < rows; ++v) {
    const uint16_t * row = depth_mm.ptr<uint16_t>(v);
    for (int u = 0; u < cols; ++u) {
      const int z = row[u];
      if (z == 0) {
        continue;
      }
      const double threshold = std::max(cfg.jump_abs_mm, cfg.jump_rel * z);
      int valid_neighbours = 0;
      bool jump = false;
      for (int dv = -1; dv <= 1 && !jump; ++dv) {
        const int nv = v + dv;
        if (nv < 0 || nv >= rows) {
          continue;
        }
        for (int du = -1; du <= 1; ++du) {
          const int nu = u + du;
          if ((dv == 0 && du == 0) || nu < 0 || nu >= cols) {
            continue;
          }
          const int n = depth_mm.at<uint16_t>(nv, nu);
          if (n == 0) {
            continue;
          }
          ++valid_neighbours;
          if (std::abs(n - z) > threshold) {
            jump = true;
            break;
          }
        }
      }
      if (jump) {
        out.at<uint16_t>(v, u) = 0;
        ++counts.removed_jump;
      } else if (valid_neighbours < cfg.min_valid_neighbours) {
        out.at<uint16_t>(v, u) = 0;
        ++counts.removed_isolated;
      }
    }
  }
  if (stats != nullptr) {
    *stats = counts;
  }
  return out;
}

}  // namespace head_rgbd_cpp
