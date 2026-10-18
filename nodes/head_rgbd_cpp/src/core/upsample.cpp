#include "head_rgbd_cpp/upsample.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <stdexcept>

namespace head_rgbd_cpp
{

namespace
{
constexpr double MAX_DEPTH_MM_F = 65535.0;
constexpr double BYTE_MAX = 255.0;
}  // namespace

cv::Mat guided_upsample(
  const cv::Mat & depth_mm, const cv::Mat & guide_bgr, const UpsamplingConfig & cfg, UpsampleStats * stats)
{
  if (depth_mm.type() != CV_16UC1) {
    throw std::invalid_argument("guided_upsample: depth must be CV_16UC1");
  }
  if (guide_bgr.type() != CV_8UC3) {
    throw std::invalid_argument("guided_upsample: guide must be CV_8UC3");
  }
  const cv::Size size = depth_mm.size() * UPSAMPLE_FACTOR;
  if (guide_bgr.size() != size) {
    throw std::invalid_argument("guided_upsample: guide must be exactly twice the depth size");
  }

  cv::Mat d0_u16;
  cv::resize(depth_mm, d0_u16, size, 0.0, 0.0, cv::INTER_NEAREST);
  const cv::Mat valid = d0_u16 > 0;  // CV_8UC1, 255 valid
  cv::Mat d0;
  cv::Mat mask;
  d0_u16.convertTo(d0, CV_32FC1);
  valid.convertTo(mask, CV_32FC1, 1.0 / BYTE_MAX);

  UpsampleStats local;
  local.valid_in = cv::countNonZero(depth_mm);

  cv::Mat out(size, CV_16UC1, cv::Scalar(0));
  if (local.valid_in > 0) {
    cv::Mat grey;
    cv::cvtColor(guide_bgr, grey, cv::COLOR_BGR2GRAY);
    grey.convertTo(grey, CV_32FC1, 1.0 / BYTE_MAX);

    cv::Mat num;
    cv::Mat den;
    cv::ximgproc::guidedFilter(grey, d0.mul(mask), num, cfg.radius, cfg.eps);
    cv::ximgproc::guidedFilter(grey, mask, den, cfg.radius, cfg.eps);

    // Distance of every pixel to the nearest valid pixel: nonzero pixels of the inverted mask to the nearest zero.
    cv::Mat distance;
    cv::distanceTransform(~valid, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

    for (int v = 0; v < size.height; ++v) {
      const auto * num_row = num.ptr<float>(v);
      const auto * den_row = den.ptr<float>(v);
      const auto * dist_row = distance.ptr<float>(v);
      auto * out_row = out.ptr<uint16_t>(v);
      for (int u = 0; u < size.width; ++u) {
        if (dist_row[u] > static_cast<float>(cfg.radius)) {
          ++local.rejected_distance;
        } else if (den_row[u] < static_cast<float>(cfg.min_support)) {
          ++local.rejected_support;
        } else {
          const double z = static_cast<double>(num_row[u]) / static_cast<double>(den_row[u]);
          if (z >= 0.5) {
            out_row[u] = static_cast<uint16_t>(std::lround(std::min(z, MAX_DEPTH_MM_F)));
          }
        }
      }
    }
  }
  local.valid_out = cv::countNonZero(out);
  if (stats != nullptr) {
    *stats = local;
  }
  return out;
}

}  // namespace head_rgbd_cpp
