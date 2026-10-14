#include "depthd/depth.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <stdexcept>

#include "depthd/postprocess_kernels.hpp"

namespace depthd
{

void disparity_to_depth_mm(const cv::cuda::GpuMat & disp16, double f_px, double baseline_m, double min_depth_m,
                           double max_depth_m, cv::cuda::GpuMat & depth_mm, cv::cuda::Stream & stream)
{
  if (disp16.type() != CV_16SC1) throw std::invalid_argument("disparity must be CV_16SC1 (disparity * 16)");
  if (!(f_px > 0.0) || !(baseline_m > 0.0)) throw std::invalid_argument("f_px and baseline_m must be positive");
  if (!(min_depth_m >= 0.0) || !(max_depth_m > min_depth_m))
    throw std::invalid_argument("depth range must satisfy 0 <= min_depth_m < max_depth_m");

  // Z_mm = f * B * 1000 / (d16 / 16), one kernel and no temporaries (it runs on every frame).
  const double numerator = f_px * baseline_m * 1000.0 * kDisparityScale;
  const double min_mm = std::round(min_depth_m * 1000.0);
  const double max_mm = std::min(std::round(max_depth_m * 1000.0), 65535.0);
  depth_mm.create(disp16.size(), CV_16UC1);
  kernels::disparity_to_depth({disp16.ptr<short>(), disp16.step}, depth_mm.ptr<unsigned short>(), depth_mm.step,
                              disp16.cols, disp16.rows, static_cast<float>(numerator), static_cast<int>(min_mm),
                              static_cast<int>(max_mm), cv::cuda::StreamAccessor::getStream(stream));
}

cv::cuda::GpuMat disparity_to_depth_mm(const cv::cuda::GpuMat & disp16, double f_px, double baseline_m,
                                       double min_depth_m, double max_depth_m, cv::cuda::Stream & stream)
{
  cv::cuda::GpuMat out;
  disparity_to_depth_mm(disp16, f_px, baseline_m, min_depth_m, max_depth_m, out, stream);
  return out;
}

}  // namespace depthd
