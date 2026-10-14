#include "depthd/depth.hpp"

#include <cmath>
#include <opencv2/cudaarithm.hpp>
#include <stdexcept>

namespace depthd
{

void disparity_to_depth_mm(const cv::cuda::GpuMat & disp16, double f_px, double baseline_m, double min_depth_m,
                           double max_depth_m, cv::cuda::GpuMat & depth_mm, cv::cuda::Stream & stream)
{
  if (disp16.type() != CV_16SC1) throw std::invalid_argument("disparity must be CV_16SC1 (disparity * 16)");
  if (!(f_px > 0.0) || !(baseline_m > 0.0)) throw std::invalid_argument("f_px and baseline_m must be positive");
  if (!(min_depth_m >= 0.0) || !(max_depth_m > min_depth_m))
    throw std::invalid_argument("depth range must satisfy 0 <= min_depth_m < max_depth_m");

  // Z_mm = f * B * 1000 / (d16 / 16)
  const double numerator = f_px * baseline_m * 1000.0 * kDisparityScale;
  const double min_mm = std::round(min_depth_m * 1000.0);
  const double max_mm = std::min(std::round(max_depth_m * 1000.0), 65535.0);

  cv::cuda::GpuMat d32, z32, bad, out_of_range;
  disp16.convertTo(d32, CV_32F, stream);
  cv::cuda::compare(d32, cv::Scalar(0), bad, cv::CMP_LE, stream);  // d <= 0, includes INT16_MIN
  cv::cuda::divide(cv::Scalar(numerator), d32, z32, 1.0, -1, stream);
  z32.convertTo(depth_mm, CV_16U, stream);  // rounds to nearest, saturates at 65535
  cv::cuda::GpuMat zf;
  depth_mm.convertTo(zf, CV_32F, stream);
  cv::cuda::compare(zf, cv::Scalar(min_mm), out_of_range, cv::CMP_LT, stream);
  cv::cuda::bitwise_or(bad, out_of_range, bad, cv::noArray(), stream);
  cv::cuda::compare(zf, cv::Scalar(max_mm), out_of_range, cv::CMP_GT, stream);
  cv::cuda::bitwise_or(bad, out_of_range, bad, cv::noArray(), stream);
  depth_mm.setTo(cv::Scalar(0), bad, stream);
}

cv::cuda::GpuMat disparity_to_depth_mm(const cv::cuda::GpuMat & disp16, double f_px, double baseline_m,
                                       double min_depth_m, double max_depth_m, cv::cuda::Stream & stream)
{
  cv::cuda::GpuMat out;
  disparity_to_depth_mm(disp16, f_px, baseline_m, min_depth_m, max_depth_m, out, stream);
  return out;
}

}  // namespace depthd
