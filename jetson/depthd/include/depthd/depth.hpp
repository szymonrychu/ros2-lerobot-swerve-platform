// Disparity to depth on the GPU. Output is the ToF contract: CV_16UC1 millimetres, 0 = invalid.
#ifndef DEPTHD_DEPTH_HPP_
#define DEPTHD_DEPTH_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

namespace depthd
{

// Fixed point scale of the matcher disparity (CV_16S holds disparity * 16).
constexpr int kDisparityScale = 16;

// Z = f * B / d in millimetres, rounded to uint16 and saturated at 65535 (never wrapped).
// Pixels are 0 where d <= 0 (this covers the matchers' negative invalid marker), where d == INT16_MIN, and where the
// rounded Z falls outside [round(min_depth_m * 1000), round(max_depth_m * 1000)].
// disp16: CV_16SC1 (disparity * 16). Throws std::invalid_argument on a wrong type, non-positive f or baseline, or
// an empty or inverted depth range. Result is created in `depth_mm` (CV_16UC1, same size) and ordered on `stream`.
void disparity_to_depth_mm(const cv::cuda::GpuMat & disp16, double f_px, double baseline_m, double min_depth_m,
                           double max_depth_m, cv::cuda::GpuMat & depth_mm,
                           cv::cuda::Stream & stream = cv::cuda::Stream::Null());

cv::cuda::GpuMat disparity_to_depth_mm(const cv::cuda::GpuMat & disp16, double f_px, double baseline_m,
                                       double min_depth_m, double max_depth_m,
                                       cv::cuda::Stream & stream = cv::cuda::Stream::Null());

}  // namespace depthd

#endif  // DEPTHD_DEPTH_HPP_
