#include "depthd/temporal.hpp"

#include <opencv2/core/cuda_stream_accessor.hpp>
#include <stdexcept>

#include "depthd/postprocess_kernels.hpp"

namespace depthd
{
namespace
{
void check_params(double alpha, int motion_threshold)
{
  if (!(alpha > 0.0 && alpha <= 1.0)) throw std::invalid_argument("temporal alpha must be in (0, 1]");
  if (motion_threshold < 0) throw std::invalid_argument("temporal motion_threshold must be >= 0");
}
}  // namespace

TemporalFilter::TemporalFilter(double alpha, int motion_threshold) : alpha_(alpha), motion_threshold_(motion_threshold)
{
  check_params(alpha, motion_threshold);
}

void TemporalFilter::set_params(double alpha, int motion_threshold)
{
  check_params(alpha, motion_threshold);
  alpha_ = alpha;
  motion_threshold_ = motion_threshold;
}

void TemporalFilter::reset() { have_history_ = false; }

void TemporalFilter::apply(cv::cuda::GpuMat & disp, const cv::cuda::GpuMat & grey, cv::cuda::Stream & stream)
{
  if (disp.type() != CV_16SC1 || disp.empty()) throw std::invalid_argument("temporal: disparity must be non-empty CV_16SC1");
  if (grey.type() != CV_8UC1 || grey.size() != disp.size())
    throw std::invalid_argument("temporal: grey must be CV_8UC1 of the disparity size");
  if (state_.size() != disp.size()) {
    cv::cuda::createContinuous(disp.size(), CV_32FC1, state_);
    cv::cuda::createContinuous(disp.size(), CV_8UC1, prev_grey_);
    have_history_ = false;
  }
  kernels::temporal({reinterpret_cast<short *>(disp.data), disp.step}, {grey.data, grey.step},
                    reinterpret_cast<float *>(state_.data), prev_grey_.data, disp.cols, disp.rows,
                    static_cast<float>(alpha_), motion_threshold_, !have_history_,
                    cv::cuda::StreamAccessor::getStream(stream));
  have_history_ = true;
}

}  // namespace depthd
