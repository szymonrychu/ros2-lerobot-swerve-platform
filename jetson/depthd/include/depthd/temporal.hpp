// Motion-gated temporal filter on the disparity map (design decision D4: motion by frame difference, no odometry).
#ifndef DEPTHD_TEMPORAL_HPP_
#define DEPTHD_TEMPORAL_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

namespace depthd
{

// Per-pixel exponential moving average over valid disparity pixels:
//   out = alpha * new + (1 - alpha) * history
// The history of a pixel is dropped (out = new) where the grey frame difference exceeds motion_threshold, on the first
// frame, after reset() and after a size change. A pixel that is invalid (<= 0) stays invalid and loses its history, so
// the filter never turns an invalid pixel valid. One fused kernel, in place, on the given stream.
class TemporalFilter
{
public:
  // alpha in (0, 1] is the weight of the new frame; motion_threshold >= 0 in grey levels. Throws std::invalid_argument.
  TemporalFilter(double alpha, int motion_threshold);

  void set_params(double alpha, int motion_threshold);

  // disp: CV_16SC1 disparity * 16, filtered in place; grey: CV_8UC1 of the same size (the left image of the same
  // frame). Throws std::invalid_argument on a wrong type or size.
  void apply(cv::cuda::GpuMat & disp, const cv::cuda::GpuMat & grey,
             cv::cuda::Stream & stream = cv::cuda::Stream::Null());

  // Forgets all history; the next frame passes through.
  void reset();

  double alpha() const { return alpha_; }
  int motion_threshold() const { return motion_threshold_; }

private:
  double alpha_;
  int motion_threshold_;
  bool have_history_ = false;
  cv::cuda::GpuMat state_;       // CV_32FC1, filtered disparity * 16, <= 0 means none
  cv::cuda::GpuMat prev_grey_;   // CV_8UC1
};

}  // namespace depthd

#endif  // DEPTHD_TEMPORAL_HPP_
