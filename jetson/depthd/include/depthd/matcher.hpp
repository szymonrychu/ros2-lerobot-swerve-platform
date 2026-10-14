// The common stereo matcher interface (design decision D3: the matcher is picked by benchmark).
// Implementations: cuda_sgm (cv::cuda::StereoSGM, matcher_cuda_sgm.cpp); vpi (matcher_vpi.cpp) and libsgm
// (matcher_libsgm.cpp) sit behind make_matcher as well.
#ifndef DEPTHD_MATCHER_HPP_
#define DEPTHD_MATCHER_HPP_

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <string>

#include "depthd/config.hpp"

namespace depthd
{

class StereoMatcher
{
public:
  virtual ~StereoMatcher() = default;

  // left/right: rectified CV_8UC1 images of identical size. Returns CV_16SC1 disparity * 16 (left-image
  // referenced); non-positive values are invalid. Throws std::invalid_argument on a wrong type or size mismatch.
  virtual cv::cuda::GpuMat compute(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right,
                                   cv::cuda::Stream & stream = cv::cuda::Stream::Null()) = 0;

  virtual std::string name() const = 0;

  // True when compute() already applies a left-right consistency check (max diff 1 px): the post-processing
  // lr_check stage is then skipped and the matcher is not run a second time for the right-referenced disparity.
  virtual bool supports_lr_check() const = 0;
};

// Builds the matcher named by config.matcher using processing.num_disparities, p1, p2 and uniqueness_ratio.
// Throws std::invalid_argument for an unknown name or unsupported parameters (cuda_sgm: 64, 128 or 256 disparities).
std::unique_ptr<StereoMatcher> make_matcher(const Processing & processing);
inline std::unique_ptr<StereoMatcher> make_matcher(const Config & config) { return make_matcher(config.processing); }

// Shared by implementations: throws std::invalid_argument unless both are non-empty CV_8UC1 of the same size.
void check_matcher_inputs(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right);

// cuda_sgm factory (matcher_cuda_sgm.cpp).
std::unique_ptr<StereoMatcher> make_cuda_sgm_matcher(const Processing & processing);
// vpi factory (matcher_vpi.cpp): VPI 1.1 CUDA stereo, 64, 128 or 256 disparities.
std::unique_ptr<StereoMatcher> make_vpi_matcher(const Processing & processing);
// libsgm factory (matcher_libsgm.cpp): libSGM 3.1.0, 64, 128 or 256 disparities.
std::unique_ptr<StereoMatcher> make_libsgm_matcher(const Processing & processing);

}  // namespace depthd

#endif  // DEPTHD_MATCHER_HPP_
