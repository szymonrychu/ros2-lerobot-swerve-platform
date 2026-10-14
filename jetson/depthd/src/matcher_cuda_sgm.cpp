// cv::cuda::StereoSGM (OpenCV 4.8 CUDA, cudastereo) behind the common matcher interface.
#include <opencv2/cudastereo.hpp>
#include <stdexcept>
#include <string>

#include "depthd/matcher.hpp"

namespace depthd
{
namespace
{

class CudaSgmMatcher : public StereoMatcher
{
public:
  explicit CudaSgmMatcher(const Processing & p)
  : sgm_(cv::cuda::createStereoSGM(0, p.num_disparities, p.p1, p.p2, p.uniqueness_ratio,
                                   cv::cuda::StereoSGM::MODE_HH4))
  {
  }

  cv::cuda::GpuMat compute(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right,
                           cv::cuda::Stream & stream) override
  {
    check_matcher_inputs(left, right);
    cv::cuda::GpuMat disp;
    sgm_->compute(left, right, disp, stream);
    return disp;  // CV_16SC1, disparity * 16
  }

  std::string name() const override { return "cuda_sgm"; }
  bool supports_lr_check() const override { return false; }

private:
  cv::Ptr<cv::cuda::StereoSGM> sgm_;
};

}  // namespace

std::unique_ptr<StereoMatcher> make_cuda_sgm_matcher(const Processing & p)
{
  if (p.num_disparities != 64 && p.num_disparities != 128 && p.num_disparities != 256)
    throw std::invalid_argument("cuda_sgm supports 64, 128 or 256 disparities, got " +
                                std::to_string(p.num_disparities));
  return std::make_unique<CudaSgmMatcher>(p);
}

}  // namespace depthd
