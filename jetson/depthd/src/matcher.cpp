#include "depthd/matcher.hpp"

#include <stdexcept>

namespace depthd
{

void check_matcher_inputs(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right)
{
  if (left.empty() || right.empty()) throw std::invalid_argument("matcher inputs must not be empty");
  if (left.type() != CV_8UC1 || right.type() != CV_8UC1) throw std::invalid_argument("matcher inputs must be CV_8UC1");
  if (left.size() != right.size()) throw std::invalid_argument("matcher inputs must have the same size");
}

std::unique_ptr<StereoMatcher> make_matcher(const Processing & processing)
{
  if (processing.matcher == "cuda_sgm") return make_cuda_sgm_matcher(processing);
  if (processing.matcher == "vpi") return make_vpi_matcher(processing);
  if (processing.matcher == "libsgm") return make_libsgm_matcher(processing);
  throw std::invalid_argument("unknown matcher '" + processing.matcher + "'");
}

}  // namespace depthd
