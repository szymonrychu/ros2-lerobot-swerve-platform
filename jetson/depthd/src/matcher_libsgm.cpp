// libSGM 3.1.0 (fixstars, fetched in CMakeLists.txt) behind the common matcher interface, device-pointer API.
// Subpixel output (4 fractional bits, equal to the interface's disparity * 16), left-right consistency check and
// uniqueness are all done inside libSGM; its invalid marker is a negative value, which the interface also treats
// as invalid.
#include <libsgm.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "depthd/matcher.hpp"

namespace depthd
{
namespace
{

constexpr int kLrMaxDiff = 1;

class LibsgmMatcher : public StereoMatcher
{
public:
  explicit LibsgmMatcher(const Processing & p) : p_(p) {}

  cv::cuda::GpuMat compute(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right,
                           cv::cuda::Stream & stream) override
  {
    check_matcher_inputs(left, right);
    if (left.size() != size_ || left.step != src_pitch_) create(left.size(), left.step);

    // libSGM runs on the default stream and synchronises; finish the caller's pending work first.
    stream.waitForCompletion();
    cv::cuda::GpuMat disp(left.size(), CV_16SC1);
    if (static_cast<int>(disp.step / sizeof(int16_t)) != dst_pitch_) {
      create(left.size(), left.step, static_cast<int>(disp.step / sizeof(int16_t)));
    }
    sgm_->execute(left.data, right.data, disp.data);
    return disp;  // CV_16SC1, disparity * 16, negative = invalid
  }

  std::string name() const override { return "libsgm"; }
  bool supports_lr_check() const override { return false; }  // the check runs inside; no right disparity is returned

private:
  void create(const cv::Size & size, size_t src_step, int dst_pitch = 0)
  {
    if (dst_pitch == 0) {
      cv::cuda::GpuMat probe(size, CV_16SC1);
      dst_pitch = static_cast<int>(probe.step / sizeof(int16_t));
    }
    sgm::StereoSGM::Parameters params(p_.p1, p_.p2, 1.0f - static_cast<float>(p_.uniqueness_ratio) / 100.0f, true,
                                      sgm::PathType::SCAN_8PATH, 0, kLrMaxDiff);
    sgm_ = std::make_unique<sgm::StereoSGM>(size.width, size.height, p_.num_disparities, 8, 16,
                                            static_cast<int>(src_step), dst_pitch, sgm::EXECUTE_INOUT_CUDA2CUDA, params);
    size_ = size;
    src_pitch_ = src_step;
    dst_pitch_ = dst_pitch;
  }

  Processing p_;
  cv::Size size_{0, 0};
  size_t src_pitch_ = 0;
  int dst_pitch_ = 0;
  std::unique_ptr<sgm::StereoSGM> sgm_;
};

}  // namespace

std::unique_ptr<StereoMatcher> make_libsgm_matcher(const Processing & p)
{
  if (p.num_disparities != 64 && p.num_disparities != 128 && p.num_disparities != 256)
    throw std::invalid_argument("libsgm supports 64, 128 or 256 disparities, got " + std::to_string(p.num_disparities));
  return std::make_unique<LibsgmMatcher>(p);
}

}  // namespace depthd
