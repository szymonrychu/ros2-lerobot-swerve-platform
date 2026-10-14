// NVIDIA VPI 1.1 stereo disparity estimator (CUDA backend, C API) behind the common matcher interface.
//
// VPI 1.1 on the Nano exposes only windowSize and maxDisparity (plus a confidence threshold and, for PVA+NVENC
// only, a quality knob); there is no VIC or PVA stereo path on this board. The U8 inputs are wrapped from the
// GpuMat memory (no copy), the Q10.5 disparity is converted to disparity * 16 and pixels whose confidence is
// below kVpiConfidenceMin are marked invalid.
#include <vpi/CUDAInterop.h>
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/StereoDisparity.h>

#include <memory>
#include <opencv2/cudaarithm.hpp>
#include <stdexcept>
#include <string>

#include "depthd/matcher.hpp"

namespace depthd
{
namespace
{

constexpr int kWindowSize = 5;
constexpr int kMaxDisparityLimit = 256;

void vpi_check(VPIStatus status, const char * what)
{
  if (status == VPI_SUCCESS) return;
  char msg[VPI_MAX_STATUS_MESSAGE_LENGTH] = {};
  vpiGetLastStatusMessage(msg, sizeof(msg));
  throw std::runtime_error(std::string("vpi ") + what + " failed: " + vpiStatusGetName(status) + ": " + msg);
}

struct StreamDeleter
{
  void operator()(VPIStream s) const { vpiStreamDestroy(s); }
};
struct PayloadDeleter
{
  void operator()(VPIPayload p) const { vpiPayloadDestroy(p); }
};
struct ImageDeleter
{
  void operator()(VPIImage i) const { vpiImageDestroy(i); }
};
using StreamPtr = std::unique_ptr<std::remove_pointer_t<VPIStream>, StreamDeleter>;
using PayloadPtr = std::unique_ptr<std::remove_pointer_t<VPIPayload>, PayloadDeleter>;
using ImagePtr = std::unique_ptr<std::remove_pointer_t<VPIImage>, ImageDeleter>;

// Wraps the device memory of a single-plane GpuMat; the GpuMat must outlive the image.
ImagePtr wrap(const cv::cuda::GpuMat & m, VPIImageFormat format, VPIPixelType pixel_type)
{
  VPIImageData data = {};
  data.format = format;
  data.numPlanes = 1;
  data.planes[0].pixelType = pixel_type;
  data.planes[0].width = m.cols;
  data.planes[0].height = m.rows;
  data.planes[0].pitchBytes = static_cast<int32_t>(m.step);
  data.planes[0].data = m.data;
  VPIImage img = nullptr;
  vpi_check(vpiImageCreateCUDAMemWrapper(&data, VPI_BACKEND_CUDA, &img), "vpiImageCreateCUDAMemWrapper");
  return ImagePtr(img);
}

class VpiMatcher : public StereoMatcher
{
public:
  explicit VpiMatcher(int max_disparity) : max_disparity_(max_disparity)
  {
    VPIStream s = nullptr;
    vpi_check(vpiStreamCreate(VPI_BACKEND_CUDA, &s), "vpiStreamCreate");
    stream_.reset(s);
  }

  cv::cuda::GpuMat compute(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right,
                           cv::cuda::Stream & stream) override
  {
    check_matcher_inputs(left, right);
    if (left.size() != size_) create_payload(left.size());

    cv::cuda::GpuMat disp16s(left.size(), CV_16UC1);  // Q10.5, unsigned: the CUDA backend accepts only U16
    confidence_.create(left.size(), CV_16UC1);

    // VPI runs on its own stream: finish the caller's pending work (e.g. rectification) first.
    stream.waitForCompletion();
    ImagePtr l = wrap(left, VPI_IMAGE_FORMAT_U8, VPI_PIXEL_TYPE_U8);
    ImagePtr r = wrap(right, VPI_IMAGE_FORMAT_U8, VPI_PIXEL_TYPE_U8);
    ImagePtr d = wrap(disp16s, VPI_IMAGE_FORMAT_U16, VPI_PIXEL_TYPE_U16);
    ImagePtr c = wrap(confidence_, VPI_IMAGE_FORMAT_U16, VPI_PIXEL_TYPE_U16);

    VPIStereoDisparityEstimatorParams params;
    vpi_check(vpiInitStereoDisparityEstimatorParams(&params), "vpiInitStereoDisparityEstimatorParams");
    params.windowSize = kWindowSize;
    params.maxDisparity = max_disparity_;
    params.confidenceThreshold = 1;  // keep every pixel here; the cut is applied below from the confidence map
    vpi_check(vpiSubmitStereoDisparityEstimator(stream_.get(), VPI_BACKEND_CUDA, payload_.get(), l.get(), r.get(),
                                                d.get(), c.get(), &params),
              "vpiSubmitStereoDisparityEstimator");
    vpi_check(vpiStreamSync(stream_.get()), "vpiStreamSync");

    // Q10.5 -> disparity * 16, then low confidence -> invalid (-1).
    cv::cuda::GpuMat disp;
    disp16s.convertTo(disp, CV_16SC1, 0.5, stream);
    cv::cuda::GpuMat low;
    cv::cuda::compare(confidence_, cv::Scalar(kVpiConfidenceMin), low, cv::CMP_LT, stream);
    disp.setTo(cv::Scalar(-1), low, stream);
    return disp;
  }

  std::string name() const override { return "vpi"; }
  bool supports_lr_check() const override { return false; }

  // Confidence (0..65535) below which a pixel is invalid; VPI's documented default; the synthetic
  // shifted-pair test keeps 100% of its pixels at this value and an uncorrelated pair keeps almost none (README).
  static constexpr int kVpiConfidenceMin = 32767;

private:
  void create_payload(const cv::Size & size)
  {
    VPIStereoDisparityEstimatorCreationParams cp;
    vpi_check(vpiInitStereoDisparityEstimatorCreationParams(&cp), "vpiInitStereoDisparityEstimatorCreationParams");
    cp.maxDisparity = max_disparity_;
    VPIPayload p = nullptr;
    vpi_check(vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, size.width, size.height, VPI_IMAGE_FORMAT_U8, &cp, &p),
              "vpiCreateStereoDisparityEstimator");
    payload_.reset(p);
    size_ = size;
  }

  int max_disparity_;
  cv::Size size_{0, 0};
  StreamPtr stream_;
  PayloadPtr payload_;
  cv::cuda::GpuMat confidence_;
};

}  // namespace

std::unique_ptr<StereoMatcher> make_vpi_matcher(const Processing & p)
{
  if (p.num_disparities != 64 && p.num_disparities != 128 && p.num_disparities != kMaxDisparityLimit)
    throw std::invalid_argument("vpi supports 64, 128 or 256 disparities, got " + std::to_string(p.num_disparities));
  return std::make_unique<VpiMatcher>(p.num_disparities);
}

}  // namespace depthd
