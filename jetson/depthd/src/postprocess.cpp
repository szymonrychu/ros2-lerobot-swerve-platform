#include "depthd/postprocess.hpp"

#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <stdexcept>

#include "depthd/postprocess_kernels.hpp"

namespace depthd
{
namespace
{
enum Stage_ { kLr, kConfidence, kSpeckle, kBilateral, kHoleFill, kWls, kTemporal };
const char * const kNames[PostProcessor::kStageCount] = {"lr_check", "confidence", "speckle",
                                                         "bilateral", "hole_fill",  "wls",
                                                         "temporal"};

// Bilateral filter window (ndisp is the value range of the disparity * 16 map): radius in pixels, refinement iterations.
constexpr int kBilateralRadius = 3;
constexpr int kBilateralIterations = 1;
// WLS smoothing, the values OpenCV recommends for a 16-bit disparity * 16 map.
constexpr double kWlsLambda = 8000.0;
constexpr double kWlsSigma = 1.5;
constexpr int kWlsFillWindow = 33;
constexpr int kDispScale = 16;

void cuda_check(cudaError_t e, const char * what)
{
  if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
}

// WLS treats every value as a disparity, so invalid pixels are first replaced by the mean of the valid pixels around
// them (the global mean where there are none), filtered, and set back to 0: WLS must not fill what stays invalid.
cv::Mat wls_filter(cv::ximgproc::DisparityWLSFilter & wls, const cv::Mat & disp, const cv::Mat & left)
{
  const cv::Mat valid = disp > 0;
  if (cv::countNonZero(valid) == 0) return cv::Mat(disp.size(), CV_16SC1, cv::Scalar(0));
  cv::Mat d32, weight, sum_d, sum_w, filled, input, output;
  disp.convertTo(d32, CV_32F);
  d32.setTo(0, ~valid);
  valid.convertTo(weight, CV_32F, 1.0 / 255.0);
  cv::boxFilter(d32, sum_d, -1, cv::Size(kWlsFillWindow, kWlsFillWindow), cv::Point(-1, -1), false);
  cv::boxFilter(weight, sum_w, -1, cv::Size(kWlsFillWindow, kWlsFillWindow), cv::Point(-1, -1), false);
  const double global_mean = cv::sum(d32)[0] / cv::countNonZero(valid);
  cv::max(sum_w, 1e-6, sum_w);
  cv::divide(sum_d, sum_w, filled);
  filled.setTo(global_mean, sum_w < 0.5);
  filled.convertTo(input, CV_16S);
  disp.copyTo(input, valid);
  wls.filter(input, left, output);
  output.setTo(0, ~valid);
  return output;
}

kernels::Disp plane(cv::cuda::GpuMat & m) { return {reinterpret_cast<short *>(m.data), m.step}; }
kernels::CDisp cplane(const cv::cuda::GpuMat & m) { return {reinterpret_cast<const short *>(m.data), m.step}; }
}  // namespace

struct PostProcessor::Workspace
{
  cv::Size size;
  cv::cuda::GpuMat labels, sizes, flag, scratch;
  cv::cuda::HostMem flag_host{1, 1, CV_32SC1, cv::cuda::HostMem::PAGE_LOCKED};
  cv::Ptr<cv::cuda::DisparityBilateralFilter> bilateral;
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls;
};

cv::cuda::GpuMat compute_right_disparity(StereoMatcher & matcher, const cv::cuda::GpuMat & left,
                                         const cv::cuda::GpuMat & right, cv::cuda::Stream & stream)
{
  check_matcher_inputs(left, right);
  cv::cuda::GpuMat left_mirror, right_mirror, mirrored, out;
  cv::cuda::flip(left, left_mirror, 1, stream);
  cv::cuda::flip(right, right_mirror, 1, stream);
  mirrored = matcher.compute(right_mirror, left_mirror, stream);  // the mirrored right image plays the left
  // cv::cuda::flip has no signed 16-bit path; the bits are the same as CV_16UC1.
  out.create(mirrored.size(), CV_16SC1);
  const cv::cuda::GpuMat src_bits(mirrored.rows, mirrored.cols, CV_16UC1, mirrored.data, mirrored.step);
  cv::cuda::GpuMat dst_bits(out.rows, out.cols, CV_16UC1, out.data, out.step);
  cv::cuda::flip(src_bits, dst_bits, 1, stream);
  return out;
}

PostProcessor::PostProcessor(const Config & config, bool matcher_native_lr_check)
: cfg_(config.postprocess),
  num_disparities_(config.processing.num_disparities),
  native_lr_(matcher_native_lr_check),
  stages_(kStageCount),
  temporal_(config.postprocess.temporal.alpha, config.postprocess.temporal.motion_threshold),
  ws_(new Workspace)
{
  for (int i = 0; i < kStageCount; ++i) {
    stages_[i].name = kNames[i];
    cuda_check(cudaEventCreate(&stages_[i].begin), "cudaEventCreate");
    cuda_check(cudaEventCreate(&stages_[i].end), "cudaEventCreate");
  }
}

PostProcessor::~PostProcessor()
{
  for (auto & s : stages_) {
    cudaEventDestroy(s.begin);
    cudaEventDestroy(s.end);
  }
}

void PostProcessor::set_postprocess(const Postprocess & postprocess)
{
  std::lock_guard<std::mutex> lock(mutex_);
  temporal_.set_params(postprocess.temporal.alpha, postprocess.temporal.motion_threshold);
  cfg_ = postprocess;
}

Postprocess PostProcessor::postprocess() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cfg_;
}

bool PostProcessor::needs_right_disparity() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cfg_.lr_check.enabled && !native_lr_;
}

void PostProcessor::finalize()
{
  for (auto & s : stages_) {
    if (!s.pending) continue;
    cuda_check(cudaEventSynchronize(s.end), "cudaEventSynchronize");
    cuda_check(cudaEventElapsedTime(&s.ms, s.begin, s.end), "cudaEventElapsedTime");
    s.total_ms += s.ms;
    ++s.runs;
    s.pending = false;
  }
}

std::vector<StageTiming> PostProcessor::timings()
{
  finalize();
  std::vector<StageTiming> out;
  for (const auto & s : stages_) {
    StageTiming t;
    t.name = s.name;
    t.ran = s.ran;
    t.ms = s.ran ? s.ms : 0.f;
    t.runs = s.runs;
    t.mean_ms = s.runs ? static_cast<float>(s.total_ms / s.runs) : 0.f;
    out.push_back(t);
  }
  return out;
}

template <class Fn>
void PostProcessor::timed(int stage, bool enabled, cudaStream_t stream, Fn && fn)
{
  Stage & s = stages_[stage];
  s.ran = enabled;
  if (!enabled) return;
  cuda_check(cudaEventRecord(s.begin, stream), "cudaEventRecord");
  fn();
  cuda_check(cudaEventRecord(s.end, stream), "cudaEventRecord");
  s.pending = true;
}

void PostProcessor::ensure_workspace(cv::Size size)
{
  if (ws_->size == size) return;
  cv::cuda::createContinuous(size, CV_32SC1, ws_->labels);
  cv::cuda::createContinuous(size, CV_32SC1, ws_->sizes);
  ws_->flag.create(1, 1, CV_32SC1);
  ws_->scratch.create(size, CV_16SC1);
  ws_->size = size;
}

void PostProcessor::run(cv::cuda::GpuMat & disp, const cv::cuda::GpuMat & left, const cv::cuda::GpuMat * right_disp,
                        cv::cuda::Stream & stream)
{
  if (disp.empty() || disp.type() != CV_16SC1) throw std::invalid_argument("postprocess: disparity must be non-empty CV_16SC1");
  if (left.type() != CV_8UC1 || left.size() != disp.size())
    throw std::invalid_argument("postprocess: left must be CV_8UC1 of the disparity size");

  Postprocess cfg;
  bool native;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cfg = cfg_;
    native = native_lr_;
  }
  const bool lr = cfg.lr_check.enabled && !native;
  if (lr) {
    if (right_disp == nullptr)
      throw std::invalid_argument("postprocess: lr_check is on and the matcher has no native check, right_disp is required");
    if (right_disp->type() != CV_16SC1 || right_disp->size() != disp.size())
      throw std::invalid_argument("postprocess: right_disp must be CV_16SC1 of the disparity size");
  }

  finalize();
  const int w = disp.cols, h = disp.rows;
  cudaStream_t s = cv::cuda::StreamAccessor::getStream(stream);
  const bool any_cc = cfg.speckle.enabled || cfg.hole_fill.enabled;
  if (any_cc || cfg.wls.enabled || cfg.bilateral.enabled) ensure_workspace(disp.size());
  auto label = [&](kernels::Grouping g, int tol16) {
    kernels::label_components(cplane(disp), g, tol16, w, h, reinterpret_cast<int *>(ws_->labels.data),
                              reinterpret_cast<int *>(ws_->sizes.data), reinterpret_cast<int *>(ws_->flag.data),
                              reinterpret_cast<int *>(ws_->flag_host.data), s);
  };

  timed(kLr, lr, s, [&] { kernels::lr_check(plane(disp), cplane(*right_disp), w, h, cfg.lr_check.max_diff * kDispScale, s); });
  timed(kConfidence, cfg.confidence.enabled, s, [&] { kernels::confidence(plane(disp), w, h, num_disparities_, s); });
  timed(kSpeckle, cfg.speckle.enabled, s, [&] {
    label(kernels::Grouping::kSimilarValid, cfg.speckle.max_diff * kDispScale);
    kernels::speckle_apply(plane(disp), reinterpret_cast<int *>(ws_->labels.data),
                           reinterpret_cast<int *>(ws_->sizes.data), w, h, cfg.speckle.max_size, s);
  });
  // Filters are built on first use, outside the timed window: a stage reports its steady-state cost.
  if (cfg.bilateral.enabled && !ws_->bilateral)
    ws_->bilateral = cv::cuda::createDisparityBilateralFilter(num_disparities_ * kDispScale, kBilateralRadius, kBilateralIterations);
  if (cfg.wls.enabled && !ws_->wls) {
    ws_->wls = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    ws_->wls->setLambda(kWlsLambda);
    ws_->wls->setSigmaColor(kWlsSigma);
  }
  timed(kBilateral, cfg.bilateral.enabled, s, [&] {
    cv::cuda::GpuMat smoothed, invalid;
    ws_->bilateral->apply(disp, left, smoothed, stream);
    cv::cuda::compare(disp, cv::Scalar(0), invalid, cv::CMP_LE, stream);  // smoothing never validates a pixel
    smoothed.setTo(cv::Scalar(0), invalid, stream);
    smoothed.copyTo(disp, stream);
  });
  timed(kHoleFill, cfg.hole_fill.enabled, s, [&] {
    label(kernels::Grouping::kInvalid, 0);
    kernels::hole_fill(cplane(disp), plane(ws_->scratch), {left.data, left.step},
                       reinterpret_cast<int *>(ws_->labels.data), reinterpret_cast<int *>(ws_->sizes.data), w, h,
                       cfg.hole_fill.max_hole_px, s);
    ws_->scratch.copyTo(disp, stream);
  });
  timed(kWls, cfg.wls.enabled, s, [&] {
    cv::Mat disp_cpu, left_cpu;
    disp.download(disp_cpu, stream);
    left.download(left_cpu, stream);
    stream.waitForCompletion();
    cv::Mat filtered = wls_filter(*ws_->wls, disp_cpu, left_cpu);
    disp.upload(filtered, stream);
  });
  if (cfg.temporal.enabled)
    timed(kTemporal, true, s, [&] { temporal_.apply(disp, left, stream); });
  else {
    stages_[kTemporal].ran = false;
    temporal_.reset();  // a stale history must not survive a switch-off
  }
}

}  // namespace depthd
