// GPU post-processing of the matcher's disparity map (design decision D4), one timed, switchable stage at a time.
#ifndef DEPTHD_POSTPROCESS_HPP_
#define DEPTHD_POSTPROCESS_HPP_

#include <cuda_runtime.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <string>
#include <vector>

#include "depthd/config.hpp"
#include "depthd/matcher.hpp"
#include "depthd/temporal.hpp"

namespace depthd
{

// Per-stage cost of the last run (CUDA events on the run's stream) and over all runs. A disabled stage is not
// launched: ran is false, ms is 0 and nothing is recorded.
struct StageTiming
{
  std::string name;
  bool ran = false;
  float ms = 0.f;       // last run
  float mean_ms = 0.f;  // over the runs in which the stage ran
  uint64_t runs = 0;    // number of runs in which the stage ran
};

// Disparity of the right image (right-referenced, CV_16SC1 * 16) from any matcher: both images are mirrored, swapped,
// matched and the result mirrored back. Use it when needs_right_disparity() is true.
cv::cuda::GpuMat compute_right_disparity(StereoMatcher & matcher, const cv::cuda::GpuMat & left,
                                         const cv::cuda::GpuMat & right,
                                         cv::cuda::Stream & stream = cv::cuda::Stream::Null());

// Runs, in this order and each only when its config switch is on:
//   lr_check, confidence, speckle, bilateral, hole_fill, wls, temporal
// on a CV_16SC1 disparity * 16 map (invalid <= 0), in place. Every stage that runs writes 0 for invalid pixels; no
// stage turns an invalid pixel valid except hole_fill (small holes only) and no stage touches data when disabled.
// The configuration may change between runs (set_postprocess); a run and a set_postprocess may come from different
// threads, two concurrent runs may not.
class PostProcessor
{
public:
  static constexpr int kStageCount = 7;

  // matcher_native_lr_check is StereoMatcher::supports_lr_check() of the matcher that produced the disparity: its
  // output is already left-right checked, so the lr_check stage is skipped.
  explicit PostProcessor(const Config & config, bool matcher_native_lr_check = false);
  ~PostProcessor();
  PostProcessor(const PostProcessor &) = delete;
  PostProcessor & operator=(const PostProcessor &) = delete;

  void set_postprocess(const Postprocess & postprocess);
  Postprocess postprocess() const;

  // True when lr_check is on and the matcher has no native check: run() then needs right_disp.
  bool needs_right_disparity() const;

  // disp: CV_16SC1 disparity * 16, processed in place. left: CV_8UC1 rectified left image of the same size (guidance
  // for the bilateral filter, hole fill and temporal motion). right_disp: right-referenced disparity, only read by
  // lr_check. Throws std::invalid_argument on a wrong type or size, or when lr_check needs right_disp and it is null.
  void run(cv::cuda::GpuMat & disp, const cv::cuda::GpuMat & left, const cv::cuda::GpuMat * right_disp = nullptr,
           cv::cuda::Stream & stream = cv::cuda::Stream::Null());

  // Timings of the last run in stage order (waits for that run's stream to reach its last stage).
  std::vector<StageTiming> timings();

private:
  struct Stage
  {
    std::string name;
    cudaEvent_t begin = nullptr, end = nullptr;
    bool ran = false;
    bool pending = false;
    float ms = 0.f;
    double total_ms = 0.0;
    uint64_t runs = 0;
  };
  struct Workspace;

  template <class Fn>
  void timed(int stage, bool enabled, cudaStream_t stream, Fn && fn);
  void finalize();
  void ensure_workspace(cv::Size size);

  mutable std::mutex mutex_;
  Postprocess cfg_;
  int num_disparities_;
  bool native_lr_;
  std::vector<Stage> stages_;
  TemporalFilter temporal_;
  std::unique_ptr<Workspace> ws_;
};

}  // namespace depthd

#endif  // DEPTHD_POSTPROCESS_HPP_
