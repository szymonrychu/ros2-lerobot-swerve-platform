// The monocular depth stage (design D5): a TensorRT FP16 engine (Depth Anything V2 Small, MiDaS v2.1 small as the
// fallback) fills the pixels stereo left invalid, published as the `fused` wire kind (/stereo/depth/fused on the Pi).
//
//   match thread, Pipeline depth listener (after the depth frame is final):
//     mono enabled, model loaded, worker idle and the next 1 / mono.rate_hz slot reached (average <= rate_hz)?
//       copy rectified left (GPU) and depth mm (host) into the stage's own buffers --> worker
//   worker thread, its own lowest-priority CUDA stream (the stereo stream has the highest priority):
//     resize + normalise -> TensorRT -> resize to the processing size -> download
//     align_and_fill (scale_align.hpp) -> Pipeline::publish_fused (same header as the depth frame)
//
// The stage never writes into the depth path: it reads copies, and `depth` goes out exactly as without it. A frame
// arriving while the worker is busy or before the rate period is simply not used (no queue). Every buffer is
// allocated once and reused. A model that does not load sets mono_state "error: ..." and fused stays off; depth is
// unaffected.
#ifndef DEPTHD_MONO_HPP_
#define DEPTHD_MONO_HPP_

#include <cuda_runtime_api.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core/cuda.hpp>
#include <string>
#include <thread>
#include <vector>

#include "depthd/config.hpp"
#include "depthd/health.hpp"
#include "depthd/pipeline.hpp"
#include "depthd/scale_align.hpp"

namespace depthd
{

// ImageNet normalisation of both exported models (jetson/models/README.md, graph contract), RGB order.
constexpr float kMonoMean[3] = {0.485F, 0.456F, 0.406F};
constexpr float kMonoStd[3] = {0.229F, 0.224F, 0.225F};

// A CUDA stream with a scheduling priority, usable as a cv::cuda::Stream. Lower numbers run first on the GPU:
// highest is the stereo stream, lowest the mono stream.
class PriorityStream
{
public:
  enum class Priority
  {
    lowest,
    highest
  };
  explicit PriorityStream(Priority priority);
  ~PriorityStream();
  PriorityStream(const PriorityStream &) = delete;
  PriorityStream & operator=(const PriorityStream &) = delete;

  cv::cuda::Stream & stream() { return cv_stream_; }
  int priority() const;  // cudaStreamGetPriority

private:
  cudaStream_t raw_ = nullptr;
  cv::cuda::Stream cv_stream_;
};

// A monocular depth network: grey image in, relative inverse depth (larger is nearer) out.
class MonoModel
{
public:
  virtual ~MonoModel() = default;
  // grey: CV_8UC1; inv_out: CV_32FC1 at out_size. Enqueued on `stream`; the caller synchronises.
  virtual void infer(
    const cv::cuda::GpuMat & grey, cv::Size out_size, cv::cuda::GpuMat & inv_out, cv::cuda::Stream & stream) = 0;
  virtual cv::Size input_size() const = 0;
};

// A serialized TensorRT engine with input `image` (1x3xHxW float) and output `inverse_depth` (1x1xHxW float).
// Throws std::runtime_error when the file is missing or not such an engine.
class TrtMonoModel : public MonoModel
{
public:
  explicit TrtMonoModel(const std::string & engine_path);
  ~TrtMonoModel() override;
  void infer(
    const cv::cuda::GpuMat & grey, cv::Size out_size, cv::cuda::GpuMat & inv_out, cv::cuda::Stream & stream) override;
  cv::Size input_size() const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

using MonoModelFactory = std::function<std::unique_ptr<MonoModel>(const Mono &)>;

struct MonoStats
{
  uint64_t runs = 0;       // inferences
  uint64_t published = 0;  // fused frames sent
  uint64_t no_fit = 0;     // runs without a fit (no fused frame)
  double fps = 0.0;        // inferences per second
  int last_inliers = 0;
  double last_fill_ratio = 0.0;
  double last_ms = 0.0;    // hand-off to publish
};

class MonoStage
{
public:
  // factory defaults to TrtMonoModel(mono.engine_path). `pipeline` and `metrics` must outlive the stage.
  MonoStage(const Config & config, Pipeline & pipeline, Metrics & metrics, MonoModelFactory factory = nullptr,
            LogFn log = nullptr);
  ~MonoStage();
  MonoStage(const MonoStage &) = delete;
  MonoStage & operator=(const MonoStage &) = delete;

  // Registers the pipeline's depth listener and starts the worker (which loads the model while mono is enabled).
  void start();
  void stop();
  MonoStats stats() const;

  struct Impl;

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace depthd

#endif  // DEPTHD_MONO_HPP_
