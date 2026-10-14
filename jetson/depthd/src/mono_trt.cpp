#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudawarping.hpp>
#include <stdexcept>
#include <utility>

#include "depthd/mono.hpp"

namespace depthd
{
namespace wire = ros2_common_cpp::depth_wire;

namespace
{
constexpr const char * kInputName = "image";
constexpr const char * kOutputName = "inverse_depth";
constexpr int kWorkerPollMs = 200;  // worker wake-up without a job: load on enable, mono fps refresh

void cuda_check(cudaError_t e, const char * what)
{
  if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
}

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) std::cerr << "depthd: tensorrt: " << msg << std::endl;
  }
};

TrtLogger & trt_logger()
{
  static TrtLogger logger;
  return logger;
}

// Height and width: the last two dimensions of an NCHW (or NHW) binding.
cv::Size hw(const nvinfer1::Dims & d) { return d.nbDims >= 2 ? cv::Size(d.d[d.nbDims - 1], d.d[d.nbDims - 2]) : cv::Size(); }

int64_t volume(const nvinfer1::Dims & d)
{
  int64_t v = 1;
  for (int i = 0; i < d.nbDims; ++i) v *= d.d[i];
  return v;
}
}  // namespace

// ---- PriorityStream ----
PriorityStream::PriorityStream(Priority priority)
{
  int least = 0;
  int greatest = 0;
  cuda_check(cudaDeviceGetStreamPriorityRange(&least, &greatest), "cudaDeviceGetStreamPriorityRange");
  cuda_check(
    cudaStreamCreateWithPriority(&raw_, cudaStreamDefault, priority == Priority::highest ? greatest : least),
    "cudaStreamCreateWithPriority");
  cv_stream_ = cv::cuda::StreamAccessor::wrapStream(raw_);
}

PriorityStream::~PriorityStream()
{
  cv_stream_ = cv::cuda::Stream::Null();  // drop the (non-owning) wrapper before the stream goes
  cudaStreamDestroy(raw_);
}

int PriorityStream::priority() const
{
  int p = 0;
  cudaStreamGetPriority(raw_, &p);
  return p;
}

// ---- TrtMonoModel ----
struct TrtMonoModel::Impl
{
  // Destroyed in reverse order: context, engine, runtime.
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;
  std::unique_ptr<nvinfer1::IExecutionContext> context;
  std::vector<void *> bindings;
  int input = -1;
  int output = -1;
  cv::Size in_size;
  cv::Size out_size;
  cv::cuda::GpuMat resized;  // CV_8UC1 at in_size, reused

  ~Impl()
  {
    context.reset();
    engine.reset();
    runtime.reset();
    for (void * p : bindings) {
      if (p) cudaFree(p);
    }
  }
};

TrtMonoModel::TrtMonoModel(const std::string & engine_path) : impl_(std::make_unique<Impl>())
{
  std::ifstream f(engine_path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot read TensorRT engine " + engine_path);
  const std::vector<char> blob((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  Impl & im = *impl_;
  im.runtime.reset(nvinfer1::createInferRuntime(trt_logger()));
  if (!im.runtime) throw std::runtime_error("TensorRT runtime could not be created");
  im.engine.reset(im.runtime->deserializeCudaEngine(blob.data(), blob.size()));
  if (!im.engine) throw std::runtime_error("not a TensorRT engine for this GPU and version: " + engine_path);
  im.input = im.engine->getBindingIndex(kInputName);
  im.output = im.engine->getBindingIndex(kOutputName);
  if (im.input < 0 || im.output < 0 || im.engine->getNbBindings() != 2) {
    throw std::runtime_error(engine_path + ": expected bindings 'image' and 'inverse_depth'");
  }
  const nvinfer1::Dims in = im.engine->getBindingDimensions(im.input);
  const nvinfer1::Dims out = im.engine->getBindingDimensions(im.output);
  if (in.nbDims != 4 || in.d[0] != 1 || in.d[1] != 3 || volume(out) != static_cast<int64_t>(hw(out).area())) {
    throw std::runtime_error(engine_path + ": expected input 1x3xHxW and output 1x1xHxW");
  }
  im.in_size = hw(in);
  im.out_size = hw(out);
  im.bindings.assign(2, nullptr);
  cuda_check(cudaMalloc(&im.bindings[static_cast<std::size_t>(im.input)], sizeof(float) * volume(in)), "cudaMalloc");
  cuda_check(cudaMalloc(&im.bindings[static_cast<std::size_t>(im.output)], sizeof(float) * volume(out)), "cudaMalloc");
  im.context.reset(im.engine->createExecutionContext());
  if (!im.context) throw std::runtime_error("TensorRT execution context could not be created");
}

TrtMonoModel::~TrtMonoModel() = default;

cv::Size TrtMonoModel::input_size() const { return impl_->in_size; }

void TrtMonoModel::infer(
  const cv::cuda::GpuMat & grey, cv::Size out_size, cv::cuda::GpuMat & inv_out, cv::cuda::Stream & stream)
{
  Impl & im = *impl_;
  if (grey.type() != CV_8UC1) throw std::invalid_argument("TrtMonoModel::infer: grey must be CV_8UC1");
  cv::cuda::resize(grey, im.resized, im.in_size, 0, 0, cv::INTER_LINEAR, stream);
  // The grey image stands in for each RGB channel, normalised with that channel's mean and std (planar CHW).
  auto * input = static_cast<float *>(im.bindings[static_cast<std::size_t>(im.input)]);
  const std::size_t plane = static_cast<std::size_t>(im.in_size.area());
  for (int c = 0; c < 3; ++c) {
    cv::cuda::GpuMat dst(im.in_size, CV_32FC1, input + c * plane);
    im.resized.convertTo(
      dst, CV_32FC1, 1.0 / (255.0 * kMonoStd[c]), -static_cast<double>(kMonoMean[c]) / kMonoStd[c], stream);
  }
  if (!im.context->enqueueV2(im.bindings.data(), cv::cuda::StreamAccessor::getStream(stream), nullptr)) {
    throw std::runtime_error("TensorRT enqueue failed");
  }
  const cv::cuda::GpuMat out(im.out_size, CV_32FC1, im.bindings[static_cast<std::size_t>(im.output)]);
  cv::cuda::resize(out, inv_out, out_size, 0, 0, cv::INTER_LINEAR, stream);
}

// ---- MonoStage ----
struct MonoStage::Impl
{
  const Config config;
  Pipeline & pipeline;
  Metrics & metrics;
  MonoModelFactory factory;
  LogFn log;
  AlignParams align;

  PriorityStream low{PriorityStream::Priority::lowest};
  std::unique_ptr<MonoModel> model;
  std::atomic<bool> ready{false};
  bool load_failed = false;

  std::mutex mutex;
  std::condition_variable cv;
  bool started = false;
  bool stopping = false;
  bool busy = false;  // a frame was handed over and is not finished
  bool job = false;   // ... and the worker has not taken it yet
  int64_t last_hand_ns = 0;
  std::thread worker;

  // The handed-over frame and the worker's buffers, allocated once and reused. The listener writes them only while
  // !busy, the worker reads them only while busy.
  cv::cuda::GpuMat grey;
  cv::cuda::HostMem depth_host;
  wire::Header header;
  int64_t handed_ns = 0;
  cv::cuda::GpuMat inv;
  cv::cuda::HostMem inv_host;
  std::vector<uint16_t> fused;
  bool infer_error_logged = false;

  mutable std::mutex stats_mutex;
  MonoStats stats;
  RateMeter rate;

  Impl(const Config & c, Pipeline & p, Metrics & m, MonoModelFactory f, LogFn l)
  : config(c), pipeline(p), metrics(m), factory(std::move(f)), log(std::move(l))
  {
    if (!factory) {
      factory = [](const Mono & mono) { return std::unique_ptr<MonoModel>(new TrtMonoModel(mono.engine_path)); };
    }
    align.min_depth_m = config.processing.min_depth_m;
    align.max_depth_m = config.processing.max_depth_m;
  }

  void say(const std::string & line) const
  {
    if (log) log(line);
  }

  // Match thread: take the frame when it is due and the worker is free; otherwise return at once.
  void on_depth(const DepthFrame & frame)
  {
    if (!ready.load()) return;
    const Mono settings = pipeline.mono_settings();
    if (!settings.enabled || settings.rate_hz <= 0.0) return;
    const int64_t now = steady_ns();
    const auto period = static_cast<int64_t>(1e9 / settings.rate_hz);
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (stopping || busy) return;
      if (last_hand_ns && now - last_hand_ns < period) return;
      // Hand-offs land on stereo frames, so a strict period would run below rate_hz; advancing by one period keeps
      // the average at rate_hz (never above it) unless the stage fell more than a period behind.
      last_hand_ns = (last_hand_ns && now - last_hand_ns < 2 * period) ? last_hand_ns + period : now;
      busy = true;
    }
    frame.rectified_left.copyTo(grey, frame.stream);
    frame.depth_mm.download(depth_host, frame.stream);
    frame.stream.waitForCompletion();
    header = frame.header;
    handed_ns = now;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      job = true;
    }
    cv.notify_all();
  }

  void load(const Mono & settings)
  {
    pipeline.set_mono_state("loading");
    try {
      model = factory(settings);
      if (!model) throw std::runtime_error("no model");
      ready = true;
      say("mono model " + settings.model + " loaded from " + settings.engine_path);
      if (pipeline.mono_settings().enabled) pipeline.set_mono_state("ok");
    } catch (const std::exception & e) {
      load_failed = true;
      pipeline.set_mono_state(std::string("error: ") + e.what());
      say(std::string("mono disabled, the model did not load: ") + e.what());
    }
  }

  void run()
  {
    MonoStats s;
    try {
      model->infer(grey, grey.size(), inv, low.stream());
      inv.download(inv_host, low.stream());
      low.stream().waitForCompletion();
    } catch (const std::exception & e) {
      try {
        low.stream().waitForCompletion();
      } catch (const std::exception &) {
      }
      if (!infer_error_logged) say(std::string("mono inference failed: ") + e.what());
      infer_error_logged = true;
      pipeline.set_mono_state(std::string("error: inference: ") + e.what());
      return;
    }
    if (infer_error_logged) {
      infer_error_logged = false;
      pipeline.set_mono_state("ok");
    }
    const FillResult r = align_and_fill(inv_host.createMatHeader(), depth_host.createMatHeader(), align, fused);
    const int64_t now = steady_ns();
    const double ms = static_cast<double>(now - handed_ns) / 1e6;
    metrics.observe_stage_ms("mono", ms);
    metrics.record_mono_fit(r.fit.ok, r.fit.inliers, r.fill_ratio());
    bool published = false;
    if (r.fit.ok) {
      wire::Header h = header;
      h.stage_mono_ms = static_cast<float>(ms);
      published = pipeline.publish_fused(h, fused);
    }
    const std::lock_guard<std::mutex> lock(stats_mutex);
    rate.tick(now);
    ++stats.runs;
    if (published) ++stats.published;
    if (!r.fit.ok) ++stats.no_fit;
    stats.last_inliers = r.fit.inliers;
    stats.last_fill_ratio = r.fill_ratio();
    stats.last_ms = ms;
    stats.fps = rate.rate(now);
  }

  void worker_loop()
  {
    for (;;) {
      bool have_job = false;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait_for(lock, std::chrono::milliseconds(kWorkerPollMs), [&] { return stopping || job; });
        if (stopping) return;
        have_job = job;
        job = false;
      }
      const Mono settings = pipeline.mono_settings();
      if (settings.enabled && !model && !load_failed) load(settings);
      // Re-enabled live: the pipeline reports "unavailable" until the stage is serving again.
      if (settings.enabled && ready.load() && !infer_error_logged &&
          pipeline.health_inputs().mono_state == "unavailable") {
        pipeline.set_mono_state("ok");
      }
      if (have_job) run();
      {
        const std::lock_guard<std::mutex> lock(mutex);
        if (have_job) busy = false;
      }
      double fps = 0.0;
      {
        const std::lock_guard<std::mutex> lock(stats_mutex);
        fps = stats.fps = rate.rate(steady_ns());
      }
      metrics.set_mono_fps(fps);
    }
  }
};

MonoStage::MonoStage(
  const Config & config, Pipeline & pipeline, Metrics & metrics, MonoModelFactory factory, LogFn log)
: impl_(std::make_unique<Impl>(config, pipeline, metrics, std::move(factory), std::move(log)))
{
}

MonoStage::~MonoStage() { stop(); }

void MonoStage::start()
{
  Impl & im = *impl_;
  {
    const std::lock_guard<std::mutex> lock(im.mutex);
    if (im.started) return;
    im.started = true;
    im.stopping = false;
  }
  im.pipeline.set_depth_listener([&im](const DepthFrame & f) { im.on_depth(f); });
  im.worker = std::thread([&im] { im.worker_loop(); });
}

void MonoStage::stop()
{
  Impl & im = *impl_;
  {
    const std::lock_guard<std::mutex> lock(im.mutex);
    if (!im.started) return;
    im.started = false;
    im.stopping = true;
  }
  im.pipeline.set_depth_listener(nullptr);
  im.cv.notify_all();
  if (im.worker.joinable()) im.worker.join();
}

MonoStats MonoStage::stats() const
{
  const std::lock_guard<std::mutex> lock(impl_->stats_mutex);
  return impl_->stats;
}

}  // namespace depthd
