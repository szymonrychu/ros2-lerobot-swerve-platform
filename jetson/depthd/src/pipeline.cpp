#include "depthd/pipeline.hpp"

#include <time.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <opencv2/core/cuda.hpp>
#include <thread>
#include <utility>

#include "depthd/depth.hpp"
#include "depthd/matcher.hpp"
#include "depthd/postprocess.hpp"
#include "depthd/rectify.hpp"

namespace depthd
{
namespace wire = ros2_common_cpp::depth_wire;

namespace
{
int64_t clock_realtime_ns()
{
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

template <std::size_t N>
std::array<double, N> flatten(const cv::Mat & m)
{
  std::array<double, N> out{};
  const cv::Mat d = m.reshape(1, 1);
  for (int i = 0; i < d.cols && static_cast<std::size_t>(i) < N; ++i) out[static_cast<std::size_t>(i)] = d.at<double>(0, i);
  return out;
}

// One frame between the match and the encode thread.
struct Slot
{
  std::vector<uint16_t> depth;
  wire::Header header;
};

// The calibrated state, swapped atomically: rectification maps and every header field derived from them.
struct CalibratedState
{
  std::shared_ptr<const Rectification> rect;
  double f_px = 0.0;
  double baseline_m = 0.0;
  std::array<double, 9> k{};   // unrectified left K at processing size
  std::array<double, 5> d{};   // left distortion, first five terms (k1 k2 p1 p2 k3)
  std::array<double, 12> p{};  // rectified left projection P1
  std::array<double, 9> rgb_k{};  // left K scaled to the rgb size
};
}  // namespace

struct Pipeline::Impl
{
  const Config config;
  FrameSource & source;
  Metrics & metrics;
  PipelineOptions opt;
  Publisher * publisher = nullptr;

  std::unique_ptr<StereoMatcher> matcher;
  std::unique_ptr<PostProcessor> post;
  std::string matcher_name;

  std::shared_ptr<const CalibratedState> cal;  // std::atomic_load / atomic_store

  // Threads and the slot hand-off.
  std::mutex mutex;
  std::condition_variable cv;
  bool started = false;
  bool stopping = false;
  std::atomic<bool> accepting{false};
  std::atomic<bool> source_up{false};
  std::optional<StereoPair> pending;
  std::vector<int> free_slots;
  std::deque<int> ready;
  std::array<Slot, Pipeline::kSlots> slots;
  std::size_t in_flight = 0;
  std::size_t in_flight_max = 0;
  std::thread match_thread;
  std::thread encode_thread;
  std::thread monitor_thread;

  // Match thread only: one CUDA stream and buffers reused for every pair.
  cv::cuda::Stream stream;
  cv::cuda::GpuMat rect_left;
  cv::cuda::GpuMat rect_right;
  cv::cuda::GpuMat depth_mm;
  std::array<cv::cuda::Event, 4> events;
  bool error_logged = false;

  std::mutex listener_mutex;
  DepthListener listener;

  mutable std::mutex latest_mutex;
  std::optional<StereoPair> latest;

  std::mutex rgb_mutex;
  std::vector<uint8_t> rgb_buffer;

  // Counters, rates and the values carried in every header.
  mutable std::mutex stats_mutex;
  PipelineCounters n;
  RateMeter depth_rate;
  int64_t last_depth_ns = 0;
  int64_t last_pair_ns = 0;
  float stage_capture_ms = 0.F;
  float stage_rectify_ms = 0.F;
  float stage_match_ms = 0.F;
  float stage_post_ms = 0.F;
  float stage_encode_ms = 0.F;
  double gpu_temp_c = 0.0;
  uint64_t nvgpu_errors = 0;
  uint64_t pairing_dropped = 0;
  Mono mono;
  std::string mono_state;

  Impl(const Config & c, FrameSource & s, Metrics & m, PipelineOptions o)
  : config(c), source(s), metrics(m), opt(std::move(o)), mono(c.mono)
  {
    if (!opt.realtime_now) opt.realtime_now = clock_realtime_ns;
    matcher = make_matcher(config);
    matcher_name = matcher->name();
    post = std::make_unique<PostProcessor>(config, matcher->supports_lr_check());
    mono_state = mono.enabled ? "unavailable" : "disabled";
    for (int i = Pipeline::kSlots - 1; i >= 0; --i) free_slots.push_back(i);
  }

  void log(const std::string & line) const
  {
    if (opt.log) opt.log(line);
  }
  int64_t now() const { return opt.realtime_now(); }
  std::shared_ptr<const CalibratedState> calibration() const { return std::atomic_load(&cal); }

  void count_drop(uint64_t PipelineCounters::*field, const char * reason)
  {
    {
      const std::lock_guard<std::mutex> lock(stats_mutex);
      ++(n.*field);
    }
    metrics.inc_drop(reason);
  }

  bool too_old(int64_t capture_ns) const
  {
    const int64_t age = now() - capture_ns;
    return age > opt.max_pair_age_ns || age < -opt.max_pair_age_ns;
  }

  // Health values carried in a header.
  void fill_health(wire::Header & h)
  {
    const std::lock_guard<std::mutex> lock(stats_mutex);
    h.pipeline_fps = static_cast<float>(depth_rate.rate(now()));
    h.stage_capture_ms = stage_capture_ms;
    h.stage_rectify_ms = stage_rectify_ms;
    h.stage_match_ms = stage_match_ms;
    h.stage_post_ms = stage_post_ms;
    h.stage_encode_ms = stage_encode_ms;
    h.pairs_dropped_total = dropped_total_locked();
    h.gpu_temp_c = static_cast<float>(gpu_temp_c);
    h.nvgpu_errors_total = nvgpu_errors;
  }

  uint64_t dropped_total_locked() const
  {
    return pairing_dropped + n.dropped_busy + n.dropped_stale + n.dropped_bad_frame + n.dropped_error + n.dropped_send;
  }

  // ---- source callbacks (capture threads): never block on the pipeline ----
  void on_pair(const StereoPair & pair)
  {
    if (!accepting.load()) return;
    {
      const std::lock_guard<std::mutex> lock(latest_mutex);
      latest = pair;
    }
    bool replaced = false;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      replaced = pending.has_value();
      pending = pair;
    }
    cv.notify_all();
    {
      const std::lock_guard<std::mutex> lock(stats_mutex);
      ++n.pairs_received;
      last_pair_ns = pair.capture_realtime_ns;
      if (replaced) ++n.dropped_busy;
    }
    if (replaced) metrics.inc_drop("busy");
  }

  void on_rgb(const RgbJpeg & jpeg)
  {
    if (!accepting.load() || !publisher || !jpeg.data || jpeg.size == 0) return;
    if (source.status().left_error) {
      count_drop(&PipelineCounters::rgb_dropped_camera_error, "rgb_camera_error");
      return;
    }
    if (too_old(jpeg.capture_realtime_ns)) {
      count_drop(&PipelineCounters::rgb_dropped_stale, "rgb_stale");
      return;
    }
    wire::Header h;
    h.kind = wire::Kind::kRgb;
    h.sequence = jpeg.sequence;
    h.capture_realtime_ns = jpeg.capture_realtime_ns;
    h.width = static_cast<uint32_t>(config.rgb.width);
    h.height = static_cast<uint32_t>(config.rgb.height);
    h.frame_id = config.frame_ids.rgb;
    if (const auto c = calibration()) {
      h.k = c->rgb_k;
      h.d = c->d;
      h.calibrated = true;
    }
    fill_health(h);
    bool sent = false;
    {
      const std::lock_guard<std::mutex> lock(rgb_mutex);
      rgb_buffer.assign(jpeg.data, jpeg.data + jpeg.size);
      sent = publisher->send_rgb(h, rgb_buffer);
    }
    if (sent) {
      {
        const std::lock_guard<std::mutex> lock(stats_mutex);
        ++n.rgb_published;
      }
      metrics.inc_frames("rgb");
    }
  }

  // ---- match thread ----
  void match_loop()
  {
    for (;;) {
      StereoPair pair;
      int slot = -1;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return stopping || (pending.has_value() && !free_slots.empty()); });
        if (stopping) return;
        pair = std::move(*pending);
        pending.reset();
        slot = free_slots.back();
        free_slots.pop_back();
        ++in_flight;
        in_flight_max = std::max(in_flight_max, in_flight);
      }
      const bool handed_on = process(pair, slots[static_cast<std::size_t>(slot)]);
      {
        const std::lock_guard<std::mutex> lock(mutex);
        if (handed_on) {
          ready.push_back(slot);
        } else {
          free_slots.push_back(slot);
          --in_flight;
        }
      }
      cv.notify_all();
    }
  }

  // Runs one pair through the GPU stages into `slot`. False when the pair was dropped.
  bool process(const StereoPair & pair, Slot & slot)
  {
    const CaptureStatus status = source.status();
    if (status.state == CaptureState::camera_error || status.left_error || status.right_error) {
      count_drop(&PipelineCounters::dropped_camera_error, "camera_error");
      return false;
    }
    const auto c = calibration();
    if (!c) {
      count_drop(&PipelineCounters::dropped_uncalibrated, "uncalibrated");
      return false;
    }
    const cv::Size size = c->rect->size();
    if (pair.left.type() != CV_8UC1 || pair.right.type() != CV_8UC1 || pair.left.size() != size ||
        pair.right.size() != size) {
      count_drop(&PipelineCounters::dropped_bad_frame, "bad_frame");
      return false;
    }
    // G5: the age is judged when matching starts, after waiting for a slot.
    const int64_t age_ns = now() - pair.capture_realtime_ns;
    if (age_ns > opt.max_pair_age_ns || age_ns < -opt.max_pair_age_ns) {
      count_drop(&PipelineCounters::dropped_stale, "stale");
      return false;
    }
    const std::size_t pixels = static_cast<std::size_t>(size.area());
    if (slot.depth.size() != pixels) slot.depth.resize(pixels);
    float rectify_ms = 0.F;
    float match_ms = 0.F;
    float post_ms = 0.F;
    try {
      events[0].record(stream);
      c->rect->rectify(pair.left, pair.right, rect_left, rect_right, stream);
      events[1].record(stream);
      // The right-referenced disparity first: it is copied out, so a matcher that reuses its output buffer cannot
      // overwrite it with the left one.
      cv::cuda::GpuMat right_disp;
      const bool need_right = post->needs_right_disparity();
      if (need_right) right_disp = compute_right_disparity(*matcher, rect_left, rect_right, stream);
      cv::cuda::GpuMat disp = matcher->compute(rect_left, rect_right, stream);
      events[2].record(stream);
      post->run(disp, rect_left, need_right ? &right_disp : nullptr, stream);
      disparity_to_depth_mm(
        disp, c->f_px, c->baseline_m, config.processing.min_depth_m, config.processing.max_depth_m, depth_mm, stream);
      cv::Mat host(size, CV_16UC1, slot.depth.data());
      depth_mm.download(host, stream);
      events[3].record(stream);
      stream.waitForCompletion();
      rectify_ms = cv::cuda::Event::elapsedTime(events[0], events[1]);
      match_ms = cv::cuda::Event::elapsedTime(events[1], events[2]);
      post_ms = cv::cuda::Event::elapsedTime(events[2], events[3]);
      for (const StageTiming & t : post->timings()) {
        if (t.ran) metrics.observe_stage_ms("post_" + t.name, t.ms);
      }
    } catch (const std::exception & e) {
      if (!error_logged) {
        log(std::string("dropping pairs: a GPU stage failed: ") + e.what());
        error_logged = true;
      }
      try {
        stream.waitForCompletion();
      } catch (const std::exception &) {
      }
      count_drop(&PipelineCounters::dropped_error, "error");
      return false;
    }
    error_logged = false;

    const float capture_ms = static_cast<float>(age_ns) / 1e6F;
    metrics.observe_stage_ms("capture", capture_ms);
    metrics.observe_stage_ms("rectify", rectify_ms);
    metrics.observe_stage_ms("match", match_ms);
    metrics.observe_stage_ms("post", post_ms);
    {
      const std::lock_guard<std::mutex> lock(stats_mutex);
      stage_capture_ms = capture_ms;
      stage_rectify_ms = rectify_ms;
      stage_match_ms = match_ms;
      stage_post_ms = post_ms;
    }

    wire::Header & h = slot.header;
    h = wire::Header{};
    h.kind = wire::Kind::kDepth;
    h.sequence = pair.left_sequence;
    h.capture_realtime_ns = pair.capture_realtime_ns;
    h.right_minus_left_ns = pair.right_minus_left_ns;
    h.width = static_cast<uint32_t>(size.width);
    h.height = static_cast<uint32_t>(size.height);
    h.frame_id = config.frame_ids.depth;
    h.k = c->k;
    h.d = c->d;
    h.p = c->p;
    h.baseline_m = c->baseline_m;
    h.calibrated = true;
    fill_health(h);

    DepthListener l;
    {
      const std::lock_guard<std::mutex> lock(listener_mutex);
      l = listener;
    }
    if (l) {
      try {
        l(DepthFrame{depth_mm, rect_left, h, stream});
      } catch (const std::exception & e) {
        log(std::string("depth listener failed: ") + e.what());
      }
    }
    return true;
  }

  // ---- encode thread: zstd + send ----
  void encode_loop()
  {
    for (;;) {
      int slot = -1;
      {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return stopping || !ready.empty(); });
        if (stopping) return;
        slot = ready.front();
        ready.pop_front();
      }
      Slot & s = slots[static_cast<std::size_t>(slot)];
      const auto t0 = std::chrono::steady_clock::now();
      const bool sent = publisher->send_depth(s.header, s.depth);
      const float encode_ms =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
      metrics.observe_stage_ms("encode", encode_ms);
      {
        const std::lock_guard<std::mutex> lock(stats_mutex);
        stage_encode_ms = encode_ms;
        if (sent) {
          ++n.depth_published;
          last_depth_ns = now();
          depth_rate.tick(last_depth_ns);
        } else {
          ++n.dropped_send;
        }
      }
      if (sent) metrics.inc_frames("depth");
      {
        const std::lock_guard<std::mutex> lock(mutex);
        free_slots.push_back(slot);
        --in_flight;
      }
      cv.notify_all();
    }
  }

  // ---- monitor thread: board health and metrics, once per interval ----
  void monitor_loop()
  {
    NvgpuErrorCounter nvgpu(opt.kmsg_path);
    if (!opt.kmsg_path.empty() && !nvgpu.available()) {
      log("nvgpu error counting unavailable: " + nvgpu.error());
    }
    for (;;) {
      const auto temp = read_gpu_temp_c(opt.thermal_root);
      const uint64_t errors = nvgpu.poll();
      const PairingStats pairing = source.pairing_stats();
      double fps = 0.0;
      {
        const std::lock_guard<std::mutex> lock(stats_mutex);
        if (temp) gpu_temp_c = *temp;
        if (errors > nvgpu_errors) log("nvgpu errors in the kernel log: " + std::to_string(errors));
        nvgpu_errors = errors;
        pairing_dropped = pairing.dropped_skew + pairing.dropped_orphan;
        fps = depth_rate.rate(now());
      }
      if (temp) metrics.set_gpu_temp_c(*temp);
      metrics.set_nvgpu_errors(errors);
      metrics.set_pairing(pairing);
      metrics.set_fps(fps);
      if (publisher) metrics.set_zmq_send_drops(publisher->send_drops());
      std::unique_lock<std::mutex> lock(mutex);
      if (cv.wait_for(lock, std::chrono::milliseconds(opt.monitor_interval_ms), [&] { return stopping; })) return;
    }
  }
};

Pipeline::Pipeline(const Config & config, FrameSource & source, Metrics & metrics, PipelineOptions options)
: impl_(std::make_unique<Impl>(config, source, metrics, std::move(options)))
{
  Impl & im = *impl_;
  source.set_pair_callback([&im](const StereoPair & pair) { im.on_pair(pair); });
  source.set_rgb_callback([&im](const RgbJpeg & jpeg) { im.on_rgb(jpeg); });
}

Pipeline::~Pipeline() { stop(); }

bool Pipeline::start(Publisher & publisher, std::string * error)
{
  Impl & im = *impl_;
  {
    const std::lock_guard<std::mutex> lock(im.mutex);
    if (im.started) return start_source(error);
    im.started = true;
    im.stopping = false;
    im.publisher = &publisher;
  }
  im.match_thread = std::thread([&im] { im.match_loop(); });
  im.encode_thread = std::thread([&im] { im.encode_loop(); });
  im.monitor_thread = std::thread([&im] { im.monitor_loop(); });
  im.accepting = true;
  return start_source(error);
}

bool Pipeline::start_source(std::string * error)
{
  Impl & im = *impl_;
  if (im.source_up.load()) return true;
  std::string err;
  if (!im.source.start(&err)) {
    if (error) *error = err;
    return false;
  }
  im.source_up = true;
  return true;
}

bool Pipeline::source_running() const { return impl_->source_up.load(); }

void Pipeline::stop()
{
  Impl & im = *impl_;
  im.accepting = false;
  if (im.source_up.exchange(false)) im.source.stop();
  {
    const std::lock_guard<std::mutex> lock(im.mutex);
    if (!im.started) return;
    im.stopping = true;
    im.started = false;
  }
  im.cv.notify_all();
  for (std::thread * t : {&im.match_thread, &im.encode_thread, &im.monitor_thread}) {
    if (t->joinable()) t->join();
  }
  const std::lock_guard<std::mutex> lock(im.mutex);
  im.pending.reset();
  im.ready.clear();
  im.free_slots.clear();
  for (int i = kSlots - 1; i >= 0; --i) im.free_slots.push_back(i);
  im.in_flight = 0;
}

void Pipeline::set_calibration(const StereoCalibration & calibration)
{
  auto state = std::make_shared<CalibratedState>();
  state->rect = std::make_shared<const Rectification>(calibration);
  state->f_px = calibration.P1.at<double>(0, 0);
  state->baseline_m = calibration.baseline_m;
  state->k = flatten<9>(calibration.K1);
  state->d = flatten<5>(calibration.D1);
  state->p = flatten<12>(calibration.P1);
  const Config & c = impl_->config;
  const cv::Size rgb_size(c.rgb.width, c.rgb.height);
  state->rgb_k = rgb_size == calibration.size ? state->k
                                              : flatten<9>(scale_calibration(calibration, rgb_size).K1);
  std::atomic_store(&impl_->cal, std::shared_ptr<const CalibratedState>(std::move(state)));
  impl_->log(
    "calibration active: rms " + std::to_string(calibration.rms) + " px, baseline " +
    std::to_string(calibration.baseline_m) + " m");
}

bool Pipeline::calibrated() const { return impl_->calibration() != nullptr; }

std::string Pipeline::apply_config(const Config & config)
{
  Impl & im = *impl_;
  im.post->set_postprocess(config.postprocess);
  im.source.set_jpeg_quality(config.rgb.jpeg_quality);
  const std::lock_guard<std::mutex> lock(im.stats_mutex);
  im.mono.enabled = config.mono.enabled;
  im.mono.rate_hz = config.mono.rate_hz;
  if (!im.mono.enabled) {
    im.mono_state = "disabled";
  } else if (im.mono_state == "disabled") {
    im.mono_state = "unavailable";
  }
  return "";
}

void Pipeline::set_mono_state(const std::string & state)
{
  const std::lock_guard<std::mutex> lock(impl_->stats_mutex);
  impl_->mono_state = state;
}

void Pipeline::set_depth_listener(DepthListener listener)
{
  const std::lock_guard<std::mutex> lock(impl_->listener_mutex);
  impl_->listener = std::move(listener);
}

bool Pipeline::publish_fused(wire::Header header, const std::vector<uint16_t> & depth_mm)
{
  Impl & im = *impl_;
  const auto c = im.calibration();
  const CaptureStatus status = im.source.status();
  const bool camera_ok = status.state != CaptureState::camera_error && !status.left_error && !status.right_error;
  if (!im.publisher || !im.accepting.load() || !c || !camera_ok || im.too_old(header.capture_realtime_ns)) {
    im.count_drop(&PipelineCounters::fused_rejected, "fused_rejected");
    return false;
  }
  header.kind = wire::Kind::kFused;
  header.frame_id = im.config.frame_ids.depth;
  header.k = c->k;
  header.d = c->d;
  header.p = c->p;
  header.baseline_m = c->baseline_m;
  header.calibrated = true;
  im.fill_health(header);
  if (!im.publisher->send_fused(header, depth_mm)) return false;
  {
    const std::lock_guard<std::mutex> lock(im.stats_mutex);
    ++im.n.fused_published;
  }
  im.metrics.inc_frames("fused");
  return true;
}

std::optional<CapturedPair> Pipeline::latest_pair() const
{
  const Impl & im = *impl_;
  std::optional<StereoPair> pair;
  {
    const std::lock_guard<std::mutex> lock(im.latest_mutex);
    pair = im.latest;
  }
  if (!pair || pair->left.empty() || pair->right.empty()) return std::nullopt;
  const int64_t age = im.now() - pair->capture_realtime_ns;
  if (age > im.opt.max_capture_pair_age_ns || age < -im.opt.max_capture_pair_age_ns) return std::nullopt;
  CapturedPair out;
  pair->left.download(out.left);
  pair->right.download(out.right);
  out.left_capture_realtime_ns = pair->capture_realtime_ns;
  out.right_capture_realtime_ns = pair->capture_realtime_ns + pair->right_minus_left_ns;
  return out;
}

HealthInputs Pipeline::health_inputs() const
{
  const Impl & im = *impl_;
  HealthInputs in;
  in.capture = im.source.status();
  in.calibrated = calibrated();
  in.matcher = im.matcher_name;
  in.degraded_after_ms = im.config.health.degraded_after_s * 1000.0;
  const int64_t now = im.now();
  const std::lock_guard<std::mutex> lock(im.stats_mutex);
  in.depth_published = im.n.depth_published > 0;
  in.last_depth_age_ms = im.last_depth_ns ? static_cast<double>(now - im.last_depth_ns) / 1e6 : -1.0;
  in.fps = im.depth_rate.rate(now);
  in.last_pair_age_ms = im.last_pair_ns ? static_cast<double>(now - im.last_pair_ns) / 1e6 : -1.0;
  in.pairs_dropped_total = im.dropped_total_locked();
  in.gpu_temp_c = im.gpu_temp_c;
  in.nvgpu_errors_total = im.nvgpu_errors;
  in.mono_state = im.mono_state;
  return in;
}

PipelineCounters Pipeline::counters() const
{
  const std::lock_guard<std::mutex> lock(impl_->stats_mutex);
  return impl_->n;
}

std::string Pipeline::matcher_name() const { return impl_->matcher_name; }

std::size_t Pipeline::in_flight_max() const
{
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->in_flight_max;
}

}  // namespace depthd
