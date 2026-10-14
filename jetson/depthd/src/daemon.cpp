#include "depthd/daemon.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <utility>

#include "depthd/publisher.hpp"

namespace depthd
{

int check_config(const std::string & path, std::string & message)
{
  const Result r = load_config(path);
  if (!r.ok()) {
    message = "invalid config: " + r.error;
    return 1;
  }
  message = "ok: " + path;
  return 0;
}

struct Daemon::Impl
{
  DaemonOptions opt;
  Config config;
  std::string config_error;
  Metrics metrics;
  std::unique_ptr<FrameSource> source;
  std::unique_ptr<Pipeline> pipeline;
  std::unique_ptr<HttpApi> api;
  std::unique_ptr<Publisher> publisher;

  mutable std::mutex mutex;
  std::condition_variable cv;
  bool stopping = false;
  bool started = false;
  std::thread supervisor;
  std::atomic<int> http_port{0};
  std::atomic<int> pub_port{0};
  std::string http_reason;  // guarded by mutex
  std::string pub_reason;
  std::string source_reason;

  explicit Impl(DaemonOptions o) : opt(std::move(o)) {}

  void log(const std::string & line) const
  {
    if (opt.log) opt.log(line);
  }

  void build()
  {
    const Result loaded = load_config(opt.config_path);
    if (loaded.ok()) {
      config = loaded.config;
    } else {
      config_error = "config error: " + loaded.error;
      log(config_error + " (serving /health only until the file is fixed and depthd restarted)");
    }
    if (!opt.bind_address.empty()) config.transport.bind_address = opt.bind_address;

    if (config_error.empty()) {
      try {
        source = opt.make_source ? opt.make_source(config, opt.log)
                                 : std::make_unique<StereoCaptureSource>(config, opt.log);
        PipelineOptions po = opt.pipeline;
        if (!po.log) po.log = opt.log;
        pipeline = std::make_unique<Pipeline>(config, *source, metrics, po);
      } catch (const std::exception & e) {
        config_error = std::string("processing.matcher: ") + e.what();
        log(config_error);
        pipeline.reset();
        source.reset();
      }
    }

    HttpApiHooks hooks;
    hooks.health = [this] { return health(); };
    if (pipeline) {
      Pipeline * p = pipeline.get();
      hooks.capture_pair = [p] { return p->latest_pair(); };
      hooks.apply_config = [p](const Config & c) { return p->apply_config(c); };
      hooks.on_calibration = [p](const StereoCalibration & cal) { p->set_calibration(cal); };
    }
    api = std::make_unique<HttpApi>(config, metrics, hooks);
    if (pipeline && !pipeline->calibrated()) {
      log("uncalibrated: " + api->calibration_status().reason + " (rgb only, no depth)");
    }
  }

  // Binds what is not bound yet and starts the pipeline; retried until everything runs or stop().
  void supervise()
  {
    auto last_source_try = std::chrono::steady_clock::now() - std::chrono::hours(1);
    bool pipeline_started = false;
    for (;;) {
      if (http_port.load() == 0) {
        if (api->start(opt.http_port)) {
          http_port = api->bound_port();
          log("HTTP API on " + config.transport.bind_address + ":" + std::to_string(http_port.load()));
          set_reason(http_reason, "");
        } else {
          set_reason(
            http_reason, "binding the HTTP API on " + config.transport.bind_address + ":" +
                           std::to_string(opt.http_port < 0 ? config.transport.http_port : opt.http_port) +
                           " failed (retrying)");
        }
      }
      if (pipeline && !publisher) {
        try {
          Transport transport = config.transport;
          if (opt.pub_port >= 0) transport.pub_port = opt.pub_port;  // 0 (any port) is not a valid config value
          publisher = std::make_unique<Publisher>(transport);
          pub_port = publisher->bound_port();
          log("ZeroMQ PUB on " + config.transport.bind_address + ":" + std::to_string(pub_port.load()));
          set_reason(pub_reason, "");
        } catch (const std::exception & e) {
          set_reason(pub_reason, std::string(e.what()) + " (retrying)");
        }
      }
      if (pipeline && publisher) {
        const auto now = std::chrono::steady_clock::now();
        if (!pipeline_started) {
          std::string err;
          last_source_try = now;
          pipeline_started = true;
          set_reason(source_reason, pipeline->start(*publisher, &err) ? "" : err);
        } else if (!pipeline->source_running() &&
                   now - last_source_try >= std::chrono::milliseconds(opt.source_retry_interval_ms)) {
          std::string err;
          last_source_try = now;
          set_reason(source_reason, pipeline->start_source(&err) ? "" : err);
        }
      }
      const bool done = http_port.load() != 0 && (!pipeline || (publisher && pipeline->source_running()));
      std::unique_lock<std::mutex> lock(mutex);
      const std::chrono::milliseconds wait =
        done ? std::chrono::milliseconds(std::chrono::hours(24))
             : std::chrono::milliseconds(std::min(opt.retry_interval_ms, opt.source_retry_interval_ms));
      if (cv.wait_for(lock, wait, [&] { return stopping; })) return;
    }
  }

  void set_reason(std::string & field, const std::string & reason)
  {
    bool changed = false;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      changed = field != reason;
      field = reason;
    }
    if (changed && !reason.empty()) log(reason);
  }

  Health health() const
  {
    HealthInputs in;
    if (pipeline) in = pipeline->health_inputs();
    in.config_error = config_error;
    if (!pipeline) {
      in.capture.state = CaptureState::starting;
      in.capture.reason = "not started";
    }
    if (!in.calibrated && api) in.calibration_reason = api->calibration_status().reason;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      for (const std::string * r : {&http_reason, &pub_reason}) {
        if (!r->empty() && in.startup_reason.empty()) in.startup_reason = *r;
      }
      // A camera that could not be started reports camera_error through the source status; this covers sources
      // whose status does not say so.
      if (!source_reason.empty() && in.capture.state != CaptureState::camera_error) {
        in.capture.state = CaptureState::camera_error;
        in.capture.reason = source_reason;
      }
    }
    return compose_health(in);
  }
};

Daemon::Daemon(DaemonOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}

Daemon::~Daemon() { stop(); }

void Daemon::start()
{
  Impl & im = *impl_;
  {
    const std::lock_guard<std::mutex> lock(im.mutex);
    if (im.started) return;
    im.started = true;
  }
  im.build();
  im.supervisor = std::thread([&im] { im.supervise(); });
}

void Daemon::stop()
{
  Impl & im = *impl_;
  {
    const std::lock_guard<std::mutex> lock(im.mutex);
    if (!im.started || im.stopping) return;
    im.stopping = true;
  }
  im.cv.notify_all();
  if (im.supervisor.joinable()) im.supervisor.join();
  if (im.pipeline) im.pipeline->stop();
  if (im.api) im.api->stop();
  im.log("stopped");
}

Health Daemon::health() const { return impl_->health(); }
int Daemon::http_port() const { return impl_->http_port.load(); }
int Daemon::pub_port() const { return impl_->pub_port.load(); }

}  // namespace depthd
