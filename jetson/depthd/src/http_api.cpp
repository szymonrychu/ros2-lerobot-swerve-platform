#include "depthd/http_api.hpp"

#include <sys/stat.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

#include "../third_party/httplib.h"

namespace depthd
{
using nlohmann::json;

constexpr double Metrics::kStageBoundsMs[Metrics::kStageBuckets];

namespace
{
// ---- Prometheus text ----
std::string num(double v)
{
  std::ostringstream o;
  o << std::setprecision(15) << v;
  return o.str();
}

void histogram_lines(
  std::ostringstream & o, const std::string & name, const std::string & labels, const double * bounds, std::size_t n,
  const uint64_t * cumulative, uint64_t count, double sum)
{
  const std::string sep = labels.empty() ? "" : labels + ",";
  for (std::size_t i = 0; i < n; ++i) {
    o << name << "_bucket{" << sep << "le=\"" << num(bounds[i]) << "\"} " << cumulative[i] << "\n";
  }
  o << name << "_bucket{" << sep << "le=\"+Inf\"} " << count << "\n";
  const std::string braces = labels.empty() ? "" : "{" + labels + "}";
  o << name << "_sum" << braces << " " << num(sum) << "\n";
  o << name << "_count" << braces << " " << count << "\n";
}

// ---- config helpers ----
void flatten(const std::string & prefix, const json & value, std::map<std::string, json> & out)
{
  if (value.is_object()) {
    for (auto it = value.begin(); it != value.end(); ++it) {
      flatten(prefix.empty() ? it.key() : prefix + "." + it.key(), it.value(), out);
    }
  } else {
    out[prefix] = value;
  }
}

// Sets root[a][b][c] for the dotted key; false when the path runs through a non-object (so the key cannot exist).
bool set_path(json & root, const std::string & dotted, const json & value)
{
  json * node = &root;
  std::size_t start = 0;
  for (;;) {
    const std::size_t dot = dotted.find('.', start);
    const std::string part = dotted.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
    if (!node->is_object()) {
      return false;
    }
    if (dot == std::string::npos) {
      (*node)[part] = value;
      return true;
    }
    node = &(*node)[part];  // creates a null for an unknown part; from_json then rejects the unknown key
    if (node->is_null()) {
      *node = json::object();
    }
    start = dot + 1;
  }
}

std::string utc_iso(std::time_t t)
{
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

std::string file_created_utc(const std::string & path)
{
  try {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (fs.isOpened()) {
      const cv::FileNode node = fs["created_utc"];
      if (node.isString()) {
        return static_cast<std::string>(node);
      }
    }
  } catch (const cv::Exception &) {
    // fall through to the modification time
  }
  struct stat st
  {
  };
  return stat(path.c_str(), &st) == 0 ? utc_iso(st.st_mtime) : std::string();
}

json to_json(const CalibrationStatus & s)
{
  return {
    {"status", s.status},
    {"reason", s.reason},
    {"rms", s.rms ? json(*s.rms) : json(nullptr)},
    {"baseline_m", s.baseline_m ? json(*s.baseline_m) : json(nullptr)},
    {"created_utc", s.created_utc.empty() ? json(nullptr) : json(s.created_utc)},
    {"path", s.path},
    {"last_reload_error", s.last_reload_error}};
}

void respond_json(httplib::Response & res, int status, const json & body)
{
  res.status = status;
  res.set_content(body.dump(), "application/json");
}

void respond_error(httplib::Response & res, int status, const std::string & message, const json & extra = json::object())
{
  json body = extra;
  body["error"] = message;
  respond_json(res, status, body);
}

bool encode_png(const cv::Mat & image, std::vector<uint8_t> & out)
{
  return image.type() == CV_8UC1 && !image.empty() && cv::imencode(".png", image, out);
}

void add_part(
  std::string & body, const std::string & boundary, const std::string & name, const std::string & content_type,
  const std::string & data, const std::string & filename = "")
{
  body += "--" + boundary + "\r\nContent-Disposition: form-data; name=\"" + name + "\"";
  if (!filename.empty()) {
    body += "; filename=\"" + filename + "\"";
  }
  body += "\r\nContent-Type: " + content_type + "\r\n\r\n" + data + "\r\n";
}
}  // namespace

// ---- Metrics ----
void Metrics::inc_frames(const std::string & kind)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  ++frames_[kind];
}

void Metrics::inc_drop(const std::string & reason)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  ++drops_[reason];
}

void Metrics::set_pairing(const PairingStats & stats)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  pairing_ = stats;
}

void Metrics::observe_stage_ms(const std::string & stage, double ms)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  StageHistogram & h = stages_[stage];
  std::size_t i = 0;
  while (i < kStageBuckets && ms > kStageBoundsMs[i]) {
    ++i;
  }
  ++h.counts[i];
  ++h.count;
  h.sum_ms += ms;
}

void Metrics::set_fps(double fps)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  fps_ = fps;
}

void Metrics::set_gpu_temp_c(double celsius)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  gpu_temp_c_ = celsius;
}

void Metrics::set_nvgpu_errors(uint64_t total)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  nvgpu_errors_ = total;
}

void Metrics::set_zmq_send_drops(uint64_t total)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  zmq_send_drops_ = total;
}

std::string Metrics::render() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream o;
  o << "# HELP depthd_frames_total Messages published, by kind.\n# TYPE depthd_frames_total counter\n";
  for (const char * kind : {"depth", "fused", "rgb"}) {
    const auto it = frames_.find(kind);
    o << "depthd_frames_total{kind=\"" << kind << "\"} " << (it == frames_.end() ? 0 : it->second) << "\n";
  }
  for (const auto & kv : frames_) {
    if (kv.first != "depth" && kv.first != "fused" && kv.first != "rgb") {
      o << "depthd_frames_total{kind=\"" << kv.first << "\"} " << kv.second << "\n";
    }
  }
  o << "# HELP depthd_pairs_total Stereo pairs emitted by the pairer.\n# TYPE depthd_pairs_total counter\n";
  o << "depthd_pairs_total " << pairing_.pairs_emitted << "\n";
  o << "# HELP depthd_drops_total Dropped frames and pairs, by reason.\n# TYPE depthd_drops_total counter\n";
  o << "depthd_drops_total{reason=\"skew\"} " << pairing_.dropped_skew << "\n";
  o << "depthd_drops_total{reason=\"orphan\"} " << pairing_.dropped_orphan << "\n";
  for (const auto & kv : drops_) {
    o << "depthd_drops_total{reason=\"" << kv.first << "\"} " << kv.second << "\n";
  }
  o << "# HELP depthd_pair_skew_ms Left/right capture skew of every candidate pair, milliseconds.\n"
       "# TYPE depthd_pair_skew_ms histogram\n";
  const auto cumulative = pairing_.skew.cumulative();
  histogram_lines(
    o, "depthd_pair_skew_ms", "", SkewHistogram::kBoundsMs.data(), SkewHistogram::kBuckets, cumulative.data(),
    pairing_.skew.count, pairing_.skew.sum_ms);
  o << "# HELP depthd_camera_realignments_total Runtime camera-phase re-alignments (right-camera restarts), by trigger.\n"
       "# TYPE depthd_camera_realignments_total counter\n";
  o << "depthd_camera_realignments_total{reason=\"skew\"} " << pairing_.realignments_skew << "\n";
  o << "depthd_camera_realignments_total{reason=\"stall\"} " << pairing_.realignments_stall << "\n";
  o << "# HELP depthd_stage_time_ms Time spent in each pipeline stage, milliseconds.\n"
       "# TYPE depthd_stage_time_ms histogram\n";
  for (const auto & kv : stages_) {
    uint64_t cum[kStageBuckets];
    uint64_t running = 0;
    for (std::size_t i = 0; i < kStageBuckets; ++i) {
      running += kv.second.counts[i];
      cum[i] = running;
    }
    histogram_lines(
      o, "depthd_stage_time_ms", "stage=\"" + kv.first + "\"", kStageBoundsMs, kStageBuckets, cum, kv.second.count,
      kv.second.sum_ms);
  }
  o << "# HELP depthd_fps Depth frames per second.\n# TYPE depthd_fps gauge\ndepthd_fps " << num(fps_) << "\n";
  o << "# HELP depthd_gpu_temp_celsius GPU temperature.\n# TYPE depthd_gpu_temp_celsius gauge\n"
    << "depthd_gpu_temp_celsius " << num(gpu_temp_c_) << "\n";
  o << "# HELP depthd_nvgpu_errors_total nvgpu kernel errors seen since boot.\n# TYPE depthd_nvgpu_errors_total counter\n"
    << "depthd_nvgpu_errors_total " << nvgpu_errors_ << "\n";
  o << "# HELP depthd_zmq_send_drops_total Messages dropped because the ZeroMQ send high-water mark was full.\n"
       "# TYPE depthd_zmq_send_drops_total counter\n"
    << "depthd_zmq_send_drops_total " << zmq_send_drops_ << "\n";
  return o.str();
}

// ---- HttpApi ----
struct HttpApi::Impl
{
  Metrics & metrics;
  HttpApiHooks hooks;
  mutable std::mutex mutex;
  Config config;
  bool calibrated = false;
  CalibrationStatus calibration;
  httplib::Server server;
  std::thread thread;
  std::atomic<bool> running{false};

  Impl(const Config & c, Metrics & m, HttpApiHooks h) : metrics(m), hooks(std::move(h)), config(c) {}

  HttpApi::ReloadResult reload()
  {
    Config cfg;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      cfg = config;
    }
    const CalibrationLoad load = load_calibration(
      cfg.calibration.path, cv::Size(cfg.processing.width, cfg.processing.height), cfg.calibration.max_rms_px);
    HttpApi::ReloadResult result;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (load.ok()) {
        calibrated = true;
        calibration = CalibrationStatus{};
        calibration.status = "ok";
        calibration.rms = load.calibration->rms;
        calibration.baseline_m = load.calibration->baseline_m;
        calibration.created_utc = file_created_utc(cfg.calibration.path);
        calibration.path = cfg.calibration.path;
      } else if (calibrated) {
        calibration.last_reload_error = load.reason;
      } else {
        calibration = CalibrationStatus{};
        calibration.status = "uncalibrated";
        calibration.reason = load.reason;
        calibration.path = cfg.calibration.path;
        calibration.last_reload_error = load.reason;
      }
      result.ok = load.ok();
      result.status = calibration;
    }
    if (load.ok() && hooks.on_calibration) {
      hooks.on_calibration(*load.calibration);
    }
    return result;
  }

  void handle_put_config(const httplib::Request & req, httplib::Response & res)
  {
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::exception & e) {
      respond_error(res, 400, std::string("body is not valid JSON: ") + e.what());
      return;
    }
    if (!body.is_object()) {
      respond_error(res, 400, "body must be a JSON object of config keys");
      return;
    }
    // One writer at a time, so two PUTs cannot interleave their validate/apply/store.
    const std::lock_guard<std::mutex> lock(mutex);
    json merged = to_json(config);
    std::map<std::string, json> changes;
    flatten("", body, changes);
    for (const auto & kv : changes) {
      if (!set_path(merged, kv.first, kv.second)) {
        respond_error(res, 400, "unknown config key " + kv.first);
        return;
      }
    }
    const Result parsed = from_json(merged);
    if (!parsed.ok()) {
      respond_error(res, 400, parsed.error);
      return;
    }
    std::map<std::string, json> before;
    std::map<std::string, json> after;
    flatten("", to_json(config), before);
    flatten("", to_json(parsed.config), after);
    json restart_keys = json::array();
    std::string joined;
    for (const auto & kv : after) {
      if (before[kv.first] != kv.second && !runtime_mutable(kv.first)) {
        restart_keys.push_back(kv.first);
        joined += (joined.empty() ? "" : ", ") + kv.first;
      }
    }
    if (!restart_keys.empty()) {
      respond_error(
        res, 409, joined + (restart_keys.size() == 1 ? " needs" : " need") + " a restart",
        {{"key", restart_keys.front()}, {"keys", restart_keys}});
      return;
    }
    if (hooks.apply_config) {
      const std::string rejected = hooks.apply_config(parsed.config);
      if (!rejected.empty()) {
        respond_error(res, 400, rejected);
        return;
      }
    }
    config = parsed.config;
    respond_json(res, 200, to_json(config));
  }

  void handle_capture(httplib::Response & res)
  {
    std::optional<CapturedPair> pair;
    if (hooks.capture_pair) {
      pair = hooks.capture_pair();
    }
    if (!pair) {
      respond_error(res, 503, "no fresh stereo pair", {{"reason", "no fresh stereo pair"}});
      return;
    }
    std::vector<uint8_t> left_png;
    std::vector<uint8_t> right_png;
    if (!encode_png(pair->left, left_png) || !encode_png(pair->right, right_png)) {
      respond_error(res, 500, "captured pair is not a non-empty 8-bit grey image");
      return;
    }
    const std::string boundary = "depthd-capture-boundary-7f3a9c";
    std::string body;
    add_part(body, boundary, "left", "image/png", std::string(left_png.begin(), left_png.end()), "left.png");
    add_part(body, boundary, "right", "image/png", std::string(right_png.begin(), right_png.end()), "right.png");
    add_part(body, boundary, "left_capture_realtime_ns", "text/plain", std::to_string(pair->left_capture_realtime_ns));
    add_part(body, boundary, "right_capture_realtime_ns", "text/plain", std::to_string(pair->right_capture_realtime_ns));
    body += "--" + boundary + "--\r\n";
    res.status = 200;
    res.set_content(body, "multipart/form-data; boundary=" + boundary);
  }

  void route()
  {
    server.Get("/health", [this](const httplib::Request &, httplib::Response & res) {
      const Health h = hooks.health ? hooks.health() : Health{};
      respond_json(
        res, 200,
        {{"state", h.state},
         {"reason", h.reason},
         {"fps", h.fps},
         {"last_pair_age_ms", h.last_pair_age_ms},
         {"pairs_dropped_total", h.pairs_dropped_total},
         {"gpu_temp_c", h.gpu_temp_c},
         {"nvgpu_errors_total", h.nvgpu_errors_total},
         {"matcher", h.matcher},
         {"mono_state", h.mono_state}});
    });
    server.Get("/config", [this](const httplib::Request &, httplib::Response & res) {
      const std::lock_guard<std::mutex> lock(mutex);
      respond_json(res, 200, to_json(config));
    });
    server.Put("/config", [this](const httplib::Request & req, httplib::Response & res) { handle_put_config(req, res); });
    server.Get("/calibration", [this](const httplib::Request &, httplib::Response & res) {
      const std::lock_guard<std::mutex> lock(mutex);
      respond_json(res, 200, to_json(calibration));
    });
    server.Post("/calibration/reload", [this](const httplib::Request &, httplib::Response & res) {
      const HttpApi::ReloadResult r = reload();
      json body = to_json(r.status);
      if (!r.ok) {
        body["reason"] = r.status.last_reload_error;
      }
      respond_json(res, r.ok ? 200 : 422, body);
    });
    server.Get("/calibration/capture", [this](const httplib::Request &, httplib::Response & res) { handle_capture(res); });
    server.Get("/metrics", [this](const httplib::Request &, httplib::Response & res) {
      res.status = 200;
      res.set_content(metrics.render(), "text/plain; version=0.0.4; charset=utf-8");
    });
  }
};

HttpApi::HttpApi(const Config & config, Metrics & metrics, HttpApiHooks hooks)
: impl_(std::make_unique<Impl>(config, metrics, std::move(hooks)))
{
  impl_->route();
  impl_->reload();
}

HttpApi::~HttpApi() { stop(); }

bool HttpApi::start(int port)
{
  if (impl_->running) {
    return true;
  }
  const Config cfg = config();
  const std::string & host = cfg.transport.bind_address;
  if (port < 0) {
    port = cfg.transport.http_port;
  }
  if (port == 0) {
    port_ = impl_->server.bind_to_any_port(host);
    if (port_ <= 0) {
      return false;
    }
  } else {
    if (!impl_->server.bind_to_port(host, port)) {
      return false;
    }
    port_ = port;
  }
  impl_->running = true;
  impl_->thread = std::thread([this] { impl_->server.listen_after_bind(); });
  // Server::stop() before listen_after_bind() has started is lost and the thread would serve forever.
  for (int i = 0; i < 1000 && !impl_->server.is_running(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return impl_->server.is_running();
}

void HttpApi::stop()
{
  if (!impl_->running.exchange(false)) {
    return;
  }
  impl_->server.stop();
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
}

Config HttpApi::config() const
{
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->config;
}

CalibrationStatus HttpApi::calibration_status() const
{
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->calibration;
}

HttpApi::ReloadResult HttpApi::reload_calibration() { return impl_->reload(); }

}  // namespace depthd
