// depthd HTTP API (cpp-httplib) on transport.bind_address:transport.http_port, and the Prometheus metrics it serves.
//
//   GET  /health               JSON state of the pipeline (fed by HttpApiHooks::health)
//   GET  /config               the running config as JSON
//   PUT  /config               change runtime_mutable keys: 200 new config, 409 names a key that needs a restart,
//                              400 validation message; nothing changes on any error
//   GET  /calibration          status, reason and the loaded file's metadata
//   POST /calibration/reload   reload calibration.path: 200 new status, 422 reason
//   GET  /calibration/capture  latest unrectified pair as multipart/form-data (PNG parts), 503 when none is fresh
//   GET  /metrics              Prometheus text, depthd_* metrics
//
// The API owns the running Config and the calibration file state; the pipeline feeds it through HttpApiHooks and
// Metrics, so T15 (pipeline), T20 (calibration tool) and T28 (bridge metrics) share one definition.
#ifndef DEPTHD_HTTP_API_HPP_
#define DEPTHD_HTTP_API_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <thread>

#include "depthd/calibration.hpp"
#include "depthd/config.hpp"
#include "depthd/pairing.hpp"

namespace depthd
{

// Thread-safe counters, gauges and histograms rendered by GET /metrics.
class Metrics
{
public:
  // depthd_frames_total{kind}: messages produced, kind is "depth", "fused" or "rgb".
  void inc_frames(const std::string & kind);
  // depthd_drops_total{reason}: pipeline drops other than the pairing ones (e.g. "encode", "busy").
  void inc_drop(const std::string & reason);
  // Pairing counters: depthd_drops_total{reason="skew"|"orphan"}, depthd_pairs_total and depthd_pair_skew_ms.
  void set_pairing(const PairingStats & stats);
  // depthd_stage_time_ms{stage}: per-stage time histogram (capture, rectify, match, post, mono, encode, ...).
  void observe_stage_ms(const std::string & stage, double ms);
  void set_fps(double fps);
  void set_gpu_temp_c(double celsius);
  void set_nvgpu_errors(uint64_t total);
  void set_zmq_send_drops(uint64_t total);
  // Monocular stage: depthd_mono_fps, and after the first fit attempt depthd_mono_fit_inliers and
  // depthd_mono_fill_ratio (filled share of the stereo holes); depthd_mono_fits_total{result="ok"|"rejected"}.
  void set_mono_fps(double fps);
  void record_mono_fit(bool ok, int inliers, double fill_ratio);

  std::string render() const;  // Prometheus text exposition format 0.0.4

  static constexpr std::size_t kStageBuckets = 9;
  static constexpr double kStageBoundsMs[kStageBuckets] = {1, 2, 5, 10, 20, 50, 100, 200, 500};

private:
  struct StageHistogram
  {
    uint64_t counts[kStageBuckets + 1] = {};  // per bucket, the last is above the last bound
    uint64_t count = 0;
    double sum_ms = 0.0;
  };

  mutable std::mutex mutex_;
  std::map<std::string, uint64_t> frames_;
  std::map<std::string, uint64_t> drops_;
  std::map<std::string, StageHistogram> stages_;
  PairingStats pairing_;
  double fps_ = 0.0;
  double gpu_temp_c_ = 0.0;
  uint64_t nvgpu_errors_ = 0;
  uint64_t zmq_send_drops_ = 0;
  double mono_fps_ = 0.0;
  bool mono_fit_seen_ = false;
  int mono_inliers_ = 0;
  double mono_fill_ratio_ = 0.0;
  uint64_t mono_fits_ok_ = 0;
  uint64_t mono_fits_rejected_ = 0;
};

// GET /health body. state is ok | uncalibrated | camera_error | starting.
struct Health
{
  std::string state = "starting";
  std::string reason;
  double fps = 0.0;
  double last_pair_age_ms = -1.0;  // -1: no pair seen yet
  uint64_t pairs_dropped_total = 0;
  double gpu_temp_c = 0.0;
  uint64_t nvgpu_errors_total = 0;
  std::string matcher;
  std::string mono_state;
};

// The latest unrectified pair, GRAY8 at processing size (CV_8UC1 cv::Mat on the CPU).
struct CapturedPair
{
  cv::Mat left;
  cv::Mat right;
  int64_t left_capture_realtime_ns = 0;
  int64_t right_capture_realtime_ns = 0;
};

// What the pipeline plugs in. Every hook is optional; the endpoint answers with a neutral default without it.
struct HttpApiHooks
{
  std::function<Health()> health;
  // Latest unrectified pair, or nullopt when there is no fresh one (the endpoint answers 503).
  std::function<std::optional<CapturedPair>()> capture_pair;
  // Called with the validated new config before it becomes the running one; a non-empty return rejects the change
  // (400 with that message) and nothing is applied.
  std::function<std::string(const Config &)> apply_config;
  // Called with every successfully (re)loaded calibration, including the one loaded at start.
  std::function<void(const StereoCalibration &)> on_calibration;
};

// GET /calibration body.
struct CalibrationStatus
{
  std::string status = "uncalibrated";  // ok | uncalibrated
  std::string reason;                   // why uncalibrated, or why the last reload failed
  std::optional<double> rms;
  std::optional<double> baseline_m;
  std::string last_reload_error;  // set when the last reload failed (an earlier calibration may still be active)
  std::string created_utc;  // the file's created_utc key when present, else its modification time (UTC ISO 8601)
  std::string path;
};

class HttpApi
{
public:
  // Loads the calibration file once (see calibration_status()). metrics must outlive the API.
  HttpApi(const Config & config, Metrics & metrics, HttpApiHooks hooks = {});
  ~HttpApi();
  HttpApi(const HttpApi &) = delete;
  HttpApi & operator=(const HttpApi &) = delete;

  // Binds transport.bind_address:<port> and serves on a background thread. port defaults to transport.http_port;
  // pass 0 for any free port (tests; bound_port() then tells which). False when the bind fails.
  bool start(int port = -1);
  void stop();
  int bound_port() const { return port_; }

  Config config() const;
  CalibrationStatus calibration_status() const;
  // Reloads calibration.path. ok is true when the file is valid; on failure the previously active calibration (if
  // any) stays active and the reason is returned.
  struct ReloadResult
  {
    bool ok = false;
    CalibrationStatus status;
  };
  ReloadResult reload_calibration();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  int port_ = 0;
};

}  // namespace depthd

#endif  // DEPTHD_HTTP_API_HPP_
