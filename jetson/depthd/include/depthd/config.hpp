// depthd configuration: the one schema every stage, the HTTP API and Ansible share.
// config/depthd.yaml is the documented example and equals the struct defaults.
#ifndef DEPTHD_CONFIG_HPP_
#define DEPTHD_CONFIG_HPP_

#include <nlohmann/json.hpp>
#include <string>

namespace depthd
{

struct SensorMode
{
  int width = 1640;
  int height = 1232;
  int fps = 30;
  bool operator==(const SensorMode & o) const { return width == o.width && height == o.height && fps == o.fps; }
};

// capture.exposure: continuous AE with the right camera following the left (exposure_control.hpp), or the old
// settle-then-lock mode.
struct ExposureConfig
{
  std::string mode = "follow";  // follow | lock_after_settle
  int measure_interval_frames = 2;
  double follow_threshold = 0.05;
  int follow_settle_frames = 2;
  int follow_transition_frames = 30;  // right frames a re-pin withholds pairs for at most
  double follow_starvation_s = 10.0;  // right not converged this long: fall back to lock_after_settle
  int max_exposure_us = 20000;
  double max_gain = 8.0;
  int saturation_level = 250;
  double saturated_high = 0.02;
  double saturated_low = 0.005;
  double compensation_step = 0.25;
  double compensation_min = -2.0;
  int compensation_hold_frames = 15;
  bool operator==(const ExposureConfig & o) const
  {
    return mode == o.mode && measure_interval_frames == o.measure_interval_frames &&
           follow_threshold == o.follow_threshold && follow_settle_frames == o.follow_settle_frames &&
           follow_transition_frames == o.follow_transition_frames && follow_starvation_s == o.follow_starvation_s &&
           max_exposure_us == o.max_exposure_us && max_gain == o.max_gain && saturation_level == o.saturation_level &&
           saturated_high == o.saturated_high && saturated_low == o.saturated_low &&
           compensation_step == o.compensation_step && compensation_min == o.compensation_min &&
           compensation_hold_frames == o.compensation_hold_frames;
  }
};

struct Capture
{
  int left_sensor_id = 1;
  int right_sensor_id = 0;
  SensorMode sensor_mode;
  int settle_frames = 30;  // exposure.mode lock_after_settle: frames before aelock/awblock
  ExposureConfig exposure;
  double pair_window_ms = 8.0;
  bool zero_copy = true;
  // Runtime phase re-alignment (PhaseDriftMonitor): re-run the right-camera alignment when the median pair skew of the
  // trailing realign_window_s exceeds realign_skew_ms, or when no pair came for realign_stall_s; at most once per
  // realign_cooldown_s.
  double realign_skew_ms = 4.0;
  double realign_window_s = 10.0;
  double realign_stall_s = 5.0;
  double realign_cooldown_s = 30.0;
  bool operator==(const Capture & o) const
  {
    return left_sensor_id == o.left_sensor_id && right_sensor_id == o.right_sensor_id && sensor_mode == o.sensor_mode &&
           settle_frames == o.settle_frames && exposure == o.exposure &&
           pair_window_ms == o.pair_window_ms && zero_copy == o.zero_copy && realign_skew_ms == o.realign_skew_ms &&
           realign_window_s == o.realign_window_s && realign_stall_s == o.realign_stall_s &&
           realign_cooldown_s == o.realign_cooldown_s;
  }
};

struct Processing
{
  int width = 640;
  int height = 480;
  std::string matcher = "cuda_sgm";  // cuda_sgm | vpi | libsgm
  int num_disparities = 128;         // 64 | 128 | 256
  int p1 = 10;
  int p2 = 120;
  int uniqueness_ratio = 5;
  double min_depth_m = 0.2;
  double max_depth_m = 6.0;
  bool operator==(const Processing & o) const
  {
    return width == o.width && height == o.height && matcher == o.matcher && num_disparities == o.num_disparities &&
           p1 == o.p1 && p2 == o.p2 && uniqueness_ratio == o.uniqueness_ratio && min_depth_m == o.min_depth_m &&
           max_depth_m == o.max_depth_m;
  }
};

struct Toggle
{
  bool enabled = true;
  bool operator==(const Toggle & o) const { return enabled == o.enabled; }
};

struct LrCheck
{
  bool enabled = true;
  int max_diff = 1;
  bool operator==(const LrCheck & o) const { return enabled == o.enabled && max_diff == o.max_diff; }
};

struct Speckle
{
  bool enabled = true;
  int max_size = 200;
  int max_diff = 1;
  bool operator==(const Speckle & o) const
  {
    return enabled == o.enabled && max_size == o.max_size && max_diff == o.max_diff;
  }
};

struct HoleFill
{
  bool enabled = true;
  int max_hole_px = 64;
  bool operator==(const HoleFill & o) const { return enabled == o.enabled && max_hole_px == o.max_hole_px; }
};

struct Temporal
{
  bool enabled = true;
  double alpha = 0.4;
  int motion_threshold = 12;
  bool operator==(const Temporal & o) const
  {
    return enabled == o.enabled && alpha == o.alpha && motion_threshold == o.motion_threshold;
  }
};

struct Postprocess
{
  LrCheck lr_check;
  Toggle confidence;
  Speckle speckle;
  Toggle bilateral;
  HoleFill hole_fill;
  Temporal temporal;
  Toggle wls{false};
  bool operator==(const Postprocess & o) const
  {
    return lr_check == o.lr_check && confidence == o.confidence && speckle == o.speckle && bilateral == o.bilateral &&
           hole_fill == o.hole_fill && temporal == o.temporal && wls == o.wls;
  }
};

struct Mono
{
  bool enabled = true;
  std::string model = "dav2_small";  // dav2_small | midas_small
  std::string engine_path = "/var/lib/depthd/models/dav2_small_266.engine";
  double rate_hz = 2.0;
  bool operator==(const Mono & o) const
  {
    return enabled == o.enabled && model == o.model && engine_path == o.engine_path && rate_hz == o.rate_hz;
  }
};

struct Rgb
{
  bool enabled = true;
  int width = 640;
  int height = 480;
  int jpeg_quality = 80;
  double fps = 10.0;
  bool operator==(const Rgb & o) const
  {
    return enabled == o.enabled && width == o.width && height == o.height && jpeg_quality == o.jpeg_quality &&
           fps == o.fps;
  }
};

struct Transport
{
  std::string bind_address = "10.77.0.2";
  int pub_port = 5550;
  int http_port = 8090;
  int zstd_level = 1;
  int send_hwm = 2;
  bool operator==(const Transport & o) const
  {
    return bind_address == o.bind_address && pub_port == o.pub_port && http_port == o.http_port &&
           zstd_level == o.zstd_level && send_hwm == o.send_hwm;
  }
};

struct Calibration
{
  std::string path = "/var/lib/depthd/stereo.yaml";
  double max_rms_px = 0.5;
  bool operator==(const Calibration & o) const { return path == o.path && max_rms_px == o.max_rms_px; }
};

struct FrameIds
{
  std::string depth = "stereo_left_optical_frame";
  std::string rgb = "overview_camera_optical_frame";
  bool operator==(const FrameIds & o) const { return depth == o.depth && rgb == o.rgb; }
};

// health: when /health turns degraded.
struct HealthConfig
{
  double degraded_after_s = 5.0;  // calibrated, but no depth frame for longer than this
  bool operator==(const HealthConfig & o) const { return degraded_after_s == o.degraded_after_s; }
};

struct Config
{
  Capture capture;
  Processing processing;
  Postprocess postprocess;
  Mono mono;
  Rgb rgb;
  Transport transport;
  Calibration calibration;
  FrameIds frame_ids;
  HealthConfig health;
  bool operator==(const Config & o) const
  {
    return capture == o.capture && processing == o.processing && postprocess == o.postprocess && mono == o.mono &&
           rgb == o.rgb && transport == o.transport && calibration == o.calibration && frame_ids == o.frame_ids &&
           health == o.health;
  }
  bool operator!=(const Config & o) const { return !(*this == o); }
};

// Outcome of a load: config is valid only when error is empty. The error names the offending key (dotted path).
struct Result
{
  Config config;
  std::string error;
  bool ok() const { return error.empty(); }
};

// Empty string when valid, else the first violation naming its key.
std::string validate(const Config & config);

Result load_config(const std::string & path);
Result parse_config(const std::string & yaml_text);

nlohmann::json to_json(const Config & config);
// Missing keys keep their defaults; unknown keys, wrong types and invalid values are rejected.
Result from_json(const nlohmann::json & json);

// True for postprocess.*, mono.enabled, mono.rate_hz and rgb.jpeg_quality: the keys the HTTP API may change live.
bool runtime_mutable(const std::string & key);

}  // namespace depthd

#endif  // DEPTHD_CONFIG_HPP_
