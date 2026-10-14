#include "depthd/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace depthd
{
namespace
{
using json = nlohmann::json;

struct Failure : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

[[noreturn]] void fail(const std::string & key, const std::string & why) { throw Failure(key + ": " + why); }

// Reads one JSON object section, tracking which keys were consumed so leftovers are rejected as unknown.
class Section
{
public:
  Section(const json & node, std::string path) : path_(std::move(path))
  {
    if (!node.is_object()) {
      fail(path_.empty() ? "<root>" : path_, "expected a mapping");
    }
    node_ = &node;
  }

  std::string key(const std::string & name) const { return path_.empty() ? name : path_ + "." + name; }

  // Returns the child object section when present, else nullptr.
  const json * child(const std::string & name)
  {
    seen_.insert(name);
    const auto it = node_->find(name);
    return it == node_->end() ? nullptr : &*it;
  }

  void get(const std::string & name, int & out)
  {
    if (const json * v = child(name)) {
      if (!v->is_number_integer()) {
        fail(key(name), "expected an integer");
      }
      out = v->get<int>();
    }
  }
  void get(const std::string & name, double & out)
  {
    if (const json * v = child(name)) {
      if (!v->is_number()) {
        fail(key(name), "expected a number");
      }
      out = v->get<double>();
    }
  }
  void get(const std::string & name, bool & out)
  {
    if (const json * v = child(name)) {
      if (!v->is_boolean()) {
        fail(key(name), "expected true or false");
      }
      out = v->get<bool>();
    }
  }
  void get(const std::string & name, std::string & out)
  {
    if (const json * v = child(name)) {
      if (!v->is_string()) {
        fail(key(name), "expected a string");
      }
      out = v->get<std::string>();
    }
  }

  // Sub-section reader: calls fn(Section&) when the child exists.
  template <typename Fn>
  void sub(const std::string & name, Fn fn)
  {
    if (const json * v = child(name)) {
      Section s(*v, key(name));
      fn(s);
      s.finish();
    }
  }

  void finish() const
  {
    for (auto it = node_->begin(); it != node_->end(); ++it) {
      if (seen_.count(it.key()) == 0) {
        fail(key(it.key()), "unknown key");
      }
    }
  }

private:
  const json * node_;
  std::string path_;
  std::set<std::string> seen_;
};

void read_toggle(Section & s, bool & enabled) { s.get("enabled", enabled); }

Config read_config(const json & root)
{
  Config c;
  Section top(root, "");
  top.sub("capture", [&](Section & s) {
    s.get("left_sensor_id", c.capture.left_sensor_id);
    s.get("right_sensor_id", c.capture.right_sensor_id);
    s.sub("sensor_mode", [&](Section & m) {
      m.get("width", c.capture.sensor_mode.width);
      m.get("height", c.capture.sensor_mode.height);
      m.get("fps", c.capture.sensor_mode.fps);
    });
    s.get("settle_frames", c.capture.settle_frames);
    s.sub("exposure", [&](Section & e) {
      auto & x = c.capture.exposure;
      e.get("mode", x.mode);
      e.get("measure_interval_frames", x.measure_interval_frames);
      e.get("follow_threshold", x.follow_threshold);
      e.get("follow_settle_frames", x.follow_settle_frames);
      e.get("follow_transition_frames", x.follow_transition_frames);
      e.get("follow_starvation_s", x.follow_starvation_s);
      e.get("max_exposure_us", x.max_exposure_us);
      e.get("max_gain", x.max_gain);
      e.get("saturation_level", x.saturation_level);
      e.get("saturated_high", x.saturated_high);
      e.get("saturated_low", x.saturated_low);
      e.get("compensation_step", x.compensation_step);
      e.get("compensation_min", x.compensation_min);
      e.get("compensation_hold_frames", x.compensation_hold_frames);
    });
    s.get("pair_window_ms", c.capture.pair_window_ms);
    s.get("zero_copy", c.capture.zero_copy);
    s.get("realign_skew_ms", c.capture.realign_skew_ms);
    s.get("realign_window_s", c.capture.realign_window_s);
    s.get("realign_stall_s", c.capture.realign_stall_s);
    s.get("realign_cooldown_s", c.capture.realign_cooldown_s);
  });
  top.sub("processing", [&](Section & s) {
    auto & p = c.processing;
    s.get("width", p.width);
    s.get("height", p.height);
    s.get("matcher", p.matcher);
    s.get("num_disparities", p.num_disparities);
    s.get("p1", p.p1);
    s.get("p2", p.p2);
    s.get("uniqueness_ratio", p.uniqueness_ratio);
    s.get("min_depth_m", p.min_depth_m);
    s.get("max_depth_m", p.max_depth_m);
  });
  top.sub("postprocess", [&](Section & s) {
    auto & p = c.postprocess;
    s.sub("lr_check", [&](Section & t) {
      read_toggle(t, p.lr_check.enabled);
      t.get("max_diff", p.lr_check.max_diff);
    });
    s.sub("confidence", [&](Section & t) { read_toggle(t, p.confidence.enabled); });
    s.sub("speckle", [&](Section & t) {
      read_toggle(t, p.speckle.enabled);
      t.get("max_size", p.speckle.max_size);
      t.get("max_diff", p.speckle.max_diff);
    });
    s.sub("bilateral", [&](Section & t) { read_toggle(t, p.bilateral.enabled); });
    s.sub("hole_fill", [&](Section & t) {
      read_toggle(t, p.hole_fill.enabled);
      t.get("max_hole_px", p.hole_fill.max_hole_px);
    });
    s.sub("temporal", [&](Section & t) {
      read_toggle(t, p.temporal.enabled);
      t.get("alpha", p.temporal.alpha);
      t.get("motion_threshold", p.temporal.motion_threshold);
    });
    s.sub("wls", [&](Section & t) { read_toggle(t, p.wls.enabled); });
  });
  top.sub("mono", [&](Section & s) {
    s.get("enabled", c.mono.enabled);
    s.get("model", c.mono.model);
    s.get("engine_path", c.mono.engine_path);
    s.get("rate_hz", c.mono.rate_hz);
  });
  top.sub("rgb", [&](Section & s) {
    s.get("enabled", c.rgb.enabled);
    s.get("width", c.rgb.width);
    s.get("height", c.rgb.height);
    s.get("jpeg_quality", c.rgb.jpeg_quality);
    s.get("fps", c.rgb.fps);
  });
  top.sub("transport", [&](Section & s) {
    s.get("bind_address", c.transport.bind_address);
    s.get("pub_port", c.transport.pub_port);
    s.get("http_port", c.transport.http_port);
    s.get("zstd_level", c.transport.zstd_level);
    s.get("send_hwm", c.transport.send_hwm);
  });
  top.sub("calibration", [&](Section & s) {
    s.get("path", c.calibration.path);
    s.get("max_rms_px", c.calibration.max_rms_px);
  });
  top.sub("frame_ids", [&](Section & s) {
    s.get("depth", c.frame_ids.depth);
    s.get("rgb", c.frame_ids.rgb);
  });
  top.sub("health", [&](Section & s) { s.get("degraded_after_s", c.health.degraded_after_s); });
  top.finish();
  return c;
}

template <typename T>
bool one_of(const T & v, std::initializer_list<T> set)
{
  return std::find(set.begin(), set.end(), v) != set.end();
}

void check(const Config & c)
{
  const auto positive = [](const char * key, double v) {
    if (!(v > 0.0)) {
      fail(key, "must be > 0");
    }
  };
  const auto div8 = [](const char * key, int v) {
    if (v <= 0 || v % 8 != 0) {
      fail(key, "must be a positive multiple of 8, got " + std::to_string(v));
    }
  };
  const auto port = [](const char * key, int v) {
    if (v < 1024 || v > 65535) {
      fail(key, "must be within 1024-65535, got " + std::to_string(v));
    }
  };
  const auto range = [](const char * key, double v, double lo, double hi) {
    if (!(v >= lo && v <= hi)) {
      fail(key, "must be within " + std::to_string(lo) + ".." + std::to_string(hi));
    }
  };
  const auto non_negative = [](const char * key, int v) {
    if (v < 0) {
      fail(key, "must be >= 0");
    }
  };
  const auto non_empty = [](const char * key, const std::string & v) {
    if (v.empty()) {
      fail(key, "must not be empty");
    }
  };

  non_negative("capture.left_sensor_id", c.capture.left_sensor_id);
  non_negative("capture.right_sensor_id", c.capture.right_sensor_id);
  if (c.capture.left_sensor_id == c.capture.right_sensor_id) {
    fail("capture.right_sensor_id", "must differ from capture.left_sensor_id");
  }
  positive("capture.sensor_mode.width", c.capture.sensor_mode.width);
  positive("capture.sensor_mode.height", c.capture.sensor_mode.height);
  positive("capture.sensor_mode.fps", c.capture.sensor_mode.fps);
  non_negative("capture.settle_frames", c.capture.settle_frames);
  positive("capture.pair_window_ms", c.capture.pair_window_ms);
  positive("capture.realign_skew_ms", c.capture.realign_skew_ms);
  positive("capture.realign_window_s", c.capture.realign_window_s);
  positive("capture.realign_stall_s", c.capture.realign_stall_s);
  if (!(c.capture.realign_cooldown_s >= 0.0)) {
    fail("capture.realign_cooldown_s", "must be >= 0");
  }
  const auto & x = c.capture.exposure;
  if (!one_of<std::string>(x.mode, {"follow", "lock_after_settle"})) {
    fail("capture.exposure.mode", "unknown mode '" + x.mode + "' (follow, lock_after_settle)");
  }
  range("capture.exposure.measure_interval_frames", x.measure_interval_frames, 1, 1000);
  if (!(x.follow_threshold > 0.0 && x.follow_threshold < 1.0)) {
    fail("capture.exposure.follow_threshold", "must be > 0 and < 1");
  }
  non_negative("capture.exposure.follow_settle_frames", x.follow_settle_frames);
  range("capture.exposure.follow_transition_frames", x.follow_transition_frames, 1, 100000);
  positive("capture.exposure.follow_starvation_s", x.follow_starvation_s);
  // Argus limits nvarguscamerasrc enforces on the IMX219: 34 us (the plugin rejects less) .. 683709 us, gain 1..10.625.
  range("capture.exposure.max_exposure_us", x.max_exposure_us, 34, 683709);
  range("capture.exposure.max_gain", x.max_gain, 1.0, 10.625);
  range("capture.exposure.saturation_level", x.saturation_level, 1, 255);
  range("capture.exposure.saturated_high", x.saturated_high, 0.0, 1.0);
  if (!(x.saturated_low >= 0.0 && x.saturated_low < x.saturated_high)) {
    fail("capture.exposure.saturated_low", "must be >= 0 and < capture.exposure.saturated_high");
  }
  if (!(x.compensation_step > 0.0 && x.compensation_step <= 2.0)) {
    fail("capture.exposure.compensation_step", "must be > 0 and <= 2");
  }
  range("capture.exposure.compensation_min", x.compensation_min, -2.0, 0.0);
  non_negative("capture.exposure.compensation_hold_frames", x.compensation_hold_frames);

  div8("processing.width", c.processing.width);
  div8("processing.height", c.processing.height);
  if (!one_of<std::string>(c.processing.matcher, {"cuda_sgm", "vpi", "libsgm"})) {
    fail("processing.matcher", "unknown matcher '" + c.processing.matcher + "' (cuda_sgm, vpi, libsgm)");
  }
  if (!one_of(c.processing.num_disparities, {64, 128, 256})) {
    fail("processing.num_disparities", "must be 64, 128 or 256, got " + std::to_string(c.processing.num_disparities));
  }
  non_negative("processing.p1", c.processing.p1);
  if (c.processing.p2 < c.processing.p1) {
    fail("processing.p2", "must be >= processing.p1");
  }
  range("processing.uniqueness_ratio", c.processing.uniqueness_ratio, 0, 100);
  positive("processing.min_depth_m", c.processing.min_depth_m);
  if (c.processing.max_depth_m <= c.processing.min_depth_m) {
    fail("processing.max_depth_m", "must be > processing.min_depth_m");
  }

  non_negative("postprocess.lr_check.max_diff", c.postprocess.lr_check.max_diff);
  non_negative("postprocess.speckle.max_size", c.postprocess.speckle.max_size);
  non_negative("postprocess.speckle.max_diff", c.postprocess.speckle.max_diff);
  non_negative("postprocess.hole_fill.max_hole_px", c.postprocess.hole_fill.max_hole_px);
  range("postprocess.temporal.alpha", c.postprocess.temporal.alpha, 0.0, 1.0);
  non_negative("postprocess.temporal.motion_threshold", c.postprocess.temporal.motion_threshold);

  if (!one_of<std::string>(c.mono.model, {"dav2_small", "midas_small"})) {
    fail("mono.model", "unknown model '" + c.mono.model + "' (dav2_small, midas_small)");
  }
  non_empty("mono.engine_path", c.mono.engine_path);
  positive("mono.rate_hz", c.mono.rate_hz);

  div8("rgb.width", c.rgb.width);
  div8("rgb.height", c.rgb.height);
  range("rgb.jpeg_quality", c.rgb.jpeg_quality, 1, 100);
  positive("rgb.fps", c.rgb.fps);

  non_empty("transport.bind_address", c.transport.bind_address);
  port("transport.pub_port", c.transport.pub_port);
  port("transport.http_port", c.transport.http_port);
  if (c.transport.pub_port == c.transport.http_port) {
    fail("transport.http_port", "must differ from transport.pub_port");
  }
  range("transport.zstd_level", c.transport.zstd_level, 1, 22);
  range("transport.send_hwm", c.transport.send_hwm, 1, 1000);

  non_empty("calibration.path", c.calibration.path);
  positive("calibration.max_rms_px", c.calibration.max_rms_px);

  non_empty("frame_ids.depth", c.frame_ids.depth);
  non_empty("frame_ids.rgb", c.frame_ids.rgb);

  positive("health.degraded_after_s", c.health.degraded_after_s);
}

json yaml_to_json(const YAML::Node & n)
{
  switch (n.Type()) {
    case YAML::NodeType::Null:
      return nullptr;
    case YAML::NodeType::Map: {
      json o = json::object();
      for (const auto & kv : n) {
        o[kv.first.as<std::string>()] = yaml_to_json(kv.second);
      }
      return o;
    }
    case YAML::NodeType::Sequence: {
      json a = json::array();
      for (const auto & e : n) {
        a.push_back(yaml_to_json(e));
      }
      return a;
    }
    default:
      break;
  }
  const std::string s = n.Scalar();
  if (n.Tag() == "!") {  // quoted scalar: always a string
    return s;
  }
  if (s == "true" || s == "false") {
    return s == "true";
  }
  try {
    size_t used = 0;
    const long long i = std::stoll(s, &used);
    if (used == s.size()) {
      return i;
    }
    used = 0;
    const double d = std::stod(s, &used);
    if (used == s.size() && std::isfinite(d)) {
      return d;
    }
  } catch (const std::exception &) {
  }
  try {
    size_t used = 0;
    const double d = std::stod(s, &used);
    if (used == s.size() && std::isfinite(d)) {
      return d;
    }
  } catch (const std::exception &) {
  }
  return s;
}

}  // namespace

std::string validate(const Config & config)
{
  try {
    check(config);
  } catch (const Failure & e) {
    return e.what();
  }
  return "";
}

Result from_json(const json & root)
{
  Result r;
  try {
    r.config = read_config(root);
    check(r.config);
  } catch (const Failure & e) {
    r.config = Config{};
    r.error = e.what();
  }
  return r;
}

Result parse_config(const std::string & yaml_text)
{
  YAML::Node doc;
  try {
    doc = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    return Result{Config{}, std::string("yaml: ") + e.what()};
  }
  if (doc.IsNull()) {
    return from_json(json::object());
  }
  try {
    return from_json(yaml_to_json(doc));
  } catch (const YAML::Exception & e) {
    return Result{Config{}, std::string("yaml: ") + e.what()};
  }
}

Result load_config(const std::string & path)
{
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path);
  } catch (const YAML::BadFile &) {
    return Result{Config{}, path + ": cannot read config file"};
  } catch (const YAML::Exception & e) {
    return Result{Config{}, path + ": yaml: " + e.what()};
  }
  Result r = doc.IsNull() ? from_json(json::object()) : from_json(yaml_to_json(doc));
  if (!r.ok()) {
    r.error = path + ": " + r.error;
  }
  return r;
}

json to_json(const Config & c)
{
  const auto toggle = [](bool enabled) { return json{{"enabled", enabled}}; };
  const auto & p = c.postprocess;
  return json{
    {"capture",
     {{"left_sensor_id", c.capture.left_sensor_id},
      {"right_sensor_id", c.capture.right_sensor_id},
      {"sensor_mode",
       {{"width", c.capture.sensor_mode.width},
        {"height", c.capture.sensor_mode.height},
        {"fps", c.capture.sensor_mode.fps}}},
      {"settle_frames", c.capture.settle_frames},
      {"exposure",
       {{"mode", c.capture.exposure.mode},
        {"measure_interval_frames", c.capture.exposure.measure_interval_frames},
        {"follow_threshold", c.capture.exposure.follow_threshold},
        {"follow_settle_frames", c.capture.exposure.follow_settle_frames},
        {"follow_transition_frames", c.capture.exposure.follow_transition_frames},
        {"follow_starvation_s", c.capture.exposure.follow_starvation_s},
        {"max_exposure_us", c.capture.exposure.max_exposure_us},
        {"max_gain", c.capture.exposure.max_gain},
        {"saturation_level", c.capture.exposure.saturation_level},
        {"saturated_high", c.capture.exposure.saturated_high},
        {"saturated_low", c.capture.exposure.saturated_low},
        {"compensation_step", c.capture.exposure.compensation_step},
        {"compensation_min", c.capture.exposure.compensation_min},
        {"compensation_hold_frames", c.capture.exposure.compensation_hold_frames}}},
      {"pair_window_ms", c.capture.pair_window_ms},
      {"zero_copy", c.capture.zero_copy},
      {"realign_skew_ms", c.capture.realign_skew_ms},
      {"realign_window_s", c.capture.realign_window_s},
      {"realign_stall_s", c.capture.realign_stall_s},
      {"realign_cooldown_s", c.capture.realign_cooldown_s}}},
    {"processing",
     {{"width", c.processing.width},
      {"height", c.processing.height},
      {"matcher", c.processing.matcher},
      {"num_disparities", c.processing.num_disparities},
      {"p1", c.processing.p1},
      {"p2", c.processing.p2},
      {"uniqueness_ratio", c.processing.uniqueness_ratio},
      {"min_depth_m", c.processing.min_depth_m},
      {"max_depth_m", c.processing.max_depth_m}}},
    {"postprocess",
     {{"lr_check", {{"enabled", p.lr_check.enabled}, {"max_diff", p.lr_check.max_diff}}},
      {"confidence", toggle(p.confidence.enabled)},
      {"speckle", {{"enabled", p.speckle.enabled}, {"max_size", p.speckle.max_size}, {"max_diff", p.speckle.max_diff}}},
      {"bilateral", toggle(p.bilateral.enabled)},
      {"hole_fill", {{"enabled", p.hole_fill.enabled}, {"max_hole_px", p.hole_fill.max_hole_px}}},
      {"temporal",
       {{"enabled", p.temporal.enabled},
        {"alpha", p.temporal.alpha},
        {"motion_threshold", p.temporal.motion_threshold}}},
      {"wls", toggle(p.wls.enabled)}}},
    {"mono",
     {{"enabled", c.mono.enabled},
      {"model", c.mono.model},
      {"engine_path", c.mono.engine_path},
      {"rate_hz", c.mono.rate_hz}}},
    {"rgb",
     {{"enabled", c.rgb.enabled},
      {"width", c.rgb.width},
      {"height", c.rgb.height},
      {"jpeg_quality", c.rgb.jpeg_quality},
      {"fps", c.rgb.fps}}},
    {"transport",
     {{"bind_address", c.transport.bind_address},
      {"pub_port", c.transport.pub_port},
      {"http_port", c.transport.http_port},
      {"zstd_level", c.transport.zstd_level},
      {"send_hwm", c.transport.send_hwm}}},
    {"calibration", {{"path", c.calibration.path}, {"max_rms_px", c.calibration.max_rms_px}}},
    {"frame_ids", {{"depth", c.frame_ids.depth}, {"rgb", c.frame_ids.rgb}}},
    {"health", {{"degraded_after_s", c.health.degraded_after_s}}}};
}

bool runtime_mutable(const std::string & key)
{
  static const std::string kPostprocess = "postprocess.";
  if (key.size() > kPostprocess.size() && key.compare(0, kPostprocess.size(), kPostprocess) == 0) {
    return true;
  }
  return key == "mono.enabled" || key == "mono.rate_hz" || key == "rgb.jpeg_quality";
}

}  // namespace depthd
