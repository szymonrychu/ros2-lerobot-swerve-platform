// The 3D map status contract (see status.hpp).
#include "map3d_manager_cpp/status.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace map3d_manager_cpp
{

namespace
{

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr const char * STATE_MAPPING = "mapping";
constexpr const char * STATE_PAUSED = "paused";
constexpr const char * STATE_WAITING = "waiting_for_inputs";
constexpr const char * STATE_ERROR = "error";

bool is_fresh(const std::optional<double> & stamp, double now, double stale_s)
{
  return stamp.has_value() && now - *stamp <= stale_s;
}

nlohmann::json optional_json(const std::optional<double> & v)
{
  return v ? nlohmann::json(*v) : nlohmann::json(nullptr);
}

std::string describe(const StatusReport & s, const StatusInputs & in)
{
  std::ostringstream out;
  if (s.state == STATE_ERROR) {
    out << in.last_error;
  } else if (s.state == STATE_WAITING) {
    out << "waiting for inputs:";
    for (const auto & name : s.missing_inputs) {
      out << ' ' << name;
    }
  } else if (s.state == STATE_PAUSED) {
    out << "mapping paused";
  } else if (!s.nodes) {
    out << "mapping, no graph yet";
  } else if (s.correction_ok && !*s.correction_ok) {
    out << "3D map left slam_toolbox's frame: graph correction " << *s.graph_correction_m << " m, "
        << *s.graph_correction_deg << " deg";
  } else {
    out << "mapping, " << *s.nodes << " nodes";
  }
  return out.str();
}

}  // namespace

StatusReport build_status(const StatusInputs & in, const Config & config)
{
  StatusReport s;
  s.stamp_s = in.now_s;
  s.correction_warn_m = config.correction_warn_m;
  s.correction_warn_deg = config.correction_warn_deg;

  if (!is_fresh(in.rgb_info_stamp_s, in.now_s, config.input_stale_s)) {
    s.missing_inputs.push_back(INPUT_RGB_CAMERA_INFO);
  }
  if (!is_fresh(in.depth_info_stamp_s, in.now_s, config.input_stale_s)) {
    s.missing_inputs.push_back(INPUT_DEPTH_CAMERA_INFO);
  }
  if (!is_fresh(in.info_stamp_s, in.now_s, config.input_stale_s)) {
    s.missing_inputs.push_back(INPUT_RTABMAP_INFO);
  }

  const bool error =
    !in.last_error.empty() && in.last_error_stamp_s && in.now_s - *in.last_error_stamp_s <= ERROR_WINDOW_S;
  if (!in.paused && !s.missing_inputs.empty()) {
    s.state = STATE_WAITING;
  } else if (in.paused) {
    s.state = STATE_PAUSED;
  } else if (error) {
    s.state = STATE_ERROR;
  } else {
    s.state = STATE_MAPPING;
  }

  if (in.graph) {
    const GraphSample & g = *in.graph;
    s.nodes = g.nodes;
    s.graph_correction_m = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
    s.graph_correction_deg = std::abs(g.yaw_rad) * RAD_TO_DEG;
    s.correction_ok = *s.graph_correction_m <= config.correction_warn_m + 1e-9 &&
                      *s.graph_correction_deg <= config.correction_warn_deg + 1e-9;
    s.last_update_age_s = in.now_s - g.stamp_s;
  }
  if (in.db_size_bytes) {
    s.db_size_mb = static_cast<double>(*in.db_size_bytes) / BYTES_PER_MB;
  }
  if (in.memory_bytes) {
    s.memory_mb = static_cast<double>(*in.memory_bytes) / BYTES_PER_MB;
  }
  s.last_save = in.last_save;
  s.message = describe(s, in);
  return s;
}

std::string to_json(const StatusReport & s)
{
  using nlohmann::json;
  json j;
  j["stamp_s"] = s.stamp_s;
  j["state"] = s.state;
  j["nodes"] = s.nodes ? json(*s.nodes) : json(nullptr);
  j["db_size_mb"] = optional_json(s.db_size_mb);
  j["memory_mb"] = optional_json(s.memory_mb);
  j["graph_correction_m"] = optional_json(s.graph_correction_m);
  j["graph_correction_deg"] = optional_json(s.graph_correction_deg);
  j["correction_warn_m"] = s.correction_warn_m;
  j["correction_warn_deg"] = s.correction_warn_deg;
  j["correction_ok"] = s.correction_ok ? json(*s.correction_ok) : json(nullptr);
  j["missing_inputs"] = s.missing_inputs;
  j["last_update_age_s"] = optional_json(s.last_update_age_s);
  j["last_save"] = s.last_save ? json{{"name", s.last_save->name}, {"stamp_s", s.last_save->stamp_s}} : json(nullptr);
  j["message"] = s.message;
  return j.dump();
}

std::optional<std::uint64_t> read_cgroup_memory_bytes(const std::string & unit, const std::string & root)
{
  std::ifstream file(std::filesystem::path(root) / unit / "memory.current");
  std::uint64_t value = 0;
  if (!(file >> value)) {
    return std::nullopt;
  }
  return value;
}

}  // namespace map3d_manager_cpp
