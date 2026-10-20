#include "ground_estimator_cpp/state.hpp"

#include <cmath>

namespace ground_estimator_cpp
{

namespace
{

using ordered_json = nlohmann::ordered_json;

constexpr double DECIMAL_BASE = 10.0;
constexpr const char * PRIOR_IMU = "imu";
constexpr const char * PRIOR_BASE = "base_link";
constexpr const char * FRAME_BASE = "base_link";

// Round to JSON_DECIMALS; never -0.
double round_decimals(double value)
{
  const double scale = std::pow(DECIMAL_BASE, JSON_DECIMALS);
  const double rounded = std::round(value * scale) / scale;
  return rounded == 0.0 ? 0.0 : rounded;
}

ordered_json vec2(const Eigen::Vector2d & v)
{
  return ordered_json::array({round_decimals(v.x()), round_decimals(v.y())});
}

ordered_json polygon_json(const std::vector<Eigen::Vector2d> & polygon)
{
  ordered_json out = ordered_json::array();
  for (const auto & p : polygon) {
    out.push_back(vec2(p));
  }
  return out;
}

ordered_json area_json(const WarningArea & area)
{
  ordered_json out = ordered_json::object();
  out["kind"] = area.kind;
  out["polygon"] = polygon_json(area.polygon);
  out["height_m"] = round_decimals(area.height_m);
  out["slope_deg"] = round_decimals(area.slope_deg);
  out["source"] = area.source;
  return out;
}

ordered_json region_json(const Region & region)
{
  ordered_json edge = ordered_json::object();
  edge["point"] = vec2(region.point);
  edge["direction"] = vec2(region.direction);
  edge["side"] = region.side;
  ordered_json out = ordered_json::object();
  out["name"] = region.name;
  out["kind"] = region.kind;
  out["class"] = region.cls;
  out["height_m"] = round_decimals(region.height_m);
  out["frame"] = FRAME_BASE;
  out["edge"] = edge;
  out["distance_m"] = round_decimals(region.distance_m);
  out["points"] = region.points;
  return out;
}

const char * level_name(Level level)
{
  switch (level) {
    case Level::ok:
      return "ok";
    case Level::warning:
      return "warning";
    case Level::error:
      return "error";
  }
  return "ok";
}

ordered_json plane_json(const StateInputs & in)
{
  const TrackedPlane & tracked = *in.plane;
  const Eigen::Vector3d & n = tracked.plane.normal;
  ordered_json out = ordered_json::object();
  out["normal"] = ordered_json::array({round_decimals(n.x()), round_decimals(n.y()), round_decimals(n.z())});
  out["height_m"] = round_decimals(-tracked.plane.d / n.z());  // floor z at the base_link origin
  out["inliers"] = in.inliers;
  out["inlier_ratio"] = round_decimals(in.inlier_ratio);
  out["source"] = tracked.source == PlaneSource::fit ? "fit" : "held";
  out["age_s"] = round_decimals(tracked.age_s);
  return out;
}

ordered_json colour_json(const ColourInputs & colour)
{
  ordered_json out = ordered_json::object();
  out["model_ok"] = colour.model_ok;
  out["floor_samples"] = colour.floor_samples;
  out["mismatch_ratio"] =
    colour.stats.mismatch_ratio ? ordered_json(round_decimals(*colour.stats.mismatch_ratio)) : ordered_json(nullptr);
  out["stamp_ns"] = colour.stamp_ns;
  return out;
}

}  // namespace

ordered_json build_state(const StateInputs & in)
{
  const bool has_plane = in.plane.has_value();
  const ordered_json null = nullptr;
  ordered_json reason = null;
  if (!has_plane) {
    reason = in.reject ? std::string(to_string(*in.reject)) : in.reason;
  }
  const bool has_slope = has_plane && in.slope.has_value();

  ordered_json traversability = ordered_json::object();
  traversability["max_step_m"] = round_decimals(max_step_m(in.traversability));
  traversability["max_slope_deg"] = round_decimals(in.traversability.max_slope_deg);

  ordered_json regions = ordered_json::array();
  ordered_json warnings = ordered_json::array();
  if (has_plane) {
    for (const auto & region : in.regions) {
      regions.push_back(region_json(region));
    }
    for (const auto & area : in.warnings) {
      warnings.push_back(area_json(area));
    }
  }

  ordered_json out = ordered_json::object();
  out["stamp_ns"] = in.stamp_ns;
  out["frame"] = in.frame;
  out["prior"] = in.prior.from_imu ? PRIOR_IMU : PRIOR_BASE;
  out["imu_ok"] = in.imu_ok;
  out["plane"] = has_plane ? plane_json(in) : null;
  out["reason"] = reason;
  out["slope_deg"] = has_slope ? ordered_json(round_decimals(in.slope->total_deg)) : null;
  out["slope_roll_deg"] = has_slope ? ordered_json(round_decimals(in.slope->roll_deg)) : null;
  out["slope_pitch_deg"] = has_slope ? ordered_json(round_decimals(in.slope->pitch_deg)) : null;
  out["slope_level"] = has_slope ? ordered_json(level_name(in.slope->level)) : null;
  out["base_tilt_deg"] = round_decimals(in.base_tilt_deg);
  out["plane_vs_base_deg"] =
    has_plane && in.plane_vs_base_deg ? ordered_json(round_decimals(*in.plane_vs_base_deg)) : null;
  out["traversability"] = traversability;
  out["obstacle_points"] = has_plane ? in.obstacle_points : 0;
  out["drop_points"] = has_plane ? in.drop_points : 0;
  out["regions"] = regions;
  out["warnings"] = warnings;
  out["colour"] = in.colour ? colour_json(*in.colour) : null;
  return out;
}

std::string to_state_string(const StateInputs & in) { return build_state(in).dump(); }

std::string to_warning_areas_string(
  int64_t stamp_ns, const std::string & frame, const std::vector<WarningArea> & areas_in_frame)
{
  ordered_json areas = ordered_json::array();
  for (const auto & area : areas_in_frame) {
    areas.push_back(area_json(area));
  }
  ordered_json out = ordered_json::object();
  out["stamp_ns"] = stamp_ns;
  out["frame"] = frame;
  out["areas"] = areas;
  return out.dump();
}

}  // namespace ground_estimator_cpp
