// The JSON contracts of the node: /ground/state (contract ground-state, keys fixed in STATE_KEYS) and
// /ground/warning_areas (contract ground-warning-areas, consumed by costmap_zones).
#ifndef GROUND_ESTIMATOR_CPP__STATE_HPP_
#define GROUND_ESTIMATOR_CPP__STATE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ground_estimator_cpp/colour.hpp"
#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/edges.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"
#include "ground_estimator_cpp/tracker.hpp"
#include "ground_estimator_cpp/traversability.hpp"

namespace ground_estimator_cpp
{

// Keys of the state document, in output order.
// clang-format off
inline constexpr std::array<std::string_view, 19> STATE_KEYS = {
  "stamp_ns",       "frame",          "prior",          "imu_ok",      "head_moving", "plane",    "reason",
  "slope_deg",      "slope_roll_deg", "slope_pitch_deg", "slope_level", "base_tilt_deg", "plane_vs_base_deg",
  "traversability", "obstacle_points", "drop_points",    "regions",     "warnings", "colour"};
// clang-format on

// Decimals of every float in the documents.
inline constexpr int JSON_DECIMALS = 4;

// What the RGB-D labelling last produced.
struct ColourInputs
{
  LabelStats stats;               // mismatch_ratio is published
  bool model_ok = false;          // FloorColourModel::ready()
  std::size_t floor_samples = 0;  // FloorColourModel::samples()
  int64_t stamp_ns = 0;           // stamp of the labelled RGB-D cloud
};

// Everything one state document is built from.
struct StateInputs
{
  int64_t stamp_ns = 0;
  std::string frame = "base_link";
  Prior prior;
  bool imu_ok = false;
  bool head_moving = false;           // the last depth cloud was gated by the head-motion gate
  std::optional<TrackedPlane> plane;  // nullopt: no valid fit or held plane
  std::size_t inliers = 0;            // of the fit the plane comes from
  double inlier_ratio = 0.0;
  std::optional<FitReject> reject;  // why the fit failed; wins over reason
  std::string reason;               // "no_tf" or "no_cloud" when there is no fit result
  std::optional<Slope> slope;
  double base_tilt_deg = 0.0;
  std::optional<double> plane_vs_base_deg;
  std::size_t obstacle_points = 0;
  std::size_t drop_points = 0;
  std::vector<Region> regions;        // with cls set, in base_link
  std::vector<WarningArea> warnings;  // in base_link
  TraversabilityConfig traversability;
  std::optional<ColourInputs> colour;  // nullopt: colour disabled or no RGB-D cloud labelled yet
};

// The state document. Without a plane: plane, slope_*, slope_level and plane_vs_base_deg are null, the counts are 0 and
// regions and warnings are empty (no fabricated numbers).
// @param in the inputs.
// @return the document with keys in STATE_KEYS order (ordered_json), floats rounded to JSON_DECIMALS.
nlohmann::ordered_json build_state(const StateInputs & in);

// @param in the inputs.
// @return the compact dump of build_state.
std::string to_state_string(const StateInputs & in);

// The warning areas document for costmap_zones.
// @param stamp_ns stamp of the areas.
// @param frame frame of the polygons.
// @param areas_in_frame areas already converted to frame; may be empty (a real "no warning" state).
// @return compact JSON {stamp_ns, frame, areas: [{kind, polygon, height_m, slope_deg, source}]}.
std::string to_warning_areas_string(
  int64_t stamp_ns, const std::string & frame, const std::vector<WarningArea> & areas_in_frame);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__STATE_HPP_
