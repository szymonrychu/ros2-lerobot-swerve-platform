#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "ground_estimator_cpp/state.hpp"

namespace
{

using ground_estimator_cpp::build_state;
using ground_estimator_cpp::ColourInputs;
using ground_estimator_cpp::PlaneSource;
using ground_estimator_cpp::Region;
using ground_estimator_cpp::STATE_KEYS;
using ground_estimator_cpp::StateInputs;
using ground_estimator_cpp::to_state_string;
using ground_estimator_cpp::to_warning_areas_string;
using ground_estimator_cpp::TrackedPlane;
using ground_estimator_cpp::WarningArea;
using json = nlohmann::ordered_json;

// Keys of a JSON object in document order.
std::vector<std::string> keys_in_order(const std::string & text)
{
  std::vector<std::string> keys;
  const nlohmann::ordered_json document = nlohmann::ordered_json::parse(text);
  for (const auto & item : document.items()) {
    keys.push_back(item.key());
  }
  return keys;
}

double as_double(const json & value) { return value.get<double>(); }

int64_t as_int(const json & value) { return value.get<int64_t>(); }

bool as_bool(const json & value) { return value.get<bool>(); }

std::vector<std::string> state_keys() { return {STATE_KEYS.begin(), STATE_KEYS.end()}; }

std::string read_file(const std::string & name)
{
  std::ifstream in(std::string(GROUND_TEST_DATA_DIR) + "/" + name);
  EXPECT_TRUE(in.good()) << name;
  std::stringstream ss;
  ss << in.rdbuf();
  std::string text = ss.str();
  while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
    text.pop_back();
  }
  return text;
}

WarningArea ramp_area()
{
  WarningArea area;
  area.kind = "ramp";
  area.polygon = {{0.5, -0.2}, {0.9, -0.2}, {0.9, 0.31234}, {0.5, 0.3}};
  area.height_m = 0.0123456;
  area.slope_deg = 5.55555;
  area.source = "tof";
  return area;
}

WarningArea colour_area()
{
  WarningArea area;
  area.kind = "colour";
  area.polygon = {{1.0, 0.1}, {1.2, 0.1}, {1.1, 0.4}};
  area.height_m = 0.001;
  area.slope_deg = 0.0;
  area.source = "colour";
  return area;
}

// The inputs of the example fixtures: a held plane, a slope, one obstacle and one warning region, colour state.
StateInputs full_inputs()
{
  StateInputs in;
  in.stamp_ns = 1760000000123456789LL;
  in.frame = "base_link";
  in.prior.from_imu = true;
  in.imu_ok = true;
  TrackedPlane tracked;
  tracked.plane.normal = Eigen::Vector3d(0.0, 0.0523, 0.9986);
  tracked.plane.d = 0.0412;
  tracked.age_s = 0.31234;
  tracked.source = PlaneSource::held;
  in.plane = tracked;
  in.inliers = 812;
  in.inlier_ratio = 0.654321;
  ground_estimator_cpp::Slope slope;
  slope.total_deg = 3.0001234;
  slope.roll_deg = 0.5;
  slope.pitch_deg = -2.99999;
  slope.level = ground_estimator_cpp::Level::warning;
  in.slope = slope;
  in.base_tilt_deg = 1.23456;
  in.plane_vs_base_deg = 3.0;
  in.obstacle_points = 42;
  in.drop_points = 7;
  Region obstacle;
  obstacle.name = "step_0";
  obstacle.kind = "step";
  obstacle.cls = "obstacle";
  obstacle.height_m = 0.0987654;
  obstacle.point = Eigen::Vector2d(0.8, 0.0);
  obstacle.direction = Eigen::Vector2d(0.0, 1.0);
  obstacle.side = "left";
  obstacle.distance_m = 0.8;
  obstacle.points = 120;
  Region warning;
  warning.name = "drop_0";
  warning.kind = "drop";
  warning.cls = "warning";
  warning.height_m = -0.03;
  warning.point = Eigen::Vector2d(1.2, 0.5);
  warning.direction = Eigen::Vector2d(1.0, 0.0);
  warning.side = "right";
  warning.distance_m = 1.3;
  warning.points = 55;
  in.regions = {obstacle, warning};
  in.warnings = {ramp_area(), colour_area()};
  ColourInputs colour;
  colour.model_ok = true;
  colour.stats.mismatch_ratio = 0.0123456;
  colour.floor_samples = 5000;
  colour.stamp_ns = 1760000000023456789LL;
  in.colour = colour;
  return in;
}

// A state without a plane: counts and lists must be empty even when the caller left values in the inputs.
StateInputs planeless_inputs()
{
  StateInputs in;
  in.stamp_ns = 5;
  in.prior.from_imu = false;
  in.imu_ok = false;
  in.reason = "no_tf";
  in.base_tilt_deg = 0.0;
  return in;
}

}  // namespace

TEST(GroundState, KeyOrderEqualsStateKeysForFullState)
{
  EXPECT_EQ(keys_in_order(to_state_string(full_inputs())), state_keys());
}

TEST(GroundState, KeyOrderEqualsStateKeysForPlaneLessState)
{
  EXPECT_EQ(keys_in_order(to_state_string(planeless_inputs())), state_keys());
}

TEST(GroundState, StateKeysAreTheContractOrder)
{
  // clang-format off
  const std::vector<std::string> expected = {
    "stamp_ns",      "frame",           "prior",          "imu_ok",     "plane",      "reason",
    "slope_deg",     "slope_roll_deg",  "slope_pitch_deg", "slope_level", "base_tilt_deg", "plane_vs_base_deg",
    "traversability", "obstacle_points", "drop_points",    "regions",    "warnings",   "colour"};
  // clang-format on
  EXPECT_EQ(state_keys(), expected);
}

TEST(GroundState, PlaneLessStateHasNullsAndZeroCounts)
{
  StateInputs in = planeless_inputs();
  in.obstacle_points = 9;  // stale caller values must not leak
  in.drop_points = 4;
  in.regions = full_inputs().regions;
  in.warnings = full_inputs().warnings;
  const json j = build_state(in);
  for (const char * key :
       {"plane", "slope_deg", "slope_roll_deg", "slope_pitch_deg", "slope_level", "plane_vs_base_deg"}) {
    EXPECT_TRUE(j.at(key).is_null()) << key;
  }
  EXPECT_EQ(j.at("obstacle_points"), 0);
  EXPECT_EQ(j.at("drop_points"), 0);
  EXPECT_TRUE(j.at("regions").is_array());
  EXPECT_TRUE(j.at("regions").empty());
  EXPECT_TRUE(j.at("warnings").empty());
  EXPECT_EQ(j.at("reason"), "no_tf");
  EXPECT_EQ(j.at("prior"), "base_link");
  EXPECT_FALSE(as_bool(j.at("imu_ok")));
  EXPECT_TRUE(j.at("colour").is_null());
}

TEST(GroundState, FitRejectIsTheReasonString)
{
  StateInputs in = planeless_inputs();
  in.reject = ground_estimator_cpp::FitReject::low_inlier_ratio;
  in.reason = "ignored";
  EXPECT_EQ(build_state(in).at("reason"), "low_inlier_ratio");
}

TEST(GroundState, ReasonIsNullWhenPlanePresent)
{
  StateInputs in = full_inputs();
  in.reason = "no_cloud";
  EXPECT_TRUE(build_state(in).at("reason").is_null());
}

TEST(GroundState, PlaneAndSlopeShapes)
{
  const json j = build_state(full_inputs());
  const json & plane = j.at("plane");
  EXPECT_EQ(plane.at("normal"), json::array({0.0, 0.0523, 0.9986}));
  EXPECT_NEAR(as_double(plane.at("height_m")), -0.0413, 1e-9);
  EXPECT_EQ(plane.at("inliers"), 812);
  EXPECT_DOUBLE_EQ(as_double(plane.at("inlier_ratio")), 0.6543);
  EXPECT_EQ(plane.at("source"), "held");
  EXPECT_DOUBLE_EQ(as_double(plane.at("age_s")), 0.3123);
  EXPECT_DOUBLE_EQ(as_double(j.at("slope_deg")), 3.0001);
  EXPECT_DOUBLE_EQ(as_double(j.at("slope_roll_deg")), 0.5);
  EXPECT_DOUBLE_EQ(as_double(j.at("slope_pitch_deg")), -3.0);
  EXPECT_EQ(j.at("slope_level"), "warning");
  EXPECT_EQ(j.at("prior"), "imu");
  EXPECT_DOUBLE_EQ(as_double(j.at("base_tilt_deg")), 1.2346);
  EXPECT_DOUBLE_EQ(as_double(j.at("plane_vs_base_deg")), 3.0);
  EXPECT_EQ(as_int(j.at("stamp_ns")), 1760000000123456789LL);
  EXPECT_EQ(j.at("frame"), "base_link");
}

TEST(GroundState, FitSourceAndLevels)
{
  StateInputs in = full_inputs();
  in.plane->source = PlaneSource::fit;
  in.slope->level = ground_estimator_cpp::Level::error;
  json j = build_state(in);
  EXPECT_EQ(j.at("plane").at("source"), "fit");
  EXPECT_EQ(j.at("slope_level"), "error");
  in.slope->level = ground_estimator_cpp::Level::ok;
  EXPECT_EQ(build_state(in).at("slope_level"), "ok");
}

TEST(GroundState, TraversabilityIsTheConfiguredRuleEvenWithoutPlane)
{
  for (const StateInputs & in : {full_inputs(), planeless_inputs()}) {
    const json t = build_state(in).at("traversability");
    EXPECT_DOUBLE_EQ(as_double(t.at("max_step_m")), 0.06);
    EXPECT_DOUBLE_EQ(as_double(t.at("max_slope_deg")), 15.0);
    EXPECT_EQ(keys_in_order(t.dump()), (std::vector<std::string>{"max_step_m", "max_slope_deg"}));
  }
}

TEST(GroundState, RegionsCarryClassAndEdgeShape)
{
  const json regions = build_state(full_inputs()).at("regions");
  ASSERT_EQ(regions.size(), 2u);
  const json & r = regions[0];
  EXPECT_EQ(
    keys_in_order(r.dump()),
    (std::vector<std::string>{"name", "kind", "class", "height_m", "frame", "edge", "distance_m", "points"}));
  EXPECT_EQ(r.at("name"), "step_0");
  EXPECT_EQ(r.at("kind"), "step");
  EXPECT_EQ(r.at("class"), "obstacle");
  EXPECT_DOUBLE_EQ(as_double(r.at("height_m")), 0.0988);
  EXPECT_EQ(r.at("frame"), "base_link");
  EXPECT_EQ(r.at("edge").at("point"), json::array({0.8, 0.0}));
  EXPECT_EQ(r.at("edge").at("direction"), json::array({0.0, 1.0}));
  EXPECT_EQ(r.at("edge").at("side"), "left");
  EXPECT_EQ(keys_in_order(r.at("edge").dump()), (std::vector<std::string>{"point", "direction", "side"}));
  EXPECT_DOUBLE_EQ(as_double(r.at("distance_m")), 0.8);
  EXPECT_EQ(r.at("points"), 120);
  EXPECT_EQ(regions[1].at("class"), "warning");
  EXPECT_EQ(regions[1].at("kind"), "drop");
}

TEST(GroundState, WarningsCarryTheirShape)
{
  const json warnings = build_state(full_inputs()).at("warnings");
  ASSERT_EQ(warnings.size(), 2u);
  const json & w = warnings[0];
  EXPECT_EQ(keys_in_order(w.dump()), (std::vector<std::string>{"kind", "polygon", "height_m", "slope_deg", "source"}));
  EXPECT_EQ(w.at("kind"), "ramp");
  EXPECT_EQ(w.at("source"), "tof");
  ASSERT_EQ(w.at("polygon").size(), 4u);
  EXPECT_EQ(w.at("polygon")[2], json::array({0.9, 0.3123}));
  EXPECT_DOUBLE_EQ(as_double(w.at("height_m")), 0.0123);
  EXPECT_DOUBLE_EQ(as_double(w.at("slope_deg")), 5.5556);
  EXPECT_EQ(warnings[1].at("kind"), "colour");
  EXPECT_EQ(warnings[1].at("source"), "colour");
}

TEST(WarningAreasJson, NullHeightAndSlopeWrittenAsNull)
{
  WarningArea area = colour_area();
  area.height_m = std::nullopt;
  area.slope_deg = std::nullopt;
  StateInputs in = full_inputs();
  in.warnings = {area};
  const json w = build_state(in).at("warnings")[0];
  EXPECT_TRUE(w.at("height_m").is_null());
  EXPECT_TRUE(w.at("slope_deg").is_null());
  EXPECT_EQ(keys_in_order(w.dump()), (std::vector<std::string>{"kind", "polygon", "height_m", "slope_deg", "source"}));
  EXPECT_NE(to_warning_areas_string(1, "base_link", {area}).find("\"height_m\":null"), std::string::npos);
}

TEST(GroundState, ColourShape)
{
  json c = build_state(full_inputs()).at("colour");
  EXPECT_EQ(
    keys_in_order(c.dump()), (std::vector<std::string>{"model_ok", "floor_samples", "mismatch_ratio", "stamp_ns"}));
  EXPECT_TRUE(as_bool(c.at("model_ok")));
  EXPECT_EQ(c.at("floor_samples"), 5000);
  EXPECT_DOUBLE_EQ(as_double(c.at("mismatch_ratio")), 0.0123);
  EXPECT_EQ(as_int(c.at("stamp_ns")), 1760000000023456789LL);

  StateInputs in = full_inputs();
  in.colour->stats.mismatch_ratio.reset();
  in.colour->model_ok = false;
  c = build_state(in).at("colour");
  EXPECT_TRUE(c.at("mismatch_ratio").is_null());
  EXPECT_FALSE(as_bool(c.at("model_ok")));

  in.colour.reset();
  EXPECT_TRUE(build_state(in).at("colour").is_null());
}

TEST(GroundState, ColourIsKeptWithoutPlaneWhenPresent)
{
  StateInputs in = planeless_inputs();
  in.colour = full_inputs().colour;
  EXPECT_TRUE(build_state(in).at("colour").is_object());
}

TEST(GroundState, FloatsAreRoundedToFourDecimals)
{
  StateInputs in = full_inputs();
  in.base_tilt_deg = 0.123449;
  EXPECT_DOUBLE_EQ(build_state(in).at("base_tilt_deg").get<double>(), 0.1234);
  in.base_tilt_deg = 0.123451;
  EXPECT_DOUBLE_EQ(build_state(in).at("base_tilt_deg").get<double>(), 0.1235);
  in.base_tilt_deg = -0.00001;
  EXPECT_EQ(to_state_string(in).find("e-"), std::string::npos);
}

TEST(GroundState, DumpIsCompactAndParsesBack)
{
  const std::string text = to_state_string(full_inputs());
  EXPECT_EQ(text.find(' '), std::string::npos);
  EXPECT_EQ(text.find('\n'), std::string::npos);
  const json j = json::parse(text);
  EXPECT_EQ(j, build_state(full_inputs()));
}

TEST(GroundState, ExampleFixtureEqualsToStateString)
{
  // The fixture is example data with an 8 deg limit, not the default rule.
  StateInputs in = full_inputs();
  in.traversability.max_step_fraction = 0.3333;  // the example document predates the 60 mm step
  in.traversability.max_slope_deg = 8.0;
  EXPECT_EQ(read_file("state_example.json"), to_state_string(in));
}

TEST(WarningAreas, KeyOrderAndShape)
{
  const std::string text = to_warning_areas_string(1760000000123456789LL, "map", {ramp_area(), colour_area()});
  EXPECT_EQ(keys_in_order(text), (std::vector<std::string>{"stamp_ns", "frame", "areas"}));
  const json j = json::parse(text);
  EXPECT_EQ(as_int(j.at("stamp_ns")), 1760000000123456789LL);
  EXPECT_EQ(j.at("frame"), "map");
  ASSERT_EQ(j.at("areas").size(), 2u);
  const auto & a = j.at("areas")[0];
  EXPECT_EQ(keys_in_order(a.dump()), (std::vector<std::string>{"kind", "polygon", "height_m", "slope_deg", "source"}));
  EXPECT_EQ(a.at("polygon")[3], json::array({0.5, 0.3}));
  EXPECT_DOUBLE_EQ(as_double(a.at("height_m")), 0.0123);
  EXPECT_DOUBLE_EQ(as_double(a.at("slope_deg")), 5.5556);
  EXPECT_EQ(text.find(' '), std::string::npos);
}

TEST(WarningAreas, EmptyListIsValid)
{
  const std::string text = to_warning_areas_string(7, "map", {});
  EXPECT_EQ(text, R"({"stamp_ns":7,"frame":"map","areas":[]})");
}

TEST(WarningAreas, ExampleFixtureEqualsToWarningAreasString)
{
  EXPECT_EQ(
    read_file("warning_areas_example.json"),
    to_warning_areas_string(1760000000123456789LL, "map", {ramp_area(), colour_area()}));
}
