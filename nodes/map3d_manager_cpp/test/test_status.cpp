// The 3D map status contract (see status.hpp).
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

#include "map3d_manager_cpp/status.hpp"

namespace m3 = map3d_manager_cpp;
using nlohmann::json;

namespace
{

constexpr double NOW = 1000.0;

m3::StatusInputs fresh_inputs()
{
  m3::StatusInputs in;
  in.now_s = NOW;
  in.info_stamp_s = NOW - 0.5;
  in.rgb_info_stamp_s = NOW - 0.2;
  in.depth_info_stamp_s = NOW - 0.2;
  return in;
}

m3::GraphSample graph(double x, double y, double yaw_rad)
{
  m3::GraphSample g;
  g.stamp_s = NOW - 1.0;
  g.nodes = 42;
  g.x = x;
  g.y = y;
  g.yaw_rad = yaw_rad;
  return g;
}

json status_json(const m3::StatusInputs & in, const m3::Config & cfg = m3::Config())
{
  return json::parse(m3::to_json(m3::build_status(in, cfg)));
}

}  // namespace

TEST(Map3dStatus, MappingWhenAllInputsFresh)
{
  const json j = status_json(fresh_inputs());
  EXPECT_EQ(j["state"], "mapping");
  EXPECT_TRUE(j["missing_inputs"].empty());
  EXPECT_DOUBLE_EQ(j["stamp_s"].get<double>(), NOW);
}

TEST(Map3dStatus, WaitingWhenNothingSeen)
{
  m3::StatusInputs in;
  in.now_s = NOW;
  const json j = status_json(in);
  EXPECT_EQ(j["state"], "waiting_for_inputs");
  EXPECT_EQ(j["missing_inputs"], json::array({"rgb_camera_info", "depth_camera_info", "rtabmap_info"}));
  EXPECT_FALSE(j["message"].get<std::string>().empty());
}

TEST(Map3dStatus, StaleInputIsMissingFreshIsNot)
{
  m3::StatusInputs in = fresh_inputs();
  in.depth_info_stamp_s = NOW - 10.0;  // stale (input_stale_s = 3)
  json j = status_json(in);
  EXPECT_EQ(j["state"], "waiting_for_inputs");
  EXPECT_EQ(j["missing_inputs"], json::array({"depth_camera_info"}));

  in.depth_info_stamp_s = NOW - 2.9;  // still fresh
  j = status_json(in);
  EXPECT_EQ(j["state"], "mapping");
  EXPECT_TRUE(j["missing_inputs"].empty());
}

TEST(Map3dStatus, PausedOutranksMissingInputs)
{
  m3::StatusInputs in;
  in.now_s = NOW;
  in.paused = true;
  json j = status_json(in);
  EXPECT_EQ(j["state"], "paused");
  EXPECT_EQ(j["missing_inputs"].size(), 3u);  // still reported

  in = fresh_inputs();
  in.paused = true;
  EXPECT_EQ(status_json(in)["state"], "paused");
}

TEST(Map3dStatus, ErrorWithinMinuteOnly)
{
  m3::StatusInputs in = fresh_inputs();
  in.last_error = "reset failed";
  in.last_error_stamp_s = NOW - 30.0;
  json j = status_json(in);
  EXPECT_EQ(j["state"], "error");
  EXPECT_NE(j["message"].get<std::string>().find("reset failed"), std::string::npos);

  in.last_error_stamp_s = NOW - 61.0;
  EXPECT_EQ(status_json(in)["state"], "mapping");

  in.last_error = "";
  in.last_error_stamp_s = NOW;
  EXPECT_EQ(status_json(in)["state"], "mapping");
}

TEST(Map3dStatus, ErrorPrecedenceFollowsSpecOrder)
{
  m3::StatusInputs in = fresh_inputs();
  in.last_error = "reset failed";
  in.last_error_stamp_s = NOW - 5.0;

  in.paused = true;
  EXPECT_EQ(status_json(in)["state"], "paused");

  in.paused = false;
  in.rgb_info_stamp_s = std::nullopt;
  EXPECT_EQ(status_json(in)["state"], "waiting_for_inputs");

  in.paused = true;
  EXPECT_EQ(status_json(in)["state"], "paused");
}

TEST(Map3dStatus, NullFieldsBeforeData)
{
  m3::StatusInputs in;
  in.now_s = NOW;
  const json j = status_json(in);
  for (const char * key :
       {"nodes", "db_size_mb", "memory_mb", "graph_correction_m", "graph_correction_deg", "correction_ok",
        "last_update_age_s", "last_save"}) {
    ASSERT_TRUE(j.contains(key)) << key;
    EXPECT_TRUE(j[key].is_null()) << key;
  }
  EXPECT_DOUBLE_EQ(j["correction_warn_m"].get<double>(), 0.05);
  EXPECT_DOUBLE_EQ(j["correction_warn_deg"].get<double>(), 2.0);
}

TEST(Map3dStatus, CorrectionUnderThresholdsIsOk)
{
  m3::StatusInputs in = fresh_inputs();
  in.graph = graph(0.03, 0.04, 0.0);  // norm 0.05 == threshold
  json j = status_json(in);
  EXPECT_NEAR(j["graph_correction_m"].get<double>(), 0.05, 1e-12);
  EXPECT_DOUBLE_EQ(j["graph_correction_deg"].get<double>(), 0.0);
  EXPECT_EQ(j["correction_ok"], true);
  EXPECT_EQ(j["nodes"], 42);
  EXPECT_NEAR(j["last_update_age_s"].get<double>(), 1.0, 1e-12);
}

TEST(Map3dStatus, TranslationOverThresholdIsNotOk)
{
  m3::StatusInputs in = fresh_inputs();
  in.graph = graph(0.06, 0.0, 0.0);
  EXPECT_EQ(status_json(in)["correction_ok"], false);
  in.graph = graph(0.0, 0.0, 0.0);
  in.graph->z = -0.2;  // z counts in the norm
  EXPECT_EQ(status_json(in)["correction_ok"], false);
}

TEST(Map3dStatus, YawOverThresholdIsNotOkWhateverTheSign)
{
  m3::StatusInputs in = fresh_inputs();
  const double three_deg = 3.0 * M_PI / 180.0;
  in.graph = graph(0.0, 0.0, -three_deg);
  json j = status_json(in);
  EXPECT_NEAR(j["graph_correction_deg"].get<double>(), 3.0, 1e-9);
  EXPECT_EQ(j["correction_ok"], false);

  in.graph = graph(0.0, 0.0, 1.0 * M_PI / 180.0);
  EXPECT_EQ(status_json(in)["correction_ok"], true);
}

TEST(Map3dStatus, ThresholdsComeFromConfig)
{
  m3::Config cfg;
  cfg.correction_warn_m = 0.5;
  cfg.correction_warn_deg = 10.0;
  m3::StatusInputs in = fresh_inputs();
  in.graph = graph(0.3, 0.0, 5.0 * M_PI / 180.0);
  const json j = status_json(in, cfg);
  EXPECT_EQ(j["correction_ok"], true);
  EXPECT_DOUBLE_EQ(j["correction_warn_m"].get<double>(), 0.5);
  EXPECT_DOUBLE_EQ(j["correction_warn_deg"].get<double>(), 10.0);
}

TEST(Map3dStatus, JsonRoundTripOfEveryField)
{
  m3::StatusInputs in = fresh_inputs();
  in.graph = graph(0.0, 0.0, 0.0);
  in.db_size_bytes = 5 * 1024 * 1024;
  in.memory_bytes = 300ull * 1024 * 1024;
  in.last_save = m3::LastSave{"kitchen", NOW - 20.0};
  const json j = status_json(in);
  EXPECT_DOUBLE_EQ(j["stamp_s"].get<double>(), NOW);
  EXPECT_EQ(j["state"], "mapping");
  EXPECT_EQ(j["nodes"], 42);
  EXPECT_DOUBLE_EQ(j["db_size_mb"].get<double>(), 5.0);
  EXPECT_DOUBLE_EQ(j["memory_mb"].get<double>(), 300.0);
  EXPECT_DOUBLE_EQ(j["graph_correction_m"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(j["graph_correction_deg"].get<double>(), 0.0);
  EXPECT_EQ(j["correction_ok"], true);
  EXPECT_TRUE(j["missing_inputs"].is_array());
  EXPECT_DOUBLE_EQ(j["last_update_age_s"].get<double>(), 1.0);
  EXPECT_EQ(j["last_save"]["name"], "kitchen");
  EXPECT_DOUBLE_EQ(j["last_save"]["stamp_s"].get<double>(), NOW - 20.0);
  EXPECT_TRUE(j["message"].is_string());
  EXPECT_EQ(j.size(), 14u);
}

TEST(Map3dCgroup, ReadsMemoryCurrent)
{
  const auto root = std::filesystem::temp_directory_path() / "map3d_cgroup_test";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root / "unit.service");
  {
    std::ofstream(root / "unit.service" / "memory.current") << "123456789\n";
  }
  const auto v = m3::read_cgroup_memory_bytes("unit.service", root.string());
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, 123456789u);
  EXPECT_FALSE(m3::read_cgroup_memory_bytes("other.service", root.string()).has_value());
  {
    std::ofstream(root / "unit.service" / "memory.current") << "max\n";
  }
  EXPECT_FALSE(m3::read_cgroup_memory_bytes("unit.service", root.string()).has_value());
  std::filesystem::remove_all(root);
}
