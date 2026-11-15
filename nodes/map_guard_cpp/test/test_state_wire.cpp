// The guard state JSON on /map_guard/state (read by mcp_server's localisation health).
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "map_guard_cpp/state_wire.hpp"

namespace mg = map_guard_cpp;

TEST(StateWire, CarriesEveryKeyInOrder)
{
  mg::GuardStatus status;
  status.state = mg::State::DISTURBED;
  status.reasons = {"carried", "rotated"};
  status.agreement_ratio = 0.03;
  status.ratio_age_s = 0.4;
  status.median_ratio = 0.05;
  status.healthy_ratio = 0.6;
  status.dry_run = false;
  status.held = false;
  status.stamp = 1760000000.5;
  const auto json = nlohmann::ordered_json::parse(mg::state_json(status));
  std::vector<std::string> keys;
  for (const auto & [key, value] : json.items()) {
    keys.push_back(key);
  }
  EXPECT_EQ(
    keys, (std::vector<std::string>{
            "state", "reasons", "agreement_ratio", "ratio_age_s", "median_ratio", "healthy_ratio", "dry_run", "held",
            "stamp"}));
  EXPECT_EQ(json.at("state"), "disturbed");
  EXPECT_EQ(json.at("reasons").get<std::vector<std::string>>(), (std::vector<std::string>{"carried", "rotated"}));
  EXPECT_DOUBLE_EQ(json.at("agreement_ratio").get<double>(), 0.03);
  EXPECT_DOUBLE_EQ(json.at("ratio_age_s").get<double>(), 0.4);
  EXPECT_DOUBLE_EQ(json.at("median_ratio").get<double>(), 0.05);
  EXPECT_DOUBLE_EQ(json.at("healthy_ratio").get<double>(), 0.6);
  EXPECT_FALSE(json.at("dry_run").get<bool>());
  EXPECT_FALSE(json.at("held").get<bool>());
  EXPECT_DOUBLE_EQ(json.at("stamp").get<double>(), 1760000000.5);
}

TEST(StateWire, MissingValuesAreNull)
{
  mg::GuardStatus status;
  status.healthy_ratio = 0.6;
  status.held = true;
  const auto json = nlohmann::json::parse(mg::state_json(status));
  EXPECT_EQ(json.at("state"), "ok");
  EXPECT_TRUE(json.at("reasons").is_array() && json.at("reasons").empty());
  EXPECT_TRUE(json.at("agreement_ratio").is_null());
  EXPECT_TRUE(json.at("ratio_age_s").is_null());
  EXPECT_TRUE(json.at("median_ratio").is_null());
  EXPECT_TRUE(json.at("dry_run").get<bool>());
  EXPECT_TRUE(json.at("held").get<bool>());
}

TEST(StateWire, EveryStateHasItsName)
{
  for (const auto state : mg::ALL_STATES) {
    mg::GuardStatus status;
    status.state = state;
    EXPECT_EQ(nlohmann::json::parse(mg::state_json(status)).at("state"), mg::state_name(state));
  }
}
