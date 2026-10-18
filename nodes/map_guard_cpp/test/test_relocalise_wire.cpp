// The wire contract with map_manager: the relocalise command keys and the result parsing.
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "map_guard_cpp/relocalise_wire.hpp"

namespace mg = map_guard_cpp;

TEST(RelocaliseWire, CommandCarriesEveryKey)
{
  const auto json = nlohmann::json::parse(mg::relocalise_command("map_guard-3", 12.5));
  EXPECT_EQ(json.size(), 4U);
  EXPECT_EQ(json.at("op"), "relocalise");
  EXPECT_EQ(json.at("request_id"), "map_guard-3");
  EXPECT_DOUBLE_EQ(json.at("since").get<double>(), 12.5);
  EXPECT_TRUE(json.at("discard_on_fail").get<bool>());
}

TEST(RelocaliseWire, ParsesFullResult)
{
  const auto r =
    mg::parse_result(R"({"request_id":"map_guard-1","ok":true,"data":{"state":"localised","discarded_pois":4}})");
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->request_id, "map_guard-1");
  EXPECT_TRUE(r->ok);
  EXPECT_EQ(r->state, "localised");
  EXPECT_EQ(r->discarded_pois, 4);
}

TEST(RelocaliseWire, DataIsOptional)
{
  const auto r = mg::parse_result(R"({"request_id":"x","ok":false})");
  ASSERT_TRUE(r.has_value());
  EXPECT_FALSE(r->ok);
  EXPECT_EQ(r->state, "");
  EXPECT_EQ(r->discarded_pois, 0);
}

TEST(RelocaliseWire, IgnoresWrongTypedData)
{
  const auto r = mg::parse_result(R"({"request_id":"x","ok":true,"data":{"state":3,"discarded_pois":"a"}})");
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->state, "");
  EXPECT_EQ(r->discarded_pois, 0);
}

TEST(RelocaliseWire, RejectsInvalid)
{
  EXPECT_FALSE(mg::parse_result("not json"));
  EXPECT_FALSE(mg::parse_result("[1]"));
  EXPECT_FALSE(mg::parse_result(R"({"ok":true})"));
  EXPECT_FALSE(mg::parse_result(R"({"request_id":"x"})"));
  EXPECT_FALSE(mg::parse_result(R"({"request_id":1,"ok":true})"));
  EXPECT_FALSE(mg::parse_result(R"({"request_id":"x","ok":"yes"})"));
}
