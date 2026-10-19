// The map3d command/result JSON contract (see command_json.hpp).
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "map3d_manager_cpp/command_json.hpp"

namespace m3 = map3d_manager_cpp;
using nlohmann::json;

namespace
{

const m3::Command * command(const std::variant<m3::Command, m3::ParseError> & v)
{
  return std::get_if<m3::Command>(&v);
}
const m3::ParseError * error(const std::variant<m3::Command, m3::ParseError> & v)
{
  return std::get_if<m3::ParseError>(&v);
}

}  // namespace

TEST(Map3dCommand, EachActionParses)
{
  const std::pair<const char *, m3::Action> cases[] = {
    {"save", m3::Action::SAVE},
    {"reset", m3::Action::RESET},
    {"pause", m3::Action::PAUSE},
    {"resume", m3::Action::RESUME}};
  for (const auto & [text, action] : cases) {
    const auto parsed = m3::parse_command(std::string("{\"request_id\":\"r1\",\"action\":\"") + text + "\"}");
    ASSERT_NE(command(parsed), nullptr) << text;
    EXPECT_EQ(command(parsed)->action, action);
    EXPECT_EQ(command(parsed)->request_id, "r1");
    EXPECT_EQ(command(parsed)->name, "slam_map");
    EXPECT_STREQ(m3::action_name(action), text);
  }
}

TEST(Map3dCommand, NameIsRead)
{
  const auto parsed = m3::parse_command(R"({"request_id":"r","action":"save","name":"Lab-1_a"})");
  ASSERT_NE(command(parsed), nullptr);
  EXPECT_EQ(command(parsed)->name, "Lab-1_a");
  const auto longest =
    m3::parse_command(R"({"request_id":"r","action":"save","name":")" + std::string(64, 'a') + "\"}");
  EXPECT_NE(command(longest), nullptr);
}

TEST(Map3dCommand, BadNameIsInvalidWithId)
{
  for (const std::string & name : std::vector<std::string>{"", "a b", "../x", "a/b", "a.db", std::string(65, 'a')}) {
    const auto parsed = m3::parse_command(json{{"request_id", "r9"}, {"action", "save"}, {"name", name}}.dump());
    ASSERT_NE(error(parsed), nullptr) << name;
    EXPECT_EQ(error(parsed)->request_id, "r9");
  }
  const auto numeric = m3::parse_command(R"({"request_id":"r9","action":"save","name":5})");
  ASSERT_NE(error(numeric), nullptr);
  EXPECT_EQ(error(numeric)->request_id, "r9");
}

TEST(Map3dCommand, RequestIdBounds)
{
  const auto ok64 = m3::parse_command(R"({"request_id":")" + std::string(64, 'x') + R"(","action":"reset"})");
  EXPECT_NE(command(ok64), nullptr);
  for (const std::string & text : std::vector<std::string>{
         R"({"action":"save"})", R"({"request_id":"","action":"save"})", R"({"request_id":5,"action":"save"})",
         R"({"request_id":")" + std::string(65, 'x') + R"(","action":"save"})"}) {
    const auto parsed = m3::parse_command(text);
    ASSERT_NE(error(parsed), nullptr) << text;
    EXPECT_FALSE(error(parsed)->request_id.has_value()) << text;
  }
}

TEST(Map3dCommand, UnknownActionIsInvalidWithId)
{
  for (const char * text :
       {R"({"request_id":"r2","action":"explode"})", R"({"request_id":"r2"})", R"({"request_id":"r2","action":3})"}) {
    const auto parsed = m3::parse_command(text);
    ASSERT_NE(error(parsed), nullptr) << text;
    EXPECT_EQ(error(parsed)->request_id, "r2");
    EXPECT_FALSE(error(parsed)->message.empty());
  }
}

TEST(Map3dCommand, NotJsonIsDropped)
{
  for (const char * text : {"", "save", "{not json", "[1,2]", "42", "null"}) {
    const auto parsed = m3::parse_command(text);
    ASSERT_NE(error(parsed), nullptr) << text;
    EXPECT_FALSE(error(parsed)->request_id.has_value()) << text;
  }
}

TEST(Map3dResult, WithoutDbFields)
{
  m3::Result r;
  r.request_id = "r1";
  r.status = m3::Status::BUSY;
  r.message = "a save is running";
  r.action = "save";
  const json doc = json::parse(m3::to_json(r));
  EXPECT_EQ(
    doc, (json{{"request_id", "r1"}, {"status", "busy"}, {"message", "a save is running"}, {"action", "save"}}));
}

TEST(Map3dResult, WithDbFields)
{
  m3::Result r;
  r.request_id = "r2";
  r.message = "saved";
  r.action = "save";
  r.db_path = "/var/lib/ros2/rtabmap/slam_map.db";
  r.db_size_mb = 12.5;
  const json doc = json::parse(m3::to_json(r));
  EXPECT_EQ(doc["status"], "ok");
  EXPECT_EQ(doc["db_path"], "/var/lib/ros2/rtabmap/slam_map.db");
  EXPECT_DOUBLE_EQ(doc["db_size_mb"].get<double>(), 12.5);
}

TEST(Map3dResult, EveryStatusName)
{
  EXPECT_STREQ(m3::status_name(m3::Status::OK), "ok");
  EXPECT_STREQ(m3::status_name(m3::Status::ERROR), "error");
  EXPECT_STREQ(m3::status_name(m3::Status::BUSY), "busy");
  EXPECT_STREQ(m3::status_name(m3::Status::INVALID), "invalid");
  EXPECT_STREQ(m3::status_name(m3::Status::TIMEOUT), "timeout");
}
