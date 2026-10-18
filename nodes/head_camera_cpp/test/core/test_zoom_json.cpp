#include <gtest/gtest.h>

#include <array>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <variant>

#include "head_camera_cpp/config.hpp"
#include "head_camera_cpp/zoom_json.hpp"

namespace
{

using head_camera_cpp::ParseError;
using head_camera_cpp::ZoomConfig;
using head_camera_cpp::ZoomRequest;
using head_camera_cpp::ZoomResult;
using head_camera_cpp::ZoomStatus;
using nlohmann::json;

// Parse with the default limits.
std::variant<ZoomRequest, ParseError> parse(const std::string & text)
{
  return head_camera_cpp::parse_request(text, ZoomConfig{});
}

// A valid request object, for tests to modify.
json valid_request() { return json{{"request_id", "x"}, {"u", 0.5}, {"v", 0.5}, {"zoom", 2}}; }

// Assert the text is rejected with a ParseError whose request_id is expected_id (nullopt: none).
void expect_error(const std::string & text, const std::optional<std::string> & expected_id)
{
  const auto parsed = parse(text);
  ASSERT_TRUE(std::holds_alternative<ParseError>(parsed)) << text;
  const auto & error = std::get<ParseError>(parsed);
  EXPECT_EQ(error.request_id, expected_id) << text;
  EXPECT_FALSE(error.message.empty()) << text;
}

// Assert valid_request() with key set to value (or erased when value is discarded) is rejected, keeping id "x".
void expect_field_rejected(const std::string & key, const json & value)
{
  json j = valid_request();
  if (value.is_discarded()) {
    j.erase(key);
  } else {
    j[key] = value;
  }
  expect_error(j.dump(), "x");
}

}  // namespace

TEST(ZoomJson, FullValidRequest)
{
  json j = valid_request();
  j["request_id"] = "r1";
  j["u"] = 0.62;
  j["v"] = 0.41;
  j["zoom"] = 4;
  j["max_px"] = 512;
  j["af"] = false;
  const auto parsed = parse(j.dump());
  ASSERT_TRUE(std::holds_alternative<ZoomRequest>(parsed));
  const auto & r = std::get<ZoomRequest>(parsed);
  EXPECT_EQ(r.request_id, "r1");
  EXPECT_DOUBLE_EQ(r.u, 0.62);
  EXPECT_DOUBLE_EQ(r.v, 0.41);
  EXPECT_EQ(r.zoom, 4);
  EXPECT_EQ(r.max_px, 512);
  EXPECT_FALSE(r.af);
}

TEST(ZoomJson, DefaultsAndIntegerCoordinates)
{
  json j = valid_request();
  j["u"] = 0;
  j["v"] = 1;
  j["zoom"] = 8;
  const auto parsed = parse(j.dump());
  ASSERT_TRUE(std::holds_alternative<ZoomRequest>(parsed));
  const auto & r = std::get<ZoomRequest>(parsed);
  EXPECT_EQ(r.max_px, 1024);
  EXPECT_TRUE(r.af);
  EXPECT_DOUBLE_EQ(r.u, 0.0);
  EXPECT_DOUBLE_EQ(r.v, 1.0);
}

TEST(ZoomJson, InvalidFieldsKeepRequestId)
{
  const json missing = json::value_t::discarded;
  expect_field_rejected("u", -0.1);
  expect_field_rejected("u", 1.1);
  expect_field_rejected("u", "0.5");
  expect_field_rejected("u", missing);
  expect_field_rejected("v", -0.1);
  expect_field_rejected("v", 1.1);
  expect_field_rejected("v", missing);
  expect_field_rejected("zoom", 3);
  expect_field_rejected("zoom", 2.5);
  expect_field_rejected("zoom", missing);
  expect_field_rejected("max_px", 100);
  expect_field_rejected("max_px", 2000);
  expect_field_rejected("max_px", 512.5);
  expect_field_rejected("af", "yes");
  expect_field_rejected("extra", 1);
}

TEST(ZoomJson, LimitsComeFromConfig)
{
  ZoomConfig limits;
  limits.factors = {2};
  limits.max_max_px = 800;
  json j = valid_request();
  j["zoom"] = 4;
  EXPECT_TRUE(std::holds_alternative<ParseError>(head_camera_cpp::parse_request(j.dump(), limits)));
  j["zoom"] = 2;
  j["max_px"] = 900;
  EXPECT_TRUE(std::holds_alternative<ParseError>(head_camera_cpp::parse_request(j.dump(), limits)));
  j["max_px"] = 800;
  EXPECT_TRUE(std::holds_alternative<ZoomRequest>(head_camera_cpp::parse_request(j.dump(), limits)));
}

TEST(ZoomJson, UnusableRequestIdGivesNoId)
{
  json j = valid_request();
  j.erase("request_id");
  expect_error(j.dump(), std::nullopt);
  for (const json & bad : {json(""), json(7), json(nullptr)}) {
    j["request_id"] = bad;
    expect_error(j.dump(), std::nullopt);
  }
}

TEST(ZoomJson, NotJsonOrNotObject)
{
  expect_error("not json", std::nullopt);
  expect_error("", std::nullopt);
  expect_error("[1, 2]", std::nullopt);
  expect_error("42", std::nullopt);
  expect_error("\"text\"", std::nullopt);
}

namespace
{
ZoomResult ok_result()
{
  ZoomResult r;
  r.request_id = "r1";
  r.status = ZoomStatus::Ok;
  r.message = "";
  r.af_state = "focused";
  r.lens_position = 2.5;
  r.crop = std::array<int, 4>{1000, 600, 1152, 648};
  r.out = std::array<int, 2>{1024, 576};
  r.stamp_ns = 1700000000123456789LL;
  r.phase_ms = {120.0, 300.5, 90.0, 410.0};
  return r;
}

}  // namespace

TEST(ZoomJson, OkResultHasExactlyTheContractKeys)
{
  const json j = json::parse(head_camera_cpp::to_json(ok_result()));
  const std::set<std::string> expected{"request_id", "status", "message",  "af_state", "lens_position", "crop",
                                       "full",       "out",    "stamp_ns", "stamp_s",  "phase_ms"};
  std::set<std::string> keys;
  for (const auto & item : j.items()) {
    keys.insert(item.key());
  }
  EXPECT_EQ(keys, expected);
  EXPECT_EQ(j["status"], "ok");
  EXPECT_TRUE(j["stamp_ns"].is_number_integer());
  EXPECT_EQ(j["stamp_ns"].get<int64_t>(), 1700000000123456789LL);
  EXPECT_NEAR(j["stamp_s"].get<double>(), 1700000000.123456789, 1e-3);
  EXPECT_TRUE(j["lens_position"].is_number());
  EXPECT_EQ(j["crop"], json::array({1000, 600, 1152, 648}));
  EXPECT_EQ(j["full"], json::array({4608, 2592}));
  EXPECT_EQ(j["out"], json::array({1024, 576}));
  const std::set<std::string> phases{"af", "switch", "capture", "restore"};
  std::set<std::string> phase_keys;
  for (const auto & item : j["phase_ms"].items()) {
    phase_keys.insert(item.key());
    EXPECT_TRUE(item.value().is_number());
  }
  EXPECT_EQ(phase_keys, phases);
  EXPECT_DOUBLE_EQ(j["phase_ms"]["switch"].get<double>(), 300.5);
}

TEST(ZoomJson, BusyResultWritesNulls)
{
  ZoomResult r;
  r.request_id = "r9";
  r.status = ZoomStatus::Busy;
  r.message = "another zoom is running";
  const json j = json::parse(head_camera_cpp::to_json(r));
  EXPECT_EQ(j["status"], "busy");
  EXPECT_TRUE(j["crop"].is_null());
  EXPECT_TRUE(j["out"].is_null());
  EXPECT_TRUE(j["lens_position"].is_null());
  EXPECT_TRUE(j["stamp_ns"].is_number_integer());
  EXPECT_TRUE(j["full"].is_array());
  EXPECT_EQ(j.size(), 11u);
}

TEST(ZoomJson, StatusStringsAndImageCarrying)
{
  EXPECT_EQ(head_camera_cpp::to_string(ZoomStatus::Ok), "ok");
  EXPECT_EQ(head_camera_cpp::to_string(ZoomStatus::AfFailed), "af_failed");
  EXPECT_EQ(head_camera_cpp::to_string(ZoomStatus::Busy), "busy");
  EXPECT_EQ(head_camera_cpp::to_string(ZoomStatus::Timeout), "timeout");
  EXPECT_EQ(head_camera_cpp::to_string(ZoomStatus::CameraError), "camera_error");
  EXPECT_EQ(head_camera_cpp::to_string(ZoomStatus::Invalid), "invalid");
  EXPECT_TRUE(head_camera_cpp::carries_image(ZoomStatus::Ok));
  EXPECT_TRUE(head_camera_cpp::carries_image(ZoomStatus::AfFailed));
  EXPECT_FALSE(head_camera_cpp::carries_image(ZoomStatus::Busy));
  EXPECT_FALSE(head_camera_cpp::carries_image(ZoomStatus::Timeout));
  EXPECT_FALSE(head_camera_cpp::carries_image(ZoomStatus::CameraError));
  EXPECT_FALSE(head_camera_cpp::carries_image(ZoomStatus::Invalid));
}

TEST(ZoomJson, InvalidResultFromParseError)
{
  const ParseError error{std::string("abc"), "zoom: must be one of 2, 4, 8"};
  const ZoomResult r = head_camera_cpp::invalid_result(error);
  EXPECT_EQ(r.request_id, "abc");
  EXPECT_EQ(r.status, ZoomStatus::Invalid);
  EXPECT_EQ(r.message, error.message);
  const json j = json::parse(head_camera_cpp::to_json(r));
  EXPECT_EQ(j["status"], "invalid");
  EXPECT_EQ(j["request_id"], "abc");
}
