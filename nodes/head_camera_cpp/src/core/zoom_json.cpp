#include "head_camera_cpp/zoom_json.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include <set>

namespace head_camera_cpp
{

namespace
{

using nlohmann::json;

constexpr const char * KEY_REQUEST_ID = "request_id";
constexpr double NS_PER_S = 1e9;

// True when j is a number holding an integral value (2 and 2.0 count, 2.5 does not).
bool is_integral(const json & j)
{
  if (j.is_number_integer()) {
    return true;
  }
  return j.is_number_float() && std::floor(j.get<double>()) == j.get<double>();
}

// Read a required number in [0, 1] into out; return an error text or empty.
std::string read_unit(const json & j, const char * key, double & out)
{
  const auto it = j.find(key);
  if (it == j.end()) {
    return std::string(key) + ": missing";
  }
  if (!it->is_number()) {
    return std::string(key) + ": must be a number";
  }
  out = it->get<double>();
  if (!(out >= 0.0 && out <= 1.0)) {
    return std::string(key) + ": must be in [0, 1]";
  }
  return "";
}

}  // namespace

std::variant<ZoomRequest, ParseError> parse_request(const std::string & text, const ZoomConfig & limits)
{
  const json j = json::parse(text, nullptr, false);
  if (j.is_discarded()) {
    return ParseError{std::nullopt, "not valid JSON"};
  }
  if (!j.is_object()) {
    return ParseError{std::nullopt, "not a JSON object"};
  }
  const auto id_it = j.find(KEY_REQUEST_ID);
  if (id_it == j.end() || !id_it->is_string() || id_it->get<std::string>().empty()) {
    return ParseError{std::nullopt, "request_id: must be a non-empty string"};
  }
  ZoomRequest request;
  request.request_id = id_it->get<std::string>();
  request.max_px = limits.default_max_px;
  const auto fail = [&request](const std::string & message) { return ParseError{request.request_id, message}; };

  static const std::set<std::string> KNOWN{KEY_REQUEST_ID, "u", "v", "zoom", "max_px", "af"};
  for (const auto & item : j.items()) {
    if (KNOWN.count(item.key()) == 0) {
      return fail("unknown key: " + item.key());
    }
  }
  for (const auto & [key, out] : {std::pair<const char *, double *>{"u", &request.u}, {"v", &request.v}}) {
    const std::string error = read_unit(j, key, *out);
    if (!error.empty()) {
      return fail(error);
    }
  }
  const auto zoom_it = j.find("zoom");
  if (zoom_it == j.end()) {
    return fail("zoom: missing");
  }
  if (!is_integral(*zoom_it)) {
    return fail("zoom: must be an integer");
  }
  const double zoom = zoom_it->get<double>();
  const auto factor = std::find_if(
    limits.factors.begin(), limits.factors.end(), [zoom](int f) { return static_cast<double>(f) == zoom; });
  if (factor == limits.factors.end()) {
    return fail("zoom: not an allowed factor");
  }
  request.zoom = *factor;
  const auto px_it = j.find("max_px");
  if (px_it != j.end()) {
    if (!is_integral(*px_it)) {
      return fail("max_px: must be an integer");
    }
    const double px = px_it->get<double>();
    if (px < limits.min_max_px || px > limits.max_max_px) {
      return fail(
        "max_px: must be in [" + std::to_string(limits.min_max_px) + ", " + std::to_string(limits.max_max_px) + "]");
    }
    request.max_px = static_cast<int>(px);
  }
  const auto af_it = j.find("af");
  if (af_it != j.end()) {
    if (!af_it->is_boolean()) {
      return fail("af: must be a boolean");
    }
    request.af = af_it->get<bool>();
  }
  return request;
}

std::string to_string(ZoomStatus status)
{
  switch (status) {
    case ZoomStatus::Ok:
      return "ok";
    case ZoomStatus::AfFailed:
      return "af_failed";
    case ZoomStatus::Busy:
      return "busy";
    case ZoomStatus::Timeout:
      return "timeout";
    case ZoomStatus::CameraError:
      return "camera_error";
    case ZoomStatus::Invalid:
      return "invalid";
  }
  return "invalid";
}

bool carries_image(ZoomStatus status) { return status == ZoomStatus::Ok || status == ZoomStatus::AfFailed; }

std::string to_json(const ZoomResult & result)
{
  json j;
  j[KEY_REQUEST_ID] = result.request_id;
  j["status"] = to_string(result.status);
  j["message"] = result.message;
  j["af_state"] = result.af_state;
  j["lens_position"] = result.lens_position ? json(*result.lens_position) : json(nullptr);
  j["crop"] = result.crop ? json(*result.crop) : json(nullptr);
  j["full"] = result.full;
  j["out"] = result.out ? json(*result.out) : json(nullptr);
  j["stamp_ns"] = result.stamp_ns;
  j["stamp_s"] = static_cast<double>(result.stamp_ns) / NS_PER_S;
  j["phase_ms"] = {
    {"af", result.phase_ms.af},
    {"switch", result.phase_ms.switch_mode},
    {"capture", result.phase_ms.capture},
    {"restore", result.phase_ms.restore}};
  return j.dump();
}

ZoomResult invalid_result(const ParseError & error)
{
  ZoomResult result;
  result.request_id = error.request_id.value_or("");
  result.status = ZoomStatus::Invalid;
  result.message = error.message;
  return result;
}

}  // namespace head_camera_cpp
