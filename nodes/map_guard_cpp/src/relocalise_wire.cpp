#include "map_guard_cpp/relocalise_wire.hpp"

#include <nlohmann/json.hpp>

namespace map_guard_cpp
{

std::string relocalise_command(const std::string & request_id, double since)
{
  return nlohmann::ordered_json{
    {"op", "relocalise"}, {"request_id", request_id}, {"since", since}, {"discard_on_fail", true}}
    .dump();
}

std::optional<RelocaliseResult> parse_result(const std::string & text)
{
  const auto json = nlohmann::json::parse(text, nullptr, false);
  const bool valid = json.is_object() && json.contains("request_id") && json["request_id"].is_string() &&
                     json.contains("ok") && json["ok"].is_boolean();
  if (!valid) {
    return std::nullopt;
  }
  RelocaliseResult result;
  result.request_id = json["request_id"].get<std::string>();
  result.ok = json["ok"].get<bool>();
  const auto data = json.find("data");
  if (data != json.end() && data->is_object()) {
    if (const auto state = data->find("state"); state != data->end() && state->is_string()) {
      result.state = state->get<std::string>();
    }
    if (const auto pois = data->find("discarded_pois"); pois != data->end() && pois->is_number_integer()) {
      result.discarded_pois = pois->get<int>();
    }
  }
  return result;
}

}  // namespace map_guard_cpp
