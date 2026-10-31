#include "arm_drive_wait_bt_cpp/readiness.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace arm_drive_wait_bt_cpp
{

std::optional<DrivePoseState> parse_state(const std::string & json)
{
  const nlohmann::json doc = nlohmann::json::parse(json, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    return std::nullopt;
  }
  const auto state = doc.find("state");
  const auto reason = doc.find("reason");
  const auto enabled = doc.find("enabled");
  const auto stamp_ns = doc.find("stamp_ns");
  if (
    state == doc.end() || !state->is_string() || reason == doc.end() || !reason->is_string() || enabled == doc.end() ||
    !enabled->is_boolean() || stamp_ns == doc.end() || !stamp_ns->is_number_integer()) {
    return std::nullopt;
  }
  return DrivePoseState{
    state->get<std::string>(), reason->get<std::string>(), enabled->get<bool>(), stamp_ns->get<int64_t>()};
}

bool state_waits(const DrivePoseState & state)
{
  return state.state == LIFTING || state.state == LOWERING || (state.state == IDLE && state.enabled);
}

Readiness evaluate(const std::optional<DrivePoseState> & latest, double age_s, double elapsed_s, double timeout_s)
{
  if (!latest.has_value() || age_s > FRESH_S) {
    return Readiness{true, NO_FRESH_STATE};
  }
  if (!state_waits(*latest)) {
    return Readiness{true, latest->state};
  }
  if (elapsed_s >= timeout_s) {
    return Readiness{true, TIMEOUT};
  }
  return Readiness{false, ""};
}

}  // namespace arm_drive_wait_bt_cpp
