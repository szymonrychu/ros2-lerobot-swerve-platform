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

bool state_is_ready(const DrivePoseState & state, int64_t start_ns)
{
  if (state.stamp_ns < start_ns) {
    return false;
  }
  return state.state == LIFTED || state.state == DISABLED || state.state.rfind(SKIPPED_PREFIX, 0) == 0;
}

Readiness evaluate(const std::optional<DrivePoseState> & latest, int64_t start_ns, double elapsed_s, double timeout_s)
{
  if (latest.has_value() && state_is_ready(*latest, start_ns)) {
    return Readiness{true, latest->state};
  }
  if (elapsed_s >= timeout_s) {
    return Readiness{true, TIMEOUT};
  }
  return Readiness{false, ""};
}

}  // namespace arm_drive_wait_bt_cpp
