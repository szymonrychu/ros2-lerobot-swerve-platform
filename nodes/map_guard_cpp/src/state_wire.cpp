#include "map_guard_cpp/state_wire.hpp"

#include <nlohmann/json.hpp>

namespace map_guard_cpp
{

namespace
{

nlohmann::ordered_json optional_number(const std::optional<double> & value)
{
  return value ? nlohmann::ordered_json(*value) : nlohmann::ordered_json(nullptr);
}

}  // namespace

std::string state_json(const GuardStatus & status)
{
  return nlohmann::ordered_json{
    {"state", state_name(status.state)},
    {"reasons", status.reasons},
    {"agreement_ratio", optional_number(status.agreement_ratio)},
    {"ratio_age_s", optional_number(status.ratio_age_s)},
    {"median_ratio", optional_number(status.median_ratio)},
    {"healthy_ratio", status.healthy_ratio},
    {"dry_run", status.dry_run},
    {"held", status.held},
    {"stamp", status.stamp}}
    .dump();
}

}  // namespace map_guard_cpp
