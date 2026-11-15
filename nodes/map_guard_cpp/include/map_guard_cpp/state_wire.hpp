// The guard state published on state_topic (default /map_guard/state) for mcp_server's localisation health. ROS-free
// so tests link it.
#ifndef MAP_GUARD_CPP__STATE_WIRE_HPP_
#define MAP_GUARD_CPP__STATE_WIRE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "map_guard_cpp/guard.hpp"

namespace map_guard_cpp
{

// One snapshot of the guard. Values that were never measured stay nullopt and are published as null.
struct GuardStatus
{
  State state = State::OK;
  std::vector<std::string> reasons;       // the current episode's reasons, sorted; empty in OK
  std::optional<double> agreement_ratio;  // latest scan-to-map agreement sample
  std::optional<double> ratio_age_s;      // seconds since that sample
  int unscored_scans = 0;                 // scans since the last scored one with too few endpoints on known map cells
  std::optional<double> median_ratio;     // rolling median of the mismatch monitor
  double healthy_ratio = 0.0;
  bool dry_run = true;
  bool held = false;   // map_manager reports the loaded map as not localised
  double stamp = 0.0;  // epoch seconds of the snapshot
};

// The status as a JSON object: state, reasons, agreement_ratio, ratio_age_s, unscored_scans, median_ratio,
// healthy_ratio, dry_run, held and stamp, in that order (null for a missing value).
std::string state_json(const GuardStatus & status);

// True when next differs from the last published status in state, held or reasons: such a change is published at
// once, not only at state_publish_hz. Ratios, ages, counts and the stamp alone wait for the periodic publish.
bool status_changed(const GuardStatus & published, const GuardStatus & next);

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__STATE_WIRE_HPP_
