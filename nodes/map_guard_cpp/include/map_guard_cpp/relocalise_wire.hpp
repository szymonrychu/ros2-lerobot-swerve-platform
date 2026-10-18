// The wire contract with map_manager: the relocalise command and the parsing of its result. ROS-free so tests link it.
#ifndef MAP_GUARD_CPP__RELOCALISE_WIRE_HPP_
#define MAP_GUARD_CPP__RELOCALISE_WIRE_HPP_

#include <optional>
#include <string>

namespace map_guard_cpp
{

// The relocalise command JSON for map_manager (discard_on_fail: map_manager clears the live POIs on failure).
std::string relocalise_command(const std::string & request_id, double since);

// The parts of a map_manager result the guard acts on.
struct RelocaliseResult
{
  std::string request_id;
  bool ok = false;
  std::string state;
  int discarded_pois = 0;
};

// Parse a map_manager result message; nullopt when the text is not a valid result (JSON object with request_id and
// ok; data.state and data.discarded_pois are optional).
std::optional<RelocaliseResult> parse_result(const std::string & text);

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__RELOCALISE_WIRE_HPP_
