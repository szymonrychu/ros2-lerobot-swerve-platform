// The status contract of the 3D map manager: one JSON line on status_topic (/map3d/status_server), built from plain
// inputs so it needs no ROS.
//
//   {"stamp_s": number,                       // now
//    "state": "mapping"|"paused"|"waiting_for_inputs"|"error",
//    "nodes": int|null,                       // node count of the last mapGraph, null before the first one
//    "db_size_mb": number|null,               // size of the database file, null when unknown
//    "memory_mb": number|null,                // cgroup memory.current of the RTAB-Map unit, null when unknown
//    "graph_correction_m": number|null,       // |translation| of MapGraph.map_to_odom
//    "graph_correction_deg": number|null,     // |yaw| of MapGraph.map_to_odom, degrees
//    "correction_warn_m": number, "correction_warn_deg": number,   // the configured thresholds
//    "correction_ok": bool|null,              // both corrections <= their thresholds; null until a graph arrived
//    "missing_inputs": [str],                 // of rgb_camera_info, depth_camera_info, rtabmap_info
//    "last_update_age_s": number|null,        // now - stamp of the last mapGraph, null before the first one
//    "last_save": {"name": str, "stamp_s": number}|null,
//    "message": str}
//
// graph_correction_* is RTAB-Map's MapGraph.map_to_odom: identity while the 3D map shares slam_toolbox's map frame
// (RTAB-Map publishes no map->odom TF of its own). Anything else means RTAB-Map built its own links and the 3D map
// has left slam_toolbox's frame; correction_ok is then false.
//
// Rules: an input is missing when never seen or older than Config::input_stale_s. State: "waiting_for_inputs" when
// any input is missing and not paused; "paused" when paused; "error" when a last error is set and is at most
// ERROR_WINDOW_S old; else "mapping". Rules apply in this order, so waiting and paused outrank error. Unknown values are JSON null, never 0.
#ifndef MAP3D_MANAGER_CPP__STATUS_HPP_
#define MAP3D_MANAGER_CPP__STATUS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "map3d_manager_cpp/config.hpp"

namespace map3d_manager_cpp
{

constexpr double ERROR_WINDOW_S = 60.0;
constexpr double BYTES_PER_MB = 1024.0 * 1024.0;
constexpr const char * INPUT_RGB_CAMERA_INFO = "rgb_camera_info";
constexpr const char * INPUT_DEPTH_CAMERA_INFO = "depth_camera_info";
constexpr const char * INPUT_RTABMAP_INFO = "rtabmap_info";
constexpr const char * DEFAULT_CGROUP_ROOT = "/sys/fs/cgroup/system.slice";

struct LastSave
{
  std::string name;
  double stamp_s = 0.0;
};

// The last mapGraph: its stamp, node count and map_to_odom (translation in metres, yaw in radians).
struct GraphSample
{
  double stamp_s = 0.0;
  int nodes = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw_rad = 0.0;
};

// Everything the status line is derived from. Times are seconds on one clock (the node's wall clock).
struct StatusInputs
{
  double now_s = 0.0;
  std::optional<GraphSample> graph;          // last mapGraph
  std::optional<double> info_stamp_s;        // last rtabmap Info
  std::optional<double> rgb_info_stamp_s;    // last rgb camera_info
  std::optional<double> depth_info_stamp_s;  // last depth camera_info
  bool paused = false;
  std::optional<std::uint64_t> db_size_bytes;
  std::optional<std::uint64_t> memory_bytes;
  std::optional<LastSave> last_save;
  std::string last_error;                    // empty when none
  std::optional<double> last_error_stamp_s;  // when last_error was set
};

// The built status; optional members are JSON null when unset.
struct StatusReport
{
  double stamp_s = 0.0;
  std::string state;
  std::optional<int> nodes;
  std::optional<double> db_size_mb;
  std::optional<double> memory_mb;
  std::optional<double> graph_correction_m;
  std::optional<double> graph_correction_deg;
  double correction_warn_m = 0.0;
  double correction_warn_deg = 0.0;
  std::optional<bool> correction_ok;
  std::vector<std::string> missing_inputs;
  std::optional<double> last_update_age_s;
  std::optional<LastSave> last_save;
  std::string message;
};

// Derive the status from the inputs and the thresholds/staleness of the config.
StatusReport build_status(const StatusInputs & inputs, const Config & config);

// The JSON line of a status (see the header comment).
std::string to_json(const StatusReport & status);

// Bytes in <root>/<unit>/memory.current, nullopt when the file is absent or unparsable.
std::optional<std::uint64_t> read_cgroup_memory_bytes(
  const std::string & unit, const std::string & root = DEFAULT_CGROUP_ROOT);

}  // namespace map3d_manager_cpp

#endif  // MAP3D_MANAGER_CPP__STATUS_HPP_
