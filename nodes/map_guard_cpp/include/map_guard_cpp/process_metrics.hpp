// The process_* series that prometheus_client's ProcessCollector exports for every Python node (Linux /proc), so the
// C++ port keeps the same CPU and memory series as nodes/map_guard (the cut-over CPU check reads
// process_cpu_seconds_total). Unlabelled, as in Python:
//   process_cpu_seconds_total, process_virtual_memory_bytes, process_resident_memory_bytes,
//   process_start_time_seconds, process_open_fds, process_max_fds
#ifndef MAP_GUARD_CPP__PROCESS_METRICS_HPP_
#define MAP_GUARD_CPP__PROCESS_METRICS_HPP_

#include <prometheus/counter.h>
#include <prometheus/gauge.h>

#include "ros2_common_cpp/metrics.hpp"

namespace map_guard_cpp
{

// Root of the proc filesystem read for the current process.
constexpr const char * PROC_SELF = "/proc/self";
// Minimum seconds between two /proc reads in update_if_due.
constexpr double PROCESS_METRICS_PERIOD_S = 1.0;

class ProcessMetrics
{
public:
  // Registers the series in metrics; no value is set until the first successful update (no placeholder data).
  explicit ProcessMetrics(ros2_common_cpp::Metrics & metrics);

  // Reads /proc/self and sets every series. Returns false (and leaves the series as they were) when /proc cannot
  // be read, e.g. on a non-Linux host.
  bool update();

  // update() at most once per PROCESS_METRICS_PERIOD_S of now (monotonic seconds). Returns whether it ran and
  // succeeded.
  bool update_if_due(double now);

private:
  ros2_common_cpp::Metrics & metrics_;
  prometheus::Counter * cpu_seconds_ = nullptr;
  double last_cpu_seconds_ = 0.0;
  double last_update_ = -1.0;
  bool updated_ = false;
};

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__PROCESS_METRICS_HPP_
