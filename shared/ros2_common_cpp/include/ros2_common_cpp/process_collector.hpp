// The process_* series that prometheus_client's ProcessCollector exports for every Python node, read from Linux
// /proc on each collection (each scrape), as in Python. Every ros2_common_cpp::Metrics registers one, so every C++
// node exports the same CPU and memory series as its Python counterpart (a cut-over CPU check reads
// process_cpu_seconds_total for both). Unlabelled, as in Python:
//   process_cpu_seconds_total, process_virtual_memory_bytes, process_resident_memory_bytes,
//   process_start_time_seconds, process_open_fds, process_max_fds
#ifndef ROS2_COMMON_CPP__PROCESS_COLLECTOR_HPP_
#define ROS2_COMMON_CPP__PROCESS_COLLECTOR_HPP_

#include <prometheus/collectable.h>
#include <prometheus/metric_family.h>

#include <string>
#include <vector>

namespace ros2_common_cpp
{

// The proc directory of the current process.
constexpr const char * PROC_SELF = "/proc/self";
// The system-wide proc stat file holding btime (boot time, epoch seconds).
constexpr const char * PROC_STAT = "/proc/stat";

class ProcessCollector : public prometheus::Collectable
{
public:
  // proc_self: the process directory to read (tests pass an unreadable one).
  explicit ProcessCollector(std::string proc_self = PROC_SELF);

  // Reads proc_self and returns the six families. Returns no family at all when anything cannot be read, e.g. on a
  // non-Linux host (no placeholder data).
  std::vector<prometheus::MetricFamily> Collect() const override;

private:
  std::string proc_self_;
};

}  // namespace ros2_common_cpp

#endif  // ROS2_COMMON_CPP__PROCESS_COLLECTOR_HPP_
