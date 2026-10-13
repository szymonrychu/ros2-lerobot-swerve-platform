#include "ros2_common_cpp/process_collector.hpp"

#include <sys/resource.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ros2_common_cpp
{

namespace
{

// Fields of <proc>/stat (1-based, as in proc(5)) read by prometheus_client's ProcessCollector.
constexpr std::size_t STAT_FIRST_AFTER_COMM = 3;
constexpr std::size_t STAT_UTIME = 14;
constexpr std::size_t STAT_STIME = 15;
constexpr std::size_t STAT_STARTTIME = 22;
constexpr std::size_t STAT_VSIZE = 23;
constexpr std::size_t STAT_RSS = 24;

// The whitespace-separated fields of <proc>/stat after the ")" that ends comm, which may contain spaces.
// Index 0 is field 3 (state). Empty when the file cannot be read.
std::vector<std::string> stat_fields_after_comm(const std::string & proc)
{
  std::ifstream file(proc + "/stat");
  std::string line;
  if (!file || !std::getline(file, line)) {
    return {};
  }
  const auto close = line.rfind(')');
  if (close == std::string::npos) {
    return {};
  }
  std::istringstream rest(line.substr(close + 1));
  std::vector<std::string> fields;
  for (std::string field; rest >> field;) {
    fields.push_back(field);
  }
  return fields;
}

// The btime line of /proc/stat (boot time, epoch seconds), or -1.
double boot_time_s()
{
  std::ifstream file(PROC_STAT);
  for (std::string line; std::getline(file, line);) {
    if (line.rfind("btime ", 0) == 0) {
      return std::stod(line.substr(6));
    }
  }
  return -1.0;
}

prometheus::MetricFamily family(
  const std::string & name, const std::string & help, prometheus::MetricType type, double value)
{
  prometheus::ClientMetric metric;
  if (type == prometheus::MetricType::Counter) {
    metric.counter.value = value;
  } else {
    metric.gauge.value = value;
  }
  return prometheus::MetricFamily{name, help, type, {metric}};
}

}  // namespace

ProcessCollector::ProcessCollector(std::string proc_self) : proc_self_(std::move(proc_self)) {}

std::vector<prometheus::MetricFamily> ProcessCollector::Collect() const
{
  const auto fields = stat_fields_after_comm(proc_self_);
  if (fields.size() <= STAT_RSS - STAT_FIRST_AFTER_COMM) {
    return {};
  }
  const double ticks = static_cast<double>(sysconf(_SC_CLK_TCK));
  const double page = static_cast<double>(sysconf(_SC_PAGESIZE));
  const double btime = boot_time_s();
  if (ticks <= 0.0 || page <= 0.0 || btime < 0.0) {
    return {};
  }
  const auto field = [&fields](std::size_t number) { return std::stod(fields[number - STAT_FIRST_AFTER_COMM]); };

  std::size_t open_fds = 0;
  std::error_code error;
  for (std::filesystem::directory_iterator it(proc_self_ + "/fd", error), end; !error && it != end;
       it.increment(error)) {
    ++open_fds;
  }
  if (error) {
    return {};
  }
  rlimit limit{};
  if (getrlimit(RLIMIT_NOFILE, &limit) != 0) {
    return {};
  }

  using prometheus::MetricType;
  return {
    family(
      "process_cpu_seconds_total", "Total user and system CPU time spent in seconds.", MetricType::Counter,
      (field(STAT_UTIME) + field(STAT_STIME)) / ticks),
    family("process_virtual_memory_bytes", "Virtual memory size in bytes.", MetricType::Gauge, field(STAT_VSIZE)),
    family(
      "process_resident_memory_bytes", "Resident memory size in bytes.", MetricType::Gauge, field(STAT_RSS) * page),
    family(
      "process_start_time_seconds", "Start time of the process since unix epoch in seconds.", MetricType::Gauge,
      field(STAT_STARTTIME) / ticks + btime),
    family("process_open_fds", "Number of open file descriptors.", MetricType::Gauge, static_cast<double>(open_fds)),
    family(
      "process_max_fds", "Maximum number of open file descriptors.", MetricType::Gauge,
      static_cast<double>(limit.rlim_cur)),
  };
}

}  // namespace ros2_common_cpp
