#include "map_guard_cpp/process_metrics.hpp"

#include <sys/resource.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace map_guard_cpp
{

namespace
{

// Fields of /proc/self/stat (1-based, as in proc(5)) read by prometheus_client's ProcessCollector.
constexpr std::size_t STAT_UTIME = 14;
constexpr std::size_t STAT_STIME = 15;
constexpr std::size_t STAT_STARTTIME = 22;
constexpr std::size_t STAT_VSIZE = 23;
constexpr std::size_t STAT_RSS = 24;

// The whitespace-separated fields of /proc/self/stat after the ")" that ends comm, which may contain spaces.
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

// The btime (boot time, epoch seconds) line of /proc/stat, or -1.
double boot_time_s()
{
  std::ifstream file("/proc/stat");
  for (std::string line; std::getline(file, line);) {
    if (line.rfind("btime ", 0) == 0) {
      return std::stod(line.substr(6));
    }
  }
  return -1.0;
}

}  // namespace

ProcessMetrics::ProcessMetrics(ros2_common_cpp::Metrics & metrics) : metrics_(metrics) {}

bool ProcessMetrics::update()
{
  const auto fields = stat_fields_after_comm(PROC_SELF);
  constexpr std::size_t FIRST = 3;  // field number of fields[0]
  if (fields.size() <= STAT_RSS - FIRST) {
    return false;
  }
  const double ticks = static_cast<double>(sysconf(_SC_CLK_TCK));
  const double page = static_cast<double>(sysconf(_SC_PAGESIZE));
  const double btime = boot_time_s();
  if (ticks <= 0.0 || page <= 0.0 || btime < 0.0) {
    return false;
  }
  const auto field = [&fields](std::size_t number) { return std::stod(fields[number - FIRST]); };
  const double cpu_seconds = (field(STAT_UTIME) + field(STAT_STIME)) / ticks;

  std::size_t open_fds = 0;
  std::error_code error;
  for (std::filesystem::directory_iterator it(std::string(PROC_SELF) + "/fd", error), end; !error && it != end;
       it.increment(error)) {
    ++open_fds;
  }
  if (error) {
    return false;
  }
  rlimit limit{};
  if (getrlimit(RLIMIT_NOFILE, &limit) != 0) {
    return false;
  }

  if (cpu_seconds_ == nullptr) {
    cpu_seconds_ = &metrics_.counter("process_cpu_seconds_total", "Total user and system CPU time spent in seconds.");
  }
  if (cpu_seconds > last_cpu_seconds_) {
    cpu_seconds_->Increment(cpu_seconds - last_cpu_seconds_);
    last_cpu_seconds_ = cpu_seconds;
  }
  metrics_.gauge("process_virtual_memory_bytes", "Virtual memory size in bytes.").Set(field(STAT_VSIZE));
  metrics_.gauge("process_resident_memory_bytes", "Resident memory size in bytes.").Set(field(STAT_RSS) * page);
  metrics_.gauge("process_start_time_seconds", "Start time of the process since unix epoch in seconds.")
    .Set(field(STAT_STARTTIME) / ticks + btime);
  metrics_.gauge("process_open_fds", "Number of open file descriptors.").Set(static_cast<double>(open_fds));
  metrics_.gauge("process_max_fds", "Maximum number of open file descriptors.")
    .Set(static_cast<double>(limit.rlim_cur));
  return true;
}

bool ProcessMetrics::update_if_due(double now)
{
  if (updated_ && now - last_update_ < PROCESS_METRICS_PERIOD_S) {
    return false;
  }
  updated_ = true;
  last_update_ = now;
  return update();
}

}  // namespace map_guard_cpp
