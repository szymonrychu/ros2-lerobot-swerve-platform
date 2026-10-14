#include "depthd/health.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace depthd
{
namespace
{
std::string no_depth_for(double age_ms)
{
  std::ostringstream r;
  r << "no depth frame for " << std::fixed << std::setprecision(1) << age_ms / 1000.0 << " s";
  return r.str();
}
}  // namespace

Health compose_health(const HealthInputs & in)
{
  Health h;
  h.fps = in.fps;
  h.last_pair_age_ms = in.last_pair_age_ms;
  h.pairs_dropped_total = in.pairs_dropped_total;
  h.gpu_temp_c = in.gpu_temp_c;
  h.nvgpu_errors_total = in.nvgpu_errors_total;
  h.matcher = in.matcher;
  h.mono_state = in.mono_state;

  if (!in.config_error.empty()) {
    h.state = "config_error";
    h.reason = in.config_error;
  } else if (in.capture.state == CaptureState::camera_error) {
    h.state = "camera_error";
    h.reason = in.capture.reason;
  } else if (!in.calibrated) {
    h.state = "uncalibrated";
    h.reason = in.calibration_reason.empty() ? "no calibration loaded" : in.calibration_reason;
  } else if (!in.startup_reason.empty()) {
    h.state = "starting";
    h.reason = in.startup_reason;
  } else if (in.depth_published && in.last_depth_age_ms > in.degraded_after_ms) {
    h.state = "degraded";
    h.reason = no_depth_for(in.last_depth_age_ms);
    if (in.capture.state == CaptureState::starting && !in.capture.reason.empty()) h.reason += ": " + in.capture.reason;
  } else if (in.capture.state == CaptureState::starting) {
    h.state = "starting";
    h.reason = in.capture.reason;
  } else if (!in.depth_published) {
    h.state = "starting";
    h.reason = "waiting for the first depth frame";
  } else {
    h.state = "ok";
    if (in.last_depth_age_ms > kDepthStallMs) h.reason = no_depth_for(in.last_depth_age_ms);
  }
  return h;
}

std::optional<double> read_gpu_temp_c(const std::string & thermal_root)
{
  DIR * dir = ::opendir(thermal_root.c_str());
  if (!dir) return std::nullopt;
  std::optional<double> out;
  while (dirent * e = ::readdir(dir)) {
    const std::string name = e->d_name;
    if (name.rfind("thermal_zone", 0) != 0) continue;
    std::ifstream type_file(thermal_root + "/" + name + "/type");
    std::string type;
    if (!(type_file >> type) || type != "GPU-therm") continue;
    std::ifstream temp_file(thermal_root + "/" + name + "/temp");
    long long milli = 0;
    if (temp_file >> milli) out = static_cast<double>(milli) / 1000.0;
    break;
  }
  ::closedir(dir);
  return out;
}

bool is_nvgpu_error(const std::string & record)
{
  if (record.empty() || record[0] == ' ') return false;
  const std::size_t semi = record.find(';');
  if (semi == std::string::npos) return false;
  const std::size_t comma = record.find(',');
  if (comma == std::string::npos || comma > semi || comma == 0) return false;
  char * end = nullptr;
  const long prio = std::strtol(record.c_str(), &end, 10);
  if (end != record.c_str() + comma || prio < 0) return false;
  if ((prio & 7) > 3) return false;  // syslog level: 0 emerg .. 3 err
  const std::string message = record.substr(semi + 1);
  return message.find("nvgpu") != std::string::npos || message.find("gk20a") != std::string::npos;
}

NvgpuErrorCounter::NvgpuErrorCounter(const std::string & path)
{
  if (path.empty()) {
    error_ = "no kernel log path";
    return;
  }
  fd_ = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) error_ = path + ": " + std::strerror(errno);
}

NvgpuErrorCounter::~NvgpuErrorCounter()
{
  if (fd_ >= 0) ::close(fd_);
}

void NvgpuErrorCounter::consume(const char * data, std::size_t size)
{
  partial_.append(data, size);
  std::size_t start = 0;
  for (std::size_t nl = partial_.find('\n'); nl != std::string::npos; nl = partial_.find('\n', start)) {
    if (is_nvgpu_error(partial_.substr(start, nl - start))) ++total_;
    start = nl + 1;
  }
  partial_.erase(0, start);
}

uint64_t NvgpuErrorCounter::poll()
{
  if (fd_ < 0) return total_;
  char buf[8192];
  for (;;) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      consume(buf, static_cast<std::size_t>(n));
      continue;
    }
    if (n < 0 && errno == EPIPE) continue;  // /dev/kmsg: records were overwritten before we read them; go on
    if (n < 0 && errno == EINTR) continue;
    break;  // 0 (end of a plain file) or EAGAIN (no new kernel record)
  }
  return total_;
}

void RateMeter::tick(int64_t now_ns)
{
  ticks_.push_back(now_ns);
  while (ticks_.size() > kWindow) ticks_.pop_front();
}

double RateMeter::rate(int64_t now_ns) const
{
  if (ticks_.size() < 2 || now_ns - ticks_.back() > kStaleNs) return 0.0;
  const double span_s = static_cast<double>(ticks_.back() - ticks_.front()) / 1e9;
  return span_s > 0.0 ? static_cast<double>(ticks_.size() - 1) / span_s : 0.0;
}

}  // namespace depthd
