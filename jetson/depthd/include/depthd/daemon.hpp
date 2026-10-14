// The depthd service: config, HTTP API, publisher, frame source and pipeline wired together, with the startup rules
// systemd and Ansible rely on.
//
//   - A config error (or a matcher the config names that cannot be built) is logged and the service stays up serving
//     /health (state config_error, the reason) on the default bind address, without opening a camera or publishing.
//   - The monocular stage (mono.hpp) runs beside the pipeline; a model that does not load is mono_state "error: ..."
//     in /health and affects nothing else.
//   - A calibration error leaves the pipeline uncalibrated: /health says why, rgb flows, depth and fused do not.
//   - A bind that fails (the link address is not up yet) is retried every retry_interval_ms; a camera start that fails
//     is retried every source_retry_interval_ms. Neither exits, so there is no crash loop.
//   - stop() returns within 2 s (systemd's SIGTERM path in main.cpp).
#ifndef DEPTHD_DAEMON_HPP_
#define DEPTHD_DAEMON_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "depthd/config.hpp"
#include "depthd/http_api.hpp"
#include "depthd/mono.hpp"
#include "depthd/pipeline.hpp"

namespace depthd
{

struct DaemonOptions
{
  std::string config_path = "/etc/depthd/config.yaml";
  // Builds the frame source once the config is valid; default: StereoCaptureSource (the cameras).
  std::function<std::unique_ptr<FrameSource>(const Config &, LogFn)> make_source;
  PipelineOptions pipeline;
  // Builds the monocular model (MonoStage); default: TrtMonoModel(mono.engine_path).
  MonoModelFactory make_mono_model;
  LogFn log;
  std::string bind_address;  // overrides transport.bind_address when set (tests: 127.0.0.1)
  int http_port = -1;        // -1: transport.http_port; 0: any free port
  int pub_port = -1;         // -1: transport.pub_port; 0: any free port
  int64_t retry_interval_ms = 2000;
  int64_t source_retry_interval_ms = 5000;
};

// `depthd --check-config <path>`: 0 and "ok: <path>" for a valid file, 1 and the reason otherwise.
int check_config(const std::string & path, std::string & message);

class Daemon
{
public:
  explicit Daemon(DaemonOptions options);
  ~Daemon();
  Daemon(const Daemon &) = delete;
  Daemon & operator=(const Daemon &) = delete;

  // Never throws for config, calibration, bind or camera errors: they are logged and reported by health().
  void start();
  void stop();

  Health health() const;
  int http_port() const;  // 0 until the HTTP API is bound
  int pub_port() const;   // 0 until the publisher is bound

  struct Impl;

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace depthd

#endif  // DEPTHD_DAEMON_HPP_
