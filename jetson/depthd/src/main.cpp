// depthd: the Jetson depth service.
//
//   depthd [--config PATH]        run (default /etc/depthd/config.yaml) until SIGTERM or SIGINT
//   depthd --check-config PATH    validate a config file: exit 0 ("ok: PATH") or 1 (the reason on stderr)
//
// Exit codes: 0 after a clean stop or a valid --check-config, 1 for an invalid config file, 2 for a usage error.
// Config and calibration errors at startup do not exit (see daemon.hpp): systemd would only restart into the same
// error, so depthd stays up and /health names it.
#include <signal.h>
#include <time.h>

#include <cstring>
#include <iostream>
#include <string>

#include "depthd/daemon.hpp"

namespace
{
constexpr const char * kDefaultConfig = "/etc/depthd/config.yaml";

int usage(const char * argv0)
{
  std::cerr << "usage: " << argv0 << " [--config PATH] | --check-config PATH\n";
  return 2;
}

void log_line(const std::string & line) { std::cerr << "depthd: " << line << std::endl; }
}  // namespace

int main(int argc, char ** argv)
{
  std::string config_path = kDefaultConfig;
  std::string check_path;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "--config" || arg == "--check-config") && i + 1 < argc) {
      (arg == "--config" ? config_path : check_path) = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      return 0;
    } else {
      return usage(argv[0]);
    }
  }
  if (!check_path.empty()) {
    std::string message;
    const int rc = depthd::check_config(check_path, message);
    (rc == 0 ? std::cout : std::cerr) << message << std::endl;
    return rc;
  }

  // Block the stop signals before any thread exists, so every thread inherits the mask and only sigwait sees them.
  sigset_t stop_signals;
  sigemptyset(&stop_signals);
  sigaddset(&stop_signals, SIGTERM);
  sigaddset(&stop_signals, SIGINT);
  pthread_sigmask(SIG_BLOCK, &stop_signals, nullptr);
  signal(SIGPIPE, SIG_IGN);

  depthd::DaemonOptions options;
  options.config_path = config_path;
  options.log = log_line;
  depthd::Daemon daemon(options);
  log_line("starting with " + config_path);
  daemon.start();

  int sig = 0;
  sigwait(&stop_signals, &sig);
  log_line(std::string("stopping on ") + (sig == SIGTERM ? "SIGTERM" : "SIGINT"));
  daemon.stop();
  return 0;
}
