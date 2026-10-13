// Prometheus metrics for the robot's C++ nodes, the counterpart of shared/ros2_metrics (Python): the same
// robot_node_info{node} = 1 and robot_node_start_time_seconds{node} series, served as Prometheus text on
// 127.0.0.1:<metrics_port> for Grafana Alloy to scrape. Built on prometheus-cpp (apt prometheus-cpp-dev).
//
//   auto port = ros2_common_cpp::resolve_metrics_port(cfg_metrics_port);   // config value, else METRICS_PORT
//   ros2_common_cpp::Metrics metrics("stereo_depth");
//   auto & frames = metrics.counter("stereo_frames_total", "Frames processed", {{"camera", "head"}});
//   auto & latency = metrics.histogram("stereo_latency_seconds", "Frame latency", {0.01, 0.05, 0.1});
//   if (port) metrics.serve(*port);
//   frames.Increment(); latency.Observe(0.02);
#ifndef ROS2_COMMON_CPP__METRICS_HPP_
#define ROS2_COMMON_CPP__METRICS_HPP_

#include <prometheus/counter.h>
#include <prometheus/exposer.h>
#include <prometheus/gauge.h>
#include <prometheus/histogram.h>
#include <prometheus/labels.h>
#include <prometheus/registry.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ros2_common_cpp
{

// Environment variable read when the node config leaves metrics_port unset (same as ros2_metrics).
constexpr const char * METRICS_PORT_ENV = "METRICS_PORT";
// Bind address: Alloy scrapes on the robot itself, so the exporter never listens beyond localhost.
constexpr const char * DEFAULT_METRICS_HOST = "127.0.0.1";

// The /metrics port: config_port when set, else METRICS_PORT, else nullopt (metrics disabled).
// Throws std::invalid_argument when a value is not a port number in 1..65535.
std::optional<std::uint16_t> resolve_metrics_port(std::optional<int> config_port);

// One registry per node. The constructor registers robot_node_info{node} = 1 and
// robot_node_start_time_seconds{node} = now (epoch seconds). counter / gauge / histogram return the child for the
// given labels, creating the family on first use; asking again with the same name and labels returns the same child.
class Metrics
{
public:
  explicit Metrics(const std::string & node);
  ~Metrics();
  Metrics(const Metrics &) = delete;
  Metrics & operator=(const Metrics &) = delete;

  prometheus::Counter & counter(
    const std::string & name, const std::string & help, const prometheus::Labels & labels = {});
  prometheus::Gauge & gauge(const std::string & name, const std::string & help, const prometheus::Labels & labels = {});
  prometheus::Histogram & histogram(
    const std::string & name, const std::string & help, const std::vector<double> & buckets,
    const prometheus::Labels & labels = {});

  // The whole registry in the Prometheus text exposition format (what GET /metrics returns).
  std::string render() const;

  // Serve GET /metrics on host:port in a background thread; port 0 picks a free port. Returns the bound port.
  // Throws std::runtime_error when the port cannot be bound or serve was already called.
  std::uint16_t serve(std::uint16_t port, const std::string & host = DEFAULT_METRICS_HOST);

  std::shared_ptr<prometheus::Registry> registry() const;

private:
  std::shared_ptr<prometheus::Registry> registry_;
  std::unique_ptr<prometheus::Exposer> exposer_;
};

}  // namespace ros2_common_cpp

#endif  // ROS2_COMMON_CPP__METRICS_HPP_
