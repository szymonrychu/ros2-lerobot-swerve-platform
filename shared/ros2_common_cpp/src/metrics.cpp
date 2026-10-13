#include "ros2_common_cpp/metrics.hpp"

#include <prometheus/text_serializer.h>

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace ros2_common_cpp
{

namespace
{

constexpr int MAX_PORT = 65535;
constexpr std::size_t EXPOSER_THREADS = 1;

std::uint16_t checked_port(long value, const std::string & source)  // NOLINT(runtime/int)
{
  if (value < 1 || value > MAX_PORT) {
    throw std::invalid_argument(source + " is not a port number (1.." + std::to_string(MAX_PORT) + ")");
  }
  return static_cast<std::uint16_t>(value);
}

}  // namespace

std::optional<std::uint16_t> resolve_metrics_port(std::optional<int> config_port)
{
  if (config_port) {
    return checked_port(*config_port, "metrics_port=" + std::to_string(*config_port));
  }
  const char * env = std::getenv(METRICS_PORT_ENV);
  std::string raw = env == nullptr ? "" : env;
  const auto first = raw.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return std::nullopt;
  }
  raw = raw.substr(first, raw.find_last_not_of(" \t\r\n") - first + 1);
  const std::string source = std::string(METRICS_PORT_ENV) + "=" + raw;
  if (raw.size() > 5 || raw.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument(source + " is not a port number (1.." + std::to_string(MAX_PORT) + ")");
  }
  return checked_port(std::stol(raw), source);
}

Metrics::Metrics(const std::string & node)
: registry_(std::make_shared<prometheus::Registry>()), process_(std::make_shared<ProcessCollector>())
{
  gauge("robot_node_info", "Our ROS2 node is running (always 1)", {{"node", node}}).Set(1);
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  gauge("robot_node_start_time_seconds", "Process start time of the node (epoch seconds)", {{"node", node}})
    .Set(std::chrono::duration<double>(now).count());
}

Metrics::~Metrics() = default;

prometheus::Counter & Metrics::counter(
  const std::string & name, const std::string & help, const prometheus::Labels & labels)
{
  return prometheus::BuildCounter().Name(name).Help(help).Register(*registry_).Add(labels);
}

prometheus::Gauge & Metrics::gauge(
  const std::string & name, const std::string & help, const prometheus::Labels & labels)
{
  return prometheus::BuildGauge().Name(name).Help(help).Register(*registry_).Add(labels);
}

prometheus::Histogram & Metrics::histogram(
  const std::string & name, const std::string & help, const std::vector<double> & buckets,
  const prometheus::Labels & labels)
{
  return prometheus::BuildHistogram()
    .Name(name)
    .Help(help)
    .Register(*registry_)
    .Add(labels, prometheus::Histogram::BucketBoundaries(buckets));
}

std::vector<prometheus::MetricFamily> Metrics::collect() const
{
  auto families = registry_->Collect();
  auto process = process_->Collect();
  families.insert(families.end(), process.begin(), process.end());
  return families;
}

std::string Metrics::render() const { return prometheus::TextSerializer().Serialize(collect()); }

std::uint16_t Metrics::serve(std::uint16_t port, const std::string & host)
{
  if (exposer_) {
    throw std::runtime_error("metrics are already served");
  }
  try {
    exposer_ = std::make_unique<prometheus::Exposer>(host + ":" + std::to_string(port), EXPOSER_THREADS);
  } catch (const std::exception & e) {
    throw std::runtime_error("cannot serve metrics on " + host + ":" + std::to_string(port) + ": " + e.what());
  }
  exposer_->RegisterCollectable(registry_);
  exposer_->RegisterCollectable(process_);
  const std::vector<int> ports = exposer_->GetListeningPorts();
  if (ports.empty()) {
    exposer_.reset();
    throw std::runtime_error("metrics exposer on " + host + " is not listening");
  }
  return static_cast<std::uint16_t>(ports.front());
}

std::shared_ptr<prometheus::Registry> Metrics::registry() const { return registry_; }

}  // namespace ros2_common_cpp
