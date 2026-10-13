// ros2_common_cpp metrics: node info series, label escaping, histogram buckets and the HTTP exposition.
#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <ctime>
#include <regex>
#include <stdexcept>
#include <string>

#include "ros2_common_cpp/metrics.hpp"

namespace rc = ros2_common_cpp;

namespace
{

bool has_line(const std::string & text, const std::string & line)
{
  return ("\n" + text).find("\n" + line + "\n") != std::string::npos;
}

// Plain HTTP/1.0 GET on 127.0.0.1:port, returning the whole response (status line, headers and body).
std::string http_get(std::uint16_t port, const std::string & path)
{
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    throw std::runtime_error("socket");
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    close(fd);
    throw std::runtime_error("connect");
  }
  const std::string request = "GET " + path + " HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n";
  if (send(fd, request.data(), request.size(), 0) != static_cast<ssize_t>(request.size())) {
    close(fd);
    throw std::runtime_error("send");
  }
  std::string response;
  char buffer[4096];
  ssize_t n = 0;
  while ((n = recv(fd, buffer, sizeof(buffer), 0)) > 0) {
    response.append(buffer, static_cast<std::size_t>(n));
  }
  close(fd);
  return response;
}

}  // namespace

TEST(Metrics, NodeInfoExactText)
{
  rc::Metrics metrics("x");
  const std::string text = metrics.render();
  EXPECT_TRUE(has_line(text, "robot_node_info{node=\"x\"} 1")) << text;
  EXPECT_TRUE(has_line(text, "# TYPE robot_node_info gauge")) << text;
  EXPECT_TRUE(has_line(text, "# TYPE robot_node_start_time_seconds gauge")) << text;
}

TEST(Metrics, StartTimeIsNowInEpochSeconds)
{
  const double before = static_cast<double>(std::time(nullptr));
  rc::Metrics metrics("x");
  std::smatch match;
  const std::string text = metrics.render();
  ASSERT_TRUE(
    std::regex_search(text, match, std::regex("\nrobot_node_start_time_seconds\\{node=\"x\"\\} ([0-9.e+]+)\n")))
    << text;
  const double started = std::stod(match[1]);
  EXPECT_GE(started, before - 1.0);
  EXPECT_LE(started, before + 5.0);
}

TEST(Metrics, LabelValuesAreEscaped)
{
  rc::Metrics metrics("x");
  metrics.counter("robot_test_events_total", "Events", {{"source", "a\"b\\c\nd"}}).Increment(2);
  EXPECT_TRUE(has_line(metrics.render(), "robot_test_events_total{source=\"a\\\"b\\\\c\\nd\"} 2")) << metrics.render();
}

TEST(Metrics, CountersAndGaugesWithLabels)
{
  rc::Metrics metrics("x");
  metrics.counter("robot_test_frames_total", "Frames", {{"camera", "left"}}).Increment();
  metrics.counter("robot_test_frames_total", "Frames", {{"camera", "left"}}).Increment();
  metrics.counter("robot_test_frames_total", "Frames", {{"camera", "right"}}).Increment();
  metrics.gauge("robot_test_depth_m", "Depth").Set(1.5);
  const std::string text = metrics.render();
  EXPECT_TRUE(has_line(text, "# TYPE robot_test_frames_total counter")) << text;
  EXPECT_TRUE(has_line(text, "robot_test_frames_total{camera=\"left\"} 2")) << text;
  EXPECT_TRUE(has_line(text, "robot_test_frames_total{camera=\"right\"} 1")) << text;
  EXPECT_TRUE(has_line(text, "robot_test_depth_m 1.5")) << text;
}

TEST(Metrics, HistogramBucketsAreCumulative)
{
  rc::Metrics metrics("x");
  auto & h = metrics.histogram("robot_test_latency_seconds", "Latency", {0.1, 0.5}, {{"stage", "match"}});
  h.Observe(0.05);
  h.Observe(0.3);
  h.Observe(2.0);
  const std::string text = metrics.render();
  EXPECT_TRUE(has_line(text, "# TYPE robot_test_latency_seconds histogram")) << text;
  EXPECT_TRUE(has_line(text, "robot_test_latency_seconds_bucket{stage=\"match\",le=\"0.1\"} 1")) << text;
  EXPECT_TRUE(has_line(text, "robot_test_latency_seconds_bucket{stage=\"match\",le=\"0.5\"} 2")) << text;
  EXPECT_TRUE(has_line(text, "robot_test_latency_seconds_bucket{stage=\"match\",le=\"+Inf\"} 3")) << text;
  EXPECT_TRUE(has_line(text, "robot_test_latency_seconds_count{stage=\"match\"} 3")) << text;
  EXPECT_TRUE(has_line(text, "robot_test_latency_seconds_sum{stage=\"match\"} 2.35")) << text;
}

TEST(Metrics, HttpGetOnLocalhostReturnsTheExposition)
{
  rc::Metrics metrics("x");
  metrics.gauge("robot_test_up", "Up").Set(1);
  const std::uint16_t port = metrics.serve(0);
  ASSERT_GT(port, 0);
  const std::string response = http_get(port, "/metrics");
  EXPECT_EQ(response.rfind("HTTP/1.", 0), 0u) << response;
  EXPECT_NE(response.find(" 200 "), std::string::npos) << response;
  EXPECT_NE(response.find("\nrobot_node_info{node=\"x\"} 1\n"), std::string::npos) << response;
  EXPECT_NE(response.find("\nrobot_test_up 1\n"), std::string::npos) << response;
  EXPECT_THROW(metrics.serve(0), std::runtime_error);
}

TEST(Metrics, ResolvePortConfigThenEnvThenDisabled)
{
  unsetenv(rc::METRICS_PORT_ENV);
  EXPECT_EQ(rc::resolve_metrics_port(std::nullopt), std::nullopt);
  EXPECT_EQ(rc::resolve_metrics_port(19105), std::optional<std::uint16_t>(19105));
  setenv(rc::METRICS_PORT_ENV, "19106", 1);
  EXPECT_EQ(rc::resolve_metrics_port(std::nullopt), std::optional<std::uint16_t>(19106));
  EXPECT_EQ(rc::resolve_metrics_port(19105), std::optional<std::uint16_t>(19105));
  setenv(rc::METRICS_PORT_ENV, " ", 1);
  EXPECT_EQ(rc::resolve_metrics_port(std::nullopt), std::nullopt);
  setenv(rc::METRICS_PORT_ENV, "http", 1);
  EXPECT_THROW(rc::resolve_metrics_port(std::nullopt), std::invalid_argument);
  setenv(rc::METRICS_PORT_ENV, "70000", 1);
  EXPECT_THROW(rc::resolve_metrics_port(std::nullopt), std::invalid_argument);
  unsetenv(rc::METRICS_PORT_ENV);
  EXPECT_THROW(rc::resolve_metrics_port(0), std::invalid_argument);
}
