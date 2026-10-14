// The Pi side of the Jetson link as a ROS 2 node: a ZeroMQ SUB thread receives depth, fused and rgb wire frames,
// decodes them (depth_wire), lets StalenessPolicy decide and republishes the survivors with the contracted QoS
// (reliable, keep last 5, like the retired stereo_depth node). A frame that fails to decode, is stale, from the future
// or is uncalibrated depth is counted and dropped; nothing is published while the Jetson is absent and no placeholder
// data is ever made up. JPEGs are republished as received: nothing is decoded on the Pi.
#ifndef JETSON_BRIDGE_CPP__BRIDGE_NODE_HPP_
#define JETSON_BRIDGE_CPP__BRIDGE_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>

#include "jetson_bridge_cpp/config.hpp"
#include "jetson_bridge_cpp/convert.hpp"
#include "ros2_common_cpp/depth_wire.hpp"
#include "ros2_common_cpp/metrics.hpp"

namespace jetson_bridge_cpp
{

constexpr int PUBLISH_QOS_DEPTH = 5;
constexpr int RECEIVE_HWM = 2;
constexpr int RECEIVE_TIMEOUT_MS = 100;
constexpr int UP_CHECK_PERIOD_MS = 100;

// Reasons of jetson_bridge_dropped_total{reason}.
constexpr const char * DROP_MALFORMED = "malformed";        // not a 3-frame message
constexpr const char * DROP_DECODE_ERROR = "decode_error";  // depth_wire rejected it
constexpr const char * DROP_STALE = "stale";                // older than max_age_ms
constexpr const char * DROP_FUTURE = "future";              // ahead of the Pi clock
constexpr const char * DROP_UNCALIBRATED = "uncalibrated";  // depth from an uncalibrated Jetson

// The bridge's series (jetson-bridge-metrics), all on one ros2_common_cpp::Metrics. Every label value is created up
// front so each series exists at 0 from the first scrape. Thread safe (prometheus-cpp children are atomic).
class BridgeMetrics
{
public:
  explicit BridgeMetrics(ros2_common_cpp::Metrics & metrics);

  // One republished frame of this kind; latency is the Pi receive time minus capture_realtime_ns, in seconds.
  void frame(ros2_common_cpp::depth_wire::Kind kind, double latency_s);
  void dropped(const std::string & reason);
  void set_up(bool up);
  // Re-exports the Jetson health of a decoded header as jetson_depthd_* gauges.
  void health(const ros2_common_cpp::depth_wire::Header & header);

  double frames_total(const std::string & kind) const;
  double dropped_total(const std::string & reason) const;
  bool up() const;

private:
  std::map<std::string, prometheus::Counter *> frames_;
  std::map<std::string, prometheus::Histogram *> latency_;
  std::map<std::string, prometheus::Counter *> dropped_;
  prometheus::Gauge * up_;
  prometheus::Gauge * fps_;
  std::map<std::string, prometheus::Gauge *> stage_ms_;
  prometheus::Gauge * pairs_dropped_;
  prometheus::Gauge * gpu_temp_;
  prometheus::Gauge * nvgpu_errors_;
};

class JetsonBridgeNode : public rclcpp::Node
{
public:
  explicit JetsonBridgeNode(const BridgeConfig & config, const std::string & node_name = "jetson_bridge_node");
  ~JetsonBridgeNode() override;

  ros2_common_cpp::Metrics & metrics() { return metrics_; }
  BridgeMetrics & bridge_metrics() { return bridge_metrics_; }
  const StalenessPolicy & policy() const { return policy_; }

private:
  void receive_loop();
  void handle(const ros2_common_cpp::depth_wire::Decoded & frame, int64_t now_ns);
  void update_up();

  BridgeConfig config_;
  ros2_common_cpp::Metrics metrics_;
  BridgeMetrics bridge_metrics_;
  StalenessPolicy policy_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fused_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::TimerBase::SharedPtr up_timer_;
  void * zmq_context_ = nullptr;
  void * zmq_socket_ = nullptr;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace jetson_bridge_cpp

#endif  // JETSON_BRIDGE_CPP__BRIDGE_NODE_HPP_
