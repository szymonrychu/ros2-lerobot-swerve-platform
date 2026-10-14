#include <zmq.h>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "jetson_bridge_cpp/bridge_node.hpp"

namespace jetson_bridge_cpp
{

namespace wire = ros2_common_cpp::depth_wire;

namespace
{

constexpr int MAX_PARTS = 8;  // frames beyond this in one message are drained and the message counted as malformed
constexpr const char * STAGES[] = {"capture", "rectify", "match", "post", "mono", "encode"};
constexpr const char * KINDS[] = {wire::kTopicDepth, wire::kTopicFused, wire::kTopicRgb};
constexpr const char * DROP_REASONS[] = {DROP_MALFORMED, DROP_DECODE_ERROR, DROP_STALE, DROP_FUTURE, DROP_UNCALIBRATED};

int64_t realtime_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
    .count();
}

std::string kind_label(wire::Kind kind) { return wire::topic_of(kind); }

// Frame -> bytes of every part of one multipart message; false when the socket timed out before the first part.
bool receive_message(void * socket, std::vector<std::vector<uint8_t>> & parts)
{
  parts.clear();
  int more = 1;
  bool first = true;
  while (more) {
    zmq_msg_t msg;
    zmq_msg_init(&msg);
    if (zmq_msg_recv(&msg, socket, 0) < 0) {
      zmq_msg_close(&msg);
      if (first) {
        return false;
      }
      parts.clear();  // the rest of a started multipart message always arrives atomically; be safe
      return true;
    }
    first = false;
    if (static_cast<int>(parts.size()) < MAX_PARTS) {
      const auto * data = static_cast<const uint8_t *>(zmq_msg_data(&msg));
      parts.emplace_back(data, data + zmq_msg_size(&msg));
    }
    size_t more_size = sizeof(more);
    zmq_getsockopt(socket, ZMQ_RCVMORE, &more, &more_size);
    zmq_msg_close(&msg);
  }
  return true;
}

}  // namespace

BridgeMetrics::BridgeMetrics(ros2_common_cpp::Metrics & metrics)
{
  const std::vector<double> latency_buckets{0.005, 0.01, 0.025, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0, 2.0};
  for (const char * kind : KINDS) {
    frames_[kind] = &metrics.counter("jetson_bridge_frames_total", "Frames republished to ROS", {{"kind", kind}});
    latency_[kind] = &metrics.histogram(
      "jetson_bridge_latency_seconds", "Pi receive time minus the Jetson capture time", latency_buckets,
      {{"kind", kind}});
  }
  for (const char * reason : DROP_REASONS) {
    dropped_[reason] =
      &metrics.counter("jetson_bridge_dropped_total", "Frames received and not republished", {{"reason", reason}});
  }
  up_ = &metrics.gauge("jetson_bridge_jetson_up", "1 while frames arrived within absent_after_ms");
  fps_ = &metrics.gauge("jetson_depthd_pipeline_fps", "Jetson pipeline rate, from the latest frame header");
  for (const char * stage : STAGES) {
    stage_ms_[stage] =
      &metrics.gauge("jetson_depthd_stage_ms", "Jetson stage time, from the latest frame header", {{"stage", stage}});
  }
  pairs_dropped_ = &metrics.gauge("jetson_depthd_pairs_dropped_total", "Jetson stereo pairs dropped (its counter)");
  gpu_temp_ = &metrics.gauge("jetson_depthd_gpu_temp_c", "Jetson GPU temperature, from the latest frame header");
  nvgpu_errors_ = &metrics.gauge("jetson_depthd_nvgpu_errors_total", "Jetson nvgpu kernel errors (its counter)");
}

void BridgeMetrics::frame(wire::Kind kind, double latency_s)
{
  const std::string label = kind_label(kind);
  frames_.at(label)->Increment();
  latency_.at(label)->Observe(latency_s < 0.0 ? 0.0 : latency_s);
}

void BridgeMetrics::dropped(const std::string & reason) { dropped_.at(reason)->Increment(); }

void BridgeMetrics::set_up(bool up) { up_->Set(up ? 1.0 : 0.0); }

void BridgeMetrics::health(const wire::Header & h)
{
  fps_->Set(h.pipeline_fps);
  stage_ms_.at("capture")->Set(h.stage_capture_ms);
  stage_ms_.at("rectify")->Set(h.stage_rectify_ms);
  stage_ms_.at("match")->Set(h.stage_match_ms);
  stage_ms_.at("post")->Set(h.stage_post_ms);
  stage_ms_.at("mono")->Set(h.stage_mono_ms);
  stage_ms_.at("encode")->Set(h.stage_encode_ms);
  pairs_dropped_->Set(static_cast<double>(h.pairs_dropped_total));
  gpu_temp_->Set(h.gpu_temp_c);
  nvgpu_errors_->Set(static_cast<double>(h.nvgpu_errors_total));
}

double BridgeMetrics::frames_total(const std::string & kind) const { return frames_.at(kind)->Value(); }

double BridgeMetrics::dropped_total(const std::string & reason) const { return dropped_.at(reason)->Value(); }

bool BridgeMetrics::up() const { return up_->Value() > 0.5; }

JetsonBridgeNode::JetsonBridgeNode(const BridgeConfig & config, const std::string & node_name)
: rclcpp::Node(node_name),
  config_(config),
  metrics_(node_name),
  bridge_metrics_(metrics_),
  policy_(config.max_age_ms, config.absent_after_ms)
{
  const rclcpp::QoS qos(PUBLISH_QOS_DEPTH);  // reliable, volatile, keep last 5
  depth_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.depth_topic, qos);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.camera_info_topic, qos);
  fused_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.fused_topic, qos);
  rgb_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(config_.rgb_topic, qos);
  rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.rgb_info_topic, qos);
  bridge_metrics_.set_up(false);

  zmq_context_ = zmq_ctx_new();
  zmq_socket_ = zmq_socket(zmq_context_, ZMQ_SUB);
  const int hwm = RECEIVE_HWM;
  const int timeout = RECEIVE_TIMEOUT_MS;
  const int linger = 0;
  zmq_setsockopt(zmq_socket_, ZMQ_RCVHWM, &hwm, sizeof(hwm));
  zmq_setsockopt(zmq_socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
  zmq_setsockopt(zmq_socket_, ZMQ_LINGER, &linger, sizeof(linger));
  for (const char * kind : KINDS) {
    zmq_setsockopt(zmq_socket_, ZMQ_SUBSCRIBE, kind, std::string(kind).size());
  }
  if (zmq_connect(zmq_socket_, config_.endpoint.c_str()) != 0) {
    const std::string reason = zmq_strerror(zmq_errno());
    zmq_close(zmq_socket_);
    zmq_ctx_term(zmq_context_);
    throw std::runtime_error("cannot connect to " + config_.endpoint + ": " + reason);
  }
  up_timer_ = create_wall_timer(std::chrono::milliseconds(UP_CHECK_PERIOD_MS), [this]() { update_up(); });
  thread_ = std::thread([this]() { receive_loop(); });
  RCLCPP_INFO(get_logger(), "subscribed to %s", config_.endpoint.c_str());
}

JetsonBridgeNode::~JetsonBridgeNode()
{
  stop_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  zmq_close(zmq_socket_);
  zmq_ctx_term(zmq_context_);
}

void JetsonBridgeNode::update_up() { bridge_metrics_.set_up(!policy_.absent(realtime_ns())); }

void JetsonBridgeNode::receive_loop()
{
  std::vector<std::vector<uint8_t>> parts;
  while (!stop_) {
    if (!receive_message(zmq_socket_, parts)) {
      continue;
    }
    const int64_t now_ns = realtime_ns();
    if (parts.size() != 3) {
      bridge_metrics_.dropped(DROP_MALFORMED);
      continue;
    }
    const std::string topic(parts[0].begin(), parts[0].end());
    const auto decoded = wire::decode(topic, parts[1].data(), parts[1].size(), parts[2].data(), parts[2].size());
    if (!decoded.ok()) {
      bridge_metrics_.dropped(DROP_DECODE_ERROR);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "dropped undecodable frame: %s", decoded.error.c_str());
      continue;
    }
    handle(decoded, now_ns);
  }
}

void JetsonBridgeNode::handle(const wire::Decoded & frame, int64_t now_ns)
{
  const wire::Header & h = frame.header;
  bridge_metrics_.health(h);
  const Verdict verdict = policy_.accept(h, now_ns);
  bridge_metrics_.set_up(true);  // an arrival of any kind: the Jetson is present
  if (verdict == Verdict::kTooOld) {
    bridge_metrics_.dropped(DROP_STALE);
    return;
  }
  if (verdict == Verdict::kFromFuture) {
    bridge_metrics_.dropped(DROP_FUTURE);
    return;
  }
  const double latency_s = static_cast<double>(now_ns - h.capture_realtime_ns) * 1e-9;
  try {
    switch (h.kind) {
      case wire::Kind::kDepth: {
        // Depth without a calibration has no metric meaning and no camera_info to go with it.
        auto info = to_camera_info(h, CameraModel::kRectified);
        if (!info) {
          bridge_metrics_.dropped(DROP_UNCALIBRATED);
          return;
        }
        depth_pub_->publish(to_depth_image(h, frame.depth_mm));
        info_pub_->publish(std::move(*info));
        break;
      }
      case wire::Kind::kFused: {
        if (!h.calibrated) {
          bridge_metrics_.dropped(DROP_UNCALIBRATED);
          return;
        }
        fused_pub_->publish(to_depth_image(h, frame.depth_mm));
        break;
      }
      case wire::Kind::kRgb: {
        rgb_pub_->publish(to_compressed(h, frame.jpeg));
        if (auto info = to_camera_info(h, CameraModel::kRaw)) {
          rgb_info_pub_->publish(std::move(*info));
        }
        break;
      }
    }
  } catch (const std::invalid_argument & e) {
    bridge_metrics_.dropped(DROP_DECODE_ERROR);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "dropped inconsistent frame: %s", e.what());
    return;
  }
  bridge_metrics_.frame(h.kind, latency_s);
}

}  // namespace jetson_bridge_cpp
