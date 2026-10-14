// The bridge node end to end: an in-process ZeroMQ PUB on loopback feeds wire frames, a ROS test node subscribes to the
// republished topics. Good frames arrive with the right fields; stale, future, undecodable, uncalibrated and absent
// cases publish nothing and are counted.
#include <gtest/gtest.h>
#include <zmq.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>
#include <vector>

#include "jetson_bridge_cpp/bridge_node.hpp"
#include "jetson_bridge_cpp/config.hpp"
#include "ros2_common_cpp/depth_wire.hpp"

namespace wire = ros2_common_cpp::depth_wire;
using namespace std::chrono_literals;

namespace
{
constexpr int64_t kMs = 1000000;

int64_t realtime_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
    .count();
}

wire::Header make_header(wire::Kind kind, int64_t capture_offset_ms = -50, bool calibrated = true)
{
  wire::Header h;
  h.kind = kind;
  h.sequence = 7;
  h.capture_realtime_ns = realtime_ns() + capture_offset_ms * kMs;
  h.width = 4;
  h.height = 2;
  h.frame_id = kind == wire::Kind::kRgb ? "overview_camera_optical_frame" : "stereo_left_optical_frame";
  if (calibrated) {
    h.k = {500.0, 0.0, 320.5, 0.0, 510.0, 240.5, 0.0, 0.0, 1.0};
    h.p = {400.0, 0.0, 300.5, 0.0, 0.0, 410.0, 200.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    h.baseline_m = 0.06;
  }
  h.calibrated = calibrated;
  h.pipeline_fps = 12.5F;
  h.stage_match_ms = 31.0F;
  h.stage_capture_ms = 4.0F;
  h.pairs_dropped_total = 9;
  h.gpu_temp_c = 55.5F;
  h.nvgpu_errors_total = 2;
  return h;
}

const std::vector<uint16_t> kDepth = {0, 100, 200, 300, 400, 500, 600, 65535};
const std::vector<uint8_t> kJpeg = {0xFF, 0xD8, 0xFF, 0xE0, 1, 2, 3, 0xFF, 0xD9};

class Publisher
{
public:
  Publisher()
  {
    ctx_ = zmq_ctx_new();
    sock_ = zmq_socket(ctx_, ZMQ_PUB);
    int linger = 0;
    zmq_setsockopt(sock_, ZMQ_LINGER, &linger, sizeof(linger));
    EXPECT_EQ(zmq_bind(sock_, "tcp://127.0.0.1:*"), 0);
    char buf[256];
    size_t len = sizeof(buf);
    zmq_getsockopt(sock_, ZMQ_LAST_ENDPOINT, buf, &len);
    endpoint_ = buf;
  }
  ~Publisher()
  {
    zmq_close(sock_);
    zmq_ctx_term(ctx_);
  }
  const std::string & endpoint() const { return endpoint_; }

  void send(const wire::Frames & f) { send_raw(f.topic, f.header, f.payload); }
  void send_raw(const std::string & topic, const std::vector<uint8_t> & header, const std::vector<uint8_t> & payload)
  {
    zmq_send(sock_, topic.data(), topic.size(), ZMQ_SNDMORE);
    zmq_send(sock_, header.data(), header.size(), ZMQ_SNDMORE);
    zmq_send(sock_, payload.data(), payload.size(), 0);
  }

private:
  void * ctx_ = nullptr;
  void * sock_ = nullptr;
  std::string endpoint_;
};

class NodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void start(int64_t max_age_ms = 500, int64_t absent_after_ms = 2000)
  {
    jetson_bridge_cpp::BridgeConfig config;
    config.endpoint = pub_.endpoint();
    config.max_age_ms = max_age_ms;
    config.absent_after_ms = absent_after_ms;
    config.metrics_port = std::nullopt;
    bridge_ = std::make_shared<jetson_bridge_cpp::JetsonBridgeNode>(config);
    probe_ = std::make_shared<rclcpp::Node>("jetson_bridge_probe");
    const rclcpp::QoS qos(5);
    depth_sub_ = probe_->create_subscription<sensor_msgs::msg::Image>(
      config.depth_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) { depth_.push_back(*m); });
    fused_sub_ = probe_->create_subscription<sensor_msgs::msg::Image>(
      config.fused_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) { fused_.push_back(*m); });
    info_sub_ = probe_->create_subscription<sensor_msgs::msg::CameraInfo>(
      config.camera_info_topic, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { info_.push_back(*m); });
    rgb_sub_ = probe_->create_subscription<sensor_msgs::msg::CompressedImage>(
      config.rgb_topic, qos, [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) { rgb_.push_back(*m); });
    rgb_info_sub_ = probe_->create_subscription<sensor_msgs::msg::CameraInfo>(
      config.rgb_info_topic, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { rgb_info_.push_back(*m); });
    exec_.add_node(probe_);
    exec_.add_node(bridge_);
  }

  // Sends frame_fn() every 20 ms (the PUB drops what is sent before the SUB joined) while spinning until done().
  template <typename F, typename D>
  bool pump(F frame_fn, D done, std::chrono::milliseconds limit = 8s)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      frame_fn();
      exec_.spin_some(20ms);
      std::this_thread::sleep_for(20ms);
      if (done()) {
        return true;
      }
    }
    return false;
  }

  void settle()
  {
    const auto end = std::chrono::steady_clock::now() + 300ms;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some(20ms);
    }
  }

  void expect_nothing_published()
  {
    settle();
    EXPECT_TRUE(depth_.empty());
    EXPECT_TRUE(fused_.empty());
    EXPECT_TRUE(info_.empty());
    EXPECT_TRUE(rgb_.empty());
    EXPECT_TRUE(rgb_info_.empty());
  }

  Publisher pub_;
  std::shared_ptr<jetson_bridge_cpp::JetsonBridgeNode> bridge_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, fused_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_, rgb_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_sub_;
  std::vector<sensor_msgs::msg::Image> depth_, fused_;
  std::vector<sensor_msgs::msg::CameraInfo> info_, rgb_info_;
  std::vector<sensor_msgs::msg::CompressedImage> rgb_;
};
}  // namespace

TEST_F(NodeTest, RepublishesDepthWithCameraInfo)
{
  start();
  const wire::Header h = make_header(wire::Kind::kDepth);
  ASSERT_TRUE(pump(
    [&] { pub_.send(wire::encode_depth(make_header(wire::Kind::kDepth), kDepth)); },
    [&] { return !depth_.empty() && !info_.empty(); }));
  const auto & img = depth_.front();
  EXPECT_EQ(img.encoding, "16UC1");
  EXPECT_EQ(img.width, 4U);
  EXPECT_EQ(img.height, 2U);
  EXPECT_EQ(img.step, 8U);
  EXPECT_EQ(img.header.frame_id, "stereo_left_optical_frame");
  ASSERT_EQ(img.data.size(), 16U);
  EXPECT_EQ(img.data[2], 100);  // little-endian millimetres
  EXPECT_EQ(img.data[14], 0xFF);
  EXPECT_NEAR(
    img.header.stamp.sec + img.header.stamp.nanosec * 1e-9, static_cast<double>(realtime_ns()) * 1e-9 - 0.05, 1.0);
  EXPECT_EQ(info_.front().header.stamp, img.header.stamp);
  EXPECT_EQ(info_.front().header.frame_id, "stereo_left_optical_frame");
  EXPECT_DOUBLE_EQ(info_.front().p[0], h.p[0]);
  EXPECT_DOUBLE_EQ(info_.front().k[0], h.p[0]);  // K taken from P for the rectified camera
  EXPECT_GE(bridge_->bridge_metrics().frames_total("depth"), 1.0);
  EXPECT_TRUE(fused_.empty());
}

TEST_F(NodeTest, RepublishesFusedWithoutCameraInfo)
{
  start();
  ASSERT_TRUE(pump(
    [&] { pub_.send(wire::encode_depth(make_header(wire::Kind::kFused), kDepth)); }, [&] { return !fused_.empty(); }));
  EXPECT_EQ(fused_.front().encoding, "16UC1");
  EXPECT_EQ(fused_.front().header.frame_id, "stereo_left_optical_frame");
  EXPECT_TRUE(depth_.empty());
  EXPECT_GE(bridge_->bridge_metrics().frames_total("fused"), 1.0);
}

TEST_F(NodeTest, RepublishesJpegUnchangedWithoutDecoding)
{
  start();
  ASSERT_TRUE(pump(
    [&] { pub_.send(wire::encode_rgb(make_header(wire::Kind::kRgb), kJpeg)); },
    [&] { return !rgb_.empty() && !rgb_info_.empty(); }));
  EXPECT_EQ(rgb_.front().format, "jpeg");
  EXPECT_EQ(rgb_.front().data, kJpeg);
  EXPECT_EQ(rgb_.front().header.frame_id, "overview_camera_optical_frame");
  EXPECT_EQ(rgb_info_.front().header.stamp, rgb_.front().header.stamp);
  EXPECT_DOUBLE_EQ(rgb_info_.front().k[0], 500.0);
  EXPECT_GE(bridge_->bridge_metrics().frames_total("rgb"), 1.0);
}

TEST_F(NodeTest, StaleFramePublishesNothingAndCounts)
{
  start(500);
  ASSERT_TRUE(pump(
    [&] { pub_.send(wire::encode_depth(make_header(wire::Kind::kDepth, -2000), kDepth)); },
    [&] { return bridge_->bridge_metrics().dropped_total("stale") >= 1.0; }));
  expect_nothing_published();
  EXPECT_EQ(bridge_->bridge_metrics().frames_total("depth"), 0.0);
}

TEST_F(NodeTest, FutureFramePublishesNothingAndCounts)
{
  start();
  ASSERT_TRUE(pump(
    [&] { pub_.send(wire::encode_depth(make_header(wire::Kind::kDepth, 5000), kDepth)); },
    [&] { return bridge_->bridge_metrics().dropped_total("future") >= 1.0; }));
  expect_nothing_published();
}

TEST_F(NodeTest, UndecodableFramePublishesNothingAndCounts)
{
  start();
  ASSERT_TRUE(pump(
    [&] { pub_.send_raw("depth", {1, 2, 3}, {4, 5, 6}); },
    [&] { return bridge_->bridge_metrics().dropped_total("decode_error") >= 1.0; }));
  expect_nothing_published();
}

TEST_F(NodeTest, UncalibratedDepthPublishesNothingAndCounts)
{
  start();
  ASSERT_TRUE(pump(
    [&] { pub_.send(wire::encode_depth(make_header(wire::Kind::kDepth, -50, false), kDepth)); },
    [&] { return bridge_->bridge_metrics().dropped_total("uncalibrated") >= 1.0; }));
  expect_nothing_published();
}

TEST_F(NodeTest, AbsentJetsonPublishesNothingAndIsDown)
{
  start(500, 300);
  std::this_thread::sleep_for(600ms);
  expect_nothing_published();
  EXPECT_FALSE(bridge_->bridge_metrics().up());
  EXPECT_NE(bridge_->metrics().render().find("jetson_bridge_jetson_up 0"), std::string::npos);
}

TEST_F(NodeTest, JetsonGoesUpThenAbsentWhenFramesStop)
{
  start(500, 400);
  ASSERT_TRUE(pump(
    [&] { pub_.send(wire::encode_depth(make_header(wire::Kind::kDepth), kDepth)); },
    [&] { return bridge_->bridge_metrics().up(); }));
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (bridge_->bridge_metrics().up() && std::chrono::steady_clock::now() < end) {
    exec_.spin_some(20ms);
  }
  EXPECT_FALSE(bridge_->bridge_metrics().up());
}

TEST_F(NodeTest, MetricsReexportJetsonHealthAndLatency)
{
  start();
  ASSERT_TRUE(pump(
    [&] { pub_.send(wire::encode_depth(make_header(wire::Kind::kDepth), kDepth)); },
    [&] { return bridge_->bridge_metrics().frames_total("depth") >= 1.0; }));
  const std::string text = bridge_->metrics().render();
  for (const char * needle :
       {"jetson_bridge_frames_total{kind=\"depth\"}", "jetson_bridge_dropped_total{reason=\"stale\"} 0",
        "jetson_bridge_latency_seconds_bucket{kind=\"depth\"", "jetson_bridge_latency_seconds_count{kind=\"depth\"}",
        "jetson_bridge_jetson_up 1", "jetson_depthd_pipeline_fps 12.5", "jetson_depthd_stage_ms{stage=\"match\"} 31",
        "jetson_depthd_stage_ms{stage=\"capture\"} 4", "jetson_depthd_pairs_dropped_total 9",
        "jetson_depthd_gpu_temp_c 55.5", "jetson_depthd_nvgpu_errors_total 2", "process_cpu_seconds_total"}) {
    EXPECT_NE(text.find(needle), std::string::npos) << needle << "\n" << text;
  }
}
