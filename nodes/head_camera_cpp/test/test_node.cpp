#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_camera_backend.hpp"
#include "head_camera_cpp/config.hpp"
#include "head_camera_cpp/encode.hpp"
#include "head_camera_cpp/head_camera_node.hpp"

namespace
{

using head_camera_cpp::Config;
using head_camera_cpp::HeadCameraNode;
using head_camera_cpp::testing::FakeCameraBackend;
using head_camera_cpp::testing::FakeClock;
using nlohmann::json;
using std::chrono::milliseconds;
using std::chrono::seconds;

constexpr int VIDEO_W = 64;
constexpr int VIDEO_H = 36;
constexpr int STILL_W = 640;
constexpr int STILL_H = 360;
constexpr auto WAIT = seconds(10);
constexpr auto FRAME_PERIOD = milliseconds(20);
constexpr double STAMP_TOLERANCE_S = 0.5;

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

Config test_config()
{
  Config config;
  config.metrics_port = std::nullopt;
  config.zoom.still_size = {STILL_W, STILL_H};
  return config;
}

// A clock that forwards to the fixture's FakeClock, so the test keeps the FakeClock after handing the node a Clock.
class ClockRef : public head_camera_cpp::Clock
{
public:
  explicit ClockRef(FakeClock & clock) : clock_(clock) {}
  TimePoint now() override { return clock_.now(); }
  void sleep_for(Duration duration) override { clock_.sleep_for(duration); }

private:
  FakeClock & clock_;
};

// A node on a fake camera plus a probe node with the subscriptions, both spun by one executor in a thread.
class NodeTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void start(const Config & config)
  {
    config_ = config;
    auto backend = std::make_unique<FakeCameraBackend>(
      head_camera_cpp::Size{STILL_W, STILL_H}, &clock_, head_camera_cpp::Size{VIDEO_W, VIDEO_H});
    backend_ = backend.get();
    auto clock = std::make_unique<ClockRef>(clock_);
    node_ = std::make_shared<HeadCameraNode>(config, std::move(backend), std::move(clock), rclcpp::NodeOptions());
    probe_ = std::make_shared<rclcpp::Node>("head_camera_probe");
    executor_.add_node(node_);
    executor_.add_node(probe_);
    spinner_ = std::thread([this] { executor_.spin(); });
    // cancel() before spin() has started is lost (spin re-arms), so teardown would hang on join.
    while (!executor_.is_spinning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    subscriptions_.clear();
    probe_.reset();
    node_.reset();
  }

  template <typename Msg>
  void listen(const std::string & topic, rclcpp::QoS qos, std::vector<Msg> & store)
  {
    subscriptions_.push_back(probe_->create_subscription<Msg>(topic, qos, [this, &store](const std::shared_ptr<Msg> m) {
      std::lock_guard<std::mutex> lock(mutex_);
      store.push_back(*m);
    }));
  }

  template <typename Msg>
  size_t size_of(const std::vector<Msg> & store)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return store.size();
  }

  template <typename Msg>
  std::vector<Msg> copy_of(const std::vector<Msg> & store)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return store;
  }

  // Emit video frames every FRAME_PERIOD until `done` or WAIT passes.
  bool emit_until(const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + WAIT;
    while (std::chrono::steady_clock::now() < deadline) {
      if (done()) {
        return true;
      }
      backend_->emit_video_frame(steady_ns());
      std::this_thread::sleep_for(FRAME_PERIOD);
    }
    return done();
  }

  bool wait_until(const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + WAIT;
    while (std::chrono::steady_clock::now() < deadline) {
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(milliseconds(10));
    }
    return done();
  }

  // Publish a zoom command once the node has subscribed to it.
  void send_command(const std::string & payload)
  {
    if (!command_pub_) {
      command_pub_ = probe_->create_publisher<std_msgs::msg::String>(
        config_.zoom_command(), rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    }
    ASSERT_TRUE(wait_until([this] { return command_pub_->get_subscription_count() > 0; }));
    std_msgs::msg::String msg;
    msg.data = payload;
    command_pub_->publish(msg);
  }

  static std::string command(const std::string & id)
  {
    return json{{"request_id", id}, {"u", 0.5}, {"v", 0.5}, {"zoom", 2}, {"max_px", 256}, {"af", false}}.dump();
  }

  void listen_zoom()
  {
    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    listen(config_.zoom_image(), reliable, zoom_images_);
    listen(config_.zoom_result(), reliable, results_);
  }

  Config config_;
  FakeClock clock_;
  FakeCameraBackend * backend_ = nullptr;
  std::shared_ptr<HeadCameraNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  std::mutex mutex_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  std::vector<sensor_msgs::msg::Image> images_;
  std::vector<sensor_msgs::msg::CompressedImage> compressed_;
  std::vector<sensor_msgs::msg::CameraInfo> infos_;
  std::vector<sensor_msgs::msg::CompressedImage> zoom_images_;
  std::vector<std_msgs::msg::String> results_;
};

rclcpp::QoS reliable_1() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); }

}  // namespace

TEST_F(NodeTest, NoCompressedMessageWithoutSubscriber)
{
  start(test_config());
  listen(config_.image_raw(), reliable_1(), images_);
  ASSERT_TRUE(emit_until([this] { return size_of(images_) >= 3; }));
  const std::string metrics = node_->metrics().render();
  EXPECT_NE(metrics.find("head_camera_frames_published_total{topic=\"image_raw_compressed\"} 0"), std::string::npos)
    << metrics;
}

class QualityTest : public NodeTest, public ::testing::WithParamInterface<int>
{
};

TEST_P(QualityTest, CompressedPayloadEqualsEncodeAtConfiguredQuality)
{
  Config config = test_config();
  config.jpeg_quality = GetParam();
  start(config);
  listen(config_.image_raw_compressed(), reliable_1(), compressed_);
  ASSERT_TRUE(emit_until([this] { return size_of(compressed_) >= 1; }));
  const auto msg = copy_of(compressed_).front();
  const auto expected =
    head_camera_cpp::encode_bgr_jpeg(backend_->video_pixels().data(), VIDEO_W, VIDEO_H, VIDEO_W * 3, GetParam());
  EXPECT_EQ(msg.data, expected);
  EXPECT_EQ(msg.format, "bgr8; jpeg compressed bgr8");
  EXPECT_EQ(msg.header.frame_id, "head_camera_optical_frame");
}

INSTANTIATE_TEST_SUITE_P(Qualities, QualityTest, ::testing::Values(70, 40));

TEST_F(NodeTest, ImageRawPublishedWithSubscriber)
{
  start(test_config());
  listen(config_.image_raw(), reliable_1(), images_);
  ASSERT_TRUE(emit_until([this] { return size_of(images_) >= 1; }));
  const auto msg = copy_of(images_).front();
  EXPECT_EQ(msg.encoding, "bgr8");
  EXPECT_EQ(msg.width, static_cast<uint32_t>(VIDEO_W));
  EXPECT_EQ(msg.height, static_cast<uint32_t>(VIDEO_H));
  EXPECT_EQ(msg.step, static_cast<uint32_t>(VIDEO_W * 3));
  EXPECT_EQ(msg.data, backend_->video_pixels());
}

TEST_F(NodeTest, ImageRawNotPublishedWithoutSubscriber)
{
  start(test_config());
  listen(config_.image_raw_compressed(), reliable_1(), compressed_);
  ASSERT_TRUE(emit_until([this] { return size_of(compressed_) >= 3; }));
  const std::string metrics = node_->metrics().render();
  EXPECT_NE(metrics.find("head_camera_frames_published_total{topic=\"image_raw\"} 0"), std::string::npos) << metrics;
}

TEST_F(NodeTest, NoCameraInfoWithoutFile)
{
  start(test_config());
  listen(config_.camera_info(), reliable_1(), infos_);
  listen(config_.image_raw(), reliable_1(), images_);
  ASSERT_TRUE(emit_until([this] { return size_of(images_) >= 3; }));
  EXPECT_EQ(size_of(infos_), 0u);
}

TEST_F(NodeTest, CameraInfoFromFileScaledWithFrameStamp)
{
  Config config = test_config();
  config.camera_info_file = std::string(HEAD_CAMERA_TEST_DATA_DIR) + "/calibration.yaml";
  start(config);
  listen(config_.camera_info(), reliable_1(), infos_);
  listen(config_.image_raw(), reliable_1(), images_);
  ASSERT_TRUE(emit_until([this] { return size_of(infos_) >= 1 && size_of(images_) >= 1; }));
  const auto info = copy_of(infos_).front();
  EXPECT_EQ(info.width, 1152u);
  EXPECT_EQ(info.height, 648u);
  EXPECT_EQ(info.header.frame_id, "head_camera_optical_frame");
  EXPECT_DOUBLE_EQ(info.k[0], 800.0);
  EXPECT_DOUBLE_EQ(info.k[2], 575.0);
  EXPECT_DOUBLE_EQ(info.k[4], 810.0);
  EXPECT_DOUBLE_EQ(info.k[5], 320.0);
  EXPECT_DOUBLE_EQ(info.p[0], 800.0);
  EXPECT_DOUBLE_EQ(info.p[2], 575.0);
  EXPECT_DOUBLE_EQ(info.p[5], 810.0);
  EXPECT_DOUBLE_EQ(info.p[6], 320.0);
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  ASSERT_EQ(info.d.size(), 5u);
  EXPECT_DOUBLE_EQ(info.d[0], 0.1);
  // The same stamp as the frame published with it.
  bool matched = false;
  for (const auto & image : copy_of(images_)) {
    matched = matched || rclcpp::Time(image.header.stamp) == rclcpp::Time(info.header.stamp);
  }
  EXPECT_TRUE(matched);
}

TEST_F(NodeTest, StampsAreRosTimeOfCapture)
{
  start(test_config());
  listen(config_.image_raw(), reliable_1(), images_);
  ASSERT_TRUE(emit_until([this] { return size_of(images_) >= 1; }));
  const auto msg = copy_of(images_).back();
  const double age = (node_->get_clock()->now() - rclcpp::Time(msg.header.stamp)).seconds();
  EXPECT_GE(age, 0.0);
  EXPECT_LT(age, STAMP_TOLERANCE_S);
}

TEST_F(NodeTest, ZoomOkPublishesImageThenResultWithEqualStamp)
{
  start(test_config());
  listen_zoom();
  send_command(command("a"));
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 1; }));
  const auto result = json::parse(copy_of(results_).front().data);
  EXPECT_EQ(result["status"], "ok");
  EXPECT_EQ(result["request_id"], "a");
  ASSERT_TRUE(wait_until([this] { return size_of(zoom_images_) >= 1; }));
  const auto image = copy_of(zoom_images_).front();
  EXPECT_EQ(image.format, "jpeg");
  EXPECT_FALSE(image.data.empty());
  EXPECT_EQ(rclcpp::Time(image.header.stamp).nanoseconds(), result["stamp_ns"].get<int64_t>());
  EXPECT_TRUE(backend_->video_running());
}

TEST_F(NodeTest, ZoomBusyAndInvalidCarryNoImage)
{
  start(test_config());
  listen_zoom();
  // Hold the first zoom inside the backend until the second request has been answered.
  std::mutex gate_mutex;
  bool release = false;
  std::condition_variable gate;
  backend_->on_call = [&](const std::string & call) {
    if (call == "configure(Still)") {
      std::unique_lock<std::mutex> lock(gate_mutex);
      gate.wait_for(lock, WAIT, [&] { return release; });
    }
  };
  send_command(command("first"));
  ASSERT_TRUE(wait_until([this] { return backend_->count("configure(Still)") >= 1; }));
  send_command(command("second"));
  send_command(json{{"request_id", "bad"}, {"u", 2.0}, {"v", 0.5}, {"zoom", 2}}.dump());
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 2; }));
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release = true;
  }
  gate.notify_all();
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 3; }));
  std::map<std::string, std::string> status;
  for (const auto & msg : copy_of(results_)) {
    const auto result = json::parse(msg.data);
    status[result["request_id"]] = result["status"];
  }
  EXPECT_EQ(status["first"], "ok");
  EXPECT_EQ(status["second"], "busy");
  EXPECT_EQ(status["bad"], "invalid");
  // Exactly one image: the first request's.
  std::this_thread::sleep_for(milliseconds(200));
  EXPECT_EQ(size_of(zoom_images_), 1u);
}

TEST_F(NodeTest, ZoomWithoutRequestIdIsDropped)
{
  start(test_config());
  listen_zoom();
  send_command("{not json");
  send_command(command("after"));
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 1; }));
  EXPECT_EQ(json::parse(copy_of(results_).front().data)["request_id"], "after");
  EXPECT_EQ(size_of(results_), 1u);
}

TEST_F(NodeTest, ZoomCameraErrorPublishesNoImage)
{
  start(test_config());
  listen_zoom();
  backend_->fail("configure(Still)");
  send_command(command("err"));
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 1; }));
  EXPECT_EQ(json::parse(copy_of(results_).front().data)["status"], "camera_error");
  std::this_thread::sleep_for(milliseconds(200));
  EXPECT_EQ(size_of(zoom_images_), 0u);
  EXPECT_TRUE(backend_->video_running());
}

TEST_F(NodeTest, PublishersAreReliable)
{
  start(test_config());
  for (const auto & topic :
       {config_.image_raw(), config_.image_raw_compressed(), config_.camera_info(), config_.zoom_image(),
        config_.zoom_result()}) {
    const auto infos = node_->get_publishers_info_by_topic(topic);
    ASSERT_EQ(infos.size(), 1u) << topic;
    EXPECT_EQ(infos[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable) << topic;
  }
}

TEST_F(NodeTest, FailedStartIsRetriedWithoutPublishing)
{
  Config config = test_config();
  config_ = config;
  auto backend = std::make_unique<FakeCameraBackend>(
    head_camera_cpp::Size{STILL_W, STILL_H}, &clock_, head_camera_cpp::Size{VIDEO_W, VIDEO_H});
  backend_ = backend.get();
  backend_->fail("start_video", 1);
  node_ = std::make_shared<HeadCameraNode>(
    config, std::move(backend), std::make_unique<ClockRef>(clock_), rclcpp::NodeOptions());
  probe_ = std::make_shared<rclcpp::Node>("head_camera_probe");
  executor_.add_node(node_);
  executor_.add_node(probe_);
  spinner_ = std::thread([this] { executor_.spin(); });
  listen(config_.image_raw(), reliable_1(), images_);
  EXPECT_EQ(size_of(images_), 0u);
  ASSERT_TRUE(wait_until([this] { return backend_->count("start_video") >= 2; }));
  ASSERT_TRUE(emit_until([this] { return size_of(images_) >= 1; }));
}

// --- mapping stream and focus lock (D2, U4) -----------------------------------------------------------------------

namespace
{

constexpr int MAP_W = 32;
constexpr int MAP_H = 18;
constexpr double MAP_RATE_HZ = 2.0;

Config mapping_config(bool with_calibration = false)
{
  Config config = test_config();
  config.mapping.enabled = true;
  config.mapping.width = MAP_W;
  config.mapping.height = MAP_H;
  config.mapping.rate_hz = MAP_RATE_HZ;
  config.mapping.jpeg_quality = 80;
  if (with_calibration) {
    config.camera_info_file = std::string(HEAD_CAMERA_TEST_DATA_DIR) + "/calibration.yaml";
  }
  return config;
}

rclcpp::QoS best_effort_1() { return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(); }

}  // namespace

TEST_F(NodeTest, MappingPublishersExistOnlyWhenEnabled)
{
  start(test_config());
  EXPECT_TRUE(node_->get_publishers_info_by_topic(config_.mapping_image_topic()).empty());
  EXPECT_TRUE(node_->get_publishers_info_by_topic(config_.mapping_camera_info_topic()).empty());
}

TEST_F(NodeTest, MappingPublishersAreBestEffort)
{
  start(mapping_config());
  for (const auto & topic : {config_.mapping_image_topic(), config_.mapping_camera_info_topic()}) {
    const auto infos = node_->get_publishers_info_by_topic(topic);
    ASSERT_EQ(infos.size(), 1u) << topic;
    EXPECT_EQ(infos[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort) << topic;
  }
}

TEST_F(NodeTest, MappingImageAtTwoHertzFromThirtyFpsWithSubscriber)
{
  start(mapping_config());
  std::vector<sensor_msgs::msg::CompressedImage> mapping;
  listen(config_.mapping_image_topic(), best_effort_1(), mapping);
  ASSERT_TRUE(wait_until([this] { return node_->count_subscribers(config_.mapping_image_topic()) > 0; }));
  ASSERT_TRUE(emit_until([&] { return size_of(mapping) >= 1; }));
  const auto first = copy_of(mapping).front();
  EXPECT_EQ(first.format, "bgr8; jpeg compressed bgr8");
  EXPECT_EQ(first.header.frame_id, "head_camera_optical_frame");
  const cv::Mat decoded = cv::imdecode(first.data, cv::IMREAD_COLOR);
  EXPECT_EQ(decoded.cols, MAP_W);
  EXPECT_EQ(decoded.rows, MAP_H);

  // Stream frames for about 2.5 s at 50 Hz from the camera thread: at most one per period lands.
  const auto start_time = std::chrono::steady_clock::now();
  mapping.clear();
  while (std::chrono::steady_clock::now() - start_time < milliseconds(2500)) {
    backend_->emit_video_frame(steady_ns());
    std::this_thread::sleep_for(FRAME_PERIOD);
  }
  EXPECT_GE(size_of(mapping), 4u);
  EXPECT_LE(size_of(mapping), 6u);
  const std::string metrics = node_->metrics().render();
  EXPECT_NE(metrics.find("head_camera_frames_published_total{topic=\"mapping_image\"}"), std::string::npos) << metrics;
}

TEST_F(NodeTest, MappingEncodesAtMostTwoPerSecondAtThirtyFps)
{
  start(mapping_config());
  std::vector<sensor_msgs::msg::CompressedImage> mapping;
  listen(config_.mapping_image_topic(), best_effort_1(), mapping);
  ASSERT_TRUE(wait_until([this] { return node_->count_subscribers(config_.mapping_image_topic()) > 0; }));
  // Frames stamped 33.3 ms apart for 3 s of stamps, delivered as fast as the worker takes them.
  const int64_t base = steady_ns();
  for (int i = 0; i < 90; ++i) {
    backend_->emit_video_frame(base + static_cast<int64_t>(i) * 33'333'333);
    std::this_thread::sleep_for(milliseconds(5));
  }
  std::this_thread::sleep_for(milliseconds(200));
  EXPECT_GE(size_of(mapping), 5u);
  EXPECT_LE(size_of(mapping), 7u);  // 3 s * 2 Hz, plus the first frame
}

TEST_F(NodeTest, NoMappingImageWithoutSubscriber)
{
  start(mapping_config());
  listen(config_.image_raw(), reliable_1(), images_);
  ASSERT_TRUE(emit_until([this] { return size_of(images_) >= 5; }));
  const std::string metrics = node_->metrics().render();
  EXPECT_NE(metrics.find("head_camera_frames_published_total{topic=\"mapping_image\"} 0"), std::string::npos)
    << metrics;
}

TEST_F(NodeTest, MappingCameraInfoOnlyWithCalibrationScaledWithSameStamp)
{
  start(mapping_config(true));
  std::vector<sensor_msgs::msg::CompressedImage> mapping;
  listen(config_.mapping_image_topic(), best_effort_1(), mapping);
  listen(config_.mapping_camera_info_topic(), best_effort_1(), infos_);
  ASSERT_TRUE(emit_until([this, &mapping] { return size_of(mapping) >= 1 && size_of(infos_) >= 1; }));
  const auto info = copy_of(infos_).front();
  EXPECT_EQ(info.width, static_cast<uint32_t>(MAP_W));
  EXPECT_EQ(info.height, static_cast<uint32_t>(MAP_H));
  EXPECT_EQ(info.header.frame_id, "head_camera_optical_frame");
  EXPECT_NEAR(info.k[0], 1600.0 * MAP_W / 2304.0, 1e-9);
  EXPECT_NEAR(info.k[2], 1150.0 * MAP_W / 2304.0, 1e-9);
  EXPECT_NEAR(info.k[4], 1620.0 * MAP_H / 1296.0, 1e-9);
  EXPECT_NEAR(info.k[5], 640.0 * MAP_H / 1296.0, 1e-9);
  bool matched = false;
  for (const auto & image : copy_of(mapping)) {
    matched = matched || rclcpp::Time(image.header.stamp) == rclcpp::Time(info.header.stamp);
  }
  EXPECT_TRUE(matched);
}

TEST_F(NodeTest, MappingImageWithoutCalibrationHasNoCameraInfo)
{
  start(mapping_config(false));
  std::vector<sensor_msgs::msg::CompressedImage> mapping;
  listen(config_.mapping_image_topic(), best_effort_1(), mapping);
  listen(config_.mapping_camera_info_topic(), best_effort_1(), infos_);
  ASSERT_TRUE(emit_until([this, &mapping] { return size_of(mapping) >= 2; }));
  EXPECT_EQ(size_of(infos_), 0u);
}

TEST_F(NodeTest, NothingMappedWhileZoomHoldsTheCamera)
{
  start(mapping_config(true));
  listen_zoom();
  std::vector<sensor_msgs::msg::CompressedImage> mapping;
  listen(config_.mapping_image_topic(), best_effort_1(), mapping);
  listen(config_.mapping_camera_info_topic(), best_effort_1(), infos_);
  ASSERT_TRUE(wait_until([this] { return node_->count_subscribers(config_.mapping_image_topic()) > 0; }));

  std::mutex gate_mutex;
  std::condition_variable gate_cv;
  bool release = false;
  std::atomic<bool> held{false};
  backend_->on_call = [&](const std::string & call) {
    if (call == "set_af" && !held.exchange(true)) {
      std::unique_lock<std::mutex> lock(gate_mutex);
      gate_cv.wait(lock, [&] { return release; });
    }
  };
  send_command(json{{"request_id", "z"}, {"u", 0.5}, {"v", 0.5}, {"zoom", 2}, {"max_px", 256}, {"af", true}}.dump());
  ASSERT_TRUE(wait_until([&] { return held.load(); }));
  for (int i = 0; i < 20; ++i) {
    backend_->emit_video_frame(steady_ns() + static_cast<int64_t>(i) * 600'000'000);
    std::this_thread::sleep_for(milliseconds(10));
  }
  std::this_thread::sleep_for(milliseconds(200));
  EXPECT_EQ(size_of(mapping), 0u);
  EXPECT_EQ(size_of(infos_), 0u);
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release = true;
  }
  gate_cv.notify_all();
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 1; }));
}

TEST_F(NodeTest, FocusLocksOnFirstMappingSubscriberAndReleasesOnLast)
{
  Config config = mapping_config();
  config.lens_position = 1.5;
  start(config);
  EXPECT_FALSE(backend_->focus_locked());
  {
    auto sub = probe_->create_subscription<sensor_msgs::msg::CompressedImage>(
      config_.mapping_image_topic(), best_effort_1(), [](const sensor_msgs::msg::CompressedImage &) {});
    ASSERT_TRUE(wait_until([this] { return backend_->focus_locked(); }));
    sub.reset();
  }
  ASSERT_TRUE(wait_until([this] { return !backend_->focus_locked(); }));
}

TEST_F(NodeTest, FocusLockIsDisabledByLockFocusFalseAtRuntime)
{
  start(mapping_config());
  EXPECT_TRUE(node_->has_parameter("mapping.lock_focus"));
  auto sub = probe_->create_subscription<sensor_msgs::msg::CompressedImage>(
    config_.mapping_image_topic(), best_effort_1(), [](const sensor_msgs::msg::CompressedImage &) {});
  ASSERT_TRUE(wait_until([this] { return backend_->focus_locked(); }));

  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("mapping.lock_focus", false)).successful);
  ASSERT_TRUE(wait_until([this] { return !backend_->focus_locked(); }));

  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("mapping.lock_focus", true)).successful);
  ASSERT_TRUE(wait_until([this] { return backend_->focus_locked(); }));
}

TEST_F(NodeTest, FocusNeverLocksWithLockFocusConfiguredFalse)
{
  Config config = mapping_config();
  config.mapping.lock_focus = false;
  start(config);
  std::vector<sensor_msgs::msg::CompressedImage> mapping;
  listen(config_.mapping_image_topic(), best_effort_1(), mapping);
  ASSERT_TRUE(emit_until([this, &mapping] { return size_of(mapping) >= 1; }));
  EXPECT_FALSE(backend_->focus_locked());
}

TEST_F(NodeTest, FocusNeverLocksWhenMappingDisabled)
{
  start(test_config());
  listen(config_.image_raw(), reliable_1(), images_);
  ASSERT_TRUE(emit_until([this] { return size_of(images_) >= 3; }));
  EXPECT_FALSE(backend_->focus_locked());
  EXPECT_FALSE(node_->has_parameter("mapping.lock_focus"));
}

TEST_F(NodeTest, ZoomWhileLockedRestoresTheMappingLock)
{
  start(mapping_config());
  listen_zoom();
  auto sub = probe_->create_subscription<sensor_msgs::msg::CompressedImage>(
    config_.mapping_image_topic(), best_effort_1(), [](const sensor_msgs::msg::CompressedImage &) {});
  ASSERT_TRUE(wait_until([this] { return backend_->focus_locked(); }));
  send_command(command("a"));
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 1; }));
  EXPECT_EQ(json::parse(copy_of(results_).front().data)["status"], "ok");
  EXPECT_TRUE(backend_->video_running());
  EXPECT_TRUE(backend_->focus_locked());
  EXPECT_TRUE(backend_->lock_at_start_video().back());
}

TEST_F(NodeTest, NoZoomInterfaceWhenDisabled)
{
  Config config = test_config();
  config.zoom.enabled = false;
  start(config);
  EXPECT_TRUE(node_->get_subscriptions_info_by_topic(config_.zoom_command()).empty());
  EXPECT_TRUE(node_->get_publishers_info_by_topic(config_.zoom_image()).empty());
  EXPECT_TRUE(node_->get_publishers_info_by_topic(config_.zoom_result()).empty());
  EXPECT_EQ(node_->metrics().render().find("head_camera_zoom_"), std::string::npos);
  listen(config_.image_raw_compressed(), reliable_1(), compressed_);
  ASSERT_TRUE(emit_until([this] { return size_of(compressed_) >= 1; }));
}

TEST_F(NodeTest, ZoomInterfaceExistsWhenEnabled)
{
  start(test_config());
  EXPECT_EQ(node_->get_subscriptions_info_by_topic(config_.zoom_command()).size(), 1u);
  EXPECT_EQ(node_->get_publishers_info_by_topic(config_.zoom_image()).size(), 1u);
  EXPECT_EQ(node_->get_publishers_info_by_topic(config_.zoom_result()).size(), 1u);
}

// --- metrics (D13) -------------------------------------------------------------------------------------------------

TEST_F(NodeTest, CaptureFpsCountsEveryCallbackFrame)
{
  start(test_config());
  // 11 frames 100 ms apart on the node's clock: ten intervals in one second is 10 fps, whether or not the
  // publisher kept up (no subscriber here, frames may be dropped from the latest-frame slot).
  for (int i = 0; i < 11; ++i) {
    ASSERT_TRUE(backend_->emit_video_frame(steady_ns()));
    clock_.advance(milliseconds(100));
  }
  const std::string metrics = node_->metrics().render();
  EXPECT_NE(metrics.find("head_camera_frames_captured_total 11\n"), std::string::npos) << metrics;
  EXPECT_NE(metrics.find("head_camera_capture_fps 10\n"), std::string::npos) << metrics;
}

TEST_F(NodeTest, CaptureFpsFallsToZeroWhenStreamStops)
{
  constexpr auto npos = std::string::npos;
  start(test_config());
  for (int i = 0; i < 11; ++i) {
    ASSERT_TRUE(backend_->emit_video_frame(steady_ns()));
    clock_.advance(milliseconds(100));
  }
  EXPECT_NE(node_->metrics().render().find("head_camera_capture_fps 10\n"), std::string::npos);
  clock_.advance(seconds(10));
  const auto stopped = [this] { return node_->metrics().render().find("head_camera_capture_fps 0\n") != npos; };
  ASSERT_TRUE(wait_until(stopped)) << node_->metrics().render();
}

TEST_F(NodeTest, ZoomMetricsCountStatusAndPhases)
{
  start(test_config());
  listen_zoom();
  backend_->delay("set_af", milliseconds(100));
  backend_->delay("configure(Still)", milliseconds(200));
  backend_->delay("wait_still_frame", milliseconds(300));
  backend_->delay("configure(Video)", milliseconds(400));
  send_command(json{{"request_id", "m"}, {"u", 0.5}, {"v", 0.5}, {"zoom", 2}, {"max_px", 256}, {"af", true}}.dump());
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 1; }));
  send_command(command("plain"));  // no AF window: the af phase is not entered
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 2; }));
  send_command(json{{"request_id", "bad"}, {"u", 2.0}, {"v", 0.5}, {"zoom", 2}}.dump());
  ASSERT_TRUE(wait_until([this] { return size_of(results_) >= 3; }));
  const std::string metrics = node_->metrics().render();
  // The fake camera never reports a focus lock, so the AF request ends af_failed.
  EXPECT_NE(metrics.find("head_camera_zoom_requests_total{status=\"af_failed\"} 1\n"), std::string::npos) << metrics;
  EXPECT_NE(metrics.find("head_camera_zoom_requests_total{status=\"ok\"} 1\n"), std::string::npos) << metrics;
  EXPECT_NE(metrics.find("head_camera_zoom_requests_total{status=\"invalid\"} 1\n"), std::string::npos) << metrics;
  const std::map<std::string, int> phase_counts{{"af", 1}, {"switch", 2}, {"capture", 2}, {"restore", 2}};
  for (const auto & [phase, count] : phase_counts) {
    const std::string line =
      "head_camera_zoom_phase_seconds_count{phase=\"" + phase + "\"} " + std::to_string(count) + "\n";
    EXPECT_NE(metrics.find(line), std::string::npos) << phase << "\n" << metrics;
  }
}
