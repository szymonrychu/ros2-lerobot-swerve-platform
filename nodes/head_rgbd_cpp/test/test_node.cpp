// HeadRgbdNode in-process: probe publishers/subscribers, a tf2_ros::Buffer fed with static and timed transforms and
// the test ToF calibration.
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <vector>

#include "head_rgbd_cpp/config.hpp"
#include "head_rgbd_cpp/head_rgbd_node.hpp"
#include "head_rgbd_cpp/registration.hpp"
#include "head_rgbd_cpp/tof_filter.hpp"

namespace
{

using head_rgbd_cpp::Config;
using head_rgbd_cpp::HeadRgbdNode;
using std::chrono::milliseconds;
using std::chrono::seconds;

constexpr const char * ODOM = "odom";
constexpr const char * TOF_FRAME = "tof_optical_frame";
constexpr const char * RGB_FRAME = "head_camera_optical_frame";
constexpr const char * NS = "/head_rgbd";
constexpr const char * TOF_TOPIC = "/tof/depth/image_raw";
constexpr const char * RGB_TOPIC = "/head_camera/mapping/image_raw/compressed";
constexpr const char * INFO_TOPIC = "/head_camera/mapping/camera_info";
constexpr int TOF_W = 240;
constexpr int TOF_H = 180;
constexpr int RGB_W = 576;
constexpr int RGB_H = 324;
constexpr double T0_S = 1000.0;
constexpr double TF_SPAN_S = 40.0;
constexpr double TF_STEP_S = 0.1;
constexpr double SET_PERIOD_S = 0.05;
constexpr double TOF_LEAD_S = 0.01;
constexpr double TOF_DEPTH_MM = 1000.0;
constexpr double TF_OFFSET_X_M = 0.03;
constexpr seconds WAIT{12};
constexpr milliseconds POLL{20};

builtin_interfaces::msg::Time stamp_of(double t_s)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_s));
  t.nanosec = static_cast<uint32_t>(std::llround((t_s - std::floor(t_s)) * 1e9));
  return t;
}

bool wait_for(const std::function<bool()> & done, std::chrono::milliseconds timeout = WAIT)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(POLL);
  }
  return done();
}

// The value of one series of a Prometheus text rendering, 0 when absent.
double series_value(const std::string & text, const std::string & series)
{
  const std::string key = "\n" + series + " ";
  const size_t at = ("\n" + text).find(key);
  return at == std::string::npos ? 0.0 : std::stod(text.substr(at + key.size() - 1));
}

template <typename Msg>
struct Inbox
{
  std::mutex mutex;
  std::vector<Msg> messages;
  void add(const Msg & m)
  {
    std::lock_guard<std::mutex> lock(mutex);
    messages.push_back(m);
  }
  size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return messages.size();
  }
  Msg first()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return messages.front();
  }
};

// The message of an inbox with the given header stamp.
template <typename Msg>
std::optional<Msg> find_stamp(Inbox<Msg> & inbox, const builtin_interfaces::msg::Time & stamp)
{
  std::lock_guard<std::mutex> lock(inbox.mutex);
  for (const auto & m : inbox.messages) {
    if (m.header.stamp == stamp) {
      return m;
    }
  }
  return std::nullopt;
}

class NodeTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  static Config base_config()
  {
    Config c;
    c.metrics_port = std::nullopt;
    c.tof_calibration_file = std::string(HEAD_RGBD_TEST_DATA_DIR) + "/tof_calibration.yaml";
    c.lazy.idle_unsubscribe_s = 1.0;
    c.cloud.rate_hz = 1.0;
    return c;
  }

  // Poses: odom -> tof rotates about z at omega_deg_s (timed), tof -> rgb is a static x offset.
  void fill_tf(double omega_deg_s, bool with_tof_tree = true)
  {
    buffer_ = std::make_shared<tf2_ros::Buffer>(
      std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), tf2::Duration(std::chrono::seconds(120)));
    geometry_msgs::msg::TransformStamped st;
    st.header.frame_id = TOF_FRAME;
    st.child_frame_id = RGB_FRAME;
    st.transform.translation.x = TF_OFFSET_X_M;
    st.transform.rotation.w = 1.0;
    buffer_->setTransform(st, "test", true);
    if (!with_tof_tree) {
      return;
    }
    for (double t = T0_S - 1.0; t <= T0_S + TF_SPAN_S; t += TF_STEP_S) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp_of(t);
      tf.header.frame_id = ODOM;
      tf.child_frame_id = TOF_FRAME;
      const double yaw = (t - T0_S) * omega_deg_s * M_PI / 180.0;
      tf.transform.rotation.z = std::sin(yaw / 2.0);
      tf.transform.rotation.w = std::cos(yaw / 2.0);
      buffer_->setTransform(tf, "test", false);
    }
  }

  void start(const Config & config)
  {
    config_ = config;
    node_ = std::make_shared<HeadRgbdNode>(config, buffer_);
    probe_ = std::make_shared<rclcpp::Node>("head_rgbd_probe");
    const auto qos = rclcpp::QoS(1).best_effort();
    tof_pub_ = probe_->create_publisher<sensor_msgs::msg::Image>(TOF_TOPIC, qos);
    rgb_pub_ = probe_->create_publisher<sensor_msgs::msg::CompressedImage>(RGB_TOPIC, qos);
    info_pub_ = probe_->create_publisher<sensor_msgs::msg::CameraInfo>(INFO_TOPIC, qos);
    executor_.add_node(node_);
    executor_.add_node(probe_);
    spinner_ = std::thread([this] { executor_.spin(); });
  }

  void subscribe_outputs()
  {
    const auto qos = rclcpp::QoS(50).best_effort();
    depth_sub_ = probe_->create_subscription<sensor_msgs::msg::Image>(
      std::string(NS) + "/depth/image_raw", qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) { depth_.add(*m); });
    depth_info_sub_ = probe_->create_subscription<sensor_msgs::msg::CameraInfo>(
      std::string(NS) + "/depth/camera_info", qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { depth_info_.add(*m); });
    rgb_sub_ = probe_->create_subscription<sensor_msgs::msg::CompressedImage>(
      std::string(NS) + "/rgb/image_raw/compressed", qos,
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) { rgb_.add(*m); });
    rgb_info_sub_ = probe_->create_subscription<sensor_msgs::msg::CameraInfo>(
      std::string(NS) + "/rgb/camera_info", qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { rgb_info_.add(*m); });
  }

  void subscribe_cloud()
  {
    cloud_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      std::string(NS) + "/points", rclcpp::QoS(50).best_effort(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { cloud_.add(*m); });
  }

  void subscribe_upsampled()
  {
    up_sub_ = probe_->create_subscription<sensor_msgs::msg::Image>(
      std::string(NS) + "/depth_upsampled/image_raw", rclcpp::QoS(50).best_effort(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) { up_.add(*m); });
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  static std::vector<uint8_t> jpeg()
  {
    cv::Mat img(RGB_H, RGB_W, CV_8UC3);
    cv::randu(img, 0, 255);
    cv::rectangle(img, cv::Rect(100, 60, 200, 150), cv::Scalar(30, 120, 220), cv::FILLED);
    std::vector<uint8_t> out;
    cv::imencode(".jpg", img, out);
    return out;
  }

  sensor_msgs::msg::Image tof_image(double t_s, int w = TOF_W, int h = TOF_H) const
  {
    sensor_msgs::msg::Image m;
    m.header.stamp = stamp_of(t_s);
    m.header.frame_id = tof_frame_;
    m.width = w;
    m.height = h;
    m.encoding = "16UC1";
    m.step = w * 2;
    m.data.resize(static_cast<size_t>(w) * h * 2);
    auto * px = reinterpret_cast<uint16_t *>(m.data.data());
    for (int i = 0; i < w * h; ++i) {
      px[i] = static_cast<uint16_t>(TOF_DEPTH_MM);
    }
    return m;
  }

  sensor_msgs::msg::CameraInfo camera_info(double t_s, double fx) const
  {
    sensor_msgs::msg::CameraInfo m;
    m.header.stamp = stamp_of(t_s);
    m.header.frame_id = RGB_FRAME;
    m.width = RGB_W;
    m.height = RGB_H;
    m.distortion_model = "plumb_bob";
    m.d = {0.01, -0.002, 0.0, 0.0, 0.0};
    m.k = {fx, 0, RGB_W / 2.0, 0, fx, RGB_H / 2.0, 0, 0, 1};
    m.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    m.p = {fx, 0, RGB_W / 2.0, 0, 0, fx, RGB_H / 2.0, 0, 0, 0, 1, 0};
    return m;
  }

  // Publish one synchronised set stamped T0 + index * SET_PERIOD_S (the ToF frame TOF_LEAD_S earlier).
  void publish_set(int index, double fx = 500.0, int tof_w = TOF_W, int tof_h = TOF_H)
  {
    const double t = T0_S + 1.0 + index * SET_PERIOD_S;
    tof_pub_->publish(tof_image(t - TOF_LEAD_S, tof_w, tof_h));
    sensor_msgs::msg::CompressedImage rgb;
    rgb.header.stamp = stamp_of(t);
    rgb.header.frame_id = RGB_FRAME;
    rgb.format = "jpeg";
    rgb.data = jpeg_;
    rgb_pub_->publish(rgb);
    info_pub_->publish(camera_info(t, fx));
  }

  // Publishes sets every SET_PERIOD_S until done() or the timeout; returns done().
  bool pump_until(const std::function<bool()> & done, double fx = 500.0, int tof_w = TOF_W, int tof_h = TOF_H)
  {
    const auto end = std::chrono::steady_clock::now() + WAIT;
    while (std::chrono::steady_clock::now() < end) {
      if (done()) {
        return true;
      }
      publish_set(index_++, fx, tof_w, tof_h);
      std::this_thread::sleep_for(milliseconds(static_cast<int>(SET_PERIOD_S * 1000)));
    }
    return done();
  }

  bool wait_inputs_subscribed()
  {
    return wait_for([this] {
      return rgb_pub_->get_subscription_count() > 0 && tof_pub_->get_subscription_count() > 0 &&
             info_pub_->get_subscription_count() > 0;
    });
  }

  double metric(const std::string & series) { return series_value(node_->metrics().render(), series); }
  double dropped(const std::string & reason)
  {
    return metric("head_rgbd_pairs_dropped_total{reason=\"" + reason + "\"}");
  }

  // Drop scenario: the pump runs until the reason counter rose, then nothing may have been published.
  void expect_drop(const std::string & reason, double fx = 500.0, int tof_w = TOF_W, int tof_h = TOF_H)
  {
    subscribe_outputs();
    ASSERT_TRUE(wait_inputs_subscribed());
    ASSERT_TRUE(pump_until([&] { return dropped(reason) >= 1.0; }, fx, tof_w, tof_h)) << reason;
    std::this_thread::sleep_for(milliseconds(300));
    EXPECT_EQ(depth_.size(), 0u);
    EXPECT_EQ(depth_info_.size(), 0u);
    EXPECT_EQ(rgb_.size(), 0u);
    EXPECT_EQ(rgb_info_.size(), 0u);
    EXPECT_EQ(metric("head_rgbd_pairs_published_total"), 0.0);
  }

  std::string tof_frame_ = TOF_FRAME;
  std::vector<uint8_t> jpeg_ = jpeg();
  int index_ = 0;
  Config config_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<HeadRgbdNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tof_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, up_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_, rgb_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  Inbox<sensor_msgs::msg::Image> depth_, up_;
  Inbox<sensor_msgs::msg::CameraInfo> depth_info_, rgb_info_;
  Inbox<sensor_msgs::msg::CompressedImage> rgb_;
  Inbox<sensor_msgs::msg::PointCloud2> cloud_;
};

}  // namespace

TEST_F(NodeTest, InputsAreSubscribedOnlyWhileAnOutputHasSubscribers)
{
  fill_tf(0.0);
  start(base_config());
  std::this_thread::sleep_for(milliseconds(2500));
  EXPECT_EQ(rgb_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(tof_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(info_pub_->get_subscription_count(), 0u);

  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());
  EXPECT_GE(tof_pub_->get_subscription_count(), 1u);
  EXPECT_GE(info_pub_->get_subscription_count(), 1u);

  depth_sub_.reset();
  depth_info_sub_.reset();
  rgb_sub_.reset();
  rgb_info_sub_.reset();
  EXPECT_TRUE(wait_for([this] { return rgb_pub_->get_subscription_count() == 0; }));
  EXPECT_EQ(tof_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(info_pub_->get_subscription_count(), 0u);
}

TEST_F(NodeTest, PublishesRegisteredPairWithTheRgbStampAndFrame)
{
  fill_tf(0.0);
  start(base_config());
  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());
  ASSERT_TRUE(
    pump_until([&] { return depth_.size() > 0 && depth_info_.size() > 0 && rgb_.size() > 0 && rgb_info_.size() > 0; }));

  const auto depth = depth_.first();
  EXPECT_EQ(depth.encoding, "16UC1");
  EXPECT_EQ(depth.width, 288u);
  EXPECT_EQ(depth.height, 162u);
  EXPECT_EQ(depth.header.frame_id, RGB_FRAME);

  // The RGB message and the infos with the depth's stamp.
  const auto & stamp = depth.header.stamp;
  ASSERT_TRUE(wait_for(
    [&] { return find_stamp(rgb_, stamp) && find_stamp(depth_info_, stamp) && find_stamp(rgb_info_, stamp); }));
  const auto rgb = *find_stamp(rgb_, stamp);
  EXPECT_EQ(rgb.header.frame_id, RGB_FRAME);
  EXPECT_EQ(rgb.data, jpeg_);
  EXPECT_EQ(rgb.format, "jpeg");
  const auto rgb_info = *find_stamp(rgb_info_, stamp);
  const auto expected_info = camera_info(0.0, 500.0);
  EXPECT_EQ(rgb_info.k, expected_info.k);
  EXPECT_EQ(rgb_info.width, static_cast<uint32_t>(RGB_W));
  const auto depth_info = *find_stamp(depth_info_, stamp);
  EXPECT_EQ(depth_info.header.frame_id, RGB_FRAME);
  EXPECT_EQ(depth_info.width, 288u);
  EXPECT_EQ(depth_info.height, 162u);
  EXPECT_EQ(depth_info.d, expected_info.d);
  EXPECT_NEAR(depth_info.k[0], 250.0, 1e-9);
  EXPECT_NEAR(depth_info.k[2], (RGB_W / 2.0 + 0.5) / 2.0 - 0.5, 1e-9);
  EXPECT_NEAR(depth_info.k[5], (RGB_H / 2.0 + 0.5) / 2.0 - 0.5, 1e-9);
  EXPECT_NEAR(depth_info.p[0], depth_info.k[0], 1e-9);
  EXPECT_NEAR(depth_info.p[2], depth_info.k[2], 1e-9);
  EXPECT_EQ(depth_info.p[3], 0.0);

  // The depth equals register_depth on the same input.
  const head_rgbd_cpp::PinholeModel tof{
    cv::Matx33d(191, 0, 120, 0, 191, 90, 0, 0, 1), cv::Vec<double, 5>(0, 0, 0, 0, 0), TOF_W, TOF_H};
  const head_rgbd_cpp::RayTable rays(tof);
  const head_rgbd_cpp::PinholeModel rgb_model{
    cv::Matx33d(500, 0, RGB_W / 2.0, 0, 500, RGB_H / 2.0, 0, 0, 1), cv::Vec<double, 5>(0.01, -0.002, 0, 0, 0), RGB_W,
    RGB_H};
  const auto out_model = head_rgbd_cpp::scale_model(rgb_model, 288, 162);
  const cv::Mat input(TOF_H, TOF_W, CV_16UC1, cv::Scalar(TOF_DEPTH_MM));
  const cv::Mat filtered = head_rgbd_cpp::filter_jump_edges(input, config_.filter);
  Eigen::Isometry3d rgb_T_tof = Eigen::Isometry3d::Identity();
  rgb_T_tof.translation() = Eigen::Vector3d(-TF_OFFSET_X_M, 0, 0);
  const cv::Mat expected = head_rgbd_cpp::register_depth(filtered, rays, rgb_T_tof, out_model, config_.depth);
  ASSERT_EQ(depth.data.size(), static_cast<size_t>(288 * 162 * 2));
  EXPECT_EQ(std::memcmp(depth.data.data(), expected.data, depth.data.size()), 0);
  EXPECT_GT(cv::countNonZero(expected), 0);
  EXPECT_GE(metric("head_rgbd_pairs_published_total"), 1.0);
}

TEST_F(NodeTest, DropsWithoutTransform)
{
  fill_tf(0.0, false);
  start(base_config());
  expect_drop("no_tf");
}

TEST_F(NodeTest, DropsWithoutCalibrationFile)
{
  fill_tf(0.0);
  Config c = base_config();
  c.tof_calibration_file = "/nonexistent/tof_calibration.yaml";
  start(c);
  expect_drop("no_calibration");
}

TEST_F(NodeTest, DropsAZeroCameraMatrix)
{
  fill_tf(0.0);
  start(base_config());
  expect_drop("no_camera_info", 0.0);
}

TEST_F(NodeTest, DropsAWrongTofSize)
{
  fill_tf(0.0);
  start(base_config());
  expect_drop("size_mismatch", 500.0, 120, 90);
}

TEST_F(NodeTest, DropsDuringFastMotion)
{
  fill_tf(20.0);
  start(base_config());
  expect_drop("motion");
}

TEST_F(NodeTest, CloudIsPublishedAtMostAtTheConfiguredRate)
{
  fill_tf(0.0);
  Config c = base_config();
  c.cloud.rate_hz = 1.0;
  start(c);
  subscribe_outputs();
  subscribe_cloud();
  ASSERT_TRUE(wait_inputs_subscribed());
  // 1.5 s of stream stamps (30 sets); at 1 Hz at most 2 clouds.
  ASSERT_TRUE(wait_for([&] { return cloud_sub_->get_publisher_count() > 0; }));
  const int first = index_;
  ASSERT_TRUE(pump_until([&] { return index_ - first >= 30 && metric("head_rgbd_pairs_published_total") >= 20.0; }));
  std::this_thread::sleep_for(milliseconds(500));
  EXPECT_GE(cloud_.size(), 1u);
  const double span_s = (index_ - first) * SET_PERIOD_S;
  EXPECT_LE(cloud_.size(), static_cast<size_t>(std::ceil(span_s * 1.0)) + 1u);
  const auto cloud = cloud_.first();
  EXPECT_EQ(cloud.header.frame_id, RGB_FRAME);
}

TEST_F(NodeTest, UpsampledTopicsExistOnlyWhenEnabled)
{
  fill_tf(0.0);
  start(base_config());
  std::this_thread::sleep_for(milliseconds(500));
  EXPECT_EQ(probe_->get_publishers_info_by_topic(std::string(NS) + "/depth_upsampled/image_raw").size(), 0u);
  EXPECT_EQ(probe_->get_publishers_info_by_topic(std::string(NS) + "/depth_upsampled/camera_info").size(), 0u);
}

TEST_F(NodeTest, UpsampledDepthHasTheRgbSizeWhenEnabled)
{
  fill_tf(0.0);
  Config c = base_config();
  c.upsampling.enabled = true;
  start(c);
  subscribe_outputs();
  subscribe_upsampled();
  ASSERT_TRUE(wait_inputs_subscribed());
  ASSERT_TRUE(pump_until([&] { return up_.size() > 0; }));
  const auto up = up_.first();
  EXPECT_EQ(up.width, static_cast<uint32_t>(RGB_W));
  EXPECT_EQ(up.height, static_cast<uint32_t>(RGB_H));
  EXPECT_EQ(up.header.frame_id, RGB_FRAME);
  EXPECT_EQ(up.encoding, "16UC1");
}
