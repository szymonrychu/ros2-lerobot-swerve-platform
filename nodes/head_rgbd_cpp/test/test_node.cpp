// HeadRgbdNode in-process: probe publishers of the D435 aligned depth, raw colour and colour camera_info (and of the
// full-FOV depth and its camera_info for the filtered stage), probe subscribers of the /head_rgbd and depth_filtered
// outputs; plus the pure helpers of the node (depth downscale, camera_info scale, set throttle).
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "head_rgbd_cpp/config.hpp"
#include "head_rgbd_cpp/head_rgbd_node.hpp"

namespace
{

using head_rgbd_cpp::Config;
using head_rgbd_cpp::HeadRgbdNode;
using std::chrono::milliseconds;
using std::chrono::seconds;

constexpr const char * COLOR_FRAME = "d435_color_optical_frame";
// The aligned depth carries the colour frame on the robot; a different frame here shows the outputs take the colour
// header.
constexpr const char * DEPTH_FRAME = "aligned_depth_test_frame";
constexpr const char * NS = "/head_rgbd";
constexpr const char * DEPTH_TOPIC = "/head/d435/aligned_depth_to_color/image_raw";
constexpr const char * COLOR_TOPIC = "/head/d435/color/image_raw";
constexpr const char * INFO_TOPIC = "/head/d435/color/camera_info";
constexpr const char * RAW_DEPTH_TOPIC = "/head/d435/depth/image_rect_raw";
constexpr const char * RAW_INFO_TOPIC = "/head/d435/depth/camera_info";
constexpr const char * FILTERED_IMAGE_TOPIC = "/head/d435/depth_filtered/image_raw";
constexpr const char * FILTERED_INFO_TOPIC = "/head/d435/depth_filtered/camera_info";
constexpr const char * RAW_DEPTH_FRAME = "d435_depth_optical_frame";
constexpr const char * COMPRESSED_FORMAT = "bgr8; jpeg compressed bgr8";
constexpr int W = 160;
constexpr int H = 90;
constexpr double FX = 80.0;
constexpr double FY = 81.0;
constexpr double CX = 79.5;
constexpr double CY = 44.5;
constexpr double T0_S = 1000.0;
constexpr double STREAM_PERIOD_S = 1.0 / 15.0;
constexpr milliseconds PUBLISH_PERIOD{30};
constexpr uint16_t BELOW_RANGE_MM = 100;
constexpr uint16_t ABOVE_RANGE_MM = 5000;
constexpr seconds WAIT{12};
// The order the downscale tries the pixels of a 2x2 block in, as (dy, dx).
constexpr std::array<std::pair<int, int>, 4> BLOCK_ORDER = {{{0, 0}, {0, 1}, {1, 0}, {1, 1}}};
constexpr milliseconds POLL{20};

builtin_interfaces::msg::Time stamp_of(double t_s)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_s));
  t.nanosec = static_cast<uint32_t>(std::llround((t_s - std::floor(t_s)) * 1e9));
  return t;
}

double seconds_of(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }

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

// An in-range depth in mm, distinct per pixel.
uint16_t in_range_mm(int x, int y) { return static_cast<uint16_t>(1000 + (x * 7 + y * 13) % 400); }

// The depth of the downscale scene: some blocks lose their top-left pixel (0), some their top-right (below range),
// some their bottom-left (above range).
uint16_t scene_mm(int x, int y)
{
  if (x % 2 == 0 && y % 2 == 0 && (x / 2 + y / 2) % 3 == 0) {
    return 0;
  }
  if (x % 2 == 1 && y % 2 == 0 && (x / 2) % 5 == 0) {
    return BELOW_RANGE_MM;
  }
  if (x % 2 == 0 && y % 2 == 1 && (x / 2) % 7 == 0) {
    return ABOVE_RANGE_MM;
  }
  return in_range_mm(x, y);
}

// A depth step: NEAR_MM left of STEP_COLUMN, FAR_MM from it on. The edge mask removes the column STEP_COLUMN (the far
// side of the jump) and keeps every other pixel.
constexpr int STEP_COLUMN = 60;
constexpr uint16_t NEAR_MM = 1000;
constexpr uint16_t FAR_MM = 1500;
uint16_t step_mm(int x, int /*y*/) { return x < STEP_COLUMN ? NEAR_MM : FAR_MM; }

// The colour of the test image at (x, y) as R, G, B: a smooth gradient that survives JPEG.
cv::Vec3b rgb_at(int x, int y) { return cv::Vec3b(static_cast<uint8_t>(x), static_cast<uint8_t>(2 * y), 128); }

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
  std::vector<Msg> all()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return messages;
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

// One input set: what the probe publishes for a stream index.
struct SetSpec
{
  std::string depth_encoding = "16UC1";
  int depth_w = W;
  int depth_h = H;
  bool with_info = true;
  double fx = FX;
  double depth_offset_s = 0.0;
  std::function<uint16_t(int, int)> depth_mm = in_range_mm;
};

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
    c.lazy.idle_unsubscribe_s = 1.0;
    c.output.rate_hz = 15.0;
    c.cloud.rate_hz = 1.0;
    // The set plumbing tests compare the published depth with the input pixel for pixel; the edge mask has its own
    // tests below (and in test/core/test_edge_mask_scene.cpp).
    c.edge_mask.enabled = false;
    return c;
  }

  void start(const Config & config)
  {
    config_ = config;
    node_ = std::make_shared<HeadRgbdNode>(config);
    probe_ = std::make_shared<rclcpp::Node>("head_rgbd_probe");
    const auto qos = rclcpp::QoS(5).best_effort();
    depth_pub_ = probe_->create_publisher<sensor_msgs::msg::Image>(DEPTH_TOPIC, qos);
    color_pub_ = probe_->create_publisher<sensor_msgs::msg::Image>(COLOR_TOPIC, qos);
    info_pub_ = probe_->create_publisher<sensor_msgs::msg::CameraInfo>(INFO_TOPIC, qos);
    raw_depth_pub_ = probe_->create_publisher<sensor_msgs::msg::Image>(RAW_DEPTH_TOPIC, qos);
    raw_info_pub_ = probe_->create_publisher<sensor_msgs::msg::CameraInfo>(RAW_INFO_TOPIC, qos);
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

  void subscribe_filtered()
  {
    const auto qos = rclcpp::QoS(50).best_effort();
    filtered_sub_ = probe_->create_subscription<sensor_msgs::msg::Image>(
      FILTERED_IMAGE_TOPIC, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) { filtered_.add(*m); });
    filtered_info_sub_ = probe_->create_subscription<sensor_msgs::msg::CameraInfo>(
      FILTERED_INFO_TOPIC, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { filtered_info_.add(*m); });
  }

  bool wait_filtered_inputs_subscribed()
  {
    return wait_for(
      [this] { return raw_depth_pub_->get_subscription_count() > 0 && raw_info_pub_->get_subscription_count() > 0; });
  }

  // Publishes full-FOV depth frames (with their camera_info unless spec.with_info is false) every PUBLISH_PERIOD until
  // done() or the timeout; returns done().
  bool pump_raw_until(const std::function<bool()> & done, const SetSpec & spec)
  {
    const auto end = std::chrono::steady_clock::now() + WAIT;
    while (std::chrono::steady_clock::now() < end) {
      if (done()) {
        return true;
      }
      const double t = set_time(raw_index_++);
      if (spec.with_info) {
        auto info = camera_info(t, spec.fx);
        info.header.frame_id = RAW_DEPTH_FRAME;
        raw_info_pub_->publish(info);
      }
      auto depth = depth_image(t, spec);
      depth.header.frame_id = RAW_DEPTH_FRAME;
      raw_depth_pub_->publish(depth);
      std::this_thread::sleep_for(PUBLISH_PERIOD);
    }
    return done();
  }

  double filtered_dropped(const std::string & reason)
  {
    return metric("head_rgbd_filtered_depth_dropped_total{reason=\"" + reason + "\"}");
  }
  double filtered_published() { return metric("head_rgbd_filtered_depth_published_total"); }

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  static sensor_msgs::msg::Image depth_image(double t_s, const SetSpec & spec)
  {
    sensor_msgs::msg::Image m;
    m.header.stamp = stamp_of(t_s);
    m.header.frame_id = DEPTH_FRAME;
    m.width = spec.depth_w;
    m.height = spec.depth_h;
    m.encoding = spec.depth_encoding;
    const int bytes_per_px = spec.depth_encoding == "16UC1" ? 2 : 4;
    m.step = spec.depth_w * bytes_per_px;
    m.data.assign(static_cast<size_t>(m.step) * spec.depth_h, 0);
    if (bytes_per_px == 2) {
      auto * px = reinterpret_cast<uint16_t *>(m.data.data());
      for (int y = 0; y < spec.depth_h; ++y) {
        for (int x = 0; x < spec.depth_w; ++x) {
          px[y * spec.depth_w + x] = spec.depth_mm(x, y);
        }
      }
    }
    return m;
  }

  static sensor_msgs::msg::Image color_image(double t_s)
  {
    sensor_msgs::msg::Image m;
    m.header.stamp = stamp_of(t_s);
    m.header.frame_id = COLOR_FRAME;
    m.width = W;
    m.height = H;
    m.encoding = "rgb8";
    m.step = W * 3;
    m.data.resize(static_cast<size_t>(m.step) * H);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const cv::Vec3b c = rgb_at(x, y);
        std::memcpy(&m.data[static_cast<size_t>(y) * m.step + x * 3], c.val, 3);
      }
    }
    return m;
  }

  // The colour camera_info; its stamp differs from the set stamp (the node keeps the latest of the colour size).
  static sensor_msgs::msg::CameraInfo camera_info(double t_s, double fx)
  {
    sensor_msgs::msg::CameraInfo m;
    m.header.stamp = stamp_of(t_s - 0.5);
    m.header.frame_id = COLOR_FRAME;
    m.width = W;
    m.height = H;
    m.distortion_model = "plumb_bob";
    m.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    m.k = {fx, 0, CX, 0, fx == 0.0 ? 0.0 : FY, CY, 0, 0, fx == 0.0 ? 0.0 : 1.0};
    m.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    m.p = {fx, 0, CX, 0, 0, fx == 0.0 ? 0.0 : FY, CY, 0, 0, 0, 1, 0};
    return m;
  }

  static double set_time(int index) { return T0_S + 1.0 + index * STREAM_PERIOD_S; }

  // Publish the set of a stream index: camera_info first, then the aligned depth and the colour image.
  void publish_set(int index, const SetSpec & spec)
  {
    const double t = set_time(index);
    if (spec.with_info) {
      info_pub_->publish(camera_info(t, spec.fx));
    }
    depth_pub_->publish(depth_image(t + spec.depth_offset_s, spec));
    color_pub_->publish(color_image(t));
  }

  // Publishes sets every PUBLISH_PERIOD until done() or the timeout; returns done().
  bool pump_until(const std::function<bool()> & done, const SetSpec & spec = SetSpec())
  {
    const auto end = std::chrono::steady_clock::now() + WAIT;
    while (std::chrono::steady_clock::now() < end) {
      if (done()) {
        return true;
      }
      publish_set(index_++, spec);
      std::this_thread::sleep_for(PUBLISH_PERIOD);
    }
    return done();
  }

  bool wait_inputs_subscribed()
  {
    return wait_for([this] {
      return depth_pub_->get_subscription_count() > 0 && color_pub_->get_subscription_count() > 0 &&
             info_pub_->get_subscription_count() > 0;
    });
  }

  double metric(const std::string & series) { return series_value(node_->metrics().render(), series); }
  double dropped(const std::string & reason) { return metric("head_rgbd_dropped_total{reason=\"" + reason + "\"}"); }
  double published() { return metric("head_rgbd_sets_published_total"); }

  void expect_nothing_published()
  {
    std::this_thread::sleep_for(milliseconds(300));
    EXPECT_EQ(depth_.size(), 0u);
    EXPECT_EQ(depth_info_.size(), 0u);
    EXPECT_EQ(rgb_.size(), 0u);
    EXPECT_EQ(rgb_info_.size(), 0u);
    EXPECT_EQ(published(), 0.0);
  }

  // Drop scenario: the pump runs until the reason counter rose, then nothing may have been published.
  void expect_drop(const std::string & reason, const SetSpec & spec)
  {
    subscribe_outputs();
    ASSERT_TRUE(wait_inputs_subscribed());
    ASSERT_TRUE(pump_until([&] { return dropped(reason) >= 1.0; }, spec)) << reason;
    expect_nothing_published();
  }

  int index_ = 0;
  int raw_index_ = 0;
  Config config_;
  std::shared_ptr<HeadRgbdNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr raw_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr filtered_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr filtered_info_sub_;
  Inbox<sensor_msgs::msg::Image> filtered_;
  Inbox<sensor_msgs::msg::CameraInfo> filtered_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_, rgb_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  Inbox<sensor_msgs::msg::Image> depth_;
  Inbox<sensor_msgs::msg::CameraInfo> depth_info_, rgb_info_;
  Inbox<sensor_msgs::msg::CompressedImage> rgb_;
  Inbox<sensor_msgs::msg::PointCloud2> cloud_;
};

}  // namespace

// --- pure helpers ---------------------------------------------------------------------------------------------------

TEST(DownscaleDepth, KeepsTheFirstValidPixelOfEachBlockInRowMajorOrder)
{
  head_rgbd_cpp::DepthConfig range;  // 0.15 .. 4.0 m
  const cv::Mat depth =
    (cv::Mat_<uint16_t>(2, 6) << 1000, 1001, 0, 2001, 0, 0,  //
     1010, 1011, 2010, 2011, 0, 3011);
  const cv::Mat out = head_rgbd_cpp::downscale_depth(depth, 2, range);
  ASSERT_EQ(out.type(), CV_16UC1);
  ASSERT_EQ(out.cols, 3);
  ASSERT_EQ(out.rows, 1);
  EXPECT_EQ(out.at<uint16_t>(0, 0), 1000);
  EXPECT_EQ(out.at<uint16_t>(0, 1), 2001);
  EXPECT_EQ(out.at<uint16_t>(0, 2), 3011);
}

TEST(DownscaleDepth, TreatsDepthOutsideTheRangeAsMissing)
{
  head_rgbd_cpp::DepthConfig range;
  range.min_m = 0.5;
  range.max_m = 2.0;
  const cv::Mat depth =
    (cv::Mat_<uint16_t>(2, 4) << 499, 2001, 500, 3000,  //
     2000, 7, 0, 0);
  const cv::Mat halved = head_rgbd_cpp::downscale_depth(depth, 2, range);
  EXPECT_EQ(halved.at<uint16_t>(0, 0), 2000);
  EXPECT_EQ(halved.at<uint16_t>(0, 1), 500);
  const cv::Mat full = head_rgbd_cpp::downscale_depth(depth, 1, range);
  ASSERT_EQ(full.size(), depth.size());
  EXPECT_EQ(full.at<uint16_t>(0, 0), 0);
  EXPECT_EQ(full.at<uint16_t>(0, 1), 0);
  EXPECT_EQ(full.at<uint16_t>(0, 2), 500);
  EXPECT_EQ(full.at<uint16_t>(1, 0), 2000);
  EXPECT_EQ(full.at<uint16_t>(1, 1), 0);
}

TEST(DownscaleDepth, AnAllMissingBlockStaysMissing)
{
  head_rgbd_cpp::DepthConfig range;
  const cv::Mat depth = (cv::Mat_<uint16_t>(2, 2) << 0, 0, 0, 50);
  const cv::Mat out = head_rgbd_cpp::downscale_depth(depth, 2, range);
  EXPECT_EQ(out.at<uint16_t>(0, 0), 0);
}

TEST(ScaleCameraInfo, DividesTheProjectionAndSizeByTheDivisor)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 640;
  info.height = 360;
  info.distortion_model = "plumb_bob";
  info.d = {0.1, 0.2, 0.0, 0.0, 0.3};
  info.k = {600, 0, 320, 0, 602, 180, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {600, 0, 320, 0, 0, 602, 180, 0, 0, 0, 1, 0};
  const auto half = head_rgbd_cpp::scale_camera_info(info, 2);
  EXPECT_EQ(half.width, 320u);
  EXPECT_EQ(half.height, 180u);
  const std::array<double, 9> k = {300, 0, 160, 0, 301, 90, 0, 0, 1};
  const std::array<double, 12> p = {300, 0, 160, 0, 0, 301, 90, 0, 0, 0, 1, 0};
  for (size_t i = 0; i < k.size(); ++i) {
    EXPECT_DOUBLE_EQ(half.k[i], k[i]) << i;
  }
  for (size_t i = 0; i < p.size(); ++i) {
    EXPECT_DOUBLE_EQ(half.p[i], p[i]) << i;
  }
  EXPECT_EQ(half.d, info.d);
  EXPECT_EQ(half.r, info.r);
  EXPECT_EQ(half.distortion_model, info.distortion_model);
  const auto same = head_rgbd_cpp::scale_camera_info(info, 1);
  EXPECT_EQ(same.k, info.k);
  EXPECT_EQ(same.p, info.p);
  EXPECT_EQ(same.width, info.width);
}

TEST(SetThrottle, KeepsTheConfiguredAverageRateOfAFasterStream)
{
  head_rgbd_cpp::SetThrottle throttle(2.0);
  int accepted = 0;
  const int64_t frame_ns = 66'666'667;  // 15 Hz
  for (int i = 0; i < 150; ++i) {       // 10 s
    accepted += throttle.due(i * frame_ns) ? 1 : 0;
  }
  EXPECT_GE(accepted, 20);
  EXPECT_LE(accepted, 21);
}

TEST(SetThrottle, AcceptsTheFirstStampAndRestartsAfterAJumpBack)
{
  head_rgbd_cpp::SetThrottle throttle(2.0);
  EXPECT_TRUE(throttle.due(10'000'000'000));
  EXPECT_FALSE(throttle.due(10'100'000'000));
  EXPECT_TRUE(throttle.due(10'500'000'000));
  EXPECT_TRUE(throttle.due(1'000'000'000));  // replay loop
  EXPECT_FALSE(throttle.due(1'200'000'000));
  EXPECT_TRUE(throttle.due(5'000'000'000));  // a gap: the next one is a full period later
  EXPECT_FALSE(throttle.due(5'400'000'000));
  EXPECT_TRUE(throttle.due(5'500'000'000));
}

// --- the node -------------------------------------------------------------------------------------------------------

TEST_F(NodeTest, InputsAreSubscribedOnlyWhileAnOutputHasSubscribers)
{
  start(base_config());
  std::this_thread::sleep_for(milliseconds(2500));
  EXPECT_EQ(depth_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(color_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(info_pub_->get_subscription_count(), 0u);

  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());

  depth_sub_.reset();
  depth_info_sub_.reset();
  rgb_sub_.reset();
  rgb_info_sub_.reset();
  EXPECT_TRUE(wait_for([this] {
    return depth_pub_->get_subscription_count() == 0 && color_pub_->get_subscription_count() == 0 &&
           info_pub_->get_subscription_count() == 0;
  }));
}

TEST_F(NodeTest, DoesNotAdvertiseUpsampledTopics)
{
  start(base_config());
  const std::vector<std::string> outputs = {
    "/depth/image_raw", "/depth/camera_info", "/rgb/image_raw/compressed", "/rgb/camera_info", "/points"};
  ASSERT_TRUE(wait_for([&] {
    for (const auto & topic : outputs) {
      if (probe_->get_publishers_info_by_topic(std::string(NS) + topic).empty()) {
        return false;
      }
    }
    return true;
  }));
  EXPECT_EQ(probe_->get_publishers_info_by_topic(std::string(NS) + "/depth_upsampled/image_raw").size(), 0u);
  EXPECT_EQ(probe_->get_publishers_info_by_topic(std::string(NS) + "/depth_upsampled/camera_info").size(), 0u);
}

TEST_F(NodeTest, PublishesTheAlignedSetWithTheColourStampAndFrame)
{
  Config c = base_config();
  c.output.divisor = 1;
  start(c);
  subscribe_outputs();
  subscribe_cloud();
  ASSERT_TRUE(wait_inputs_subscribed());
  ASSERT_TRUE(wait_for([&] { return cloud_sub_->get_publisher_count() > 0; }));
  ASSERT_TRUE(pump_until([&] {
    return depth_.size() > 0 && depth_info_.size() > 0 && rgb_.size() > 0 && rgb_info_.size() > 0 && cloud_.size() > 0;
  }));

  const auto depth = depth_.first();
  const auto & stamp = depth.header.stamp;
  // The stamp is the one of a published colour frame.
  const double t = seconds_of(stamp);
  const double index = (t - set_time(0)) / STREAM_PERIOD_S;
  EXPECT_NEAR(index, std::round(index), 1e-3);
  EXPECT_EQ(depth.header.frame_id, COLOR_FRAME);
  EXPECT_EQ(depth.encoding, "16UC1");
  EXPECT_EQ(depth.width, static_cast<uint32_t>(W));
  EXPECT_EQ(depth.height, static_cast<uint32_t>(H));
  ASSERT_EQ(depth.data.size(), static_cast<size_t>(W * H * 2));
  const auto expected = depth_image(t, SetSpec());
  EXPECT_EQ(depth.data, expected.data);

  ASSERT_TRUE(wait_for(
    [&] { return find_stamp(rgb_, stamp) && find_stamp(depth_info_, stamp) && find_stamp(rgb_info_, stamp); }));
  const auto input_info = camera_info(t, FX);
  const auto depth_info = *find_stamp(depth_info_, stamp);
  EXPECT_EQ(depth_info.header.frame_id, COLOR_FRAME);
  EXPECT_EQ(depth_info.width, static_cast<uint32_t>(W));
  EXPECT_EQ(depth_info.k, input_info.k);
  EXPECT_EQ(depth_info.p, input_info.p);
  const auto rgb_info = *find_stamp(rgb_info_, stamp);
  EXPECT_EQ(rgb_info.header.frame_id, COLOR_FRAME);
  EXPECT_EQ(rgb_info.k, input_info.k);
  EXPECT_EQ(rgb_info.d, input_info.d);
  EXPECT_EQ(rgb_info.width, static_cast<uint32_t>(W));

  const auto rgb = *find_stamp(rgb_, stamp);
  EXPECT_EQ(rgb.header.frame_id, COLOR_FRAME);
  EXPECT_EQ(rgb.format, COMPRESSED_FORMAT);
  const cv::Mat decoded = cv::imdecode(rgb.data, cv::IMREAD_COLOR);
  ASSERT_EQ(decoded.cols, W);
  ASSERT_EQ(decoded.rows, H);
  double error = 0.0;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const cv::Vec3b want = rgb_at(x, y);                // R, G, B
      const cv::Vec3b got = decoded.at<cv::Vec3b>(y, x);  // B, G, R
      error += std::abs(got[2] - want[0]) + std::abs(got[1] - want[1]) + std::abs(got[0] - want[2]);
    }
  }
  EXPECT_LT(error / (W * H * 3), 4.0);

  const auto cloud = cloud_.first();
  EXPECT_EQ(cloud.header.frame_id, COLOR_FRAME);
  EXPECT_TRUE(wait_for([&] { return find_stamp(rgb_, cloud.header.stamp).has_value(); }));
  EXPECT_GE(published(), 1.0);
}

TEST_F(NodeTest, ThrottlesSetsToTheOutputRate)
{
  Config c = base_config();
  c.output.rate_hz = 2.0;
  start(c);
  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());
  const int sets = 60;  // 4 s of 15 Hz stream stamps
  ASSERT_TRUE(pump_until([&] { return index_ >= sets; }));
  std::this_thread::sleep_for(milliseconds(500));
  const double span_s = sets * STREAM_PERIOD_S;
  const size_t limit = static_cast<size_t>(std::ceil(span_s * c.output.rate_hz)) + 1u;
  EXPECT_GE(depth_.size(), 3u);
  EXPECT_LE(depth_.size(), limit);
  EXPECT_LE(published(), static_cast<double>(limit));
  const auto messages = depth_.all();
  for (size_t i = 1; i < messages.size(); ++i) {
    const double dt = seconds_of(messages[i].header.stamp) - seconds_of(messages[i - 1].header.stamp);
    EXPECT_GE(dt, 1.0 / c.output.rate_hz - STREAM_PERIOD_S - 1e-6) << i;
  }
}

TEST_F(NodeTest, DownscalesDepthAndCameraInfoByTheDivisor)
{
  Config c = base_config();
  c.output.divisor = 2;
  start(c);
  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());
  SetSpec spec;
  spec.depth_mm = scene_mm;
  ASSERT_TRUE(pump_until([&] { return depth_.size() > 0 && depth_info_.size() > 0 && rgb_.size() > 0; }, spec));

  const auto depth = depth_.first();
  ASSERT_EQ(depth.width, static_cast<uint32_t>(W / 2));
  ASSERT_EQ(depth.height, static_cast<uint32_t>(H / 2));
  ASSERT_EQ(depth.data.size(), static_cast<size_t>(W / 2 * H / 2 * 2));
  const auto * px = reinterpret_cast<const uint16_t *>(depth.data.data());
  int fallbacks = 0;
  for (int v = 0; v < H / 2; ++v) {
    for (int u = 0; u < W / 2; ++u) {
      uint16_t want = 0;
      for (const auto & [dy, dx] : BLOCK_ORDER) {
        const uint16_t mm = scene_mm(2 * u + dx, 2 * v + dy);
        if (mm >= 150 && mm <= 4000) {
          want = mm;
          break;
        }
      }
      fallbacks += want != scene_mm(2 * u, 2 * v) ? 1 : 0;
      ASSERT_EQ(px[v * (W / 2) + u], want) << u << "," << v;
    }
  }
  EXPECT_GT(fallbacks, 0);

  const auto & stamp = depth.header.stamp;
  ASSERT_TRUE(wait_for([&] { return find_stamp(depth_info_, stamp) && find_stamp(rgb_, stamp); }));
  const auto info = *find_stamp(depth_info_, stamp);
  EXPECT_EQ(info.width, static_cast<uint32_t>(W / 2));
  EXPECT_EQ(info.height, static_cast<uint32_t>(H / 2));
  EXPECT_DOUBLE_EQ(info.k[0], FX / 2);
  EXPECT_DOUBLE_EQ(info.k[2], CX / 2);
  EXPECT_DOUBLE_EQ(info.k[4], FY / 2);
  EXPECT_DOUBLE_EQ(info.k[5], CY / 2);
  EXPECT_DOUBLE_EQ(info.k[8], 1.0);
  EXPECT_DOUBLE_EQ(info.p[0], FX / 2);
  EXPECT_DOUBLE_EQ(info.p[2], CX / 2);
  EXPECT_DOUBLE_EQ(info.p[5], FY / 2);
  EXPECT_DOUBLE_EQ(info.p[6], CY / 2);
  EXPECT_DOUBLE_EQ(info.p[10], 1.0);
  // The colour stays at full size.
  const cv::Mat decoded = cv::imdecode((*find_stamp(rgb_, stamp)).data, cv::IMREAD_COLOR);
  EXPECT_EQ(decoded.cols, W);
  EXPECT_EQ(decoded.rows, H);
}

TEST_F(NodeTest, DropsNon16uc1Depth)
{
  start(base_config());
  SetSpec spec;
  spec.depth_encoding = "32FC1";
  expect_drop("bad_input", spec);
}

TEST_F(NodeTest, DropsAMismatchedSize)
{
  start(base_config());
  SetSpec spec;
  spec.depth_w = W / 2;
  spec.depth_h = H / 2;
  expect_drop("size_mismatch", spec);
}

TEST_F(NodeTest, DropsWithoutCameraInfo)
{
  start(base_config());
  SetSpec spec;
  spec.with_info = false;
  expect_drop("no_camera_info", spec);
  // A camera_info with a zero camera matrix is no camera_info either.
  const double before = dropped("no_camera_info");
  spec.with_info = true;
  spec.fx = 0.0;
  ASSERT_TRUE(pump_until([&] { return dropped("no_camera_info") >= before + 3.0; }, spec));
  expect_nothing_published();
}

TEST_F(NodeTest, DropsFramesWithoutAnExactPartner)
{
  start(base_config());
  SetSpec spec;
  spec.depth_offset_s = 0.001;
  expect_drop("unpaired", spec);
}

TEST_F(NodeTest, CloudIsPublishedAtMostAtTheConfiguredRate)
{
  Config c = base_config();
  c.cloud.rate_hz = 1.0;
  start(c);
  subscribe_outputs();
  subscribe_cloud();
  ASSERT_TRUE(wait_inputs_subscribed());
  ASSERT_TRUE(wait_for([&] { return cloud_sub_->get_publisher_count() > 0; }));
  // 2 s of stream stamps (30 sets); at 1 Hz at most 3 clouds.
  const int first = index_;
  ASSERT_TRUE(pump_until([&] { return index_ - first >= 30 && published() >= 10.0; }));
  std::this_thread::sleep_for(milliseconds(500));
  EXPECT_GE(cloud_.size(), 1u);
  const double span_s = (index_ - first) * STREAM_PERIOD_S;
  EXPECT_LE(cloud_.size(), static_cast<size_t>(std::ceil(span_s * c.cloud.rate_hz)) + 1u);
  const auto cloud = cloud_.first();
  EXPECT_EQ(cloud.header.frame_id, COLOR_FRAME);
}

// --- pairing ---------------------------------------------------------------------------------------------------------

TEST_F(NodeTest, ApproximatePairingPublishesSetsOfAShiftedStream)
{
  Config c = base_config();
  c.pairing.policy = head_rgbd_cpp::PairingPolicy::APPROXIMATE;
  c.pairing.max_interval_s = 0.02;
  start(c);
  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());
  SetSpec spec;
  spec.depth_offset_s = 0.003;
  ASSERT_TRUE(pump_until([&] { return published() >= 3.0; }, spec));
  // The set carries the colour header.
  ASSERT_TRUE(wait_for([&] { return depth_.size() > 0; }));
  const double t = seconds_of(depth_.first().header.stamp);
  const double index = (t - set_time(0)) / STREAM_PERIOD_S;
  EXPECT_NEAR(index, std::round(index), 1e-3);
  EXPECT_EQ(depth_.first().header.frame_id, COLOR_FRAME);
  EXPECT_GE(metric("head_rgbd_pair_stamp_delta_seconds_count"), 3.0);
}

TEST_F(NodeTest, ExactPairingCountsEachUnpairedFrameOnce)
{
  start(base_config());
  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());
  SetSpec spec;
  spec.depth_offset_s = 0.001;
  const auto frames = [&] {
    return metric("head_rgbd_frames_total{stream=\"depth\"}") + metric("head_rgbd_frames_total{stream=\"colour\"}");
  };
  ASSERT_TRUE(pump_until([&] { return frames() >= 40.0; }, spec));
  std::this_thread::sleep_for(milliseconds(300));
  const double received = frames();
  const double unpaired = dropped("unpaired");
  EXPECT_EQ(metric("head_rgbd_pairs_total"), 0.0);
  EXPECT_LE(unpaired, received);
  // At most the newest PAIR_QUEUE_SIZE frames of each stream are still waiting for a partner.
  EXPECT_GE(unpaired, received - 2.0 * head_rgbd_cpp::PAIR_QUEUE_SIZE);
  // The near miss is in the histogram: every unpaired colour frame had a depth frame 1 ms later.
  EXPECT_GT(metric("head_rgbd_pair_stamp_delta_seconds_count"), 0.0);
  EXPECT_EQ(
    metric("head_rgbd_pair_stamp_delta_seconds_bucket{le=\"-0.0005\"}"),
    metric("head_rgbd_pair_stamp_delta_seconds_count"));
}

TEST_F(NodeTest, ExactPairsFillTheZeroBucketOfTheStampDeltaHistogram)
{
  start(base_config());
  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());
  ASSERT_TRUE(pump_until([&] { return metric("head_rgbd_pairs_total") >= 5.0; }));
  const double pairs = metric("head_rgbd_pairs_total");
  EXPECT_GE(metric("head_rgbd_pair_stamp_delta_seconds_count"), pairs);
  EXPECT_EQ(metric("head_rgbd_pair_stamp_delta_seconds_bucket{le=\"-0.0005\"}"), 0.0);
  EXPECT_GE(metric("head_rgbd_pair_stamp_delta_seconds_bucket{le=\"0.0005\"}"), pairs);
}

// --- edge mask on the aligned depth ----------------------------------------------------------------------------------

TEST_F(NodeTest, AlignedDepthIsMaskedBeforeTheDownscale)
{
  Config c = base_config();
  c.edge_mask.enabled = true;
  c.output.divisor = 1;
  start(c);
  subscribe_outputs();
  ASSERT_TRUE(wait_inputs_subscribed());
  SetSpec spec;
  spec.depth_mm = step_mm;
  ASSERT_TRUE(pump_until([&] { return depth_.size() > 0; }, spec));
  const auto depth = depth_.first();
  const auto * px = reinterpret_cast<const uint16_t *>(depth.data.data());
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const uint16_t want = x == STEP_COLUMN ? 0 : step_mm(x, y);
      ASSERT_EQ(px[y * W + x], want) << x << "," << y;
    }
  }
  EXPECT_GT(metric("head_rgbd_edge_mask_removed_ratio{stage=\"aligned\"}"), 0.0);
}

// --- full-rate filtered depth ----------------------------------------------------------------------------------------

TEST_F(NodeTest, FilteredDepthInputsFollowItsOwnSubscribers)
{
  start(base_config());
  std::this_thread::sleep_for(milliseconds(2500));
  EXPECT_EQ(raw_depth_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(raw_info_pub_->get_subscription_count(), 0u);

  subscribe_filtered();
  ASSERT_TRUE(wait_filtered_inputs_subscribed());
  // The /head_rgbd set inputs stay released: nobody reads the sets.
  EXPECT_EQ(depth_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(color_pub_->get_subscription_count(), 0u);

  filtered_sub_.reset();
  filtered_info_sub_.reset();
  EXPECT_TRUE(wait_for(
    [this] { return raw_depth_pub_->get_subscription_count() == 0 && raw_info_pub_->get_subscription_count() == 0; }));
}

TEST_F(NodeTest, FilteredDepthKeepsTheHeaderAndRemovesTheVeil)
{
  Config c = base_config();
  c.edge_mask.enabled = true;
  start(c);
  subscribe_filtered();
  ASSERT_TRUE(wait_filtered_inputs_subscribed());
  SetSpec spec;
  spec.depth_mm = step_mm;
  ASSERT_TRUE(pump_raw_until([&] { return filtered_.size() >= 3 && filtered_info_.size() >= 3; }, spec));

  const auto image = filtered_.first();
  EXPECT_EQ(image.header.frame_id, RAW_DEPTH_FRAME);
  const double t = seconds_of(image.header.stamp);
  const double index = (t - set_time(0)) / STREAM_PERIOD_S;
  EXPECT_NEAR(index, std::round(index), 1e-3);
  EXPECT_EQ(image.encoding, "16UC1");
  EXPECT_FALSE(image.is_bigendian);
  ASSERT_EQ(image.width, static_cast<uint32_t>(W));
  ASSERT_EQ(image.height, static_cast<uint32_t>(H));
  ASSERT_EQ(image.step, static_cast<uint32_t>(W * 2));
  const auto * px = reinterpret_cast<const uint16_t *>(image.data.data());
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const uint16_t want = x == STEP_COLUMN ? 0 : step_mm(x, y);
      ASSERT_EQ(px[y * W + x], want) << x << "," << y;
    }
  }

  ASSERT_TRUE(wait_for([&] { return find_stamp(filtered_info_, image.header.stamp).has_value(); }));
  const auto info = *find_stamp(filtered_info_, image.header.stamp);
  EXPECT_EQ(info.header, image.header);
  EXPECT_EQ(info.k, camera_info(t, FX).k);
  EXPECT_EQ(info.width, static_cast<uint32_t>(W));
  // One output per input frame: never more images than frames sent.
  EXPECT_LE(filtered_published(), static_cast<double>(raw_index_));
  EXPECT_GT(metric("head_rgbd_edge_mask_removed_ratio{stage=\"filtered\"}"), 0.0);
  EXPECT_EQ(published(), 0.0);
}

TEST_F(NodeTest, FilteredDepthPassesThroughWithTheMaskDisabled)
{
  start(base_config());
  subscribe_filtered();
  ASSERT_TRUE(wait_filtered_inputs_subscribed());
  SetSpec spec;
  spec.depth_mm = step_mm;
  ASSERT_TRUE(pump_raw_until([&] { return filtered_.size() >= 1; }, spec));
  const auto image = filtered_.first();
  EXPECT_EQ(image.data, depth_image(seconds_of(image.header.stamp), spec).data);
}

TEST_F(NodeTest, FilteredDepthNeedsACameraInfo)
{
  start(base_config());
  subscribe_filtered();
  ASSERT_TRUE(wait_filtered_inputs_subscribed());
  SetSpec spec;
  spec.with_info = false;
  ASSERT_TRUE(pump_raw_until([&] { return filtered_dropped("no_camera_info") >= 3.0; }, spec));
  std::this_thread::sleep_for(milliseconds(300));
  EXPECT_EQ(filtered_.size(), 0u);
  EXPECT_EQ(filtered_info_.size(), 0u);
  EXPECT_EQ(filtered_published(), 0.0);
}

TEST_F(NodeTest, FilteredDepthDropsNon16uc1Depth)
{
  start(base_config());
  subscribe_filtered();
  ASSERT_TRUE(wait_filtered_inputs_subscribed());
  SetSpec spec;
  spec.depth_encoding = "32FC1";
  ASSERT_TRUE(pump_raw_until([&] { return filtered_dropped("bad_input") >= 3.0; }, spec));
  std::this_thread::sleep_for(milliseconds(300));
  EXPECT_EQ(filtered_.size(), 0u);
  EXPECT_EQ(filtered_published(), 0.0);
}

TEST_F(NodeTest, DisabledFilteredDepthAdvertisesNothing)
{
  Config c = base_config();
  c.filtered_depth.enabled = false;
  start(c);
  ASSERT_TRUE(wait_for([&] { return !probe_->get_publishers_info_by_topic(c.depth_image()).empty(); }));
  EXPECT_EQ(probe_->get_publishers_info_by_topic(FILTERED_IMAGE_TOPIC).size(), 0u);
  EXPECT_EQ(probe_->get_publishers_info_by_topic(FILTERED_INFO_TOPIC).size(), 0u);
}
