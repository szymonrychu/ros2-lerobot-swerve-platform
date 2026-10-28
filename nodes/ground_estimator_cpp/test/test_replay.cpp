// The 2026-10-22 live recordings (test/data/replay/{home,box,ramp}.bin, scripts/ground_replay_export.py) replayed
// through GroundNode in-process with the shipped config/ground_estimator.yaml (design D14): reader tests, the replay
// metric functions of replay_fixture.hpp on hand-built frames, and one replay per scene that prints its metrics line
// and asserts the live-round acceptance.
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <utility>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/ground_node.hpp"
#include "replay_fixture.hpp"

namespace
{

using ground_estimator_cpp::Config;
using ground_estimator_cpp::GroundNode;
using ground_replay::ReplayFrame;
using ground_replay::ReplayScene;
using nlohmann::json;
using std::chrono::milliseconds;
using std::chrono::seconds;

constexpr const char * MAP = "map";
constexpr const char * ODOM = "odom";
constexpr const char * BASE = "base_link";
constexpr const char * TOF_FRAME = "tof_optical_frame";
constexpr const char * IMU_FRAME = "imu_link";  // frame of the recorded IMU samples (brief ground-live.md)
constexpr const char * SCENES[] = {"home", "box", "ramp"};
constexpr double MAX_STRAY_LETHAL_SHARE = 0.05;  // frames with a lethal point within 1.2 m outside the box (p95)
constexpr double MAX_CHURN_SHARE = 0.10;         // consecutive frames whose warning set changes
constexpr int64_t NS_PER_S = 1000000000LL;
// The fixtures hold 15 s of a 10 Hz ToF stream.
constexpr std::size_t MIN_CLOUDS = 140, MAX_CLOUDS = 160;
constexpr double MAX_SPAN_S = 15.0;
constexpr double MM_PER_M = 1000.0;
// The first restamped cloud lies this far after the wall clock at the replay start.
constexpr int64_t LEAD_NS = 200000000LL;
// A cloud the node processed without a plane publishes nothing: after its fit is counted, the obstacle cloud of its
// stamp gets this long to arrive before the frame is taken as unanswered.
constexpr milliseconds NO_PLANE_GRACE{500};
constexpr seconds WAIT{15};
constexpr milliseconds POLL{5};
// Hand-built frames: one warm-up frame at 0 s, the frame under test after it.
constexpr double TEST_FRAME_S = 2.5;
constexpr double HALF_CELL_M = 0.05;
// The derived XYZRGB cloud goes out at most this often (head_rgbd_cpp's cloud rate is lower than the pairs').
constexpr int64_t RGBD_MIN_PERIOD_NS = NS_PER_S;
constexpr int HALF = 2;  // the derived cloud uses the depth and the colour at half the resolution

// The guidance topics' override of the replay.
enum class Guidance
{
  off,
  on
};

std::atomic<int> g_test_index{0};

std::string replay_path(const std::string & scene)
{
  return std::string(GROUND_TEST_DATA_DIR) + "/replay/" + scene + ".bin";
}

std::string file_bytes(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

builtin_interfaces::msg::Time stamp_of_ns(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / NS_PER_S);
  t.nanosec = static_cast<uint32_t>(ns % NS_PER_S);
  return t;
}

int64_t ns_of(const builtin_interfaces::msg::Time & t) { return static_cast<int64_t>(t.sec) * NS_PER_S + t.nanosec; }

int64_t wall_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
    .count();
}

bool wait_for(const std::function<bool()> & done, milliseconds timeout = WAIT)
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

// The sum of every series of a Prometheus text rendering whose line starts with prefix.
double series_sum(const std::string & text, const std::string & prefix)
{
  double sum = 0.0;
  std::istringstream lines(text);
  for (std::string line; std::getline(lines, line);) {
    if (line.rfind(prefix, 0) == 0) {
      sum += std::stod(line.substr(line.rfind(' ') + 1));
    }
  }
  return sum;
}

// Messages keyed by stamp (ns), so a frame's outputs are found without scanning the whole run.
template <typename Msg>
struct StampedInbox
{
  std::mutex mutex;
  std::map<int64_t, Msg> by_stamp;
  void add(int64_t stamp_ns, const Msg & m)
  {
    std::lock_guard<std::mutex> lock(mutex);
    by_stamp[stamp_ns] = m;
  }
  bool has(int64_t stamp_ns)
  {
    std::lock_guard<std::mutex> lock(mutex);
    return by_stamp.count(stamp_ns) > 0;
  }
  std::size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return by_stamp.size();
  }
  std::optional<Msg> get(int64_t stamp_ns)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto it = by_stamp.find(stamp_ns);
    return it == by_stamp.end() ? std::nullopt : std::optional<Msg>(it->second);
  }
};

std::vector<Eigen::Vector3f> xyz_of(const sensor_msgs::msg::PointCloud2 & msg)
{
  std::vector<Eigen::Vector3f> out;
  if (msg.width * msg.height == 0) {
    return out;
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    out.emplace_back(*x, *y, *z);
  }
  return out;
}

geometry_msgs::msg::TransformStamped transform_of(
  const Eigen::Isometry3d & parent_T_child, const std::string & parent, const std::string & child, int64_t stamp_ns)
{
  geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(parent_T_child);
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.header.stamp = stamp_of_ns(stamp_ns);
  return tf;
}

// The XYZRGB cloud head_rgbd_cpp would build from a pair, at half the resolution (nodes/head_rgbd_cpp/src/core/cloud.cpp
// build_cloud): the depth halved by the minimum valid depth of each 2x2 block (0 when none), the colour decoded at half
// size (IMREAD_REDUCED_COLOR_2), deprojected with the halved pinhole model of the RGB camera, the colour packed as
// float rgb (colour.cpp to_labelled_msg), invalid depths left out.
// @param pair the RGB-D pair (JPEG bytes, depth in mm at the full RGB size).
// @param rgb_model the RGB camera model.
// @param stamp_ns the restamped header stamp.
// @return the cloud, frame id of the RGB camera.
sensor_msgs::msg::PointCloud2 derived_rgbd_cloud(
  const ground_replay::ReplayPair & pair, const ground_replay::CameraModel & rgb_model, int64_t stamp_ns)
{
  const cv::Mat colour = cv::imdecode(pair.jpeg, cv::IMREAD_REDUCED_COLOR_2);
  const int rows = (pair.depth_mm.rows + HALF - 1) / HALF, cols = (pair.depth_mm.cols + HALF - 1) / HALF;
  const double fx = rgb_model.K[0] / HALF, fy = rgb_model.K[4] / HALF, cx = rgb_model.K[2] / HALF,
               cy = rgb_model.K[5] / HALF;
  struct Point
  {
    float x, y, z;
    uint32_t rgb;
  };
  std::vector<Point> points;
  for (int v = 0; v < rows && v < colour.rows; ++v) {
    for (int u = 0; u < cols && u < colour.cols; ++u) {
      uint16_t depth = 0;
      for (int dv = 0; dv < HALF; ++dv) {
        for (int du = 0; du < HALF; ++du) {
          const int r = v * HALF + dv, c = u * HALF + du;
          const uint16_t d = r < pair.depth_mm.rows && c < pair.depth_mm.cols ? pair.depth_mm.at<uint16_t>(r, c) : 0;
          if (d > 0 && (depth == 0 || d < depth)) {
            depth = d;
          }
        }
      }
      if (depth == 0) {
        continue;
      }
      const double z = depth / MM_PER_M;
      const cv::Vec3b bgr = colour.at<cv::Vec3b>(v, u);
      points.push_back(Point{
        static_cast<float>((u - cx) / fx * z), static_cast<float>((v - cy) / fy * z), static_cast<float>(z),
        (static_cast<uint32_t>(bgr[2]) << 16) | (static_cast<uint32_t>(bgr[1]) << 8) | bgr[0]});
    }
  }
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = rgb_model.frame;
  msg.header.stamp = stamp_of_ns(stamp_ns);
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
    sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(&msg.data[i * msg.point_step], &points[i], sizeof(Point));
  }
  msg.is_dense = true;
  return msg;
}

// @return the camera_info of a model under the given header.
sensor_msgs::msg::CameraInfo camera_info_of(
  const ground_replay::CameraModel & model, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::CameraInfo info;
  info.header = header;
  info.width = model.width;
  info.height = model.height;
  info.distortion_model = "plumb_bob";
  info.d.assign(model.D.begin(), model.D.end());
  std::copy(model.K.begin(), model.K.end(), info.k.begin());
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {model.K[0], 0, model.K[2], 0, 0, model.K[4], model.K[5], 0, 0, 0, 1, 0};
  return info;
}

// --- hand-built frames for the metric tests ----------------------------------------------------------------------

ReplayFrame frame_at(double t_s, bool has_plane = true)
{
  ReplayFrame f;
  f.stamp_ns = static_cast<int64_t>(std::llround(t_s * NS_PER_S));
  f.has_plane = has_plane;
  f.warnings = json{{"stamp_ns", f.stamp_ns}, {"frame", MAP}, {"areas", json::array()}};
  return f;
}

// A square warning area of one cell centred at (cx, cy).
json area(const std::string & kind, double cx, double cy)
{
  const double h = HALF_CELL_M;
  return json{
    {"kind", kind},
    {"polygon", json::array({{cx - h, cy - h}, {cx + h, cy - h}, {cx + h, cy + h}, {cx - h, cy + h}})},
    {"height_m", 0.03},
    {"slope_deg", 5.0},
    {"source", "depth"}};
}

// A warm-up frame and one frame after it carrying an obstacle (or drop) point at (x, y).
std::vector<ReplayFrame> one_lethal_point(double x, double y, bool drop = false)
{
  std::vector<ReplayFrame> frames{frame_at(0.0), frame_at(TEST_FRAME_S)};
  (drop ? frames[1].drop : frames[1].obstacle).emplace_back(static_cast<float>(x), static_cast<float>(y), 0.05f);
  return frames;
}

// A warm-up frame and one frame after it carrying one warning area.
std::vector<ReplayFrame> one_area(const std::string & kind, double cx, double cy)
{
  std::vector<ReplayFrame> frames{frame_at(0.0), frame_at(TEST_FRAME_S)};
  frames[1].warnings["areas"].push_back(area(kind, cx, cy));
  return frames;
}

std::string tiny_truth_path() { return std::string(GROUND_TEST_DATA_DIR) + "/replay/tiny_truth.yaml"; }

ground_replay::SceneTruth tiny_truth() { return ground_replay::load_truth(tiny_truth_path()); }

// Writes node to a file of the test temp dir.
// @param node the YAML document.
// @param tag part of the file name.
// @return the path.
std::string write_yaml(const YAML::Node & node, const std::string & tag)
{
  const std::string path = ::testing::TempDir() + "truth_" + tag + ".yaml";
  std::ofstream out(path);
  out << node;
  return path;
}

// A warm-up frame and one frame after it carrying obstacle (or drop) points at the given (x, y).
std::vector<ReplayFrame> lethal_points(const std::vector<std::pair<double, double>> & xy, bool drop = false)
{
  std::vector<ReplayFrame> frames{frame_at(0.0), frame_at(TEST_FRAME_S)};
  for (const auto & p : xy) {
    (drop ? frames[1].drop : frames[1].obstacle)
      .emplace_back(static_cast<float>(p.first), static_cast<float>(p.second), 0.05f);
  }
  return frames;
}

// --- the replay through GroundNode -------------------------------------------------------------------------------

class ReplayTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  // The shipped config, validated, with only the input topics, the output namespace and the metrics port changed
  // (unique per test so no discovery or latched message crosses tests), and guidance.enabled set by `guidance`.
  Config guidance_config(Guidance guidance)
  {
    const YAML::Node node = YAML::LoadFile(std::string(GROUND_CONFIG_DIR) + "/ground_estimator.yaml");
    const auto errors = ground_estimator_cpp::validate(node);
    EXPECT_TRUE(errors.empty()) << (errors.empty() ? "" : errors.front());
    Config c = ground_estimator_cpp::from_yaml(node);
    const std::string prefix = "/rp" + std::to_string(g_test_index++);
    c.cloud_topic = prefix + c.cloud_topic;
    c.imu_topic = prefix + "/imu/data";
    c.rgbd_cloud_topic = prefix + "/head_rgbd/points";
    c.output_namespace = prefix + "/ground";
    c.guidance.depth_topic = prefix + c.guidance.depth_topic;
    c.guidance.info_topic = prefix + c.guidance.info_topic;
    c.guidance.rgb_topic = prefix + c.guidance.rgb_topic;
    c.guidance.depth_info_topic = prefix + c.guidance.depth_info_topic;
    c.guidance.enabled = guidance == Guidance::on;
    c.metrics_port = std::nullopt;
    return c;
  }

  void start(const Config & config)
  {
    config_ = config;
    buffer_ = std::make_shared<tf2_ros::Buffer>(
      std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), tf2::Duration(std::chrono::seconds(300)));
    node_ = std::make_shared<GroundNode>(config, buffer_);
    probe_ = std::make_shared<rclcpp::Node>("replay_probe_" + std::to_string(g_test_index.load()));
    cloud_pub_ =
      probe_->create_publisher<sensor_msgs::msg::PointCloud2>(config.cloud_topic, rclcpp::QoS(10).reliable());
    imu_pub_ = probe_->create_publisher<sensor_msgs::msg::Imu>(config.imu_topic, rclcpp::SensorDataQoS());
    const auto & g = config.guidance;
    depth_pub_ = probe_->create_publisher<sensor_msgs::msg::Image>(g.depth_topic, rclcpp::QoS(10).reliable());
    info_pub_ = probe_->create_publisher<sensor_msgs::msg::CameraInfo>(g.info_topic, rclcpp::QoS(10).reliable());
    rgb_pub_ = probe_->create_publisher<sensor_msgs::msg::CompressedImage>(g.rgb_topic, rclcpp::QoS(10).reliable());
    tof_info_pub_ =
      probe_->create_publisher<sensor_msgs::msg::CameraInfo>(g.depth_info_topic, rclcpp::QoS(10).reliable());
    rgbd_pub_ =
      probe_->create_publisher<sensor_msgs::msg::PointCloud2>(config.rgbd_cloud_topic, rclcpp::QoS(1).best_effort());
    // What the harness sent on those topics, for the tests of the harness itself.
    depth_sub_ = probe_->create_subscription<sensor_msgs::msg::Image>(
      g.depth_topic, rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) { depth_in_.add(ns_of(m->header.stamp), *m); });
    info_sub_ = probe_->create_subscription<sensor_msgs::msg::CameraInfo>(
      g.info_topic, rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { info_in_.add(ns_of(m->header.stamp), *m); });
    rgb_sub_ = probe_->create_subscription<sensor_msgs::msg::CompressedImage>(
      g.rgb_topic, rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) { rgb_in_.add(ns_of(m->header.stamp), *m); });
    tof_info_sub_ = probe_->create_subscription<sensor_msgs::msg::CameraInfo>(
      g.depth_info_topic, rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { tof_info_in_.add(ns_of(m->header.stamp), *m); });
    rgbd_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.rgbd_cloud_topic, rclcpp::QoS(1).best_effort(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { rgbd_in_.add(ns_of(m->header.stamp), *m); });
    const auto reliable = rclcpp::QoS(50).reliable();
    obstacle_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.obstacle_points(), reliable,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { obstacle_.add(ns_of(m->header.stamp), *m); });
    drop_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.drop_points(), reliable,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { drop_.add(ns_of(m->header.stamp), *m); });
    warnings_sub_ = probe_->create_subscription<std_msgs::msg::String>(
      config.warning_areas(), reliable, [this](std_msgs::msg::String::ConstSharedPtr m) {
        const json doc = json::parse(m->data);
        warnings_.add(doc["stamp_ns"].get<int64_t>(), doc);
      });
    executor_.add_node(node_);
    executor_.add_node(probe_);
    spinner_ = std::thread([this] { executor_.spin(); });
    ASSERT_TRUE(wait_for([this] {
      return cloud_pub_->get_subscription_count() > 0 && imu_pub_->get_subscription_count() > 0 &&
             obstacle_sub_->get_publisher_count() > 0 && drop_sub_->get_publisher_count() > 0 &&
             warnings_sub_->get_publisher_count() > 0 && depth_pub_->get_subscription_count() > 0 &&
             info_pub_->get_subscription_count() > 0 && rgb_pub_->get_subscription_count() > 0 &&
             tof_info_pub_->get_subscription_count() > 0 && rgbd_pub_->get_subscription_count() > 0;
    }));
  }

  // Clouds the node has finished reading (fitted or rejected) or dropped before the fit.
  double processed_clouds()
  {
    const std::string text = node_->metrics().render();
    return series_sum(text, std::string(ground_estimator_cpp::METRIC_FITS) + "{") +
           series_sum(text, std::string(ground_estimator_cpp::METRIC_DROPPED) + "{reason=\"tf\"}") +
           series_sum(text, std::string(ground_estimator_cpp::METRIC_DROPPED) + "{reason=\"bad_input\"}");
  }

  // The static transforms (map <- odom identity, base_link <- imu_link) and per cloud stamp base_link <- cloud frame
  // and odom <- base_link, restamped by offset_ns.
  void fill_tf(const ReplayScene & scene, int64_t offset_ns)
  {
    buffer_->setTransform(transform_of(Eigen::Isometry3d::Identity(), MAP, ODOM, 0), "replay", true);
    buffer_->setTransform(transform_of(scene.base_T_imu, BASE, IMU_FRAME, 0), "replay", true);
    for (const auto & cloud : scene.clouds) {
      const int64_t t = cloud.stamp_ns + offset_ns;
      buffer_->setTransform(transform_of(cloud.base_T_cloud, BASE, scene.frame_id, t), "replay", false);
      buffer_->setTransform(transform_of(cloud.odom_T_base, ODOM, BASE, t), "replay", false);
    }
    for (const auto & pair : scene.pairs) {
      const int64_t t = pair.stamp_ns + offset_ns;
      buffer_->setTransform(transform_of(pair.base_T_rgb, BASE, scene.rgb_model->frame, t), "replay", false);
      buffer_->setTransform(transform_of(pair.odom_T_base, ODOM, BASE, t), "replay", false);
    }
  }

  // Publish one RGB-D pair at its restamped time: the upsampled depth (16UC1), the RGB camera_info and the JPEG with
  // one header, and the derived XYZRGB cloud when RGBD_MIN_PERIOD_NS has passed since the last one.
  void publish_pair(const ReplayScene & scene, const ground_replay::ReplayPair & pair, int64_t stamp_ns)
  {
    std_msgs::msg::Header header;
    header.frame_id = scene.rgb_model->frame;
    header.stamp = stamp_of_ns(stamp_ns);
    sensor_msgs::msg::Image depth;
    depth.header = header;
    depth.width = static_cast<uint32_t>(pair.depth_mm.cols);
    depth.height = static_cast<uint32_t>(pair.depth_mm.rows);
    depth.encoding = "16UC1";
    depth.is_bigendian = false;
    depth.step = depth.width * sizeof(uint16_t);
    depth.data.resize(static_cast<std::size_t>(depth.step) * depth.height);
    const cv::Mat contiguous = pair.depth_mm.isContinuous() ? pair.depth_mm : pair.depth_mm.clone();
    std::memcpy(depth.data.data(), contiguous.data, depth.data.size());
    sensor_msgs::msg::CompressedImage rgb;
    rgb.header = header;
    rgb.format = "jpeg";
    rgb.data = pair.jpeg;
    depth_pub_->publish(depth);
    info_pub_->publish(camera_info_of(*scene.rgb_model, header));
    rgb_pub_->publish(rgb);
    if (last_rgbd_ns_ == 0 || stamp_ns - last_rgbd_ns_ >= RGBD_MIN_PERIOD_NS) {
      rgbd_pub_->publish(derived_rgbd_cloud(pair, *scene.rgb_model, stamp_ns));
      last_rgbd_ns_ = stamp_ns;
    }
  }

  void publish_imu(const ground_replay::ReplayImu & s, int64_t stamp_ns)
  {
    sensor_msgs::msg::Imu m;
    m.header.frame_id = IMU_FRAME;
    m.header.stamp = stamp_of_ns(stamp_ns);
    m.orientation.x = s.orientation.x();
    m.orientation.y = s.orientation.y();
    m.orientation.z = s.orientation.z();
    m.orientation.w = s.orientation.w();
    m.linear_acceleration.x = s.linear_acceleration.x();
    m.linear_acceleration.y = s.linear_acceleration.y();
    m.linear_acceleration.z = s.linear_acceleration.z();
    m.angular_velocity.x = s.angular_velocity.x();
    m.angular_velocity.y = s.angular_velocity.y();
    m.angular_velocity.z = s.angular_velocity.z();
    imu_pub_->publish(m);
  }

  void publish_cloud(const ground_replay::ReplayCloud & cloud, const std::string & frame_id, int64_t stamp_ns)
  {
    pcl::PointCloud<pcl::PointXYZ> points;
    for (const auto & p : cloud.points_mm) {
      points.push_back(pcl::PointXYZ(
        static_cast<float>(p[0] / MM_PER_M), static_cast<float>(p[1] / MM_PER_M), static_cast<float>(p[2] / MM_PER_M)));
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(points, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp_of_ns(stamp_ns);
    cloud_pub_->publish(msg);
  }

  // Publish one cloud and collect what the node published for its stamp (pattern of test_node.cpp run_frame): the
  // obstacle cloud of the stamp means the node answered with a plane, then its drop cloud and warning areas follow.
  ReplayFrame run_frame(const ground_replay::ReplayCloud & cloud, const std::string & frame_id, int64_t stamp_ns)
  {
    ReplayFrame f;
    f.stamp_ns = stamp_ns;
    f.base_T_warnings = cloud.odom_T_base.inverse();  // map <- odom is the identity
    const double before = processed_clouds();
    publish_cloud(cloud, frame_id, stamp_ns);
    EXPECT_TRUE(wait_for([&] { return obstacle_.has(stamp_ns) || processed_clouds() > before; }));
    if (!obstacle_.has(stamp_ns) && !wait_for([&] { return obstacle_.has(stamp_ns); }, NO_PLANE_GRACE)) {
      return f;
    }
    f.has_plane = true;
    EXPECT_TRUE(wait_for([&] { return drop_.has(stamp_ns) && warnings_.has(stamp_ns); }));
    f.obstacle = xyz_of(*obstacle_.get(stamp_ns));
    const auto drop = drop_.get(stamp_ns);
    f.drop = drop ? xyz_of(*drop) : std::vector<Eigen::Vector3f>();
    f.warnings = warnings_.get(stamp_ns).value_or(json());
    f.areas = ground_replay::parse_areas(f.warnings);
    return f;
  }

  // Replay a scene in real time: restamped to the wall clock keeping its relative timing, each cloud published at its
  // stamp after the IMU samples up to it, and its outputs awaited before the next cloud.
  std::vector<ReplayFrame> replay(const ReplayScene & scene, Guidance guidance)
  {
    start(guidance_config(guidance));
    std::vector<ReplayFrame> frames;
    if (scene.clouds.empty()) {
      return frames;
    }
    offset_ns_ = wall_now_ns() + LEAD_NS - scene.clouds.front().stamp_ns;
    const int64_t offset_ns = offset_ns_;
    fill_tf(scene, offset_ns);
    std::size_t next_imu = 0, next_pair = 0;
    const auto sleep_until = [](int64_t stamp_ns) {
      const int64_t wait_ns = stamp_ns - wall_now_ns();
      if (wait_ns > 0) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(wait_ns));
      }
    };
    // Pairs go out at their stamp, after the clouds stamped before them.
    const auto publish_pairs_until = [&](int64_t until_ns) {
      for (; next_pair < scene.pairs.size() && scene.pairs[next_pair].stamp_ns + offset_ns <= until_ns; ++next_pair) {
        sleep_until(scene.pairs[next_pair].stamp_ns + offset_ns);
        publish_pair(scene, scene.pairs[next_pair], scene.pairs[next_pair].stamp_ns + offset_ns);
      }
    };
    for (const auto & cloud : scene.clouds) {
      const int64_t stamp_ns = cloud.stamp_ns + offset_ns;
      publish_pairs_until(stamp_ns - 1);
      sleep_until(stamp_ns);
      if (scene.tof_model) {
        std_msgs::msg::Header header;
        header.frame_id = scene.tof_model->frame;
        header.stamp = stamp_of_ns(stamp_ns);
        tof_info_pub_->publish(camera_info_of(*scene.tof_model, header));
      }
      for (; next_imu < scene.imu.size() && scene.imu[next_imu].stamp_ns + offset_ns <= stamp_ns; ++next_imu) {
        publish_imu(scene.imu[next_imu], scene.imu[next_imu].stamp_ns + offset_ns);
      }
      frames.push_back(run_frame(cloud, scene.frame_id, stamp_ns));
      publish_pairs_until(stamp_ns);  // a pair stamped at the cloud's own stamp follows it
    }
    publish_pairs_until(std::numeric_limits<int64_t>::max());
    return frames;
  }

  // Replay a replay binary (v1 or v2) through a fresh GroundNode of the shipped config with guidance.enabled set.
  // @param path the .bin file.
  // @param guidance the value of guidance.enabled.
  // @return what the node published per cloud.
  std::vector<ReplayFrame> run_scene(const std::string & path, Guidance guidance)
  {
    return replay(ground_replay::read_replay(path), guidance);
  }

  // Replay one committed scene, check the node answered with a plane and print its metrics line.
  std::vector<ReplayFrame> replay_scene(const std::string & name)
  {
    const auto frames = run_scene(replay_path(name), Guidance::on);
    EXPECT_TRUE(std::any_of(frames.begin(), frames.end(), [](const ReplayFrame & f) { return f.has_plane; }))
      << name << ": the node never answered with a plane";
    std::cout << ground_replay::metrics_line(name, frames) << std::endl;
    return frames;
  }

  Config config_;
  int64_t offset_ns_ = 0;     // restamp offset of the running replay: wall clock minus the recorded stamps
  int64_t last_rgbd_ns_ = 0;  // stamp of the last derived XYZRGB cloud sent (0: none yet)
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<GroundNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_, drop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr warnings_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_, tof_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgbd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_, tof_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rgbd_sub_;
  StampedInbox<sensor_msgs::msg::Image> depth_in_;
  StampedInbox<sensor_msgs::msg::CameraInfo> info_in_, tof_info_in_;
  StampedInbox<sensor_msgs::msg::CompressedImage> rgb_in_;
  StampedInbox<sensor_msgs::msg::PointCloud2> rgbd_in_;
  StampedInbox<sensor_msgs::msg::PointCloud2> obstacle_, drop_;
  StampedInbox<json> warnings_;
};

// The fixture under its second name: the harness tests (pairs, derived cloud) run on the same machinery.
using Harness = ReplayTest;

}  // namespace

// --- reader ------------------------------------------------------------------------------------------------------

TEST(ReplayReader, EveryFixtureHoldsFifteenSecondsOfToFClouds)
{
  for (const char * name : SCENES) {
    SCOPED_TRACE(name);
    const ReplayScene scene = ground_replay::read_replay(replay_path(name));
    EXPECT_EQ(scene.frame_id, TOF_FRAME);
    EXPECT_GE(scene.clouds.size(), MIN_CLOUDS);
    EXPECT_LE(scene.clouds.size(), MAX_CLOUDS);
    ASSERT_FALSE(scene.clouds.empty());
    for (std::size_t i = 1; i < scene.clouds.size(); ++i) {
      EXPECT_GT(scene.clouds[i].stamp_ns, scene.clouds[i - 1].stamp_ns) << "cloud " << i;
    }
    EXPECT_LT(static_cast<double>(scene.clouds.back().stamp_ns - scene.clouds.front().stamp_ns) / NS_PER_S, MAX_SPAN_S);
    for (const auto & cloud : scene.clouds) {
      EXPECT_FALSE(cloud.points_mm.empty());
      EXPECT_NEAR(cloud.base_T_cloud.linear().determinant(), 1.0, 1e-6);
      EXPECT_NEAR(cloud.odom_T_base.linear().determinant(), 1.0, 1e-6);
    }
    EXPECT_FALSE(scene.imu.empty());
    for (const auto & s : scene.imu) {
      EXPECT_NEAR(s.orientation.norm(), 1.0, 1e-3);
    }
    EXPECT_NEAR(scene.base_T_imu.linear().determinant(), 1.0, 1e-6);
  }
}

TEST(ReplayReader, CloudsLieInFrontOfTheRobotInBaseLink)
{
  // The head ToF looks down at the floor ahead: every cloud's median point is ahead of the base and near the floor.
  const ReplayScene scene = ground_replay::read_replay(replay_path("home"));
  for (const auto & cloud : scene.clouds) {
    std::vector<double> xs, zs;
    for (const auto & p : cloud.points_mm) {
      const Eigen::Vector3d b = cloud.base_T_cloud * (Eigen::Vector3d(p[0], p[1], p[2]) / MM_PER_M);
      xs.push_back(b.x());
      zs.push_back(b.z());
    }
    std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
    std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
    EXPECT_GT(xs[xs.size() / 2], 0.3);
    EXPECT_LT(std::fabs(zs[zs.size() / 2]), 0.15);
  }
}

TEST(ReplayReader, RejectsAWrongMagicAVersionAndATruncatedFile)
{
  const std::string bytes = file_bytes(replay_path("box"));
  ASSERT_GT(bytes.size(), 16u);
  EXPECT_NO_THROW(ground_replay::decode_replay(bytes));
  std::string bad_magic = bytes;
  bad_magic[0] = 'X';
  EXPECT_THROW(ground_replay::decode_replay(bad_magic), std::runtime_error);
  std::string bad_version = bytes;
  bad_version[ground_replay::REPLAY_MAGIC_SIZE] = static_cast<char>(ground_replay::REPLAY_VERSION_PAIRS + 1);
  EXPECT_THROW(ground_replay::decode_replay(bad_version), std::runtime_error);
  EXPECT_THROW(ground_replay::decode_replay(bytes.substr(0, bytes.size() - 1)), std::runtime_error);
  EXPECT_THROW(ground_replay::decode_replay(bytes + "x"), std::runtime_error);
  EXPECT_THROW(ground_replay::read_replay(replay_path("missing")), std::runtime_error);
}

TEST(ReplayReader, DecodesV2Pairs)
{
  const ReplayScene scene = ground_replay::read_replay(replay_path("tiny_v2"));
  EXPECT_EQ(scene.version, ground_replay::REPLAY_VERSION_PAIRS);
  EXPECT_EQ(scene.clouds.size(), 3u);
  ASSERT_TRUE(scene.rgb_model.has_value());
  ASSERT_TRUE(scene.tof_model.has_value());
  EXPECT_EQ(scene.rgb_model->width, 16u);
  EXPECT_EQ(scene.rgb_model->height, 12u);
  EXPECT_EQ(scene.rgb_model->frame, "rgb_optical_frame");
  EXPECT_DOUBLE_EQ(scene.rgb_model->K[0], 30.0);
  EXPECT_DOUBLE_EQ(scene.rgb_model->D[4], 0.03);
  EXPECT_EQ(scene.tof_model->frame, TOF_FRAME);
  EXPECT_DOUBLE_EQ(scene.tof_model->K[4], 20.0);
  ASSERT_EQ(scene.pairs.size(), 2u);
  EXPECT_GT(scene.pairs[1].stamp_ns, scene.pairs[0].stamp_ns);
  for (const auto & pair : scene.pairs) {
    EXPECT_EQ(pair.depth_mm.type(), CV_16UC1);
    EXPECT_EQ(pair.depth_mm.cols, 16);
    EXPECT_EQ(pair.depth_mm.rows, 12);
    EXPECT_GT(cv::countNonZero(pair.depth_mm), 0);
    ASSERT_GT(pair.jpeg.size(), 2u);
    EXPECT_EQ(pair.jpeg[0], 0xFF);  // JPEG start of image
    EXPECT_EQ(pair.jpeg[1], 0xD8);
    const cv::Mat rgb = cv::imdecode(pair.jpeg, cv::IMREAD_COLOR);
    EXPECT_EQ(rgb.cols, 16);
    EXPECT_NEAR(pair.base_T_rgb.linear().determinant(), 1.0, 1e-6);
  }
}

TEST(ReplayReader, V1FixturesStillRead)
{
  const ReplayScene scene = ground_replay::read_replay(replay_path("home"));
  EXPECT_EQ(scene.version, ground_replay::REPLAY_VERSION);
  EXPECT_FALSE(scene.rgb_model.has_value());
  EXPECT_FALSE(scene.tof_model.has_value());
  EXPECT_TRUE(scene.pairs.empty());
  EXPECT_GE(scene.clouds.size(), MIN_CLOUDS);
}

TEST(ReplayReader, RejectsUnknownVersion)
{
  std::string bytes = file_bytes(replay_path("tiny_v2"));
  for (const char version : {static_cast<char>(0), static_cast<char>(3)}) {
    bytes[ground_replay::REPLAY_MAGIC_SIZE] = version;
    EXPECT_THROW(ground_replay::decode_replay(bytes), std::runtime_error) << "version " << static_cast<int>(version);
  }
}

TEST(ReplayReader, RejectsATruncatedGuidanceSectionAndANon16BitPng)
{
  const std::string bytes = file_bytes(replay_path("tiny_v2"));
  EXPECT_NO_THROW(ground_replay::decode_replay(bytes));
  // Every cut inside the guidance section (the last pair's PNG and the models) is a truncation.
  for (const std::size_t cut : {1u, 8u, 200u, 400u}) {
    EXPECT_THROW(ground_replay::decode_replay(bytes.substr(0, bytes.size() - cut)), std::runtime_error) << cut;
  }
  // Corrupt the last PNG's signature: it no longer decodes.
  std::string broken = bytes;
  const std::size_t png = broken.rfind("\x89PNG");
  ASSERT_NE(png, std::string::npos);
  broken[png + 1] = 'X';
  EXPECT_THROW(ground_replay::decode_replay(broken), std::runtime_error);
}

// --- metric functions on hand-built frames -----------------------------------------------------------------------

TEST(ReplayMetrics, PlaneShareCountsFramesAfterTheWarmUp)
{
  // 0 s and 1.999 s are warm-up; of 2.0, 2.5 and 3.0 s two have a plane.
  const std::vector<ReplayFrame> frames{
    frame_at(0.0, false), frame_at(1.999, false), frame_at(2.0), frame_at(2.5, false), frame_at(3.0)};
  EXPECT_DOUBLE_EQ(ground_replay::plane_share(frames), 2.0 / 3.0);
  EXPECT_TRUE(std::isnan(ground_replay::plane_share({frame_at(0.0)})));
}

TEST(ReplayMetrics, BoxLethalShareCountsObstaclePointsInTheBoxRoi)
{
  using ground_replay::box_lethal_share;
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.70, 0.20)), 1.0);
  // Each ROI edge (x 0.57 .. 0.81, y 0.02 .. 0.36): just inside counts, just outside does not.
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.575, 0.20)), 1.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.565, 0.20)), 0.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.805, 0.20)), 1.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.815, 0.20)), 0.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.70, 0.025)), 1.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.70, 0.015)), 0.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.70, 0.355)), 1.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.70, 0.365)), 0.0);
  // A drop point is not the box; the warm-up and unanswered frames are not counted.
  EXPECT_DOUBLE_EQ(box_lethal_share(one_lethal_point(0.70, 0.20, true)), 0.0);
  auto frames = one_lethal_point(0.70, 0.20);
  frames.push_back(frame_at(3.0));
  frames.push_back(frame_at(3.1, false));
  frames.back().obstacle.emplace_back(0.70f, 0.20f, 0.05f);
  frames[0].obstacle.emplace_back(0.70f, 0.20f, 0.05f);
  EXPECT_DOUBLE_EQ(box_lethal_share(frames), 0.5);
  EXPECT_TRUE(std::isnan(box_lethal_share({frame_at(0.0)})));
}

TEST(ReplayMetrics, BoxWarningShareCountsTofStepAreasCentredOnTheBox)
{
  using ground_replay::box_warning_share;
  EXPECT_DOUBLE_EQ(box_warning_share(one_area("step", 0.70, 0.20)), 1.0);
  EXPECT_DOUBLE_EQ(box_warning_share(one_area("step", 0.565, 0.20)), 0.0);  // ROI x 0.57 .. 0.81
  EXPECT_DOUBLE_EQ(box_warning_share(one_area("step", 0.70, 0.365)), 0.0);  // ROI y 0.02 .. 0.36
  EXPECT_DOUBLE_EQ(box_warning_share(one_area("ramp", 0.70, 0.20)), 0.0);
  EXPECT_DOUBLE_EQ(box_warning_share(one_area("colour", 0.70, 0.20)), 0.0);
  EXPECT_TRUE(std::isnan(box_warning_share({frame_at(0.0)})));
}

TEST(ReplayMetrics, BoardLethalFramesCountsObstacleAndDropPointsOnTheBoard)
{
  using ground_replay::board_lethal_frames;
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.45, 0.0)), 1);
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.45, 0.0, true)), 1);
  // x 0.37 .. 0.57 (the x < 0.37 gripper zone is excluded), y -0.12 .. +0.14.
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.375, 0.0)), 1);
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.365, 0.0)), 0);
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.565, 0.0)), 1);
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.575, 0.0)), 0);
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.45, -0.115)), 1);
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.45, -0.125)), 0);
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.45, 0.135)), 1);
  EXPECT_EQ(board_lethal_frames(one_lethal_point(0.45, 0.145)), 0);
  auto frames = one_lethal_point(0.45, 0.0);
  frames[0].obstacle.emplace_back(0.45f, 0.0f, 0.05f);  // warm-up
  EXPECT_EQ(board_lethal_frames(frames), 1);
}

TEST(ReplayMetrics, StrayLethalShareCountsLethalPointsAwayFromTheBoxWithin1p2m)
{
  using ground_replay::stray_lethal_share;
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(1.0, -0.5)), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(1.0, -0.5, true)), 1.0);
  // Range edge: hypot 1.195 counts, 1.205 does not.
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(1.195, 0.0)), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(1.205, 0.0)), 0.0);
  // Gripper zone edge.
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(0.375, 0.0)), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(0.365, 0.0)), 0.0);
  // The box lethal ROI is not stray; just outside its edges is.
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(0.70, 0.20)), 0.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(0.575, 0.20)), 0.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(0.565, 0.20)), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(0.815, 0.20)), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(0.70, 0.015)), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(one_lethal_point(0.70, 0.365)), 1.0);
  auto frames = one_lethal_point(1.0, -0.5);
  frames.push_back(frame_at(3.0));
  EXPECT_DOUBLE_EQ(stray_lethal_share(frames), 0.5);
}

TEST(ReplayMetrics, FarSideWarningFramesCountsRampAndStepAreasBehindTheBox)
{
  using ground_replay::far_side_warning_frames;
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.86, 0.19)), 1);
  EXPECT_EQ(far_side_warning_frames(one_area("step", 0.86, 0.19)), 1);
  EXPECT_EQ(far_side_warning_frames(one_area("colour", 0.86, 0.19)), 0);
  // Centroid edges x 0.78 .. 0.95, y 0.05 .. 0.33.
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.785, 0.19)), 1);
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.775, 0.19)), 0);
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.945, 0.19)), 1);
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.955, 0.19)), 0);
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.86, 0.055)), 1);
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.86, 0.045)), 0);
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.86, 0.325)), 1);
  EXPECT_EQ(far_side_warning_frames(one_area("ramp", 0.86, 0.335)), 0);
  // The centroid is taken in base_link: the document is in map, base_T_warnings moves it.
  auto moved = one_area("ramp", 1.86, 0.19);
  EXPECT_EQ(far_side_warning_frames(moved), 0);
  moved[1].base_T_warnings = Eigen::Translation3d(-1.0, 0.0, 0.0) * Eigen::Isometry3d::Identity();
  EXPECT_EQ(far_side_warning_frames(moved), 1);
  // Warm-up and unanswered frames are not counted; a null document has no areas.
  auto frames = one_area("ramp", 0.86, 0.19);
  frames[0].warnings["areas"].push_back(area("ramp", 0.86, 0.19));
  frames.push_back(frame_at(3.0, false));
  frames.back().warnings["areas"].push_back(area("ramp", 0.86, 0.19));
  frames.push_back(frame_at(3.1));
  frames.back().warnings = json();
  EXPECT_EQ(far_side_warning_frames(frames), 1);
}

TEST(ReplayMetrics, RampLethalFramesCountsLethalPointsOnTheRamp)
{
  using ground_replay::ramp_lethal_frames;
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(0.9, -0.3)), 1);
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(0.9, -0.3, true)), 1);
  // x 0.75 .. 1.10, y -0.45 .. -0.15.
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(0.755, -0.3)), 1);
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(0.745, -0.3)), 0);
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(1.095, -0.3)), 1);
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(1.105, -0.3)), 0);
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(0.9, -0.445)), 1);
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(0.9, -0.455)), 0);
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(0.9, -0.155)), 1);
  EXPECT_EQ(ramp_lethal_frames(one_lethal_point(0.9, -0.145)), 0);
}

TEST(ReplayMetrics, ChurnShareComparesConsecutiveAnsweredWarningSets)
{
  using ground_replay::churn_share;
  // Answered frames after the warm-up: A, A, B, B (vertices 2 mm off, same at 1 cm), A -> 2 of 4 pairs differ.
  std::vector<ReplayFrame> frames{frame_at(0.0)};
  for (double t : {2.0, 2.1, 2.2, 2.3, 2.4}) {
    frames.push_back(frame_at(t));
  }
  frames[0].warnings["areas"].push_back(area("step", 0.9, 0.0));  // warm-up, not compared
  frames[1].warnings["areas"].push_back(area("ramp", 0.9, 0.0));
  frames[2].warnings["areas"].push_back(area("ramp", 0.9, 0.0));
  frames[3].warnings["areas"].push_back(area("step", 0.9, 0.0));
  frames[4].warnings["areas"].push_back(area("step", 0.902, 0.002));
  frames[5].warnings["areas"].push_back(area("ramp", 0.9, 0.0));
  EXPECT_DOUBLE_EQ(churn_share(frames), 0.5);
  // An unanswered frame between two answered ones does not break the pair.
  std::vector<ReplayFrame> gap{frame_at(0.0), frame_at(2.0), frame_at(2.1, false), frame_at(2.2)};
  gap[1].warnings["areas"].push_back(area("ramp", 0.9, 0.0));
  gap[3].warnings["areas"].push_back(area("ramp", 0.9, 0.0));
  EXPECT_DOUBLE_EQ(churn_share(gap), 0.0);
  // An area in the gripper zone (centroid x < 0.37) is ignored; just outside it is not.
  std::vector<ReplayFrame> gripper{frame_at(0.0), frame_at(2.0), frame_at(2.1)};
  gripper[2].warnings["areas"].push_back(area("step", 0.365, 0.0));
  EXPECT_DOUBLE_EQ(churn_share(gripper), 0.0);
  gripper[2].warnings["areas"][0] = area("step", 0.375, 0.0);
  EXPECT_DOUBLE_EQ(churn_share(gripper), 1.0);
  EXPECT_TRUE(std::isnan(churn_share({frame_at(0.0), frame_at(2.0)})));
}

TEST(ReplayMetrics, MetricsLineNamesEveryMetric)
{
  const std::string line = ground_replay::metrics_line("home", one_lethal_point(0.70, 0.20));
  EXPECT_EQ(line.rfind("replay home: plane_share=1.000 box_lethal_share=1.000 board_lethal_frames=0 ", 0), 0u) << line;
  for (const char * key :
       {"stray_lethal_share=0.000", "far_side_warning_frames=0", "ramp_lethal_frames=0", "churn_share=nan"}) {
    EXPECT_NE(line.find(key), std::string::npos) << key << " in " << line;
  }
}

// --- scene truth and the truth based metrics --------------------------------------------------------------------

TEST(SceneTruth, LoadsAndRejectsMissingKeys)
{
  const ground_replay::SceneTruth truth = tiny_truth();
  ASSERT_EQ(truth.board.size(), 4u);
  ASSERT_EQ(truth.box.size(), 4u);
  ASSERT_EQ(truth.ramp.size(), 4u);
  EXPECT_DOUBLE_EQ(truth.board[2].x(), 0.57);
  EXPECT_DOUBLE_EQ(truth.box[2].y(), 0.33);
  EXPECT_DOUBLE_EQ(truth.box_front_edge[1].y(), 0.33);
  EXPECT_DOUBLE_EQ(truth.empty.y_min, -0.40);
  EXPECT_DOUBLE_EQ(truth.empty.x_max, 0.70);

  const YAML::Node full = YAML::LoadFile(tiny_truth_path());
  for (const char * key : {"board", "box", "box_front_edge", "ramp", "empty"}) {
    SCOPED_TRACE(key);
    YAML::Node broken = YAML::Clone(full);
    broken.remove(key);
    try {
      ground_replay::load_truth(write_yaml(broken, key));
      FAIL() << "a truth without " << key << " loaded";
    } catch (const std::runtime_error & e) {
      EXPECT_NE(std::string(e.what()).find(key), std::string::npos) << e.what();
    }
  }
  for (const char * key : {"x_min", "x_max", "y_min", "y_max"}) {
    SCOPED_TRACE(key);
    YAML::Node broken = YAML::Clone(full);
    broken["empty"].remove(key);
    EXPECT_THROW(ground_replay::load_truth(write_yaml(broken, key)), std::runtime_error);
  }
  YAML::Node short_polygon = YAML::Clone(full);
  short_polygon["box"] = YAML::Load("[[0.6, 0.05], [0.6, 0.33]]");
  EXPECT_THROW(ground_replay::load_truth(write_yaml(short_polygon, "short")), std::runtime_error);
  YAML::Node bad_edge = YAML::Clone(full);
  bad_edge["box_front_edge"] = YAML::Load("[[0.6, 0.05]]");
  EXPECT_THROW(ground_replay::load_truth(write_yaml(bad_edge, "edge")), std::runtime_error);
  EXPECT_THROW(ground_replay::load_truth(write_yaml(YAML::Load("[1, 2]"), "seq")), std::runtime_error);
  EXPECT_THROW(ground_replay::load_truth(replay_path("missing_truth")), std::runtime_error);
}

TEST(ReplayMetrics, WarningAreasKeepKindSourceAndNullableHeights)
{
  const json doc{
    {"areas", json::array(
                {json{
                   {"kind", "no_return"},
                   {"polygon", json::array({{0.9, 0.0}, {1.0, 0.0}, {1.0, 0.1}})},
                   {"height_m", nullptr},
                   {"slope_deg", nullptr},
                   {"source", "guidance"}},
                 area("step", 0.9, 0.0)})}};
  const auto areas = ground_replay::parse_areas(doc);
  ASSERT_EQ(areas.size(), 2u);
  EXPECT_EQ(areas[0].kind, "no_return");
  EXPECT_EQ(areas[0].source, "guidance");
  EXPECT_FALSE(areas[0].height_m.has_value());
  EXPECT_FALSE(areas[0].slope_deg.has_value());
  ASSERT_EQ(areas[0].polygon.size(), 3u);
  EXPECT_DOUBLE_EQ(areas[0].polygon[2].y(), 0.1);
  EXPECT_DOUBLE_EQ(areas[1].height_m.value(), 0.03);
  EXPECT_DOUBLE_EQ(areas[1].slope_deg.value(), 5.0);
  EXPECT_TRUE(ground_replay::parse_areas(json()).empty());
}

TEST(ReplayMetrics, BoxLethalShareTruthGrowsTheBoxBy3cm)
{
  const auto truth = tiny_truth();
  using ground_replay::box_lethal_share;
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.70, 0.20}}), truth), 1.0);
  // Box x 0.60-0.78, y 0.05-0.33 grown by 3 cm: front edge 0.57, back 0.81, sides 0.02 and 0.36.
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.575, 0.20}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.565, 0.20}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.805, 0.20}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.815, 0.20}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.70, 0.025}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.70, 0.015}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.70, 0.355}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.70, 0.365}}), truth), 0.0);
  // A drop point is not the box; an unanswered frame is not counted.
  EXPECT_DOUBLE_EQ(box_lethal_share(lethal_points({{0.70, 0.20}}, true), truth), 0.0);
  auto frames = lethal_points({{0.70, 0.20}});
  frames.push_back(frame_at(3.0));
  EXPECT_DOUBLE_EQ(box_lethal_share(frames, truth), 0.5);
  EXPECT_TRUE(std::isnan(box_lethal_share({frame_at(0.0)}, truth)));
}

TEST(ReplayMetrics, BoardLethalFramesTruthShrinksTheBoardBy2cm)
{
  const auto truth = tiny_truth();
  using ground_replay::board_lethal_frames;
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.47, 0.0}}), truth), 1);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.47, 0.0}}, true), truth), 1);
  // Board x 0.37-0.57, y -0.12-0.14 shrunk by 2 cm: x 0.39-0.55, y -0.10-0.12 (x 0.37 is also the gripper limit).
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.395, 0.0}}), truth), 1);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.385, 0.0}}), truth), 0);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.545, 0.0}}), truth), 1);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.555, 0.0}}), truth), 0);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.47, -0.095}}), truth), 1);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.47, -0.105}}), truth), 0);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.47, 0.115}}), truth), 1);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.47, 0.125}}), truth), 0);
  // The gripper zone is excluded: a board that reaches x < 0.37 does not count points there.
  auto wide = tiny_truth();
  wide.board = {{0.20, -0.12}, {0.57, -0.12}, {0.57, 0.14}, {0.20, 0.14}};
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.365, 0.0}}), wide), 0);
  EXPECT_EQ(board_lethal_frames(lethal_points({{0.375, 0.0}}), wide), 1);
}

TEST(ReplayMetrics, StrayLethalShareTruthLeavesOutTheGrownBox)
{
  const auto truth = tiny_truth();
  using ground_replay::stray_lethal_share;
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{1.0, -0.5}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{1.0, -0.5}}, true), truth), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{1.195, 0.0}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{1.205, 0.0}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{0.375, 0.0}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{0.365, 0.0}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{0.70, 0.20}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{0.575, 0.20}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{0.565, 0.20}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{0.815, 0.20}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{0.70, 0.015}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(stray_lethal_share(lethal_points({{0.70, 0.365}}), truth), 1.0);
  auto frames = lethal_points({{1.0, -0.5}});
  frames.push_back(frame_at(3.0));
  EXPECT_DOUBLE_EQ(stray_lethal_share(frames, truth), 0.5);
}

TEST(ReplayMetrics, RampLethalFramesTruthUsesTheRampFootprint)
{
  const auto truth = tiny_truth();
  using ground_replay::ramp_lethal_frames;
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{0.95, -0.3}}), truth), 1);
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{0.95, -0.3}}, true), truth), 1);
  // Ramp x 0.85-1.10, y -0.45 - -0.15.
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{0.855, -0.3}}), truth), 1);
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{0.845, -0.3}}), truth), 0);
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{1.095, -0.3}}), truth), 1);
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{1.105, -0.3}}), truth), 0);
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{0.95, -0.445}}), truth), 1);
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{0.95, -0.455}}), truth), 0);
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{0.95, -0.155}}), truth), 1);
  EXPECT_EQ(ramp_lethal_frames(lethal_points({{0.95, -0.145}}), truth), 0);
}

TEST(ReplayMetrics, EmptyLethalShareCountsObstaclePointsInTheEmptyRectangle)
{
  const auto truth = tiny_truth();
  using ground_replay::empty_lethal_share;
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.55, -0.30}}), truth), 1.0);
  // Empty x 0.40-0.70, y -0.40 - -0.20.
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.405, -0.30}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.395, -0.30}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.695, -0.30}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.705, -0.30}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.55, -0.395}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.55, -0.405}}), truth), 0.0);
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.55, -0.205}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.55, -0.195}}), truth), 0.0);
  // Drop points do not count; the gripper zone is excluded.
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.55, -0.30}}, true), truth), 0.0);
  auto gripper = tiny_truth();
  gripper.empty = {0.2, 0.5, -0.4, -0.2};
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.365, -0.30}}), gripper), 0.0);
  EXPECT_DOUBLE_EQ(empty_lethal_share(lethal_points({{0.375, -0.30}}), gripper), 1.0);
  auto frames = lethal_points({{0.55, -0.30}});
  frames.push_back(frame_at(3.0));
  EXPECT_DOUBLE_EQ(empty_lethal_share(frames, truth), 0.5);
}

namespace
{

// The box front edge runs x = 0.60, y 0.05-0.33 (width 0.28); the middle 80 % is y 0.078-0.302.
// A row of obstacle points along the edge at x = 0.60 + shift, every 1 cm over y 0.04-0.34.
std::vector<ReplayFrame> edge_row(double shift_m, double y_from = 0.04, double y_to = 0.34)
{
  std::vector<std::pair<double, double>> pts;
  for (double y = y_from; y <= y_to + 1e-9; y += 0.01) {
    pts.emplace_back(0.60 + shift_m, y);
  }
  return lethal_points(pts);
}

}  // namespace

TEST(ReplayMetrics, BoxFrontEdgeErrorIsTheP95OfTheNormalDistance)
{
  const auto truth = tiny_truth();
  using ground_replay::box_front_edge_error_m;
  EXPECT_NEAR(box_front_edge_error_m(edge_row(0.0), truth), 0.0, 1e-6);
  EXPECT_NEAR(box_front_edge_error_m(edge_row(0.015), truth), 0.015, 1e-6);
  EXPECT_NEAR(box_front_edge_error_m(edge_row(-0.015), truth), 0.015, 1e-6);
  // 0.10 m from the footprint is the search limit: points 0.09 m in front are found, 0.11 m in front are not taken.
  EXPECT_NEAR(box_front_edge_error_m(edge_row(-0.09), truth), 0.09, 1e-6);
  EXPECT_NEAR(box_front_edge_error_m(edge_row(-0.11), truth), 0.10, 1e-6);
  // No obstacle point at all: every sample counts as the search limit.
  EXPECT_NEAR(box_front_edge_error_m(lethal_points({}), truth), 0.10, 1e-6);
  // A sample needs a point within 2 cm along the edge: a row covering only y 0.04-0.12 leaves the samples from
  // y 0.158 on (more than 2 cm from any point) at the limit; 8 of 12 is over 5 % of the samples, so p95 is the limit.
  EXPECT_NEAR(box_front_edge_error_m(edge_row(0.0, 0.04, 0.12), truth), 0.10, 1e-6);
  // 2 of 12 samples missing (y 0.278 and 0.298) is 17 %: the limit.
  EXPECT_NEAR(box_front_edge_error_m(edge_row(0.0, 0.04, 0.25), truth), 0.10, 1e-6);
  // p95 drops the worst 5 % of all samples of all frames: one frame without points among 30 (12 of 360 samples,
  // 3.3 %) leaves 0, among 10 (10 %) gives the limit.
  auto many = [&](int n) {
    std::vector<ReplayFrame> frames{frame_at(0.0)};
    for (int i = 0; i < n; ++i) {
      ReplayFrame f = edge_row(0.0)[1];
      f.stamp_ns += static_cast<int64_t>(i) * NS_PER_S / 10;
      if (i == 0) {
        f.obstacle.clear();
      }
      frames.push_back(f);
    }
    return frames;
  };
  EXPECT_NEAR(box_front_edge_error_m(many(30), truth), 0.0, 1e-6);
  EXPECT_NEAR(box_front_edge_error_m(many(10), truth), 0.10, 1e-6);
  // The x < 0.37 gripper zone and warm-up / unanswered frames are not used: points of the warm-up frame are ignored.
  auto warm = edge_row(0.0);
  warm[0].obstacle.emplace_back(0.61f, 0.2f, 0.05f);
  EXPECT_NEAR(box_front_edge_error_m(warm, truth), 0.0, 1e-6);
  EXPECT_TRUE(std::isnan(box_front_edge_error_m({frame_at(0.0)}, truth)));
}

TEST(ReplayMetrics, BoxFrontEdgeErrorIgnoresPointsFarFromTheBoxAndTheGripperZone)
{
  auto truth = tiny_truth();
  // A point on the sample line but 0.2 m in front of the box is not a box point: the edge stays at the limit.
  auto far = lethal_points({{0.40, 0.2}});
  EXPECT_NEAR(ground_replay::box_front_edge_error_m(far, truth), 0.10, 1e-6);
  // A box whose front edge lies in the gripper zone: the points with x < 0.37 are not used.
  truth.box = {{0.30, 0.05}, {0.30, 0.33}, {0.50, 0.33}, {0.50, 0.05}};
  truth.box_front_edge = {Eigen::Vector2d(0.30, 0.05), Eigen::Vector2d(0.30, 0.33)};
  std::vector<std::pair<double, double>> pts;
  for (double y = 0.04; y <= 0.34; y += 0.01) {
    pts.emplace_back(0.30, y);
  }
  EXPECT_NEAR(ground_replay::box_front_edge_error_m(lethal_points(pts), truth), 0.10, 1e-6);
}

TEST(ReplayMetrics, BoxWidthErrorIsTheP95OfTheExtentAlongTheEdgeMinusTheWidth)
{
  const auto truth = tiny_truth();
  using ground_replay::box_width_error_m;
  // True width 0.28; points y 0.05-0.33.
  EXPECT_NEAR(box_width_error_m(edge_row(0.0, 0.05, 0.33), truth), 0.0, 1e-6);
  EXPECT_NEAR(box_width_error_m(edge_row(0.0, 0.04, 0.34), truth), 0.02, 1e-6);  // 0.30 - 0.28
  EXPECT_NEAR(box_width_error_m(edge_row(0.0, 0.07, 0.30), truth), 0.05, 1e-6);  // 0.23, the absolute value
  // Only points within 0.10 m of the footprint count: a point 0.5 m to the side is not part of the width.
  auto stray = edge_row(0.0, 0.05, 0.33);
  stray[1].obstacle.emplace_back(0.60f, 0.83f, 0.05f);
  EXPECT_NEAR(box_width_error_m(stray, truth), 0.0, 1e-6);
  // A frame without any such point has extent 0: its error is the width.
  EXPECT_NEAR(box_width_error_m(lethal_points({}), truth), 0.28, 1e-6);
  EXPECT_TRUE(std::isnan(box_width_error_m({frame_at(0.0)}, truth)));
}

TEST(ReplayMetrics, OutsideLethalFramesShareCountsPointsBetween3And15cmOutsideTheBox)
{
  const auto truth = tiny_truth();
  using ground_replay::outside_lethal_frames_share;
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.70, 0.20}}), truth), 0.0);   // on the box
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.575, 0.20}}), truth), 0.0);  // 2.5 cm outside
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.565, 0.20}}), truth), 1.0);  // 3.5 cm outside
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.455, 0.20}}), truth), 1.0);  // 14.5 cm outside
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.445, 0.20}}), truth), 0.0);  // 15.5 cm outside
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.70, 0.475}}), truth), 1.0);
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.70, 0.485}}), truth), 0.0);
  // Drop points are not obstacle points; the gripper zone is excluded.
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.565, 0.20}}, true), truth), 0.0);
  auto near_gripper = tiny_truth();
  near_gripper.box = {{0.45, 0.05}, {0.45, 0.33}, {0.60, 0.33}, {0.60, 0.05}};
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.365, 0.20}}), near_gripper), 0.0);
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(lethal_points({{0.375, 0.20}}), near_gripper), 1.0);
  auto frames = lethal_points({{0.565, 0.20}});
  frames.push_back(frame_at(3.0));
  EXPECT_DOUBLE_EQ(outside_lethal_frames_share(frames, truth), 0.5);
}

namespace
{

// A no_return area covering [x0, x1] x [y0, y1] in the frame's document frame (identity to base_link).
void add_no_return(ReplayFrame & f, double x0, double x1, double y0, double y1)
{
  f.warnings["areas"].push_back(json{
    {"kind", "no_return"},
    {"polygon", json::array({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}})},
    {"height_m", nullptr},
    {"slope_deg", nullptr},
    {"source", "guidance"}});
  f.areas = ground_replay::parse_areas(f.warnings);
}

}  // namespace

TEST(ReplayMetrics, NoReturnCoverShareNeeds70PercentOfTheRamp)
{
  const auto truth = tiny_truth();  // ramp x 0.85-1.10 (0.25 m), y -0.45 - -0.15
  using ground_replay::no_return_cover_share;
  auto covered = [&](double x1) {
    std::vector<ReplayFrame> frames{frame_at(0.0), frame_at(TEST_FRAME_S)};
    add_no_return(frames[1], 0.80, x1, -0.50, -0.10);
    return no_return_cover_share(frames, truth);
  };
  EXPECT_DOUBLE_EQ(covered(1.20), 1.0);
  EXPECT_DOUBLE_EQ(covered(1.035), 1.0);  // 0.185 of 0.25 = 74 %
  EXPECT_DOUBLE_EQ(covered(1.015), 0.0);  // 0.165 of 0.25 = 66 %
  // Only a different kind, or no area: not covered.
  std::vector<ReplayFrame> frames{frame_at(0.0), frame_at(TEST_FRAME_S)};
  EXPECT_DOUBLE_EQ(no_return_cover_share(frames, truth), 0.0);
  frames[1].warnings["areas"].push_back(area("ramp", 0.95, -0.3));
  frames[1].areas = ground_replay::parse_areas(frames[1].warnings);
  EXPECT_DOUBLE_EQ(no_return_cover_share(frames, truth), 0.0);
  // Two areas together cover the ramp; the share counts frames; the base_link transform of the document applies.
  std::vector<ReplayFrame> two{frame_at(0.0), frame_at(TEST_FRAME_S), frame_at(3.0)};
  add_no_return(two[1], 0.80, 0.97, -0.50, -0.10);
  add_no_return(two[1], 0.97, 1.20, -0.50, -0.10);
  add_no_return(two[2], 1.80, 2.20, -0.50, -0.10);
  two[2].base_T_warnings = Eigen::Translation3d(-1.0, 0.0, 0.0) * Eigen::Isometry3d::Identity();
  EXPECT_DOUBLE_EQ(no_return_cover_share(two, truth), 1.0);
  two[2].base_T_warnings = Eigen::Isometry3d::Identity();
  EXPECT_DOUBLE_EQ(no_return_cover_share(two, truth), 0.5);
  EXPECT_TRUE(std::isnan(no_return_cover_share({frame_at(0.0)}, truth)));
}

TEST(ReplayMetrics, NoReturnStrayShareFlagsCellsMoreThan10cmOutsideTheRamp)
{
  const auto truth = tiny_truth();
  using ground_replay::no_return_stray_share;
  auto stray = [&](double x0, double x1, double y0, double y1) {
    std::vector<ReplayFrame> frames{frame_at(0.0), frame_at(TEST_FRAME_S)};
    add_no_return(frames[1], x0, x1, y0, y1);
    return no_return_stray_share(frames, truth);
  };
  EXPECT_DOUBLE_EQ(stray(0.90, 1.05, -0.40, -0.20), 0.0);  // inside
  EXPECT_DOUBLE_EQ(stray(0.78, 1.18, -0.52, -0.08), 0.0);  // reaching 7-8 cm beyond the ramp
  EXPECT_DOUBLE_EQ(stray(0.90, 1.05, -0.04, -0.01), 1.0);  // 11 cm beyond the ramp edge y -0.15
  EXPECT_DOUBLE_EQ(stray(0.90, 1.05, -0.06, -0.02), 1.0);
  EXPECT_DOUBLE_EQ(stray(0.90, 1.05, -0.07, -0.06), 0.0);   // 8 cm beyond
  EXPECT_DOUBLE_EQ(stray(1.215, 1.30, -0.40, -0.20), 1.0);  // beyond the far edge, 11.5 cm
  // Gripper zone: a stray area with x < 0.37 is ignored.
  EXPECT_DOUBLE_EQ(stray(0.20, 0.36, 0.0, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(stray(0.38, 0.50, 0.0, 0.1), 1.0);
  // Frames without areas, other kinds: not stray; the share counts frames.
  std::vector<ReplayFrame> frames{frame_at(0.0), frame_at(TEST_FRAME_S), frame_at(3.0)};
  add_no_return(frames[1], 0.38, 0.50, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(no_return_stray_share(frames, truth), 0.5);
  EXPECT_TRUE(std::isnan(no_return_stray_share({frame_at(0.0)}, truth)));
}

TEST(ReplayMetrics, TruthMetricsLineNamesEveryMetric)
{
  const std::string line = ground_replay::truth_metrics_line("home", lethal_points({{0.70, 0.20}}), tiny_truth());
  EXPECT_EQ(line.rfind("replay home:", 0), 0u) << line;
  for (const char * key :
       {"plane_share=", "box_lethal_share=1.000", "board_lethal_frames=0", "stray_lethal_share=0.000",
        "ramp_lethal_frames=0", "churn_share=", "box_front_edge_error_m=", "box_width_error_m=",
        "outside_lethal_frames_share=", "no_return_cover_share=", "no_return_stray_share=", "empty_lethal_share="}) {
    EXPECT_NE(line.find(key), std::string::npos) << key << " in " << line;
  }
  EXPECT_EQ(line.find('\n'), std::string::npos);
}

// --- the harness: pairs and the derived XYZRGB cloud --------------------------------------------------------------

TEST(HarnessCloud, DerivedCloudHalvesDepthByBlockMinimumAndDecodesColour)
{
  // 4 x 2 depth: blocks [100, 0 / 300, 200] and [0, 0 / 0, 0]; the first has minimum valid 100, the second none.
  ground_replay::ReplayPair pair;
  pair.depth_mm = (cv::Mat_<uint16_t>(2, 4) << 100, 0, 0, 0, 300, 200, 0, 0);
  cv::Mat bgr(2, 4, CV_8UC3, cv::Scalar(10, 20, 200));  // BGR: red 200
  ASSERT_TRUE(cv::imencode(".png", bgr, pair.jpeg));    // the decoder sniffs the format; PNG keeps the colour exact
  ground_replay::CameraModel model;
  model.K = {40.0, 0.0, 2.0, 0.0, 40.0, 1.0, 0.0, 0.0, 1.0};
  model.width = 4;
  model.height = 2;
  model.frame = "rgb_optical_frame";
  const auto msg = derived_rgbd_cloud(pair, model, 7 * NS_PER_S);
  EXPECT_EQ(msg.header.frame_id, "rgb_optical_frame");
  EXPECT_EQ(ns_of(msg.header.stamp), 7 * NS_PER_S);
  ASSERT_EQ(msg.width * msg.height, 1u);  // only the first block has a depth
  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z"), rgb(msg, "rgb");
  // Halved model: fx = fy = 20, cx 1, cy 0.5; the block at pixel (0, 0) deprojects at its 0.1 m depth.
  EXPECT_NEAR(*z, 0.1, 1e-6);
  EXPECT_NEAR(*x, (0.0 - 1.0) / 20.0 * 0.1, 1e-6);
  EXPECT_NEAR(*y, (0.0 - 0.5) / 20.0 * 0.1, 1e-6);
  uint32_t packed = 0;
  std::memcpy(&packed, &*rgb, sizeof(packed));
  EXPECT_EQ(packed, (200u << 16) | (20u << 8) | 10u);
}

TEST_F(Harness, PublishesPairsOfTinyV2)
{
  const ReplayScene scene = ground_replay::read_replay(replay_path("tiny_v2"));
  ASSERT_EQ(scene.pairs.size(), 2u);
  run_scene(replay_path("tiny_v2"), Guidance::on);
  // The node's config carries the guidance topics of the shipped config under the test prefix.
  ASSERT_TRUE(wait_for([this] { return depth_in_.size() >= 2 && info_in_.size() >= 2 && rgb_in_.size() >= 2; }));
  EXPECT_EQ(depth_in_.size(), 2u);
  EXPECT_EQ(info_in_.size(), 2u);
  EXPECT_EQ(rgb_in_.size(), 2u);
  for (std::size_t i = 0; i < 2; ++i) {
    SCOPED_TRACE(i);
    const int64_t stamp = scene.pairs[i].stamp_ns + offset_ns_;
    const auto depth = depth_in_.get(stamp);
    const auto info = info_in_.get(stamp);
    const auto rgb = rgb_in_.get(stamp);
    ASSERT_TRUE(depth && info && rgb);
    // One shared header.
    EXPECT_EQ(depth->header.frame_id, scene.rgb_model->frame);
    EXPECT_EQ(info->header.frame_id, depth->header.frame_id);
    EXPECT_EQ(rgb->header.frame_id, depth->header.frame_id);
    EXPECT_EQ(depth->encoding, "16UC1");
    EXPECT_EQ(depth->width, 16u);
    EXPECT_EQ(depth->height, 12u);
    EXPECT_EQ(depth->step, 32u);
    EXPECT_EQ(depth->data.size(), 16u * 12u * 2u);
    EXPECT_EQ(std::memcmp(depth->data.data(), scene.pairs[i].depth_mm.data, depth->data.size()), 0);
    EXPECT_EQ(info->width, 16u);
    EXPECT_EQ(info->height, 12u);
    EXPECT_DOUBLE_EQ(info->k[0], scene.rgb_model->K[0]);
    EXPECT_DOUBLE_EQ(info->d[4], scene.rgb_model->D[4]);
    EXPECT_EQ(rgb->format, "jpeg");
    EXPECT_EQ(rgb->data, scene.pairs[i].jpeg);
  }
  // The ToF camera_info once per cloud, in the ToF model.
  EXPECT_EQ(tof_info_in_.size(), scene.clouds.size());
  for (const auto & cloud : scene.clouds) {
    const auto tof = tof_info_in_.get(cloud.stamp_ns + offset_ns_);
    ASSERT_TRUE(tof);
    EXPECT_EQ(tof->header.frame_id, scene.tof_model->frame);
    EXPECT_DOUBLE_EQ(tof->k[4], scene.tof_model->K[4]);
  }
  // The derived XYZRGB cloud: at most 1 Hz, so the 2 pairs of the fixture give one or two; each in the RGB frame at
  // a pair stamp, with float x y z and a packed rgb.
  ASSERT_TRUE(wait_for([this] { return rgbd_in_.size() >= 1; }));
  EXPECT_LE(rgbd_in_.size(), 2u);
  const auto first = rgbd_in_.get(scene.pairs[0].stamp_ns + offset_ns_);
  ASSERT_TRUE(first);
  EXPECT_EQ(first->header.frame_id, scene.rgb_model->frame);
  EXPECT_GT(first->width * first->height, 0u);
  std::vector<std::string> names;
  for (const auto & f : first->fields) {
    names.push_back(f.name);
  }
  EXPECT_EQ(names, (std::vector<std::string>{"x", "y", "z", "rgb"}));
  // The transforms of the pairs reached the TF buffer: base_link <- RGB frame and odom <- base_link at the stamp.
  const auto stamp = tf2::TimePoint(std::chrono::nanoseconds(scene.pairs[0].stamp_ns + offset_ns_));
  EXPECT_TRUE(buffer_->canTransform(BASE, scene.rgb_model->frame, stamp));
  EXPECT_TRUE(buffer_->canTransform(ODOM, BASE, stamp));
}

TEST_F(Harness, GuidanceOffOverridesTheConfig)
{
  const Config on = guidance_config(Guidance::on);
  const Config off = guidance_config(Guidance::off);
  EXPECT_TRUE(on.guidance.enabled);
  EXPECT_FALSE(off.guidance.enabled);
  EXPECT_NE(off.guidance.depth_topic.find("/rp"), std::string::npos);  // topics are unique per test
  EXPECT_NE(off.guidance.depth_info_topic.find("/rp"), std::string::npos);
}

// --- the scenes through GroundNode -------------------------------------------------------------------------------

// The live-round acceptance (design D14-D19, approved items A10-A14) on the real recordings, over the frames after the
// warm-up, the gripper zone x < 0.37 excluded: the 5 cm box is never lethal (temporary, user 2026-10-23: floor band
// 40 mm, lethal step 60 mm; the box reads 41-45 mm, so its step warning share is printed, not asserted), with no ramp
// or step warning behind it, the board is never lethal, the black rubber ramp is never lethal (no ToF returns on the
// rubber, D19), no lethal point lies within 1.2 m outside the box in more than 5 % of the frames (the empty-floor p95
// goal) and the warning set changes in fewer than 10 % of the consecutive frames.
TEST_F(ReplayTest, HomeSceneMeetsTheLiveRoundAcceptance)
{
  const auto frames = replay_scene("home");
  std::cout << "replay home: box_warning_share=" << ground_replay::box_warning_share(frames) << std::endl;
  EXPECT_DOUBLE_EQ(ground_replay::box_lethal_share(frames), 0.0);
  EXPECT_EQ(ground_replay::far_side_warning_frames(frames), 0);
  EXPECT_EQ(ground_replay::board_lethal_frames(frames), 0);
  EXPECT_LE(ground_replay::stray_lethal_share(frames), MAX_STRAY_LETHAL_SHARE);
  EXPECT_LT(ground_replay::churn_share(frames), MAX_CHURN_SHARE);
}

TEST_F(ReplayTest, BoxSceneMeetsTheLiveRoundAcceptance)
{
  const auto frames = replay_scene("box");
  std::cout << "replay box: box_warning_share=" << ground_replay::box_warning_share(frames) << std::endl;
  EXPECT_DOUBLE_EQ(ground_replay::box_lethal_share(frames), 0.0);
  EXPECT_EQ(ground_replay::far_side_warning_frames(frames), 0);
  EXPECT_LE(ground_replay::stray_lethal_share(frames), MAX_STRAY_LETHAL_SHARE);
  EXPECT_LT(ground_replay::churn_share(frames), MAX_CHURN_SHARE);
}

TEST_F(ReplayTest, RampSceneMeetsTheLiveRoundAcceptance)
{
  const auto frames = replay_scene("ramp");
  EXPECT_EQ(ground_replay::board_lethal_frames(frames), 0);  // the board edge is in the ramp view
  EXPECT_EQ(ground_replay::ramp_lethal_frames(frames), 0);
  EXPECT_LE(ground_replay::stray_lethal_share(frames), MAX_STRAY_LETHAL_SHARE);
  EXPECT_LT(ground_replay::churn_share(frames), MAX_CHURN_SHARE);
}
