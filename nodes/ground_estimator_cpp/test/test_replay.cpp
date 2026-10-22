// The 2026-10-22 live recordings (test/data/replay/{home,box,ramp}.bin, scripts/ground_replay_export.py) replayed
// through GroundNode in-process with the shipped config/ground_estimator.yaml (design D14): reader tests, the replay
// metric functions of replay_fixture.hpp on hand-built frames, and one replay per scene that prints its metrics line.
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
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
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
constexpr double MIN_BOX_LETHAL_SHARE = 0.95;  // frames after warm-up with obstacle points on the box
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
    {"source", "tof"}};
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
  // (unique per test so no discovery or latched message crosses tests).
  Config shipped_config()
  {
    const YAML::Node node = YAML::LoadFile(std::string(GROUND_CONFIG_DIR) + "/ground_estimator.yaml");
    const auto errors = ground_estimator_cpp::validate(node);
    EXPECT_TRUE(errors.empty()) << (errors.empty() ? "" : errors.front());
    Config c = ground_estimator_cpp::from_yaml(node);
    const std::string prefix = "/rp" + std::to_string(g_test_index++);
    c.cloud_topic = prefix + "/tof/points_filtered";
    c.imu_topic = prefix + "/imu/data";
    c.rgbd_cloud_topic = prefix + "/head_rgbd/points";
    c.output_namespace = prefix + "/ground";
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
             warnings_sub_->get_publisher_count() > 0;
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
    return f;
  }

  // Replay a scene in real time: restamped to the wall clock keeping its relative timing, each cloud published at its
  // stamp after the IMU samples up to it, and its outputs awaited before the next cloud.
  std::vector<ReplayFrame> replay(const ReplayScene & scene)
  {
    start(shipped_config());
    std::vector<ReplayFrame> frames;
    if (scene.clouds.empty()) {
      return frames;
    }
    const int64_t offset_ns = wall_now_ns() + LEAD_NS - scene.clouds.front().stamp_ns;
    fill_tf(scene, offset_ns);
    std::size_t next_imu = 0;
    for (const auto & cloud : scene.clouds) {
      const int64_t stamp_ns = cloud.stamp_ns + offset_ns;
      const int64_t wait_ns = stamp_ns - wall_now_ns();
      if (wait_ns > 0) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(wait_ns));
      }
      for (; next_imu < scene.imu.size() && scene.imu[next_imu].stamp_ns + offset_ns <= stamp_ns; ++next_imu) {
        publish_imu(scene.imu[next_imu], scene.imu[next_imu].stamp_ns + offset_ns);
      }
      frames.push_back(run_frame(cloud, scene.frame_id, stamp_ns));
    }
    return frames;
  }

  // Replay one committed scene, check the node answered with a plane and print its metrics line.
  std::vector<ReplayFrame> replay_scene(const std::string & name)
  {
    const auto frames = replay(ground_replay::read_replay(replay_path(name)));
    EXPECT_TRUE(std::any_of(frames.begin(), frames.end(), [](const ReplayFrame & f) { return f.has_plane; }))
      << name << ": the node never answered with a plane";
    std::cout << ground_replay::metrics_line(name, frames) << std::endl;
    return frames;
  }

  Config config_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<GroundNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_, drop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr warnings_sub_;
  StampedInbox<sensor_msgs::msg::PointCloud2> obstacle_, drop_;
  StampedInbox<json> warnings_;
};

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
  bad_version[ground_replay::REPLAY_MAGIC_SIZE] = static_cast<char>(ground_replay::REPLAY_VERSION + 1);
  EXPECT_THROW(ground_replay::decode_replay(bad_version), std::runtime_error);
  EXPECT_THROW(ground_replay::decode_replay(bytes.substr(0, bytes.size() - 1)), std::runtime_error);
  EXPECT_THROW(ground_replay::decode_replay(bytes + "x"), std::runtime_error);
  EXPECT_THROW(ground_replay::read_replay(replay_path("missing")), std::runtime_error);
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

// --- the scenes through GroundNode -------------------------------------------------------------------------------

TEST_F(ReplayTest, HomeSceneReplaysThroughTheNode)
{
  const auto frames = replay_scene("home");
  EXPECT_GE(ground_replay::box_lethal_share(frames), MIN_BOX_LETHAL_SHARE);
  EXPECT_EQ(ground_replay::far_side_warning_frames(frames), 0);
  EXPECT_EQ(ground_replay::board_lethal_frames(frames), 0);
}

TEST_F(ReplayTest, BoxSceneReplaysThroughTheNode)
{
  const auto frames = replay_scene("box");
  EXPECT_GE(ground_replay::box_lethal_share(frames), MIN_BOX_LETHAL_SHARE);
  EXPECT_EQ(ground_replay::far_side_warning_frames(frames), 0);
}

TEST_F(ReplayTest, RampSceneReplaysThroughTheNode) { replay_scene("ramp"); }
