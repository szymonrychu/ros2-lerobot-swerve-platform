// GroundNode in-process: probe publishers and subscribers, a tf2_ros::Buffer fed with static and timed transforms
// (map -> odom -> base_link -> tof_optical_frame / head_camera_optical_frame / imu_link) and synthetic_floor.hpp
// scenes rendered in base_link and expressed in the sensor frames.
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <utility>
#include <vector>

#include "core/synthetic_floor.hpp"
#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/ground_node.hpp"

namespace
{

using ground_estimator_cpp::Config;
using ground_estimator_cpp::GroundNode;
using ground_test::DEG;
using ground_test::Scene;
using nlohmann::json;
using std::chrono::milliseconds;
using std::chrono::seconds;

constexpr const char * MAP = "map";
constexpr const char * ODOM = "odom";
constexpr const char * BASE = "base_link";
constexpr const char * TOF_FRAME = "tof_optical_frame";
constexpr const char * CAM_FRAME = "head_camera_optical_frame";
constexpr const char * IMU_FRAME = "imu_link";
constexpr double SIGMA_M = 0.001;
constexpr int DECIMATION = 2;
constexpr double TF_BEFORE_S = 5.0;
constexpr double TF_SPAN_S = 120.0;
constexpr double TF_STEP_S = 0.05;
constexpr double CLOUD_PERIOD_S = 0.1;
constexpr seconds WAIT{15};
constexpr milliseconds POLL{20};
constexpr milliseconds SETTLE{400};
constexpr double HEIGHT_TOL_M = 0.01;
constexpr double CENTROID_TOL_M = 0.1;
constexpr uint8_t GREY = 120;
constexpr int GREY_NOISE = 3;
constexpr int TRAIN_CLOUDS = 3;
constexpr uint8_t LABEL_MISMATCH = 3;
// A4 sheet flat on the floor in front of the robot.
constexpr double SHEET_X0 = 0.6, SHEET_X1 = 0.897, SHEET_Y0 = -0.105, SHEET_Y1 = 0.105;

std::atomic<int> g_test_index{0};

builtin_interfaces::msg::Time stamp_of(double t_s)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_s));
  t.nanosec = static_cast<uint32_t>(std::llround((t_s - std::floor(t_s)) * 1e9) % 1000000000LL);
  return t;
}

double seconds_of(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }

int64_t ns_of(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

double wall_now_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
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

// The value of one series of a Prometheus text rendering, nullopt when absent.
std::optional<double> series_value(const std::string & text, const std::string & series)
{
  const std::string key = "\n" + series + " ";
  const size_t at = ("\n" + text).find(key);
  if (at == std::string::npos) {
    return std::nullopt;
  }
  return std::stod(text.substr(at + key.size() - 1));
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
  std::vector<Msg> all()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return messages;
  }
};

// The cloud of an inbox with the given header stamp.
std::optional<sensor_msgs::msg::PointCloud2> find_stamp(
  Inbox<sensor_msgs::msg::PointCloud2> & inbox, const builtin_interfaces::msg::Time & stamp)
{
  for (const auto & m : inbox.all()) {
    if (m.header.stamp == stamp) {
      return m;
    }
  }
  return std::nullopt;
}

// The parsed JSON documents of a String inbox.
std::vector<json> documents(Inbox<std_msgs::msg::String> & inbox)
{
  std::vector<json> out;
  for (const auto & m : inbox.all()) {
    out.push_back(json::parse(m.data));
  }
  return out;
}

// The first document matching pred, if any.
std::optional<json> find_doc(Inbox<std_msgs::msg::String> & inbox, const std::function<bool(const json &)> & pred)
{
  for (const auto & d : documents(inbox)) {
    if (pred(d)) {
      return d;
    }
  }
  return std::nullopt;
}

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

std::vector<uint8_t> labels_of(const sensor_msgs::msg::PointCloud2 & msg)
{
  std::vector<uint8_t> out;
  for (const auto & f : msg.fields) {
    if (f.name == "label") {
      for (size_t i = 0; i < static_cast<size_t>(msg.width) * msg.height; ++i) {
        out.push_back(msg.data[i * msg.point_step + f.offset]);
      }
    }
  }
  return out;
}

// Optical frame pitched down by pitch_deg: optical z forward, x right, y down.
Eigen::Isometry3d optical_mount(const Eigen::Vector3d & origin, double pitch_deg)
{
  Eigen::Matrix3d optical;
  optical.col(0) = -Eigen::Vector3d::UnitY();
  optical.col(1) = -Eigen::Vector3d::UnitZ();
  optical.col(2) = Eigen::Vector3d::UnitX();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = origin;
  t.linear() = Eigen::AngleAxisd(pitch_deg * DEG, Eigen::Vector3d::UnitY()).toRotationMatrix() * optical;
  return t;
}

Eigen::Isometry3d planar_pose(double x, double y, double yaw_deg)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, 0.0);
  t.linear() = Eigen::AngleAxisd(yaw_deg * DEG, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return t;
}

const Eigen::Isometry3d BASE_T_TOF = optical_mount(Eigen::Vector3d(0.15, 0.0, 0.45), 30.0);
const Eigen::Isometry3d BASE_T_CAM = optical_mount(Eigen::Vector3d(0.17, -0.03, 0.47), 30.0);
const Eigen::Isometry3d BASE_T_IMU = planar_pose(0.0, 0.0, 90.0);
const Eigen::Isometry3d ODOM_T_BASE = planar_pose(1.0, 2.0, 30.0);
const Eigen::Isometry3d MAP_T_ODOM = planar_pose(5.0, -3.0, 90.0);

geometry_msgs::msg::TransformStamped transform_of(
  const Eigen::Isometry3d & parent_T_child, const std::string & parent, const std::string & child, double t_s)
{
  geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(parent_T_child);
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.header.stamp = stamp_of(t_s);
  return tf;
}

// --- scenes, rendered in base_link -------------------------------------------------------------------------------

pcl::PointCloud<pcl::PointXYZ> scene_cloud(const Scene & scene, unsigned seed = 1)
{
  return ground_test::render(scene, SIGMA_M, 0.0, seed, DECIMATION);
}

Scene box_scene()
{
  auto scene = ground_test::floor_scene();
  ground_test::add_box(scene, Eigen::Vector3d(1.0, 0.0, 0.05), Eigen::Vector3d(0.3, 0.4, 0.10));
  return scene;
}

Scene drop_scene(double depth_m)
{
  auto scene = ground_test::floor_until(1.0);
  ground_test::add_drop_off(scene, 1.0, depth_m);
  return scene;
}

// A door sill: a 15 cm deep strip of the given height across the view at x 1.0 .. 1.15.
Scene sill_scene(double height_m)
{
  auto scene = ground_test::floor_scene();
  ground_test::add_box(scene, Eigen::Vector3d(1.075, 0.0, height_m / 2.0), Eigen::Vector3d(0.15, 0.8, height_m));
  return scene;
}

Scene ramp_scene(double deg)
{
  auto scene = ground_test::floor_until(1.0);
  scene.surfaces.push_back(ground_test::plane_surface(
    Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-std::sin(deg * DEG), 0.0, std::cos(deg * DEG)),
    [](const Eigen::Vector3d & p) { return p.x() >= 1.0; }));
  return scene;
}

Scene wall_scene()
{
  Scene scene;
  ground_test::add_wall(scene, 1.0);
  return scene;
}

bool on_sheet(const Eigen::Vector3d & p)
{
  return p.x() >= SHEET_X0 && p.x() <= SHEET_X1 && p.y() >= SHEET_Y0 && p.y() <= SHEET_Y1;
}

// The centroid of a polygon's vertices.
Eigen::Vector2d centroid(const json & polygon)
{
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  for (const auto & v : polygon) {
    sum += Eigen::Vector2d(v[0].get<double>(), v[1].get<double>());
  }
  return polygon.empty() ? sum : sum / static_cast<double>(polygon.size());
}

// Frames until a static scene's cells hold a stable class: the temporal median outputs a cell from its min_frames-th
// value on, the class hysteresis then needs enter_frames votes.
int warm_up_frames(const Config & c)
{
  const int temporal = c.temporal.enabled ? c.temporal.min_frames - 1 : 0;
  return temporal + (c.hysteresis.enabled ? c.hysteresis.enter_frames : 1);
}

// --- multi-frame scenes, base_link grids with noise keyed by the fixed-frame (odom) cell -------------------------

constexpr double CELL_M = 0.10;  // temporal.cell_size_m default: the per-cell noise is drawn per fixed-frame cell
constexpr double FLOOR_NOISE_M = 0.006;
constexpr double FAR_BAND_X0 = 1.15, FAR_BAND_X1 = 1.45;
constexpr double FAR_BAND_LOW_M = 0.04;
constexpr double FAR_SPREAD_LO_M = 0.030, FAR_SPREAD_HI_M = 0.090;
constexpr int FAR_SPREAD_STEPS = 13;
constexpr double STABLE_BOX_X0 = 0.62, STABLE_BOX_X1 = 0.93, STABLE_BOX_Y0 = 0.10, STABLE_BOX_Y1 = 0.35;
constexpr double STABLE_BOX_HEIGHT_M = 0.05;
constexpr double STABLE_BOX_EDGE_BAND_M = 0.05;
constexpr double STABLE_BOX_EDGE_LO_M = 0.035, STABLE_BOX_EDGE_HI_M = 0.065;
constexpr double BOX_TOP_NOISE_M = 0.004;
constexpr double BOX_FLOOR_NOISE_M = 0.008;
constexpr double SILL_X0 = 1.0, SILL_X1 = 1.15, SILL_HALF_WIDTH_M = 0.4;
constexpr double SILL_HEIGHT_M = 0.03;
constexpr double SILL_JITTER_M = 0.01;
constexpr double SILL_NOISE_M = 0.003;
constexpr double MAT_X0 = 1.0, MAT_LENGTH_M = 0.25, MAT_HEIGHT_M = 0.063, MAT_Y0 = -0.45, MAT_Y1 = -0.20;
constexpr double MAT_GRID_M = 0.02;
constexpr double MAT_KEEP_FRACTION = 0.3;
constexpr double MAT_SIGMA_M = 0.015;
constexpr double PLATE_HEIGHT_M = 0.06;
constexpr double STEEP_MAT_DEG = 20.0;
// Seed sets of the noisy scenes: a result must hold for other noise draws too, not one lucky seed.
constexpr unsigned SEED_SET_STRIDE = 100000U;

// The fixed-frame cell of a base_link point (the robot stands still at ODOM_T_BASE).
std::pair<int, int> odom_cell(const pcl::PointXYZ & p)
{
  const Eigen::Vector3d o = ODOM_T_BASE * Eigen::Vector3d(p.x, p.y, 0.0);
  return {static_cast<int>(std::floor(o.x() / CELL_M)), static_cast<int>(std::floor(o.y() / CELL_M))};
}

// A seed per (cell, frame, salt), so every point of a cell shares the cell's per-frame value.
unsigned cell_seed(const std::pair<int, int> & cell, int frame, unsigned salt)
{
  return salt + static_cast<unsigned>(cell.first * 7919 + cell.second * 104729 + frame * 15485863);
}

// Empty floor x 0.3 .. 1.45 m (3 cm grid, 6 mm noise) whose far band x 1.15 .. 1.45 m reads like the measured far edge
// of the ToF returns: per frame each fixed-frame cell reads one value, uniform from +4 cm up with the cell's temporal
// std (spread over 30 .. 90 mm by cell; 43 mm spans +4 .. +19 cm), plus 3 mm per-point noise.
pcl::PointCloud<pcl::PointXYZ> far_band_floor(int frame, unsigned seed_set)
{
  std::mt19937 rng(7000U + SEED_SET_STRIDE * seed_set + static_cast<unsigned>(frame));
  std::normal_distribution<double> noise(0.0, FLOOR_NOISE_M);
  pcl::PointCloud<pcl::PointXYZ> cloud = ground_test::grid_cloud(0.3, FAR_BAND_X1, -0.5, 0.5, 0.03);
  for (auto & p : cloud) {
    if (p.x < FAR_BAND_X0) {
      p.z = static_cast<float>(noise(rng));
      continue;
    }
    const auto cell = odom_cell(p);
    const int step = (((cell.first * 7 + cell.second * 3) % FAR_SPREAD_STEPS) + FAR_SPREAD_STEPS) % FAR_SPREAD_STEPS;
    const double sigma = FAR_SPREAD_LO_M + (FAR_SPREAD_HI_M - FAR_SPREAD_LO_M) * step / (FAR_SPREAD_STEPS - 1.0);
    std::mt19937 cell_rng(cell_seed(cell, frame, 1U + SEED_SET_STRIDE * seed_set));
    const double value =
      FAR_BAND_LOW_M + std::uniform_real_distribution<double>(0.0, 2.0 * std::sqrt(3.0) * sigma)(cell_rng);
    p.z = static_cast<float>(value + 0.5 * noise(rng));
  }
  return cloud;
}

// The user's 5 cm box (x 0.62 .. 0.93, y 0.10 .. 0.35) on an 8 mm noise floor, 2 cm grid: the top reads 5 cm +- 4 mm,
// the robot-facing 5 cm edge band reads uniform 3.5 .. 6.5 cm per point.
pcl::PointCloud<pcl::PointXYZ> noisy_box(int frame)
{
  std::mt19937 rng(8000U + static_cast<unsigned>(frame));
  std::normal_distribution<double> floor_noise(0.0, BOX_FLOOR_NOISE_M);
  std::normal_distribution<double> top_noise(0.0, BOX_TOP_NOISE_M);
  std::uniform_real_distribution<double> edge(STABLE_BOX_EDGE_LO_M, STABLE_BOX_EDGE_HI_M);
  pcl::PointCloud<pcl::PointXYZ> cloud = ground_test::grid_cloud(0.3, 1.4, -0.4, 0.6, 0.02);
  for (auto & p : cloud) {
    const bool on_box = p.x >= STABLE_BOX_X0 && p.x <= STABLE_BOX_X1 && p.y >= STABLE_BOX_Y0 && p.y <= STABLE_BOX_Y1;
    if (!on_box) {
      p.z = static_cast<float>(floor_noise(rng));
    } else if (p.x < STABLE_BOX_X0 + STABLE_BOX_EDGE_BAND_M) {
      p.z = static_cast<float>(edge(rng));
    } else {
      p.z = static_cast<float>(STABLE_BOX_HEIGHT_M + top_noise(rng));
    }
  }
  return cloud;
}

// A 3 cm sill (x 1.0 .. 1.15, |y| <= 0.4) on the floor, 2 cm grid, 3 mm noise; the whole sill moves by one uniform
// offset in +-jitter_m per frame.
pcl::PointCloud<pcl::PointXYZ> jittered_sill(int frame, double jitter_m)
{
  std::mt19937 rng(9000U + static_cast<unsigned>(frame));
  std::normal_distribution<double> noise(0.0, SILL_NOISE_M);
  const double offset = jitter_m > 0.0 ? std::uniform_real_distribution<double>(-jitter_m, jitter_m)(rng) : 0.0;
  pcl::PointCloud<pcl::PointXYZ> cloud = ground_test::grid_cloud(0.3, 1.4, -0.5, 0.5, 0.02);
  for (auto & p : cloud) {
    const bool on_sill = p.x >= SILL_X0 && p.x <= SILL_X1 && std::fabs(p.y) <= SILL_HALF_WIDTH_M;
    p.z = static_cast<float>((on_sill ? SILL_HEIGHT_M + offset : 0.0) + noise(rng));
  }
  return cloud;
}

// The user's black rubber mat (part ramp-synthetic-scene): a 25 cm, 63 mm (14 deg) wedge at x 1.0 .. 1.25,
// y -0.45 .. -0.20 sampled on a 2 cm grid, 30 % of the returns kept per frame with 15 mm height noise, on a 3 cm,
// 4 mm noise floor. Seed set s draws other noise and returns (seeds offset by s * SEED_SET_STRIDE).
// slope_deg other than 0 raises the mat to that slope over the same length.
pcl::PointCloud<pcl::PointXYZ> sparse_mat_ramp(int frame, unsigned seed_set, double slope_deg = 0.0)
{
  Scene mat;
  const double height_m = slope_deg > 0.0 ? MAT_LENGTH_M * std::tan(slope_deg * DEG) : MAT_HEIGHT_M;
  ground_test::add_ramp(mat, MAT_X0, MAT_LENGTH_M, height_m, MAT_Y0, MAT_Y1);
  const auto dense = ground_test::sample_grid(mat, 0.95, 1.3, -0.5, -0.15, MAT_GRID_M);
  const auto seed = 11000U + SEED_SET_STRIDE * seed_set + static_cast<unsigned>(frame);
  pcl::PointCloud<pcl::PointXYZ> cloud =
    ground_test::add_z_noise(ground_test::keep_fraction(dense, MAT_KEEP_FRACTION, seed), MAT_SIGMA_M, 0.0, 0.0, seed);
  std::mt19937 rng(seed + 500U);
  std::normal_distribution<double> noise(0.0, 0.004);
  for (auto p : ground_test::grid_cloud(0.3, 1.45, -0.6, 0.4, 0.03)) {
    const bool under_mat = p.x >= MAT_X0 && p.x <= MAT_X0 + MAT_LENGTH_M && p.y >= MAT_Y0 && p.y <= MAT_Y1;
    if (!under_mat) {
      p.z = static_cast<float>(noise(rng));
      cloud.push_back(p);
    }
  }
  return cloud;
}

// Floor scene cloud with a fresh noise seed per frame.
pcl::PointCloud<pcl::PointXYZ> floor_frame(int frame)
{
  return ground_test::render(ground_test::floor_scene(), SIGMA_M, 0.0, 600U + static_cast<unsigned>(frame), DECIMATION);
}

// What the node published for one ToF cloud.
struct FrameOutput
{
  builtin_interfaces::msg::Time stamp;
  std::size_t obstacle = 0;
  std::size_t drop = 0;
  json warnings;  // the /ground/warning_areas document of the stamp (null when none arrived)
};

bool has_kind(const json & areas, const std::string & kind, const std::string & source)
{
  return std::any_of(areas.begin(), areas.end(), [&](const json & a) {
    return a["kind"] == kind && (source.empty() || a["source"] == source);
  });
}

// Which optional transforms the buffer gets.
struct TfOptions
{
  bool tof = true;  // base_link -> tof_optical_frame
  bool map = true;  // map -> odom
};

class NodeTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  // Unique topics per test so no discovery or latched message crosses tests.
  Config base_config()
  {
    const std::string prefix = "/gt" + std::to_string(g_test_index++);
    Config c;
    c.metrics_port = std::nullopt;
    c.cloud_topic = prefix + "/tof/points_filtered";
    c.imu_topic = prefix + "/imu/data";
    c.rgbd_cloud_topic = prefix + "/head_rgbd/points";
    c.output_namespace = prefix + "/ground";
    c.imu.level_reference_roll_deg = 0.0;
    c.imu.level_reference_pitch_deg = 0.0;
    c.temporal.enabled = false;  // the static scenes below are single clouds; the temporal tests switch it on
    return c;
  }

  void fill_tf(const TfOptions & opt = TfOptions())
  {
    t0_ = wall_now_s();
    buffer_ = std::make_shared<tf2_ros::Buffer>(
      std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), tf2::Duration(std::chrono::seconds(300)));
    if (opt.tof) {
      buffer_->setTransform(transform_of(BASE_T_TOF, BASE, TOF_FRAME, 0.0), "test", true);
    }
    buffer_->setTransform(transform_of(BASE_T_CAM, BASE, CAM_FRAME, 0.0), "test", true);
    buffer_->setTransform(transform_of(BASE_T_IMU, BASE, IMU_FRAME, 0.0), "test", true);
    if (opt.map) {
      buffer_->setTransform(transform_of(MAP_T_ODOM, MAP, ODOM, 0.0), "test", true);
    }
    for (double t = t0_ - TF_BEFORE_S; t <= t0_ + TF_SPAN_S; t += TF_STEP_S) {
      buffer_->setTransform(transform_of(ODOM_T_BASE, ODOM, BASE, t), "test", false);
    }
  }

  void start(const Config & config)
  {
    config_ = config;
    node_ = std::make_shared<GroundNode>(config, buffer_);
    probe_ = std::make_shared<rclcpp::Node>("ground_probe_" + std::to_string(g_test_index.load()));
    cloud_pub_ =
      probe_->create_publisher<sensor_msgs::msg::PointCloud2>(config.cloud_topic, rclcpp::QoS(10).reliable());
    rgbd_pub_ =
      probe_->create_publisher<sensor_msgs::msg::PointCloud2>(config.rgbd_cloud_topic, rclcpp::QoS(10).best_effort());
    imu_pub_ = probe_->create_publisher<sensor_msgs::msg::Imu>(config.imu_topic, rclcpp::SensorDataQoS());
    const auto reliable = rclcpp::QoS(50).reliable();
    obstacle_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.obstacle_points(), reliable,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { obstacle_.add(*m); });
    drop_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.drop_points(), reliable, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { drop_.add(*m); });
    labelled_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.labelled_points(), rclcpp::QoS(50).best_effort(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { labelled_.add(*m); });
    warnings_sub_ = probe_->create_subscription<std_msgs::msg::String>(
      config.warning_areas(), reliable, [this](std_msgs::msg::String::ConstSharedPtr m) { warnings_.add(*m); });
    state_sub_ = probe_->create_subscription<std_msgs::msg::String>(
      config.state_topic(), rclcpp::QoS(50).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) { state_.add(*m); });
    executor_.add_node(node_);
    executor_.add_node(probe_);
    spinner_ = std::thread([this] { executor_.spin(); });
    ASSERT_TRUE(wait_for([this] {
      return cloud_pub_->get_subscription_count() > 0 && imu_pub_->get_subscription_count() > 0 &&
             obstacle_sub_->get_publisher_count() > 0 && state_sub_->get_publisher_count() > 0;
    }));
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  // A scene cloud in the ToF frame, stamped now; returns the stamp.
  builtin_interfaces::msg::Time publish_cloud(const pcl::PointCloud<pcl::PointXYZ> & base_cloud)
  {
    const Eigen::Isometry3f tof_T_base = BASE_T_TOF.inverse().cast<float>();
    pcl::PointCloud<pcl::PointXYZ> tof;
    for (const auto & p : base_cloud) {
      const Eigen::Vector3f q = tof_T_base * Eigen::Vector3f(p.x, p.y, p.z);
      tof.push_back(pcl::PointXYZ(q.x(), q.y(), q.z()));
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(tof, msg);
    msg.header.frame_id = TOF_FRAME;
    msg.header.stamp = next_stamp();
    cloud_pub_->publish(msg);
    return msg.header.stamp;
  }

  // Publish and wait until the node answered on obstacle_points (a cloud with a plane) or the timeout.
  builtin_interfaces::msg::Time cloud_and_wait(const pcl::PointCloud<pcl::PointXYZ> & base_cloud)
  {
    const auto stamp = publish_cloud(base_cloud);
    EXPECT_TRUE(wait_for([&] { return find_stamp(obstacle_, stamp).has_value(); }));
    return stamp;
  }

  // Publish the cloud warm_up_frames(config_) times, waiting for each answer; returns the last stamp, whose output
  // comes from stable cell classes.
  builtin_interfaces::msg::Time warm_cloud_and_wait(const pcl::PointCloud<pcl::PointXYZ> & base_cloud)
  {
    builtin_interfaces::msg::Time stamp;
    for (int i = 0; i < warm_up_frames(config_); ++i) {
      stamp = cloud_and_wait(base_cloud);
    }
    return stamp;
  }

  // Publish one cloud and collect the lethal point counts and the warning areas document of its stamp.
  FrameOutput run_frame(const pcl::PointCloud<pcl::PointXYZ> & base_cloud)
  {
    FrameOutput out;
    out.stamp = cloud_and_wait(base_cloud);
    EXPECT_TRUE(wait_for([&] { return find_stamp(drop_, out.stamp).has_value(); }));
    const auto obstacle = find_stamp(obstacle_, out.stamp);
    const auto drop = find_stamp(drop_, out.stamp);
    out.obstacle = obstacle ? xyz_of(*obstacle).size() : 0;
    out.drop = drop ? xyz_of(*drop).size() : 0;
    const auto doc = warnings_at(out.stamp);
    out.warnings = doc ? *doc : json();
    return out;
  }

  // Run frames 0 .. warm_up + frames - 1 of a scene and return the outputs after the warm-up (warm_up_frames when
  // negative).
  std::vector<FrameOutput> run_scene(
    const std::function<pcl::PointCloud<pcl::PointXYZ>(int)> & scene, int frames, int warm_up = -1)
  {
    std::vector<FrameOutput> out;
    warm_up = warm_up < 0 ? warm_up_frames(config_) : warm_up;
    for (int i = 0; i < warm_up + frames; ++i) {
      FrameOutput f = run_frame(scene(i));
      if (i >= warm_up) {
        out.push_back(std::move(f));
      }
    }
    return out;
  }

  // Start a node with the temporal filter on and run a scene through run_scene.
  std::vector<FrameOutput> run_temporal_scene(
    const std::function<pcl::PointCloud<pcl::PointXYZ>(int)> & scene, int frames)
  {
    fill_tf();
    Config config = base_config();
    config.temporal.enabled = true;
    start(config);
    return run_scene(scene, frames);
  }

  // Train the floor colour model on grey floor RGB-D clouds, then show the red sheet (lifted by sheet_height_m).
  void show_red_sheet(double sheet_height_m)
  {
    ASSERT_TRUE(wait_for([this] { return rgbd_pub_->get_subscription_count() > 0; }));
    cloud_and_wait(scene_cloud(ground_test::floor_scene()));
    for (int i = 0; i < TRAIN_CLOUDS; ++i) {
      const auto stamp = publish_rgbd(false);
      ASSERT_TRUE(wait_for([&] { return find_stamp(labelled_, stamp).has_value(); }));
    }
    const auto sheet_stamp = publish_rgbd(true, sheet_height_m);
    ASSERT_TRUE(wait_for([&] { return find_stamp(labelled_, sheet_stamp).has_value(); }));
  }

  // An RGB-D cloud (x, y, z, rgb as head_rgbd_cpp publishes it) of the floor scene in the camera frame: grey floor
  // with a little texture, a red sheet when with_sheet (a plate of that height when sheet_height_m > 0).
  builtin_interfaces::msg::Time publish_rgbd(bool with_sheet, double sheet_height_m = 0.0)
  {
    auto rgbd_scene = ground_test::floor_scene();
    if (with_sheet && sheet_height_m > 0.0) {
      ground_test::add_table_top(rgbd_scene, sheet_height_m, SHEET_X0, SHEET_X1, SHEET_Y0, SHEET_Y1);
    }
    const auto base = scene_cloud(rgbd_scene, 5);
    const Eigen::Isometry3d cam_T_base = BASE_T_CAM.inverse();
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = CAM_FRAME;
    msg.header.stamp = next_stamp();
    msg.height = 1;
    msg.width = static_cast<uint32_t>(base.size());
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(base.size());
    sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> r(msg, "r"), g(msg, "g"), b(msg, "b");
    size_t i = 0;
    for (const auto & p : base) {
      const Eigen::Vector3d pb(p.x, p.y, p.z);
      const Eigen::Vector3d pc = cam_T_base * pb;
      *x = static_cast<float>(pc.x());
      *y = static_cast<float>(pc.y());
      *z = static_cast<float>(pc.z());
      if (with_sheet && on_sheet(pb)) {
        *r = 200;
        *g = 30;
        *b = 30;
      } else {
        const auto jitter = static_cast<uint8_t>(GREY + static_cast<int>(i % (2 * GREY_NOISE + 1)) - GREY_NOISE);
        *r = *g = *b = jitter;
      }
      ++x, ++y, ++z, ++r, ++g, ++b, ++i;
    }
    rgbd_pub_->publish(msg);
    return msg.header.stamp;
  }

  void publish_imu(double stamp_s, double cov0 = 0.01, double w = 1.0)
  {
    sensor_msgs::msg::Imu m;
    m.header.frame_id = IMU_FRAME;
    m.header.stamp = stamp_of(stamp_s);
    m.orientation.w = w;
    m.orientation_covariance[0] = cov0;
    imu_pub_->publish(m);
  }

  // Stamps strictly increase and follow the wall clock (the state timer works on now).
  builtin_interfaces::msg::Time next_stamp()
  {
    last_stamp_s_ = std::max(wall_now_s(), last_stamp_s_ + 0.001);
    return stamp_of(last_stamp_s_);
  }

  std::optional<double> metric(const std::string & series) { return series_value(node_->metrics().render(), series); }
  double dropped(const std::string & reason)
  {
    return metric("ground_clouds_dropped_total{reason=\"" + reason + "\"}").value_or(0.0);
  }

  // The newest state document, if any.
  std::optional<json> last_state()
  {
    const auto docs = documents(state_);
    if (docs.empty()) {
      return std::nullopt;
    }
    return docs.back();
  }

  // Waits for a state document with stamp >= stamp_ns matching pred; returns it.
  std::optional<json> state_after(int64_t stamp_ns, const std::function<bool(const json &)> & pred)
  {
    std::optional<json> found;
    wait_for([&] {
      found = find_doc(state_, [&](const json & d) { return d["stamp_ns"].get<int64_t>() >= stamp_ns && pred(d); });
      return found.has_value();
    });
    return found;
  }

  std::optional<json> warnings_at(const builtin_interfaces::msg::Time & stamp)
  {
    std::optional<json> found;
    wait_for([&] {
      found = find_doc(warnings_, [&](const json & d) { return d["stamp_ns"].get<int64_t>() == ns_of(stamp); });
      return found.has_value();
    });
    return found;
  }

  double t0_ = 0.0;
  double last_stamp_s_ = 0.0;
  Config config_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<GroundNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgbd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_, drop_sub_, labelled_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr warnings_sub_, state_sub_;
  Inbox<sensor_msgs::msg::PointCloud2> obstacle_, drop_, labelled_;
  Inbox<std_msgs::msg::String> warnings_, state_;
};

bool has_area(const json & doc, const std::string & kind)
{
  for (const auto & a : doc["areas"]) {
    if (a["kind"] == kind) {
      return true;
    }
  }
  return false;
}

// Noisy grid floor of the core cell-median test (sigma 2.6 cm, 20 % outliers of +-6 cm) with an optional box.
constexpr double NOISY_SIGMA_M = 0.026;
constexpr double NOISY_OUTLIER_FRACTION = 0.2;
constexpr double NOISY_OUTLIER_RANGE_M = 0.06;
constexpr double NOISY_GRID_STEP_M = 0.04;
constexpr int NOISY_FRAMES = 20;
constexpr double BOX_HEIGHT_M = 0.30;

pcl::PointCloud<pcl::PointXYZ> noisy_floor(unsigned seed, bool with_box)
{
  pcl::PointCloud<pcl::PointXYZ> truth;
  for (const auto & p : ground_test::grid_cloud(-1.0, 1.0, -1.0, 1.0, NOISY_GRID_STEP_M)) {
    const bool in_box = p.x >= 0.85 && p.x <= 1.15 && p.y >= -0.2 && p.y <= 0.2;
    if (!with_box || !in_box) {
      truth.push_back(p);
    }
  }
  if (with_box) {
    for (const auto & p : ground_test::grid_cloud(0.85, 1.15, -0.2, 0.2, NOISY_GRID_STEP_M, BOX_HEIGHT_M)) {
      truth.push_back(p);
    }
  }
  return ground_test::add_z_noise(truth, NOISY_SIGMA_M, NOISY_OUTLIER_FRACTION, NOISY_OUTLIER_RANGE_M, seed);
}

}  // namespace

TEST_F(NodeTest, NoStateBeforeTheFirstCloud)
{
  fill_tf();
  start(base_config());
  // Several state periods (2 Hz) without any ToF cloud: no state from no data.
  std::this_thread::sleep_for(milliseconds(1500));
  EXPECT_EQ(state_.size(), 0u);
  const auto stamp = cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  EXPECT_TRUE(state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); }).has_value());
}

TEST_F(NodeTest, LevelFloorWithABoxGivesObstaclePointsAndAPlaneState)
{
  fill_tf();
  start(base_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(box_scene()));

  const auto obstacle = find_stamp(obstacle_, stamp);
  ASSERT_TRUE(obstacle.has_value());
  EXPECT_EQ(obstacle->header.frame_id, BASE);
  const auto points = xyz_of(*obstacle);
  ASSERT_GT(points.size(), 50u);
  for (const auto & p : points) {
    // The box spans x 0.85 .. 1.15, y -0.2 .. 0.2, z 0 .. 0.1 in base_link: the cloud keeps base_link coordinates.
    EXPECT_NEAR(p.x(), 1.0, 0.2);
    EXPECT_NEAR(p.y(), 0.0, 0.25);
    EXPECT_GT(p.z(), 0.02);
    EXPECT_LT(p.z(), 0.12);
  }
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  const auto drop = find_stamp(drop_, stamp);
  EXPECT_EQ(drop->width * drop->height, 0u);
  EXPECT_EQ(drop->header.frame_id, BASE);

  const auto state = state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ((*state)["frame"], BASE);
  EXPECT_NEAR((*state)["plane"]["height_m"].get<double>(), 0.0, HEIGHT_TOL_M);
  EXPECT_TRUE((*state)["reason"].is_null());
  EXPECT_GT((*state)["obstacle_points"].get<int>(), 50);
  EXPECT_EQ((*state)["drop_points"].get<int>(), 0);
  EXPECT_FALSE((*state)["plane_vs_base_deg"].is_null());
  EXPECT_LT((*state)["plane_vs_base_deg"].get<double>(), 1.0);

  // ground-metrics: counters and histogram registered, gauges set from the real values.
  EXPECT_GE(metric("ground_fits_total{result=\"ok\"}").value_or(0.0), 1.0);
  EXPECT_TRUE(metric("ground_fits_total{result=\"no_plane\"}").has_value());
  EXPECT_GE(metric("ground_fit_seconds_count").value_or(0.0), 1.0);
  EXPECT_GT(metric("ground_inlier_ratio").value_or(0.0), 0.2);
  EXPECT_GT(metric("ground_obstacle_points").value_or(0.0), 50.0);
  EXPECT_EQ(metric("ground_drop_points").value_or(-1.0), 0.0);
  EXPECT_TRUE(metric("ground_plane_vs_base_deg").has_value());
  EXPECT_TRUE(metric("ground_regions").has_value());
  EXPECT_TRUE(metric("ground_warning_areas").has_value());
  EXPECT_TRUE(metric("ground_labelled_clouds_total").has_value());
  for (const char * reason : {"tf", "no_plane_rgbd", "no_map_tf", "bad_input"}) {
    EXPECT_EQ(dropped(reason), 0.0) << reason;
    EXPECT_TRUE(metric(std::string("ground_clouds_dropped_total{reason=\"") + reason + "\"}").has_value());
  }
  EXPECT_EQ(metric("robot_node_info{node=\"ground_estimator\"}").value_or(0.0), 1.0);
  EXPECT_NE(node_->metrics().render().find("process_cpu_seconds_total"), std::string::npos);
  // No slope without a fresh IMU and no colour ratio without an RGB-D cloud: those gauges have no sample.
  EXPECT_FALSE(metric("ground_slope_deg").has_value());
  EXPECT_FALSE(metric("ground_colour_mismatch_ratio").has_value());
}

TEST_F(NodeTest, TenCentimetreDropOffIsALethalDropRegion)
{
  fill_tf();
  start(base_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(drop_scene(0.10)));
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  const auto drop = find_stamp(drop_, stamp);
  const auto points = xyz_of(*drop);
  ASSERT_GT(points.size(), 50u);
  for (const auto & p : points) {
    EXPECT_GT(p.x(), 1.0);
    EXPECT_NEAR(p.z(), -0.10, 0.02);
  }
  const auto state = state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  bool lethal_drop = false;
  for (const auto & r : (*state)["regions"]) {
    lethal_drop = lethal_drop || (r["kind"] == "drop" && r["class"] == "obstacle");
  }
  EXPECT_TRUE(lethal_drop) << state->dump();
  EXPECT_GT((*state)["drop_points"].get<int>(), 50);
}

TEST_F(NodeTest, ThreeCentimetreSillIsAStepWarningInTheMapFrame)
{
  fill_tf();
  start(base_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(sill_scene(0.03)));
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, stamp)).size(), 0u);
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  EXPECT_EQ(xyz_of(*find_stamp(drop_, stamp)).size(), 0u);

  const auto doc = warnings_at(stamp);
  ASSERT_TRUE(doc.has_value());
  EXPECT_EQ((*doc)["frame"], MAP);
  ASSERT_EQ((*doc)["areas"].size(), 1u) << doc->dump();
  const auto & area = (*doc)["areas"][0];
  EXPECT_EQ(area["kind"], "step");
  EXPECT_EQ(area["source"], "tof");
  EXPECT_NEAR(area["height_m"].get<double>(), 0.03, 0.01);
  // The sill centre (1.075, 0) in base_link, through map <- odom <- base_link.
  const Eigen::Vector3d expected = MAP_T_ODOM * ODOM_T_BASE * Eigen::Vector3d(1.075, 0.0, 0.0);
  const Eigen::Vector2d c = centroid(area["polygon"]);
  EXPECT_NEAR(c.x(), expected.x(), CENTROID_TOL_M);
  EXPECT_NEAR(c.y(), expected.y(), CENTROID_TOL_M);

  const auto state = state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  ASSERT_EQ((*state)["warnings"].size(), 1u);
  EXPECT_EQ((*state)["warnings"][0]["kind"], "step");
  bool step_warning = false;
  for (const auto & r : (*state)["regions"]) {
    step_warning = step_warning || (r["kind"] == "step" && r["class"] == "warning");
  }
  EXPECT_TRUE(step_warning) << state->dump();
  EXPECT_EQ((*state)["obstacle_points"].get<int>(), 0);
  EXPECT_EQ(metric("ground_warning_areas").value_or(0.0), 1.0);
}

TEST_F(NodeTest, FourteenDegreeRampIsARampWarningWithoutLethalPoints)
{
  fill_tf();
  start(base_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(ramp_scene(14.0)));
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, stamp)).size(), 0u);
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  EXPECT_EQ(xyz_of(*find_stamp(drop_, stamp)).size(), 0u);
  const auto doc = warnings_at(stamp);
  ASSERT_TRUE(doc.has_value());
  EXPECT_TRUE(has_area(*doc, "ramp")) << doc->dump();
}

TEST_F(NodeTest, NoMapTransformGivesNoWarningAreasAndCountsIt)
{
  fill_tf({true, false});
  start(base_config());
  const auto stamp = cloud_and_wait(scene_cloud(sill_scene(0.03)));
  ASSERT_TRUE(wait_for([&] { return dropped("no_map_tf") >= 1.0; }));
  std::this_thread::sleep_for(SETTLE);
  EXPECT_EQ(warnings_.size(), 0u);
  // The plane-relative clouds do not need the map.
  EXPECT_TRUE(find_stamp(obstacle_, stamp).has_value());
}

TEST_F(NodeTest, WallOnlyGivesNoCloudAndAPlaneNullState)
{
  fill_tf();
  start(base_config());
  const auto stamp = publish_cloud(scene_cloud(wall_scene()));
  const auto state = state_after(ns_of(stamp), [](const json &) { return true; });
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE((*state)["plane"].is_null());
  // The spec literals: a wall is either a plane whose normal is off the prior or no plane at all.
  ASSERT_TRUE((*state)["reason"].is_string()) << state->dump();
  const auto reason = (*state)["reason"].get<std::string>();
  EXPECT_TRUE(reason == "normal_off_prior" || reason == "no_plane") << reason;
  std::this_thread::sleep_for(SETTLE);
  EXPECT_EQ(obstacle_.size(), 0u);
  EXPECT_EQ(drop_.size(), 0u);
  EXPECT_EQ(warnings_.size(), 0u);
  EXPECT_GE(metric("ground_fits_total{result=\"" + (*state)["reason"].get<std::string>() + "\"}").value_or(0.0), 1.0);
}

TEST_F(NodeTest, PlaneIsHeldForHoldSecondsThenNull)
{
  fill_tf();
  Config config = base_config();
  config.fit.hold_s = 0.5;
  start(config);
  const auto floor_stamp = cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  ASSERT_TRUE(state_after(ns_of(floor_stamp), [](const json & d) { return !d["plane"].is_null(); }).has_value());

  const auto wall = scene_cloud(wall_scene());
  std::vector<builtin_interfaces::msg::Time> wall_stamps;
  for (int i = 0; i < 12; ++i) {
    wall_stamps.push_back(publish_cloud(wall));
    std::this_thread::sleep_for(milliseconds(static_cast<int>(CLOUD_PERIOD_S * 1000)));
  }
  const auto state = state_after(ns_of(wall_stamps.back()), [](const json & d) { return d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE((*state)["reason"].is_null());
  std::this_thread::sleep_for(SETTLE);
  // Wall clouds within hold_s of the floor fit are segmented against the held plane; later ones publish nothing.
  const double limit_s = seconds_of(floor_stamp) + config.fit.hold_s;
  size_t held = 0;
  for (const auto & m : obstacle_.all()) {
    EXPECT_LE(seconds_of(m.header.stamp), limit_s + 1e-6);
    held += m.header.stamp == floor_stamp ? 0 : 1;
  }
  EXPECT_GE(held, 1u);
  for (const auto & m : drop_.all()) {
    EXPECT_LE(seconds_of(m.header.stamp), limit_s + 1e-6);
  }
}

TEST_F(NodeTest, MissingTransformDropsTheCloud)
{
  fill_tf({false, true});
  start(base_config());
  const auto stamp = publish_cloud(scene_cloud(box_scene()));
  ASSERT_TRUE(wait_for([&] { return dropped("tf") >= 1.0; }));
  const auto state = state_after(ns_of(stamp), [](const json &) { return true; });
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE((*state)["plane"].is_null());
  EXPECT_EQ((*state)["reason"], "no_tf");
  std::this_thread::sleep_for(SETTLE);
  EXPECT_EQ(obstacle_.size(), 0u);
  EXPECT_EQ(drop_.size(), 0u);
  EXPECT_EQ(warnings_.size(), 0u);
  EXPECT_EQ(metric("ground_fits_total{result=\"ok\"}").value_or(0.0), 0.0);
}

TEST_F(NodeTest, FreshImuIsThePrior)
{
  fill_tf();
  start(base_config());
  publish_imu(wall_now_s());
  std::this_thread::sleep_for(SETTLE);
  const auto stamp = cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  const auto state = state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ((*state)["prior"], "imu");
  EXPECT_TRUE((*state)["imu_ok"].get<bool>());
  ASSERT_FALSE((*state)["slope_deg"].is_null());
  EXPECT_LT((*state)["slope_deg"].get<double>(), 1.0);
  EXPECT_EQ((*state)["slope_level"], "ok");
  EXPECT_TRUE(metric("ground_slope_deg").has_value());
}

TEST_F(NodeTest, StaleImuFallsBackToBaseLink)
{
  fill_tf();
  start(base_config());
  publish_imu(wall_now_s() - 2.0);
  std::this_thread::sleep_for(SETTLE);
  const auto stamp = cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  const auto state = state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ((*state)["prior"], "base_link");
  EXPECT_FALSE((*state)["imu_ok"].get<bool>());
  EXPECT_TRUE((*state)["slope_deg"].is_null());
}

TEST_F(NodeTest, ImuWithoutOrientationIsNotUsed)
{
  fill_tf();
  start(base_config());
  publish_imu(wall_now_s() + 0.2, -1.0);
  std::this_thread::sleep_for(SETTLE);
  auto stamp = cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  auto state = state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ((*state)["prior"], "base_link");

  // A quaternion that is not normalised is not used either.
  publish_imu(wall_now_s() + 0.2, 0.01, 2.0);
  std::this_thread::sleep_for(SETTLE);
  stamp = cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  state = state_after(ns_of(stamp), [](const json &) { return true; });
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ((*state)["prior"], "base_link");
}

TEST_F(NodeTest, RgbdCloudWithARedSheetIsLabelledAndGivesAColourWarningArea)
{
  fill_tf();
  start(base_config());
  ASSERT_TRUE(wait_for([this] { return rgbd_pub_->get_subscription_count() > 0; }));
  cloud_and_wait(scene_cloud(ground_test::floor_scene()));

  // Train the floor colour model on grey floor clouds, then show the sheet.
  for (int i = 0; i < TRAIN_CLOUDS; ++i) {
    const auto stamp = publish_rgbd(false);
    ASSERT_TRUE(wait_for([&] { return find_stamp(labelled_, stamp).has_value(); }));
  }
  const auto sheet_stamp = publish_rgbd(true);
  ASSERT_TRUE(wait_for([&] { return find_stamp(labelled_, sheet_stamp).has_value(); }));
  const auto labelled = *find_stamp(labelled_, sheet_stamp);
  EXPECT_EQ(labelled.header.frame_id, BASE);
  const auto labels = labels_of(labelled);
  const auto xyz = xyz_of(labelled);
  ASSERT_EQ(labels.size(), xyz.size());
  size_t mismatch = 0;
  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels[i] == LABEL_MISMATCH) {
      ++mismatch;
      // The labelled cloud is in base_link: every flagged point lies on the sheet.
      EXPECT_TRUE(on_sheet(xyz[i].cast<double>()));
    }
  }
  EXPECT_GT(mismatch, 20u);
  EXPECT_GE(metric("ground_labelled_clouds_total").value_or(0.0), TRAIN_CLOUDS + 1.0);
  EXPECT_TRUE(metric("ground_colour_mismatch_ratio").has_value());

  const auto stamp = warm_cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  const auto doc = warnings_at(stamp);
  ASSERT_TRUE(doc.has_value());
  bool colour = false;
  for (const auto & a : (*doc)["areas"]) {
    if (a["kind"] == "colour" && a["source"] == "colour") {
      colour = true;
      const Eigen::Vector3d expected =
        MAP_T_ODOM * ODOM_T_BASE * Eigen::Vector3d((SHEET_X0 + SHEET_X1) / 2.0, (SHEET_Y0 + SHEET_Y1) / 2.0, 0.0);
      const Eigen::Vector2d c = centroid(a["polygon"]);
      EXPECT_NEAR(c.x(), expected.x(), CENTROID_TOL_M);
      EXPECT_NEAR(c.y(), expected.y(), CENTROID_TOL_M);
    }
  }
  EXPECT_TRUE(colour) << doc->dump();
  // The flat sheet is a warning, never lethal.
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, stamp)).size(), 0u);
  const auto state = state_after(ns_of(stamp), [](const json & d) { return !d["colour"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE((*state)["colour"]["model_ok"].get<bool>());
}

TEST_F(NodeTest, RgbdCloudWithoutAPlaneIsNotLabelled)
{
  fill_tf();
  start(base_config());
  ASSERT_TRUE(wait_for([this] { return rgbd_pub_->get_subscription_count() > 0; }));
  publish_rgbd(false);
  ASSERT_TRUE(wait_for([&] { return dropped("no_plane_rgbd") >= 1.0; }));
  std::this_thread::sleep_for(SETTLE);
  EXPECT_EQ(labelled_.size(), 0u);
  EXPECT_EQ(state_.size(), 0u);
}

namespace
{
// A cloud of zeroed points with the given fields, all of one datatype (FLOAT32 or FLOAT64).
sensor_msgs::msg::PointCloud2 cloud_with_fields(
  const std::string & frame, const builtin_interfaces::msg::Time & stamp, const std::vector<std::string> & names,
  uint8_t datatype)
{
  constexpr uint32_t POINTS = 10;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame;
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = POINTS;
  const uint32_t size = datatype == sensor_msgs::msg::PointField::FLOAT64 ? 8u : 4u;
  for (const auto & name : names) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = static_cast<uint32_t>(msg.fields.size()) * size;
    f.datatype = datatype;
    f.count = 1;
    msg.fields.push_back(f);
  }
  msg.point_step = static_cast<uint32_t>(names.size()) * size;
  msg.row_step = msg.point_step * POINTS;
  msg.is_dense = true;
  msg.data.assign(msg.row_step, 0);
  return msg;
}
}  // namespace

TEST_F(NodeTest, TofCloudWithoutFloat32XyzIsDroppedAsBadInput)
{
  fill_tf();
  start(base_config());
  cloud_pub_->publish(
    cloud_with_fields(TOF_FRAME, next_stamp(), {"x", "y", "z"}, sensor_msgs::msg::PointField::FLOAT64));
  ASSERT_TRUE(wait_for([&] { return dropped("bad_input") >= 1.0; }));
  std::this_thread::sleep_for(SETTLE);
  EXPECT_EQ(obstacle_.size(), 0u);
  EXPECT_EQ(drop_.size(), 0u);
  EXPECT_EQ(warnings_.size(), 0u);
  // Unreadable input is no data: no state and no fit.
  EXPECT_EQ(state_.size(), 0u);
  EXPECT_EQ(metric("ground_fits_total{result=\"ok\"}").value_or(0.0), 0.0);
  EXPECT_EQ(dropped("tf"), 0.0);
}

TEST_F(NodeTest, RgbdCloudWithoutRgbIsDroppedAsBadInput)
{
  fill_tf();
  start(base_config());
  ASSERT_TRUE(wait_for([this] { return rgbd_pub_->get_subscription_count() > 0; }));
  cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  rgbd_pub_->publish(
    cloud_with_fields(CAM_FRAME, next_stamp(), {"x", "y", "z"}, sensor_msgs::msg::PointField::FLOAT32));
  ASSERT_TRUE(wait_for([&] { return dropped("bad_input") >= 1.0; }));
  std::this_thread::sleep_for(SETTLE);
  EXPECT_EQ(labelled_.size(), 0u);
  EXPECT_EQ(metric("ground_labelled_clouds_total").value_or(0.0), 0.0);
  EXPECT_EQ(dropped("no_plane_rgbd"), 0.0);
  EXPECT_EQ(dropped("tf"), 0.0);
}

TEST_F(NodeTest, RgbdIsNotSubscribedWhenColourIsDisabled)
{
  fill_tf();
  Config config = base_config();
  config.colour.enabled = false;
  start(config);
  std::this_thread::sleep_for(SETTLE);
  EXPECT_EQ(rgbd_pub_->get_subscription_count(), 0u);
}

TEST_F(NodeTest, NoisyFlatFloorGivesNoObstacleOrDropPoints)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  builtin_interfaces::msg::Time stamp;
  for (int i = 0; i < NOISY_FRAMES; ++i) {
    stamp = cloud_and_wait(noisy_floor(100 + i, false));
    ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
    if (i >= config.temporal.min_frames) {
      EXPECT_EQ(xyz_of(*find_stamp(obstacle_, stamp)).size(), 0u) << "obstacle points at cloud " << i;
      EXPECT_EQ(xyz_of(*find_stamp(drop_, stamp)).size(), 0u) << "drop points at cloud " << i;
    }
  }
  EXPECT_TRUE(state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); }).has_value());
  EXPECT_GT(metric("ground_temporal_cells").value_or(0.0), 0.0);
}

TEST_F(NodeTest, BoxOnNoisyFloorIsStillAnObstacle)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  builtin_interfaces::msg::Time stamp;
  for (int i = 0; i < warm_up_frames(config); ++i) {
    stamp = cloud_and_wait(noisy_floor(200 + i, true));
  }
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  const auto points = xyz_of(*find_stamp(obstacle_, stamp));
  ASSERT_GT(points.size(), 10u);
  for (const auto & p : points) {
    EXPECT_NEAR(p.x(), 1.0, 0.25);
    EXPECT_NEAR(p.y(), 0.0, 0.3);
    EXPECT_GT(p.z(), 0.1);
  }
}

TEST_F(NodeTest, PlaneLossResetsTheCellHistoryAndGauge)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  config.fit.hold_s = 0.3;
  config.temporal.max_age_s = 5.0;  // only the plane-loss reset can drop the history
  start(config);
  builtin_interfaces::msg::Time stamp;
  for (int i = 0; i < warm_up_frames(config); ++i) {
    stamp = cloud_and_wait(noisy_floor(300 + i, true));
  }
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  ASSERT_GT(xyz_of(*find_stamp(obstacle_, stamp)).size(), 10u);
  EXPECT_GT(metric("ground_temporal_cells").value_or(0.0), 0.0);

  const auto wall = scene_cloud(wall_scene());
  builtin_interfaces::msg::Time wall_stamp;
  for (int i = 0; i < 12; ++i) {
    wall_stamp = publish_cloud(wall);
    std::this_thread::sleep_for(milliseconds(static_cast<int>(CLOUD_PERIOD_S * 1000)));
  }
  ASSERT_TRUE(state_after(ns_of(wall_stamp), [](const json & d) { return d["plane"].is_null(); }).has_value());
  EXPECT_EQ(metric("ground_temporal_cells").value_or(-1.0), 0.0);

  // A flat floor cloud after the loss starts a new warm-up: stale box history would still give obstacle points.
  const auto flat = cloud_and_wait(noisy_floor(400, false));
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, flat).has_value(); }));
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, flat)).size(), 0u);
}

// --- stable cell classes (design D3, D5, D7, D8) ---------------------------------------------------------------------

namespace
{
constexpr int FAR_BAND_FRAMES = 80;
constexpr int BOX_FRAMES = 40;
constexpr int SILL_FRAMES = 30;
constexpr int RAMP_FRAMES = 40;
constexpr int SLOPE_TRANSIENT_FRAMES = 8;  // temporal.window_frames: the frames before a new incline's history is full
constexpr int CHURN_FRAMES = 40;
constexpr int COLOUR_FRAMES = 8;
constexpr double LETHAL_FREE_SHARE = 0.95;
constexpr double BOX_LETHAL_SHARE = 0.95;
constexpr double RAMP_SHARE = 0.90;
constexpr double IDENTICAL_PAIR_SHARE = 0.90;

double share(const std::vector<FrameOutput> & frames, const std::function<bool(const FrameOutput &)> & pred)
{
  return frames.empty() ? 0.0
                        : static_cast<double>(std::count_if(frames.begin(), frames.end(), pred)) /
                            static_cast<double>(frames.size());
}

bool lethal_free(const FrameOutput & f) { return f.obstacle == 0 && f.drop == 0; }
}  // namespace

TEST_F(NodeTest, NoisyFarBandOnEmptyFloorGivesNoLethalPointsAtP95)
{
  const auto frames = run_temporal_scene([](int i) { return far_band_floor(i, 0U); }, FAR_BAND_FRAMES);
  EXPECT_GE(share(frames, lethal_free), LETHAL_FREE_SHARE);
  // The noisy far-band cells are unknown: the spread gate counts them.
  EXPECT_GT(metric("ground_temporal_gated_cells").value_or(0.0), 0.0);
  EXPECT_TRUE(metric("ground_stable_lethal_cells").has_value());
}

// The far band with other noise draws.
TEST_F(NodeTest, NoisyFarBandGivesNoLethalPointsAtP95ForSeedSet1)
{
  const auto frames = run_temporal_scene([](int i) { return far_band_floor(i, 1U); }, FAR_BAND_FRAMES);
  EXPECT_GE(share(frames, lethal_free), LETHAL_FREE_SHARE);
}

TEST_F(NodeTest, NoisyFarBandGivesNoLethalPointsAtP95ForSeedSet2)
{
  const auto frames = run_temporal_scene([](int i) { return far_band_floor(i, 2U); }, FAR_BAND_FRAMES);
  EXPECT_GE(share(frames, lethal_free), LETHAL_FREE_SHARE);
}

TEST_F(NodeTest, NoisyFarBandGivesNoLethalPointsAtP95ForSeedSet3)
{
  const auto frames = run_temporal_scene([](int i) { return far_band_floor(i, 3U); }, FAR_BAND_FRAMES);
  EXPECT_GE(share(frames, lethal_free), LETHAL_FREE_SHARE);
}

TEST_F(NodeTest, FiveCentimetreBoxWithANoisyEdgeBandIsStablyLethal)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  const auto frames = run_scene(noisy_box, BOX_FRAMES);
  EXPECT_GE(share(frames, [](const FrameOutput & f) { return f.obstacle > 0; }), BOX_LETHAL_SHARE);
  // Only this frame's real points of stable lethal cells: every published point lies on or at the box.
  const auto points = xyz_of(*find_stamp(obstacle_, frames.back().stamp));
  ASSERT_FALSE(points.empty());
  for (const auto & p : points) {
    EXPECT_GE(p.x(), STABLE_BOX_X0 - CELL_M);
    EXPECT_LE(p.x(), STABLE_BOX_X1 + CELL_M);
    EXPECT_GE(p.y(), STABLE_BOX_Y0 - CELL_M);
    EXPECT_LE(p.y(), STABLE_BOX_Y1 + CELL_M);
    EXPECT_GT(p.z(), 0.02);
  }
  const auto state = state_after(
    ns_of(frames.back().stamp), [](const json & d) { return !d["plane"].is_null() && !d["regions"].empty(); });
  ASSERT_TRUE(state.has_value());
  bool obstacle_region = false;
  for (const auto & r : (*state)["regions"]) {
    obstacle_region = obstacle_region || (r["kind"] == "step" && r["class"] == "obstacle");
  }
  EXPECT_TRUE(obstacle_region) << state->dump();
  EXPECT_GT((*state)["obstacle_points"].get<int>(), 0);
  EXPECT_GT(metric("ground_stable_lethal_cells").value_or(0.0), 0.0);
}

TEST_F(NodeTest, ThreeCentimetreSillIsAStepWarningEveryFrameAndNeverLethal)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  const int warm_up = warm_up_frames(config);
  for (int i = 0; i < warm_up + SILL_FRAMES; ++i) {
    const auto f = run_frame(
      ground_test::render(sill_scene(SILL_HEIGHT_M), ground_test::DEFAULT_SIGMA_M, 0.0, 900U + i, DECIMATION));
    EXPECT_EQ(f.obstacle, 0u) << "frame " << i;
    EXPECT_EQ(f.drop, 0u) << "frame " << i;
    if (i >= warm_up) {
      ASSERT_FALSE(f.warnings.is_null()) << "frame " << i;
      EXPECT_TRUE(has_kind(f.warnings["areas"], "step", "tof")) << "frame " << i << ": " << f.warnings.dump();
    }
  }
  const auto state = state_after(ns_of(stamp_of(last_stamp_s_)), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(has_kind((*state)["warnings"], "step", "tof")) << state->dump();
  EXPECT_GT(metric("ground_stable_warning_cells").value_or(0.0), 0.0);
  EXPECT_EQ(metric("ground_stable_lethal_cells").value_or(-1.0), 0.0);
}

bool has_ramp(const FrameOutput & f) { return !f.warnings.is_null() && has_kind(f.warnings["areas"], "ramp", ""); }

TEST_F(NodeTest, SparseFourteenDegreeMatIsAStableRampWarning)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 0U); }, RAMP_FRAMES);
  EXPECT_GE(share(frames, has_ramp), RAMP_SHARE);
  EXPECT_GE(share(frames, lethal_free), RAMP_SHARE);
}

// The same mat with other noise draws: the ramp verdict is not one lucky seed.
TEST_F(NodeTest, SparseFourteenDegreeMatIsAStableRampWarningForSeedSet1)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 1U); }, RAMP_FRAMES);
  EXPECT_GE(share(frames, has_ramp), RAMP_SHARE);
  EXPECT_GE(share(frames, lethal_free), RAMP_SHARE);
}

TEST_F(NodeTest, SparseFourteenDegreeMatIsAStableRampWarningForSeedSet2)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 2U); }, RAMP_FRAMES);
  EXPECT_GE(share(frames, has_ramp), RAMP_SHARE);
  EXPECT_GE(share(frames, lethal_free), RAMP_SHARE);
}

TEST_F(NodeTest, SparseFourteenDegreeMatIsAStableRampWarningForSeedSet3)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 3U); }, RAMP_FRAMES);
  EXPECT_GE(share(frames, has_ramp), RAMP_SHARE);
  EXPECT_GE(share(frames, lethal_free), RAMP_SHARE);
}

// The safety side of the ramp rule: the same sparse mat raised to 20 deg is lethal.
TEST_F(NodeTest, SparseTwentyDegreeMatIsLethal)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 0U, STEEP_MAT_DEG); }, RAMP_FRAMES);
  EXPECT_GE(share(frames, [](const FrameOutput & f) { return f.obstacle > 0; }), RAMP_SHARE);
}

// A new steep incline is lethal from the first frame its cells hold a stable class, not only once the slope history
// holds a temporal window of clouds in them: until then its slope is fit over the clouds so far.
TEST_F(NodeTest, NewSparseTwentyDegreeMatIsLethalFromItsFirstStableFrame)
{
  const auto frames =
    run_temporal_scene([](int i) { return sparse_mat_ramp(i, 0U, STEEP_MAT_DEG); }, SLOPE_TRANSIENT_FRAMES);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    EXPECT_GT(frames[i].obstacle, 0u) << "frame " << i;
  }
}

// The shorter fit while the slope history fills does not make a new 14 deg mat lethal, for any seed set.
TEST_F(NodeTest, NewSparseFourteenDegreeMatIsNotLethalWhileItsSlopeHistoryFills)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 0U); }, SLOPE_TRANSIENT_FRAMES);
  EXPECT_GE(share(frames, lethal_free), RAMP_SHARE);
}

TEST_F(NodeTest, NewSparseFourteenDegreeMatIsNotLethalWhileItsSlopeHistoryFillsForSeedSet1)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 1U); }, SLOPE_TRANSIENT_FRAMES);
  EXPECT_GE(share(frames, lethal_free), RAMP_SHARE);
}

TEST_F(NodeTest, NewSparseFourteenDegreeMatIsNotLethalWhileItsSlopeHistoryFillsForSeedSet2)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 2U); }, SLOPE_TRANSIENT_FRAMES);
  EXPECT_GE(share(frames, lethal_free), RAMP_SHARE);
}

TEST_F(NodeTest, NewSparseFourteenDegreeMatIsNotLethalWhileItsSlopeHistoryFillsForSeedSet3)
{
  const auto frames = run_temporal_scene([](int i) { return sparse_mat_ramp(i, 3U); }, SLOPE_TRANSIENT_FRAMES);
  EXPECT_GE(share(frames, lethal_free), RAMP_SHARE);
}

TEST_F(NodeTest, JitteringStaticSillGivesIdenticalWarningAreas)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  const auto frames = run_scene([](int i) { return jittered_sill(i, SILL_JITTER_M); }, CHURN_FRAMES);
  std::size_t identical = 0;
  for (std::size_t i = 0; i < frames.size(); ++i) {
    ASSERT_FALSE(frames[i].warnings.is_null()) << "frame " << i;
    EXPECT_FALSE(frames[i].warnings["areas"].empty()) << "frame " << i;
    if (i > 0 && frames[i].warnings["areas"].dump() == frames[i - 1].warnings["areas"].dump()) {
      ++identical;
    }
  }
  EXPECT_GE(static_cast<double>(identical) / static_cast<double>(frames.size() - 1), IDENTICAL_PAIR_SHARE);
}

TEST_F(NodeTest, RedSheetIsAColourWarningAreaAndNeverLethal)
{
  fill_tf();
  start(base_config());
  show_red_sheet(0.0);
  const auto frames = run_scene(floor_frame, COLOUR_FRAMES);
  for (const auto & f : frames) {
    ASSERT_FALSE(f.warnings.is_null());
    EXPECT_TRUE(has_kind(f.warnings["areas"], "colour", "colour")) << f.warnings.dump();
    EXPECT_TRUE(lethal_free(f));
  }
  EXPECT_EQ(metric("ground_stable_lethal_cells").value_or(-1.0), 0.0);
}

TEST_F(NodeTest, SixCentimetreColourPlateIsAWarningAndNeverLethal)
{
  fill_tf();
  Config config = base_config();
  // A floor band wide enough that the RGB-D labelling flags the 6 cm plate as a floor colour mismatch: a colour
  // cluster 6 cm above the floor, above the 4 cm step rule. The ToF cloud is the bare floor; colour never blocks.
  config.fit.distance_m = 0.07;
  start(config);
  show_red_sheet(PLATE_HEIGHT_M);
  const auto frames = run_scene(floor_frame, COLOUR_FRAMES);
  for (const auto & f : frames) {
    ASSERT_FALSE(f.warnings.is_null());
    bool plate = false;
    for (const auto & a : f.warnings["areas"]) {
      if (a["kind"] == "colour" && a["source"] == "colour") {
        plate = true;
        EXPECT_NEAR(a["height_m"].get<double>(), PLATE_HEIGHT_M, 0.015) << f.warnings.dump();
      }
    }
    EXPECT_TRUE(plate) << f.warnings.dump();
    EXPECT_TRUE(lethal_free(f));
  }
  EXPECT_EQ(metric("ground_stable_lethal_cells").value_or(-1.0), 0.0);
}

TEST_F(NodeTest, StableClassGaugesAppearWithTheirFirstRealValues)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  for (const char * gauge :
       {"ground_temporal_gated_cells", "ground_stable_lethal_cells", "ground_stable_warning_cells"}) {
    EXPECT_FALSE(metric(gauge).has_value()) << gauge;
  }
  run_scene(noisy_box, 1);
  EXPECT_TRUE(metric("ground_temporal_gated_cells").has_value());
  EXPECT_GT(metric("ground_stable_lethal_cells").value_or(0.0), 0.0);
  EXPECT_TRUE(metric("ground_stable_warning_cells").has_value());
  run_scene([](int i) { return jittered_sill(i, 0.0); }, 1);
  EXPECT_GT(metric("ground_stable_warning_cells").value_or(0.0), 0.0);
}

TEST_F(NodeTest, PlaneLossResetsTheCellClasses)
{
  fill_tf();
  Config config = base_config();  // temporal off: only the class hysteresis delays the box
  config.fit.hold_s = 0.3;
  start(config);
  const auto box = scene_cloud(box_scene());
  const auto warm = warm_cloud_and_wait(box);
  ASSERT_GT(xyz_of(*find_stamp(obstacle_, warm)).size(), 50u);
  EXPECT_GT(metric("ground_stable_lethal_cells").value_or(0.0), 0.0);

  const auto wall = scene_cloud(wall_scene());
  builtin_interfaces::msg::Time wall_stamp;
  for (int i = 0; i < 12; ++i) {
    wall_stamp = publish_cloud(wall);
    std::this_thread::sleep_for(milliseconds(static_cast<int>(CLOUD_PERIOD_S * 1000)));
  }
  ASSERT_TRUE(state_after(ns_of(wall_stamp), [](const json & d) { return d["plane"].is_null(); }).has_value());
  EXPECT_EQ(metric("ground_stable_lethal_cells").value_or(-1.0), 0.0);
  EXPECT_EQ(metric("ground_stable_warning_cells").value_or(-1.0), 0.0);

  // The first box cloud after the loss is a new warm-up: remembered lethal cells would publish its points at once.
  const auto first = cloud_and_wait(box);
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, first)).size(), 0u);
  const auto stable = warm_cloud_and_wait(box);
  EXPECT_GT(xyz_of(*find_stamp(obstacle_, stable)).size(), 50u);
}

// --- occlusion shadow gate on the node (design D16) -------------------------------------------------------------------

namespace
{
constexpr int SHADOW_FRAMES = 30;
constexpr int TAIL_POINTS = 150;
constexpr double SHADOW_FAR_EDGE_X = STABLE_BOX_X1;  // the box's far edge in base_link
constexpr double SHADOW_DROP_SHARE = 0.01;
constexpr double SHADOW_BOX_SHARE = 0.95;
constexpr double SHADOW_TAIL_FLOOR_M = 0.0;
constexpr unsigned TAIL_SEED = 31000U;

// noisy_box plus the mixed-pixel tail behind its far (x = 0.93) edge, as the ToF at BASE_T_TOF smears it.
pcl::PointCloud<pcl::PointXYZ> box_with_tail(int frame)
{
  pcl::PointCloud<pcl::PointXYZ> cloud = noisy_box(frame);
  ground_test::add_mixed_pixel_tail(
    cloud, BASE_T_TOF.translation(), Eigen::Vector3d(SHADOW_FAR_EDGE_X, STABLE_BOX_Y0, STABLE_BOX_HEIGHT_M),
    Eigen::Vector3d(SHADOW_FAR_EDGE_X, STABLE_BOX_Y1, STABLE_BOX_HEIGHT_M), SHADOW_TAIL_FLOOR_M, TAIL_POINTS,
    TAIL_SEED + static_cast<unsigned>(frame));
  return cloud;
}

// Whether a step or ramp area of the document has its centroid beyond the box's far edge (base_link x).
bool has_far_side_area(const json & doc)
{
  if (doc.is_null()) {
    return false;
  }
  const Eigen::Isometry3d base_T_map = (MAP_T_ODOM * ODOM_T_BASE).inverse();
  for (const auto & a : doc["areas"]) {
    if (a["kind"] != "step" && a["kind"] != "ramp") {
      continue;
    }
    const Eigen::Vector2d c = centroid(a["polygon"]);
    if ((base_T_map * Eigen::Vector3d(c.x(), c.y(), 0.0)).x() > SHADOW_FAR_EDGE_X) {
      return true;
    }
  }
  return false;
}
}  // namespace

TEST_F(NodeTest, BoxWithAMixedPixelTailHasNoFarSideWarningAndAStableObstacle)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  const auto frames = run_scene(box_with_tail, SHADOW_FRAMES);
  EXPECT_GE(share(frames, [](const FrameOutput & f) { return f.obstacle > 0; }), SHADOW_BOX_SHARE);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    EXPECT_FALSE(has_far_side_area(frames[i].warnings)) << "frame " << i << ": " << frames[i].warnings.dump();
  }
  EXPECT_GT(metric("ground_occluded_points").value_or(0.0), 0.0);
}

TEST_F(NodeTest, NoisyEmptyFloorLosesFewerThanOnePercentToTheShadowGate)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  for (int i = 0; i < NOISY_FRAMES; ++i) {
    const auto cloud = ground_test::add_z_noise(
      ground_test::grid_cloud(-1.0, 1.0, -1.0, 1.0, NOISY_GRID_STEP_M), BOX_FLOOR_NOISE_M, 0.0, 0.0, 500U + i);
    const auto f = run_frame(cloud);
    const auto occluded = metric("ground_occluded_points");
    ASSERT_TRUE(occluded.has_value()) << "frame " << i;
    EXPECT_LT(*occluded, SHADOW_DROP_SHARE * static_cast<double>(cloud.size())) << "frame " << i;
    if (i >= config.temporal.min_frames) {
      EXPECT_EQ(f.obstacle, 0u) << "frame " << i;
      EXPECT_EQ(f.drop, 0u) << "frame " << i;
    }
  }
}

TEST_F(NodeTest, TenCentimetreDropOffStaysALethalDropWithTheShadowGate)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  const auto frames = run_scene(
    [](int i) {
      return ground_test::render(
        drop_scene(0.10), ground_test::DEFAULT_SIGMA_M, 0.0, 700U + static_cast<unsigned>(i), DECIMATION);
    },
    SHADOW_FRAMES);
  EXPECT_GE(share(frames, [](const FrameOutput & f) { return f.drop > 0; }), SHADOW_BOX_SHARE);
  EXPECT_TRUE(metric("ground_occluded_points").has_value());
}

TEST_F(NodeTest, OccludedPointsGaugeAppearsWithItsFirstRealValue)
{
  fill_tf();
  start(base_config());
  EXPECT_FALSE(metric("ground_occluded_points").has_value());
  cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  EXPECT_EQ(metric("ground_occluded_points").value_or(-1.0), 0.0);
}
