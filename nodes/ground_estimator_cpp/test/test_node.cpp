// GroundNode in-process: probe publishers and subscribers, a tf2_ros::Buffer fed with static and timed transforms
// (map -> odom -> base_link -> tof_optical_frame / head_camera_optical_frame / imu_link) and synthetic_floor.hpp
// scenes rendered in base_link and expressed in the sensor frames.
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
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
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
#include "ground_estimator_cpp/cell_median.hpp"
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
constexpr double TURN_TF_STEP_S = 0.02;   // TF step of a turning head
constexpr double TURN_RATE_DEG_S = 20.0;  // a turning head far above the 3 deg/s gate
constexpr double TURN_SETTLE_S = 0.15;    // after a turn starts or stops until both ends of the 0.1 s rate window agree
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
constexpr int PAIR_TRIES = 4;
constexpr milliseconds PAIR_WAIT{3000};

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
constexpr double LEGACY_FLOOR_BAND_M = 0.02;
constexpr double LEGACY_STEP_FRACTION = 0.3333;
constexpr double LEGACY_EDGE_MIN_HEIGHT_M = 0.02;

// The 10 Hz frame-count windows the fixtures of this file were calibrated for; the shipped defaults are these times 1.5
// for the 15 Hz D435 cloud (exercised by shipped_config()).
void pin_ten_hz_windows(Config & c)
{
  c.temporal.window_frames = 8;
  c.temporal.min_frames = 4;
  c.hysteresis.window_frames = 5;
  c.hysteresis.enter_frames = 3;
  c.hysteresis.exit_frames = 1;
  c.hysteresis.area_enter_frames = 3;
  c.hysteresis.area_join_frames = 8;
  c.hysteresis.area_kind_frames = 10;
  c.hysteresis.area_history_frames = 20;
  c.guidance.no_return.window_frames = 4;
  c.guidance.no_return.enter_frames = 3;
}

// The floor band, lethal step and edge point height the fixtures of this file were calibrated for.
void pin_legacy_rule(Config & c)
{
  c.bands.floor_band_m = LEGACY_FLOOR_BAND_M;
  c.traversability.max_step_fraction = LEGACY_STEP_FRACTION;
  c.guidance.edge.min_height_m = LEGACY_EDGE_MIN_HEIGHT_M;
}

constexpr double FLOOR_BUMP_M = 0.015;    // inside the 20 mm floor band
constexpr double WARNING_BUMP_M = 0.030;  // between the floor band and the 40 mm lethal step
constexpr double LETHAL_BUMP_M = 0.055;   // above the 40 mm lethal step
constexpr double LETHAL_DIP_M = 0.050;    // a drop deeper than the 40 mm step down limit
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

// --- guidance pairs: upsampled depth, its camera info and the JPEG, as head_rgbd_cpp publishes them -------------

// The guidance camera of the synthetic pairs: a 320x180 pinhole (70 deg horizontal field of view, no distortion) at
// BASE_T_CAM, principal point at the image centre.
constexpr int PAIR_W = 320, PAIR_H = 180;
constexpr double PAIR_F = 228.5;
// The ToF camera info: a 240x180 pinhole covering the 70 x 50 deg field of view of ground_test::render.
constexpr int TOF_W = 240, TOF_H = 180;
constexpr double TOF_FX = 171.4, TOF_FY = 193.0;
constexpr int JPEG_QUALITY = 95;
const cv::Vec3b FLOOR_BGR(120, 120, 120);
const cv::Vec3b BOX_BGR(60, 110, 160);
const cv::Vec3b BLACK_BGR(24, 20, 20);
constexpr double BOX_COLOUR_MIN_Z_M = 0.01;  // hits above this height take the box colour

// What a guidance pair shows: the depth geometry and, optionally, the floor hits without a depth return (black).
struct PairScene
{
  Scene scene;
  std::function<bool(const Eigen::Vector3d &)> no_return;
};

// The three messages of one pair.
struct PairMessages
{
  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo info;
  sensor_msgs::msg::CompressedImage rgb;
};

sensor_msgs::msg::CameraInfo pinhole_info(
  int w, int h, double fx, double fy, const std::string & frame, const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame;
  info.header.stamp = stamp;
  info.width = static_cast<uint32_t>(w);
  info.height = static_cast<uint32_t>(h);
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = {fx, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, 0.0, w / 2.0, 0.0, 0.0, fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

// Ray-cast the pair scene from the guidance camera: per pixel the nearest hit along the ray (x, y, 1) of the camera,
// whose ray parameter is the optical z; a hit the no_return test accepts reads 0 and black, a box hit (above
// BOX_COLOUR_MIN_Z_M) the box colour, the floor grey with a little texture.
PairMessages render_pair(const PairScene & pair, const std::string & frame, const builtin_interfaces::msg::Time & stamp)
{
  cv::Mat depth(PAIR_H, PAIR_W, CV_16UC1, cv::Scalar(0));
  cv::Mat bgr(PAIR_H, PAIR_W, CV_8UC3, cv::Scalar(0, 0, 0));
  const Eigen::Vector3d origin = BASE_T_CAM.translation();
  for (int v = 0; v < PAIR_H; ++v) {
    for (int u = 0; u < PAIR_W; ++u) {
      const Eigen::Vector3d ray((u - PAIR_W / 2.0) / PAIR_F, (v - PAIR_H / 2.0) / PAIR_F, 1.0);
      const Eigen::Vector3d dir = BASE_T_CAM.linear() * ray;
      double best = std::numeric_limits<double>::infinity();
      for (const auto & surface : pair.scene.surfaces) {
        const auto t = surface(origin, dir);
        if (t && *t > 0.0 && *t < best) {
          best = *t;
        }
      }
      if (!std::isfinite(best)) {
        continue;
      }
      const Eigen::Vector3d hit = origin + best * dir;
      if (pair.no_return && pair.no_return(hit)) {
        bgr.at<cv::Vec3b>(v, u) = BLACK_BGR;
        continue;
      }
      depth.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(best * 1000.0));
      const auto texture = static_cast<uchar>(FLOOR_BGR[0] + (u + 2 * v) % 7 - 3);
      bgr.at<cv::Vec3b>(v, u) = hit.z() > BOX_COLOUR_MIN_Z_M ? BOX_BGR : cv::Vec3b(texture, texture, texture);
    }
  }
  PairMessages out;
  out.depth.header.frame_id = frame;
  out.depth.header.stamp = stamp;
  out.depth.width = PAIR_W;
  out.depth.height = PAIR_H;
  out.depth.encoding = "16UC1";
  out.depth.is_bigendian = 0;
  out.depth.step = PAIR_W * sizeof(uint16_t);
  out.depth.data.resize(static_cast<std::size_t>(out.depth.step) * PAIR_H);
  std::memcpy(out.depth.data.data(), depth.data, out.depth.data.size());
  out.info = pinhole_info(PAIR_W, PAIR_H, PAIR_F, PAIR_F, frame, stamp);
  out.rgb.header.frame_id = frame;
  out.rgb.header.stamp = stamp;
  out.rgb.format = "jpeg";
  std::vector<uchar> jpeg;
  cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY});
  out.rgb.data.assign(jpeg.begin(), jpeg.end());
  return out;
}

// A second GroundNode on the same inputs with its own outputs (output_namespace of the main node plus a suffix), for
// frame-by-frame comparisons of two configs on identical clouds.
struct Twin
{
  Config config;
  std::shared_ptr<GroundNode> node;
  std::shared_ptr<rclcpp::Node> probe;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub, drop_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr warnings_sub;
  Inbox<sensor_msgs::msg::PointCloud2> obstacle, drop;
  Inbox<std_msgs::msg::String> warnings;
};

// Which optional transforms the buffer gets.
struct TfOptions
{
  bool tof = true;  // base_link -> tof_optical_frame
  bool map = true;  // map -> odom
  bool cam = true;  // base_link -> head_camera_optical_frame
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
    c.cloud_topic = prefix + "/head/d435/points_filtered";
    c.imu_topic = prefix + "/imu/data";
    c.rgbd_cloud_topic = prefix + "/head_rgbd/points";
    c.output_namespace = prefix + "/ground";
    c.guidance.depth_topic = prefix + "/head_rgbd/depth/image_raw";
    c.guidance.info_topic = prefix + "/head_rgbd/depth/camera_info";
    c.guidance.rgb_topic = prefix + "/head_rgbd/rgb/image_raw/compressed";
    c.guidance.depth_info_topic = prefix + "/head/d435/depth/camera_info";
    c.imu.level_reference_roll_deg = 0.0;
    c.imu.level_reference_pitch_deg = 0.0;
    c.temporal.enabled = false;  // the static scenes below are single clouds; the temporal tests switch it on
    // The fixtures of this file are calibrated for the 20 mm floor band and the 40 mm lethal step; the shipped values
    // (20 mm band, 40 mm step both ways) are exercised by shipped_rule_config().
    pin_legacy_rule(c);
    pin_ten_hz_windows(c);
    return c;
  }

  // base_config() with the shipped floor band, lethal step and edge point height (the Config defaults).
  Config shipped_rule_config()
  {
    Config c = base_config();
    const Config defaults;
    c.bands.floor_band_m = defaults.bands.floor_band_m;
    c.traversability.max_step_fraction = defaults.traversability.max_step_fraction;
    c.guidance.edge.min_height_m = defaults.guidance.edge.min_height_m;
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
    if (opt.cam) {
      buffer_->setTransform(transform_of(BASE_T_CAM, BASE, CAM_FRAME, 0.0), "test", true);
    }
    buffer_->setTransform(transform_of(BASE_T_IMU, BASE, IMU_FRAME, 0.0), "test", true);
    if (opt.map) {
      buffer_->setTransform(transform_of(MAP_T_ODOM, MAP, ODOM, 0.0), "test", true);
    }
    for (double t = t0_ - TF_BEFORE_S; t <= t0_ + TF_SPAN_S; t += TF_STEP_S) {
      buffer_->setTransform(transform_of(ODOM_T_BASE, ODOM, BASE, t), "test", false);
    }
  }

  // A camera frame whose TF history is dynamic (use TfOptions to leave its static transform out): from first_sample_s
  // to the end of the buffer it turns about base_link z at rate_deg_s from turn_start_s to turn_stop_s and holds its
  // angle outside that span. History before first_sample_s does not exist, so a rate read there is unknown.
  void set_turning_head(
    const std::string & frame, const Eigen::Isometry3d & base_T_frame, double rate_deg_s, double first_sample_s,
    double turn_start_s, double turn_stop_s)
  {
    for (double t = first_sample_s; t <= t0_ + TF_SPAN_S; t += TURN_TF_STEP_S) {
      const double turned_s = std::clamp(t, turn_start_s, turn_stop_s) - turn_start_s;
      const Eigen::Isometry3d turned(Eigen::AngleAxisd(rate_deg_s * DEG * turned_s, Eigen::Vector3d::UnitZ()));
      buffer_->setTransform(transform_of(turned * base_T_frame, BASE, frame, t), "test", false);
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
    const auto pair_qos = rclcpp::QoS(10).best_effort();
    depth_pub_ = probe_->create_publisher<sensor_msgs::msg::Image>(config.guidance.depth_topic, pair_qos);
    info_pub_ = probe_->create_publisher<sensor_msgs::msg::CameraInfo>(config.guidance.info_topic, pair_qos);
    rgb_pub_ = probe_->create_publisher<sensor_msgs::msg::CompressedImage>(config.guidance.rgb_topic, pair_qos);
    tof_info_pub_ = probe_->create_publisher<sensor_msgs::msg::CameraInfo>(config.guidance.depth_info_topic, pair_qos);
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
    for (const auto & twin : twins_) {
      executor_.add_node(twin->node);
      executor_.add_node(twin->probe);
    }
    spinner_ = std::thread([this] { executor_.spin(); });
    ASSERT_TRUE(wait_for([this] {
      const bool twins_ready = std::all_of(twins_.begin(), twins_.end(), [](const std::unique_ptr<Twin> & t) {
        return t->obstacle_sub->get_publisher_count() > 0 && t->drop_sub->get_publisher_count() > 0 &&
               t->warnings_sub->get_publisher_count() > 0;
      });
      return cloud_pub_->get_subscription_count() > twins_.size() && imu_pub_->get_subscription_count() > 0 &&
             obstacle_sub_->get_publisher_count() > 0 && state_sub_->get_publisher_count() > 0 && twins_ready;
    }));
  }

  // A twin node on the main config's inputs (call after fill_tf, before start); its outputs go to
  // <main output_namespace>_<name>.
  Twin & add_twin(const Config & main, Config twin_config, const std::string & name)
  {
    twin_config.cloud_topic = main.cloud_topic;
    twin_config.imu_topic = main.imu_topic;
    twin_config.rgbd_cloud_topic = main.rgbd_cloud_topic;
    twin_config.output_namespace = main.output_namespace + "_" + name;
    twins_.push_back(std::make_unique<Twin>());
    Twin * t = twins_.back().get();
    t->config = twin_config;
    t->node = std::make_shared<GroundNode>(twin_config, buffer_);
    t->probe = std::make_shared<rclcpp::Node>("ground_twin_probe_" + name + std::to_string(g_test_index.load()));
    const auto reliable = rclcpp::QoS(50).reliable();
    t->obstacle_sub = t->probe->create_subscription<sensor_msgs::msg::PointCloud2>(
      twin_config.obstacle_points(), reliable,
      [t](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { t->obstacle.add(*m); });
    t->drop_sub = t->probe->create_subscription<sensor_msgs::msg::PointCloud2>(
      twin_config.drop_points(), reliable, [t](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { t->drop.add(*m); });
    t->warnings_sub = t->probe->create_subscription<std_msgs::msg::String>(
      twin_config.warning_areas(), reliable, [t](std_msgs::msg::String::ConstSharedPtr m) { t->warnings.add(*m); });
    return *t;
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

  // The ToF camera info, then one guidance pair stamped now in `frame`; returns the stamp.
  builtin_interfaces::msg::Time publish_pair(const PairScene & pair, const std::string & frame = CAM_FRAME)
  {
    const auto stamp = next_stamp();
    tof_info_pub_->publish(pinhole_info(TOF_W, TOF_H, TOF_FX, TOF_FY, TOF_FRAME, stamp));
    PairMessages m = render_pair(pair, frame, stamp);
    depth_pub_->publish(m.depth);
    info_pub_->publish(m.info);
    rgb_pub_->publish(m.rgb);
    return stamp;
  }

  // ground_guidance_frames_total{result} of a node.
  static double guidance_frames(GroundNode & node, const std::string & result)
  {
    return series_value(node.metrics().render(), "ground_guidance_frames_total{result=\"" + result + "\"}")
      .value_or(0.0);
  }

  // Publish a pair until `counted` (the main node when null) counts it with `result`, at most PAIR_TRIES times (the
  // pair topics are best effort, and the first ToF camera info may arrive after its pair); returns the counted stamp.
  builtin_interfaces::msg::Time pair_and_wait(
    const PairScene & pair, const std::string & result = "ok", const std::string & frame = CAM_FRAME,
    GroundNode * counted = nullptr)
  {
    GroundNode & node = counted != nullptr ? *counted : *node_;
    builtin_interfaces::msg::Time stamp;
    const auto all_frames = [&node] {
      double sum = 0.0;
      for (const char * r : {"ok", "head_moving", "guidance_bad_input", "guidance_tf", "guidance_no_plane"}) {
        sum += guidance_frames(node, r);
      }
      return sum;
    };
    for (int attempt = 0; attempt < PAIR_TRIES; ++attempt) {
      const double before = guidance_frames(node, result);
      const double before_all = all_frames();
      stamp = publish_pair(pair, frame);
      wait_for([&] { return all_frames() > before_all; }, PAIR_WAIT);
      if (guidance_frames(node, result) > before) {
        return stamp;
      }
    }
    ADD_FAILURE() << "no guidance pair counted as " << result;
    return stamp;
  }

  // Wait for the guidance subscriptions of the main node (and of every guidance twin).
  void wait_for_pair_subscribers(std::size_t subscribers = 1)
  {
    ASSERT_TRUE(wait_for([&] {
      return depth_pub_->get_subscription_count() >= subscribers &&
             info_pub_->get_subscription_count() >= subscribers && rgb_pub_->get_subscription_count() >= subscribers &&
             tof_info_pub_->get_subscription_count() >= subscribers;
    }));
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
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_, tof_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  std::vector<std::unique_ptr<Twin>> twins_;
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

TEST_F(NodeTest, FifteenMillimetreBumpIsFloorWithNoWarningAndNoLethal)
{
  fill_tf();
  start(shipped_rule_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(sill_scene(FLOOR_BUMP_M)));
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, stamp)).size(), 0u);
  EXPECT_EQ(xyz_of(*find_stamp(drop_, stamp)).size(), 0u);
  const auto doc = warnings_at(stamp);
  ASSERT_TRUE(doc.has_value());
  EXPECT_TRUE((*doc)["areas"].empty()) << doc->dump();
  const auto state = state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE((*state)["regions"].empty()) << state->dump();
}

TEST_F(NodeTest, FifteenMillimetreDipIsFloorWithNoWarningAndNoLethal)
{
  fill_tf();
  start(shipped_rule_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(drop_scene(FLOOR_BUMP_M)));
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, stamp)).size(), 0u);
  EXPECT_EQ(xyz_of(*find_stamp(drop_, stamp)).size(), 0u);
  const auto doc = warnings_at(stamp);
  ASSERT_TRUE(doc.has_value());
  EXPECT_TRUE((*doc)["areas"].empty()) << doc->dump();
  const auto state = state_after(ns_of(stamp), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE((*state)["regions"].empty()) << state->dump();
}

TEST_F(NodeTest, ThirtyMillimetreBumpIsAStepWarningAndNeverLethal)
{
  fill_tf();
  start(shipped_rule_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(sill_scene(WARNING_BUMP_M)));
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, stamp)).size(), 0u);
  const auto doc = warnings_at(stamp);
  ASSERT_TRUE(doc.has_value());
  ASSERT_EQ((*doc)["areas"].size(), 1u) << doc->dump();
  EXPECT_EQ((*doc)["areas"][0]["kind"], "step");
}

TEST_F(NodeTest, FiftyFiveMillimetreBumpIsLethal)
{
  fill_tf();
  start(shipped_rule_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(sill_scene(LETHAL_BUMP_M)));
  ASSERT_TRUE(wait_for([&] { return find_stamp(obstacle_, stamp).has_value(); }));
  EXPECT_GT(xyz_of(*find_stamp(obstacle_, stamp)).size(), 20u);
}

TEST_F(NodeTest, SixtyFiveMillimetreBumpIsLethal)
{
  fill_tf();
  start(shipped_rule_config());
  const auto stamp = warm_cloud_and_wait(scene_cloud(sill_scene(0.065)));
  ASSERT_TRUE(wait_for([&] { return find_stamp(obstacle_, stamp).has_value(); }));
  EXPECT_GT(xyz_of(*find_stamp(obstacle_, stamp)).size(), 20u);
}

// A drop deeper than the 40 mm step limit is a lethal drop (user decision 2026-10-29: the same limit up and down). The
// plane may not tilt more than 1 degree here: with the default 10 degrees the RANSAC plane of a scene that is half
// upper and half lower floor tilts to catch both levels of a 5 cm drop (inlier ratio 0.83, plane 2 degrees off, drop
// region of 27 mm), a limit of the plane fit that the 10 cm drop of TenCentimetreDropOffIsALethalDropRegion does not meet.
TEST_F(NodeTest, FiftyMillimetreDipIsLethalAsADropDeeperThanTheStepLimit)
{
  fill_tf();
  Config config = shipped_rule_config();
  config.fit.max_normal_angle_deg = 1.0;
  start(config);
  const auto stamp = warm_cloud_and_wait(scene_cloud(drop_scene(LETHAL_DIP_M)));
  ASSERT_TRUE(wait_for([&] { return find_stamp(drop_, stamp).has_value(); }));
  EXPECT_GT(xyz_of(*find_stamp(drop_, stamp)).size(), 20u);
  EXPECT_EQ(xyz_of(*find_stamp(obstacle_, stamp)).size(), 0u);
}

// Head-motion gate (task T2): the TF of the cloud frame turns at 20 deg/s for two seconds. A cloud captured in the turn
// feeds the plane fit but does not vote or publish, is counted, and /ground/state says head_moving; once the turn
// stopped the next cloud votes again and the flag clears (the gate never holds on its own).
TEST_F(NodeTest, CloudsCapturedWhileTheHeadTurnsDoNotVoteAndTheGateReleasesWhenItStops)
{
  constexpr double TURN_S = 2.0;
  constexpr int TURN_CLOUDS = 5;
  TfOptions opt;
  opt.tof = false;
  fill_tf(opt);
  start(base_config());
  const double turn_start_s = wall_now_s();
  set_turning_head(TOF_FRAME, BASE_T_TOF, TURN_RATE_DEG_S, t0_ - TF_BEFORE_S, turn_start_s, turn_start_s + TURN_S);
  std::this_thread::sleep_for(std::chrono::duration<double>(TURN_SETTLE_S));
  std::vector<builtin_interfaces::msg::Time> gated;
  for (int i = 0; i < TURN_CLOUDS; ++i) {
    gated.push_back(publish_cloud(scene_cloud(box_scene())));
    std::this_thread::sleep_for(std::chrono::duration<double>(CLOUD_PERIOD_S));
  }
  ASSERT_TRUE(
    wait_for([&] { return metric("ground_head_moving_frames_total{input=\"cloud\"}").value_or(0.0) >= TURN_CLOUDS; }));
  EXPECT_NEAR(metric("ground_head_rate_deg_s").value_or(0.0), TURN_RATE_DEG_S, 1.0);
  const auto moving = state_after(ns_of(gated.back()), [](const json & d) { return d["head_moving"] == true; });
  ASSERT_TRUE(moving.has_value());
  EXPECT_FALSE((*moving)["plane"].is_null());  // the plane followed the head
  std::this_thread::sleep_for(SETTLE);
  for (const auto & stamp : gated) {
    EXPECT_FALSE(find_stamp(obstacle_, stamp).has_value()) << "gated cloud published an obstacle cloud";
    EXPECT_FALSE(find_stamp(drop_, stamp).has_value()) << "gated cloud published a drop cloud";
  }
  // The turn is over once both ends of the rate window lie after it.
  std::this_thread::sleep_for(std::chrono::duration<double>(TURN_S + TURN_SETTLE_S));
  const auto voting = cloud_and_wait(scene_cloud(box_scene()));
  const auto released = state_after(ns_of(voting), [](const json & d) { return d["head_moving"] == false; });
  ASSERT_TRUE(released.has_value());
  EXPECT_EQ(metric("ground_head_moving_frames_total{input=\"cloud\"}").value_or(-1.0), TURN_CLOUDS);
}

// A rate that cannot be read (the TF history of the cloud frame starts after stamp - motion.window_s) counts as still:
// the cloud votes as it did before the gate, and the unreadable rate is counted.
TEST_F(NodeTest, HeadRateThatCannotBeReadCountsAsStillAndIsCounted)
{
  TfOptions opt;
  opt.tof = false;
  fill_tf(opt);
  start(base_config());
  // History starting 1 s ahead; the cloud is stamped just after it starts, 0.1 s before which nothing is known.
  const double first_sample_s = wall_now_s() + 1.0;
  set_turning_head(TOF_FRAME, BASE_T_TOF, 0.0, first_sample_s, 0.0, 0.0);
  std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, first_sample_s + 0.01 - wall_now_s())));
  const auto stamp = cloud_and_wait(scene_cloud(box_scene()));
  EXPECT_TRUE(find_stamp(obstacle_, stamp).has_value());
  EXPECT_GE(metric("ground_head_rate_unknown_total").value_or(0.0), 1.0);
  EXPECT_EQ(metric("ground_head_moving_frames_total{input=\"cloud\"}").value_or(-1.0), 0.0);
  const auto state = state_after(ns_of(stamp), [](const json &) { return true; });
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ((*state)["head_moving"], false);
}

TEST_F(NodeTest, GateOffNeverCountsTheHeadMoving)
{
  TfOptions opt;
  opt.tof = false;
  fill_tf(opt);
  Config config = base_config();
  config.motion.enabled = false;
  start(config);
  const double turn_start_s = wall_now_s();
  set_turning_head(TOF_FRAME, BASE_T_TOF, TURN_RATE_DEG_S, t0_ - TF_BEFORE_S, turn_start_s, turn_start_s + 60.0);
  std::this_thread::sleep_for(std::chrono::duration<double>(TURN_SETTLE_S));
  const auto stamp = cloud_and_wait(scene_cloud(box_scene()));
  EXPECT_TRUE(find_stamp(obstacle_, stamp).has_value());
  EXPECT_EQ(metric("ground_head_moving_frames_total{input=\"cloud\"}").value_or(-1.0), 0.0);
  EXPECT_EQ(metric("ground_head_rate_unknown_total").value_or(-1.0), 0.0);
}

// The RGB-D cloud is stamped by the same camera: captured while the head turns it is dropped unlabelled and counted.
TEST_F(NodeTest, RgbdCloudCapturedWhileTheHeadTurnsIsNotLabelled)
{
  TfOptions opt;
  opt.cam = false;
  fill_tf(opt);
  start(base_config());
  ASSERT_TRUE(wait_for([this] { return rgbd_pub_->get_subscription_count() > 0; }));
  cloud_and_wait(scene_cloud(ground_test::floor_scene()));  // a plane
  const double turn_start_s = wall_now_s();
  set_turning_head(CAM_FRAME, BASE_T_CAM, TURN_RATE_DEG_S, t0_ - TF_BEFORE_S, turn_start_s, turn_start_s + 60.0);
  std::this_thread::sleep_for(std::chrono::duration<double>(TURN_SETTLE_S));
  publish_rgbd(false);
  ASSERT_TRUE(wait_for([&] { return metric("ground_head_moving_frames_total{input=\"rgbd\"}").value_or(0.0) >= 1.0; }));
  std::this_thread::sleep_for(SETTLE);
  EXPECT_EQ(labelled_.size(), 0u);
  EXPECT_EQ(dropped("no_plane_rgbd"), 0.0);
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
  EXPECT_EQ(area["source"], "depth");
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

// The filter's hysteresis.area_kind_frames is the only kind hold: with it at 1 a changed kind is published the frame
// the cell classes report it, not enter_frames frames later.
TEST_F(NodeTest, AreaKindChangeIsPublishedWithoutAnExtraNodeHold)
{
  constexpr int SWITCH_FRAMES = 12;
  constexpr int MAX_LAG_FRAMES = 4;
  fill_tf();
  Config config = base_config();
  config.hysteresis.area_kind_frames = 1;
  start(config);
  warm_cloud_and_wait(scene_cloud(sill_scene(0.03)));
  const auto ramp = scene_cloud(ramp_scene(14.0));
  int first_ramp = -1;
  for (int i = 0; i < SWITCH_FRAMES && first_ramp < 0; ++i) {
    const auto f = run_frame(ramp);
    if (!f.warnings.is_null() && has_kind(f.warnings["areas"], "ramp", "")) {
      first_ramp = i;
    }
  }
  ASSERT_GE(first_ramp, 0);
  EXPECT_LE(first_ramp, MAX_LAG_FRAMES);
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
constexpr int SLOPE_TRANSIENT_FRAMES = 8;  // pinned temporal.window_frames: frames to fill a new incline's history
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
      EXPECT_TRUE(has_kind(f.warnings["areas"], "step", "depth")) << "frame " << i << ": " << f.warnings.dump();
    }
  }
  const auto state = state_after(ns_of(stamp_of(last_stamp_s_)), [](const json & d) { return !d["plane"].is_null(); });
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(has_kind((*state)["warnings"], "step", "depth")) << state->dump();
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
  config.bands.floor_band_m = 0.07;
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

// --- lethal marks and the latency of a new obstacle (design D18, approved item A13) ------------------------------

namespace
{

constexpr int TRACKED_FLOOR_FRAMES = 20;  // frames of empty floor before the box is placed
constexpr int PLACED_BOX_FRAMES = 30;     // frames run after the box is placed
constexpr int MARK_FRAMES = 10;           // frames checked after the warm-up of the clean box
constexpr double FOOTPRINT_TOLERANCE_M = 0.005;

// The 5 cm box (x 0.62 .. 0.93, y 0.10 .. 0.35) with a clean 5 cm +- 4 mm top on the 8 mm noise floor, 2 cm grid (no
// noisy edge band); with_box false gives the same floor without it.
pcl::PointCloud<pcl::PointXYZ> clean_box(int frame, bool with_box = true)
{
  std::mt19937 rng(12000U + static_cast<unsigned>(frame));
  std::normal_distribution<double> floor_noise(0.0, BOX_FLOOR_NOISE_M);
  std::normal_distribution<double> top_noise(0.0, BOX_TOP_NOISE_M);
  pcl::PointCloud<pcl::PointXYZ> cloud = ground_test::grid_cloud(0.3, 1.4, -0.4, 0.6, 0.02);
  for (auto & p : cloud) {
    const bool on_box = p.x >= STABLE_BOX_X0 && p.x <= STABLE_BOX_X1 && p.y >= STABLE_BOX_Y0 && p.y <= STABLE_BOX_Y1;
    p.z = static_cast<float>(with_box && on_box ? STABLE_BOX_HEIGHT_M + top_noise(rng) : floor_noise(rng));
  }
  return cloud;
}

bool on_box_footprint(const Eigen::Vector3f & p)
{
  const double t = FOOTPRINT_TOLERANCE_M;
  return p.x() >= STABLE_BOX_X0 - t && p.x() <= STABLE_BOX_X1 + t && p.y() >= STABLE_BOX_Y0 - t &&
         p.y() <= STABLE_BOX_Y1 + t;
}

// The shipped config/ground_estimator.yaml with only the topics, the output namespace and the metrics port of the
// test's base config.
Config shipped_config(const Config & base)
{
  Config c = ground_estimator_cpp::from_yaml(YAML::LoadFile(std::string(GROUND_CONFIG_DIR) + "/ground_estimator.yaml"));
  c.cloud_topic = base.cloud_topic;
  c.imu_topic = base.imu_topic;
  c.rgbd_cloud_topic = base.rgbd_cloud_topic;
  c.output_namespace = base.output_namespace;
  c.metrics_port = std::nullopt;
  return c;
}

}  // namespace

// The cells are 10 cm in the fixed frame and the temporal filter moves every point of a cell to the cell's median: a
// cell straddling the box edge with most of its points on the box lifts its floor points to the box height. Only
// points that clear the step themselves (this frame's height and the cell's median) are lethal marks.
TEST_F(NodeTest, LethalMarksOfABoxLieOnItsFootprint)
{
  fill_tf();
  Config config = base_config();
  config.temporal.enabled = true;
  start(config);
  const auto frames = run_scene([](int i) { return clean_box(i); }, MARK_FRAMES);
  for (const auto & f : frames) {
    EXPECT_GT(f.obstacle, 0u);
    const auto cloud = find_stamp(obstacle_, f.stamp);
    ASSERT_TRUE(cloud.has_value());
    for (const auto & p : xyz_of(*cloud)) {
      EXPECT_TRUE(on_box_footprint(p)) << "lethal mark off the box at " << p.transpose();
    }
  }
}

// Latency pin: a 5 cm box placed on floor the node has tracked for 20 frames is lethal no later than
// hysteresis.enter_frames frames (0.3 s at 10 Hz) after the first frame in which the temporal filter outputs box
// heights for its cells, with the shipped config: the warning-area stability does not delay the lethal clouds.
TEST_F(NodeTest, PlacedBoxIsLethalWithinEnterFramesOfItsFirstFilteredBoxHeights)
{
  fill_tf();
  Config config = shipped_config(base_config());
  pin_legacy_rule(config);  // the rule the fixtures were calibrated for (equal to the shipped one since 2026-10-30)
  start(config);
  // The same temporal filter the node runs, on the same clouds about the true floor z = 0, tells the frame in which
  // the box cells first come out at box height.
  ground_estimator_cpp::CellMedianFilter reference(config.temporal);
  const ground_estimator_cpp::Plane floor;
  int first_filtered = -1;
  int first_lethal = -1;
  for (int i = 0; i < TRACKED_FLOOR_FRAMES + PLACED_BOX_FRAMES; ++i) {
    const bool with_box = i >= TRACKED_FLOOR_FRAMES;
    const auto cloud = clean_box(i, with_box);
    const FrameOutput f = run_frame(cloud);
    const auto filtered = reference.apply(cloud, floor, ODOM_T_BASE, static_cast<double>(ns_of(f.stamp)) * 1e-9);
    const bool box_heights = std::any_of(filtered.begin(), filtered.end(), [&](const pcl::PointXYZ & p) {
      return on_box_footprint(Eigen::Vector3f(p.x, p.y, p.z)) && p.z > config.max_step_m();
    });
    if (!with_box) {
      EXPECT_EQ(f.obstacle, 0u) << "frame " << i;
      continue;
    }
    if (first_filtered < 0 && box_heights) {
      first_filtered = i;
    }
    if (first_lethal < 0 && f.obstacle > 0) {
      first_lethal = i;
    }
  }
  ASSERT_GE(first_filtered, 0) << "the temporal filter never output the box";
  ASSERT_GE(first_lethal, 0) << "the box never became lethal";
  EXPECT_LE(first_lethal - first_filtered, config.hysteresis.enter_frames)
    << "box cells filtered at frame " << first_filtered << ", first lethal at frame " << first_lethal;
}

// --- RGB and amplitude guidance (design D3, D4, D5, D12; part guided-node) ---------------------------------------

namespace
{

// box_scene()'s box (x 0.85 .. 1.15, y -0.2 .. 0.2, 10 cm) is what the guidance pair shows; the ToF sees it with its
// near edge 4 cm short (x 0.89), the edge error of the live ToF.
constexpr double TRUE_EDGE_X = 0.85;
constexpr double TOF_EDGE_X = 0.89;
constexpr double BOX_X1 = 1.15, BOX_HALF_Y = 0.2, BOX_TOP_M = 0.10;
constexpr double EDGE_TOL_M = 0.02;  // the A3 target: the published edge within 2 cm of the true one
constexpr int GUIDED_FRAMES = 6;     // frames checked after the warm-up
// The black patch without ToF returns (x 0.75 .. 1.15, y -0.2 .. 0.2 on the floor).
constexpr double PATCH_X0 = 0.75, PATCH_X1 = 1.15, PATCH_Y0 = -0.2, PATCH_Y1 = 0.2;
constexpr int NO_RETURN_MAX_FRAMES = 12;
constexpr double STALE_AGE_S = 0.3;

Scene tof_box_scene()
{
  auto scene = ground_test::floor_scene();
  ground_test::add_box(
    scene, Eigen::Vector3d((TOF_EDGE_X + BOX_X1) / 2.0, 0.0, BOX_TOP_M / 2.0),
    Eigen::Vector3d(BOX_X1 - TOF_EDGE_X, 2.0 * BOX_HALF_Y, BOX_TOP_M));
  return scene;
}

PairScene true_box_pair() { return PairScene{box_scene(), nullptr}; }

bool in_patch(const Eigen::Vector3d & p)
{
  return p.x() >= PATCH_X0 && p.x() <= PATCH_X1 && p.y() >= PATCH_Y0 && p.y() <= PATCH_Y1;
}

// The floor with the black patch: no depth and black where the floor hit is in the patch.
PairScene black_patch_pair() { return PairScene{ground_test::floor_scene(), in_patch}; }

// The ToF floor cloud with no returns from the patch.
pcl::PointCloud<pcl::PointXYZ> floor_without_patch()
{
  pcl::PointCloud<pcl::PointXYZ> out;
  for (const auto & p : scene_cloud(ground_test::floor_scene())) {
    if (!in_patch(Eigen::Vector3d(p.x, p.y, p.z))) {
      out.push_back(p);
    }
  }
  return out;
}

// The points of the inbox's cloud of the stamp (waits for it; empty when it never arrives).
std::vector<Eigen::Vector3f> points_at(
  Inbox<sensor_msgs::msg::PointCloud2> & inbox, const builtin_interfaces::msg::Time & stamp)
{
  std::optional<sensor_msgs::msg::PointCloud2> found;
  EXPECT_TRUE(wait_for([&] {
    found = find_stamp(inbox, stamp);
    return found.has_value();
  }));
  return found ? xyz_of(*found) : std::vector<Eigen::Vector3f>();
}

// The areas of the warning-areas document of the stamp (waits for it; null when it never arrives).
json areas_at(Inbox<std_msgs::msg::String> & inbox, const builtin_interfaces::msg::Time & stamp)
{
  std::optional<json> found;
  EXPECT_TRUE(wait_for([&] {
    found = find_doc(inbox, [&](const json & d) { return d["stamp_ns"].get<int64_t>() == ns_of(stamp); });
    return found.has_value();
  }));
  return found ? (*found)["areas"] : json();
}

bool same_point(const Eigen::Vector3f & a, const Eigen::Vector3f & b) { return a == b; }

// The points of `of` that are not in `in` (exact coordinates).
std::vector<Eigen::Vector3f> extra_points(
  const std::vector<Eigen::Vector3f> & of, const std::vector<Eigen::Vector3f> & in)
{
  std::vector<Eigen::Vector3f> out;
  for (const auto & p : of) {
    if (std::none_of(in.begin(), in.end(), [&](const Eigen::Vector3f & q) { return same_point(p, q); })) {
      out.push_back(p);
    }
  }
  return out;
}

bool contains_all(const std::vector<Eigen::Vector3f> & of, const std::vector<Eigen::Vector3f> & points)
{
  return extra_points(points, of).empty();
}

double min_x(const std::vector<Eigen::Vector3f> & points)
{
  double m = std::numeric_limits<double>::infinity();
  for (const auto & p : points) {
    m = std::min(m, static_cast<double>(p.x()));
  }
  return m;
}

Config without_guidance(Config c)
{
  c.guidance.enabled = false;
  return c;
}

}  // namespace

// A3: the ToF sees the box 4 cm short; the guidance pair shows its true near edge. The published obstacle cloud is the
// stable lethal cloud of the guidance-off twin plus edge points, which reach the true edge within 2 cm and stay on the
// box; the stable points and the frame they first appear in are the twin's.
TEST_F(NodeTest, GuidedBoxGetsEdgePointsAtItsTrueEdge)
{
  fill_tf();
  const Config config = base_config();
  Twin & plain = add_twin(config, without_guidance(config), "plain");
  start(config);
  wait_for_pair_subscribers();
  cloud_and_wait(scene_cloud(tof_box_scene()));  // a plane for the first pair
  const int warm = warm_up_frames(config_);
  std::size_t edge_frames = 0;
  double last_guided = 0.0;
  for (int i = 0; i < warm + GUIDED_FRAMES; ++i) {
    pair_and_wait(true_box_pair());
    const FrameOutput f = run_frame(scene_cloud(tof_box_scene()));
    const auto guided = points_at(obstacle_, f.stamp);
    const auto stable = points_at(plain.obstacle, f.stamp);
    last_guided = static_cast<double>(guided.size());
    EXPECT_TRUE(contains_all(guided, stable)) << "frame " << i << ": a stable lethal point is missing";
    EXPECT_EQ(guided.empty(), stable.empty()) << "frame " << i << ": guidance changed when the box is lethal";
    EXPECT_EQ(points_at(drop_, f.stamp).size(), points_at(plain.drop, f.stamp).size());
    if (i < warm) {
      continue;
    }
    ASSERT_FALSE(stable.empty()) << "frame " << i;
    EXPECT_GE(min_x(stable), TOF_EDGE_X - 0.005) << "the ToF box edge is 4 cm short";
    const auto edge = extra_points(guided, stable);
    if (edge.empty()) {
      continue;
    }
    ++edge_frames;
    EXPECT_NEAR(min_x(guided), TRUE_EDGE_X, EDGE_TOL_M) << "frame " << i;
    for (const auto & p : edge) {
      EXPECT_GE(p.x(), TRUE_EDGE_X - EDGE_TOL_M) << p.transpose();
      EXPECT_LE(p.x(), BOX_X1 + EDGE_TOL_M) << p.transpose();
      EXPECT_LE(std::fabs(p.y()), BOX_HALF_Y + EDGE_TOL_M) << p.transpose();
      EXPECT_GE(p.z(), config.guidance.edge.min_height_m - 0.005) << p.transpose();
      EXPECT_LE(p.z(), BOX_TOP_M + EDGE_TOL_M) << p.transpose();
    }
  }
  EXPECT_EQ(edge_frames, static_cast<std::size_t>(GUIDED_FRAMES));
  EXPECT_GT(metric("ground_edge_points").value_or(0.0), 0.0);
  EXPECT_GE(metric("ground_guidance_frames_total{result=\"ok\"}").value_or(0.0), warm + GUIDED_FRAMES);
  EXPECT_GE(metric("ground_guidance_seconds_count").value_or(0.0), warm + GUIDED_FRAMES);
  // The obstacle_points gauge and the state count the published cloud, edge points included.
  EXPECT_TRUE(wait_for([&] { return metric("ground_obstacle_points").value_or(-1.0) == last_guided; }));
  EXPECT_TRUE(state_after(0, [&](const json & d) {
                return !d["plane"].is_null() && d["obstacle_points"].get<double>() == last_guided;
              }).has_value());
}

// D3, never alone: the pair shows a box the ToF does not see (flat floor), so there is no stable lethal point and the
// guidance adds none.
TEST_F(NodeTest, GuidanceWithoutLethalRegionAddsNoLethalPoint)
{
  fill_tf();
  start(base_config());
  wait_for_pair_subscribers();
  cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  for (int i = 0; i < warm_up_frames(config_) + GUIDED_FRAMES; ++i) {
    pair_and_wait(true_box_pair());
    const FrameOutput f = run_frame(scene_cloud(ground_test::floor_scene()));
    EXPECT_EQ(f.obstacle, 0u) << "frame " << i;
    EXPECT_EQ(f.drop, 0u) << "frame " << i;
    EXPECT_EQ(metric("ground_edge_points").value_or(-1.0), 0.0) << "frame " << i;
  }
  EXPECT_GE(metric("ground_guidance_frames_total{result=\"ok\"}").value_or(0.0), 1.0);
}

// D12: a pair older than guidance.max_age_s at the cloud stamp adds nothing (the clouds equal the guidance-off twin's);
// a fresh pair after it adds edge points again.
TEST_F(NodeTest, StaleGuidanceAddsNothing)
{
  fill_tf();
  Config config = base_config();
  config.guidance.max_age_s = STALE_AGE_S;
  Twin & plain = add_twin(config, without_guidance(config), "plain");
  start(config);
  wait_for_pair_subscribers();
  warm_cloud_and_wait(scene_cloud(tof_box_scene()));
  pair_and_wait(true_box_pair());
  std::this_thread::sleep_for(std::chrono::duration<double>(2.0 * STALE_AGE_S));
  for (int i = 0; i < GUIDED_FRAMES; ++i) {
    const FrameOutput f = run_frame(scene_cloud(tof_box_scene()));
    const auto guided = points_at(obstacle_, f.stamp);
    const auto stable = points_at(plain.obstacle, f.stamp);
    ASSERT_FALSE(stable.empty());
    EXPECT_EQ(guided.size(), stable.size()) << "frame " << i;
    EXPECT_TRUE(contains_all(guided, stable)) << "frame " << i;
    EXPECT_GE(min_x(guided), TOF_EDGE_X - 0.005) << "frame " << i;
    EXPECT_EQ(metric("ground_edge_points").value_or(-1.0), 0.0) << "frame " << i;
  }
  EXPECT_GE(metric("ground_guidance_stale_frames_total").value_or(0.0), static_cast<double>(GUIDED_FRAMES));
  // The same pair fresh: the edge points are back.
  pair_and_wait(true_box_pair());
  const FrameOutput fresh = run_frame(scene_cloud(tof_box_scene()));
  EXPECT_GT(points_at(obstacle_, fresh.stamp).size(), points_at(plain.obstacle, fresh.stamp).size());
}

// D4 (task T2): head_rgbd publishes a set every 0.5 s, so one lost set leaves a pair 1.0 s old. With the shipped
// guidance.max_age_s (1.25 s = 2.5 periods) that pair still adds its edge points and is not counted stale; the stale
// counter moves only once the newest pair is older than guidance.max_age_s.
TEST_F(NodeTest, GuidanceOfOneLostSetIsStillUsedAndStaleOnlyAfterTheMaxAge)
{
  fill_tf();
  const Config config = base_config();
  EXPECT_DOUBLE_EQ(config.guidance.max_age_s, 1.25);
  Twin & plain = add_twin(config, without_guidance(config), "plain");
  start(config);
  wait_for_pair_subscribers();
  warm_cloud_and_wait(scene_cloud(tof_box_scene()));
  pair_and_wait(true_box_pair());
  const double stale_before = metric("ground_guidance_stale_frames_total").value_or(-1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  const FrameOutput within = run_frame(scene_cloud(tof_box_scene()));
  EXPECT_GT(points_at(obstacle_, within.stamp).size(), points_at(plain.obstacle, within.stamp).size());
  EXPECT_EQ(metric("ground_guidance_stale_frames_total").value_or(-1.0), stale_before);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  const FrameOutput after = run_frame(scene_cloud(tof_box_scene()));
  EXPECT_EQ(points_at(obstacle_, after.stamp).size(), points_at(plain.obstacle, after.stamp).size());
  EXPECT_GT(metric("ground_guidance_stale_frames_total").value_or(-1.0), stale_before);
}

// A guidance pair captured while the head turns is not used (its depth and colour sit where the TF at the stamp does
// not say): counted as head_moving, no edge points, not a drop.
TEST_F(NodeTest, GuidancePairCapturedWhileTheHeadTurnsIsNotUsed)
{
  TfOptions opt;
  opt.cam = false;
  fill_tf(opt);
  start(base_config());
  wait_for_pair_subscribers();
  warm_cloud_and_wait(scene_cloud(tof_box_scene()));  // a plane for the pair
  const double turn_start_s = wall_now_s();
  set_turning_head(CAM_FRAME, BASE_T_CAM, TURN_RATE_DEG_S, t0_ - TF_BEFORE_S, turn_start_s, turn_start_s + 60.0);
  std::this_thread::sleep_for(std::chrono::duration<double>(TURN_SETTLE_S));
  pair_and_wait(true_box_pair(), "head_moving");
  EXPECT_GE(metric("ground_head_moving_frames_total{input=\"guidance\"}").value_or(0.0), 1.0);
  EXPECT_EQ(metric("ground_guidance_frames_total{result=\"ok\"}").value_or(-1.0), 0.0);
  EXPECT_EQ(dropped("guidance_tf"), 0.0);
  run_frame(scene_cloud(tof_box_scene()));
  EXPECT_EQ(metric("ground_edge_points").value_or(-1.0), 0.0);
}

// Step 2: a pair whose RGB frame has no TF is dropped (guidance_tf) and nothing from it is used.
TEST_F(NodeTest, PairWithoutTfIsDroppedAndCounted)
{
  fill_tf();
  start(base_config());
  wait_for_pair_subscribers();
  warm_cloud_and_wait(scene_cloud(tof_box_scene()));
  pair_and_wait(true_box_pair(), "guidance_tf", "camera_without_tf");
  EXPECT_GE(dropped("guidance_tf"), 1.0);
  EXPECT_EQ(metric("ground_guidance_frames_total{result=\"ok\"}").value_or(-1.0), 0.0);
  for (int i = 0; i < GUIDED_FRAMES; ++i) {
    const FrameOutput f = run_frame(scene_cloud(tof_box_scene()));
    EXPECT_GT(f.obstacle, 0u);
    EXPECT_GE(min_x(points_at(obstacle_, f.stamp)), TOF_EDGE_X - 0.005) << "frame " << i;
    EXPECT_EQ(metric("ground_edge_points").value_or(-1.0), 0.0) << "frame " << i;
  }
}

// Step 3: a pair before any plane is dropped (guidance_no_plane).
TEST_F(NodeTest, PairWithoutAPlaneIsDroppedAndCounted)
{
  fill_tf();
  start(base_config());
  wait_for_pair_subscribers();
  pair_and_wait(true_box_pair(), "guidance_no_plane");
  EXPECT_GE(dropped("guidance_no_plane"), 1.0);
  EXPECT_EQ(metric("ground_guidance_frames_total{result=\"ok\"}").value_or(-1.0), 0.0);
}

// Step 1: a pair whose JPEG does not decode is dropped (guidance_bad_input).
TEST_F(NodeTest, UndecodablePairIsDroppedAsBadInput)
{
  fill_tf();
  start(base_config());
  wait_for_pair_subscribers();
  cloud_and_wait(scene_cloud(ground_test::floor_scene()));
  for (int attempt = 0; attempt < PAIR_TRIES && dropped("guidance_bad_input") < 1.0; ++attempt) {
    const auto stamp = next_stamp();
    tof_info_pub_->publish(pinhole_info(TOF_W, TOF_H, TOF_FX, TOF_FY, TOF_FRAME, stamp));
    PairMessages m = render_pair(true_box_pair(), CAM_FRAME, stamp);
    m.rgb.data.assign(64, 0xAB);
    depth_pub_->publish(m.depth);
    info_pub_->publish(m.info);
    rgb_pub_->publish(m.rgb);
    wait_for([&] { return dropped("guidance_bad_input") >= 1.0; }, PAIR_WAIT);
  }
  EXPECT_GE(dropped("guidance_bad_input"), 1.0);
  EXPECT_GE(metric("ground_guidance_frames_total{result=\"guidance_bad_input\"}").value_or(0.0), 1.0);
  EXPECT_EQ(metric("ground_guidance_frames_total{result=\"ok\"}").value_or(-1.0), 0.0);
}

// A4: the black patch has no depth in the pair and no ToF returns; its colour is not the floor's. It becomes a
// no_return warning area (source rgb, null height and slope) in warning_areas (map frame) and in the state (base_link),
// and the obstacle and drop clouds equal the guidance-off twin's.
TEST_F(NodeTest, BlackPatchWithoutReturnsPublishesANoReturnAreaWithNullHeights)
{
  fill_tf();
  const Config config = base_config();
  Twin & plain = add_twin(config, without_guidance(config), "plain");
  start(config);
  wait_for_pair_subscribers();
  ASSERT_TRUE(wait_for([this] { return rgbd_pub_->get_subscription_count() > 1; }));
  cloud_and_wait(floor_without_patch());
  for (int i = 0; i < TRAIN_CLOUDS; ++i) {
    const auto stamp = publish_rgbd(false);
    ASSERT_TRUE(wait_for([&] { return find_stamp(labelled_, stamp).has_value(); }));
  }
  std::optional<FrameOutput> found;
  for (int i = 0; i < NO_RETURN_MAX_FRAMES && !found; ++i) {
    pair_and_wait(black_patch_pair());
    const FrameOutput f = run_frame(floor_without_patch());
    EXPECT_EQ(points_at(obstacle_, f.stamp), points_at(plain.obstacle, f.stamp)) << "frame " << i;
    EXPECT_EQ(points_at(drop_, f.stamp), points_at(plain.drop, f.stamp)) << "frame " << i;
    EXPECT_FALSE(has_kind(areas_at(plain.warnings, f.stamp), "no_return", ""));
    if (!f.warnings.is_null() && has_kind(f.warnings["areas"], "no_return", "rgb")) {
      found = f;
    }
  }
  ASSERT_TRUE(found.has_value()) << "no no_return area after " << NO_RETURN_MAX_FRAMES << " frames";
  const Eigen::Vector3d patch_base((PATCH_X0 + PATCH_X1) / 2.0, (PATCH_Y0 + PATCH_Y1) / 2.0, 0.0);
  const Eigen::Vector3d patch_map = MAP_T_ODOM * ODOM_T_BASE * patch_base;
  for (const auto & a : found->warnings["areas"]) {
    if (a["kind"] != "no_return") {
      continue;
    }
    EXPECT_EQ(a["source"], "rgb");
    EXPECT_TRUE(a["height_m"].is_null()) << a.dump();
    EXPECT_TRUE(a["slope_deg"].is_null()) << a.dump();
    const Eigen::Vector2d c = centroid(a["polygon"]);
    EXPECT_NEAR(c.x(), patch_map.x(), 2.0 * CENTROID_TOL_M);
    EXPECT_NEAR(c.y(), patch_map.y(), 2.0 * CENTROID_TOL_M);
  }
  const auto state = state_after(ns_of(found->stamp), [](const json & d) {
    return !d["plane"].is_null() && has_kind(d["warnings"], "no_return", "rgb");
  });
  ASSERT_TRUE(state.has_value());
  for (const auto & a : (*state)["warnings"]) {
    if (a["kind"] == "no_return") {
      EXPECT_TRUE(a["height_m"].is_null()) << a.dump();
      EXPECT_TRUE(a["slope_deg"].is_null()) << a.dump();
      const Eigen::Vector2d c = centroid(a["polygon"]);
      EXPECT_NEAR(c.x(), patch_base.x(), 2.0 * CENTROID_TOL_M);
      EXPECT_NEAR(c.y(), patch_base.y(), 2.0 * CENTROID_TOL_M);
    }
  }
  EXPECT_GE(metric("ground_no_return_areas").value_or(0.0), 1.0);
}

// guidance.enabled false: the node ignores the pairs and its outputs equal, frame by frame, those of a node that gets
// no pair at all; a guidance node on the same pairs differs (the pairs would change the outputs).
TEST_F(NodeTest, GuidanceDisabledBehavesAsBefore)
{
  fill_tf();
  const Config config = without_guidance(base_config());
  Config unfed = base_config();  // guidance on, its pair topics unused
  Twin & reference = add_twin(config, unfed, "reference");
  Config fed = config;
  fed.guidance.enabled = true;
  Twin & guided = add_twin(config, fed, "guided");
  start(config);
  wait_for_pair_subscribers();
  EXPECT_EQ(depth_pub_->get_subscription_count(), 1u) << "only the guided twin subscribes to the pairs";
  cloud_and_wait(scene_cloud(tof_box_scene()));
  bool guided_differs = false;
  for (int i = 0; i < warm_up_frames(config_) + GUIDED_FRAMES; ++i) {
    pair_and_wait(true_box_pair(), "ok", CAM_FRAME, guided.node.get());
    const FrameOutput f = run_frame(scene_cloud(tof_box_scene()));
    EXPECT_EQ(points_at(obstacle_, f.stamp), points_at(reference.obstacle, f.stamp)) << "frame " << i;
    EXPECT_EQ(points_at(drop_, f.stamp), points_at(reference.drop, f.stamp)) << "frame " << i;
    ASSERT_FALSE(f.warnings.is_null()) << "frame " << i;
    EXPECT_EQ(f.warnings["areas"], areas_at(reference.warnings, f.stamp)) << "frame " << i;
    guided_differs = guided_differs || points_at(guided.obstacle, f.stamp) != points_at(obstacle_, f.stamp);
  }
  EXPECT_TRUE(guided_differs);
  EXPECT_FALSE(metric("ground_guidance_frames_total{result=\"ok\"}").has_value());
  EXPECT_FALSE(metric("ground_edge_points").has_value());
  EXPECT_FALSE(metric("ground_no_return_areas").has_value());
}
