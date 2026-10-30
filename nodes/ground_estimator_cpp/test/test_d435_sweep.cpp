// GroundNode on the synthetic head D435 while the head sweeps (task T2). The tiny_truth.yaml scene (board, 5 cm box,
// ~14 deg ramp, empty floor; core/synthetic_d435.hpp) is rendered with the D435 noise model at 15 Hz while the head pans
// +-30 deg at 1 rad/s, each cloud's TF taken 0-20 ms away from its exposure, and the node's outputs are judged per
// fixed-frame cell: the box stays lethal, the board, the ramp and the empty floor never are, nothing is a drop, and a
// flicker metric (class changes per cell per 10 s) stays under its bound.
//
// The numbers asserted below are the measured worst cases over the scenarios of this file, with margin (the measured
// values are printed by every test: "d435 sweep ...").
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <set>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <utility>
#include <vector>

#include "core/synthetic_d435.hpp"
#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/ground_node.hpp"

namespace
{

using ground_estimator_cpp::Config;
using ground_estimator_cpp::GroundNode;
using nlohmann::json;

constexpr const char * MAP = "map";
constexpr const char * ODOM = "odom";
constexpr const char * BASE = "base_link";
constexpr const char * OPTICAL = "d435_depth_optical_frame";
constexpr double CELL_M = 0.10;  // temporal.cell_size_m
constexpr double TF_STEP_S = 0.02;
constexpr double TF_MARGIN_S = 1.0;
constexpr std::chrono::seconds WAIT{10};
constexpr std::chrono::milliseconds POLL{2};
constexpr std::chrono::milliseconds PUBLISH_WAIT{300};

// Scenario timeline: hold, two +-30 deg sweeps at SWEEP_RATE_DEG_S, hold.
constexpr double SWEEP_RATE_DEG_S = 57.29577951308232;  // 1 rad/s
constexpr double SWEEP_AMPLITUDE_DEG = 30.0;
constexpr double HOLD_S = 2.0;      // before the first sweep: the warm-up
constexpr double MID_HOLD_S = 2.0;  // between the sweeps
constexpr double END_HOLD_S = 2.5;  // after the last sweep
constexpr int SWEEPS = 2;
constexpr double WARM_UP_S = HOLD_S;
constexpr double EXPOSURE_OFFSET_MAX_S = 0.020;

// Robot pose in odom (static): not aligned with the cell grid.
Eigen::Isometry3d odom_T_base()
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(0.35, -0.22, 0.0);
  t.linear() = Eigen::AngleAxisd(20.0 * d435_sim::DEG, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return t;
}

double wall_now_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

builtin_interfaces::msg::Time stamp_of(double t_s)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_s));
  t.nanosec = static_cast<uint32_t>(std::llround((t_s - std::floor(t_s)) * 1e9) % 1000000000LL);
  return t;
}

int64_t ns_of(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
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

// The value of one series of a Prometheus text rendering (0 when absent).
double series_value(const std::string & text, const std::string & series)
{
  const std::string key = "\n" + series + " ";
  const size_t at = ("\n" + text).find(key);
  return at == std::string::npos ? 0.0 : std::stod(text.substr(at + key.size() - 1));
}

// Head pan (degrees) of a scenario at time t seconds after its start.
using PanPlan = std::function<double(double)>;

PanPlan static_plan(double pan_deg)
{
  return [pan_deg](double) { return pan_deg; };
}

// Hold, a +-30 deg sweep (0 -> +A -> -A -> 0), hold, a second sweep, hold.
PanPlan sweep_plan()
{
  const double leg_s = SWEEP_AMPLITUDE_DEG / SWEEP_RATE_DEG_S;
  const double sweep_s = 4.0 * leg_s;
  return [=](double t) {
    for (int k = 0; k < SWEEPS; ++k) {
      const double start = HOLD_S + k * (sweep_s + MID_HOLD_S);
      if (t < start || t >= start + sweep_s) {
        continue;
      }
      const double phase = (t - start) / leg_s;  // 0..4 legs
      if (phase < 1.0) {
        return SWEEP_AMPLITUDE_DEG * phase;
      }
      if (phase < 3.0) {
        return SWEEP_AMPLITUDE_DEG * (2.0 - phase);
      }
      return SWEEP_AMPLITUDE_DEG * (phase - 4.0);
    }
    return 0.0;
  };
}

double sweep_duration_s()
{
  return HOLD_S + SWEEPS * 4.0 * SWEEP_AMPLITUDE_DEG / SWEEP_RATE_DEG_S + (SWEEPS - 1) * MID_HOLD_S + END_HOLD_S;
}

// --- scene truth, in base_link --------------------------------------------------------------------------------------

struct Rect
{
  double x0, x1, y0, y1;
  bool contains(const Eigen::Vector2d & p, double margin = 0.0) const
  {
    return p.x() >= x0 - margin && p.x() <= x1 + margin && p.y() >= y0 - margin && p.y() <= y1 + margin;
  }
};

constexpr Rect BOARD{0.37, 0.57, -0.12, 0.14};
constexpr Rect BOX{0.60, 0.78, 0.05, 0.33};
constexpr Rect RAMP{0.85, 1.10, -0.45, -0.15};
constexpr Rect EMPTY{0.40, 0.70, -0.40, -0.20};
// Task T3 (drop safety): the scene with the floor 5 cm lower beyond x = DROP_X, past the ramp's back face (1.10 m).
constexpr double DROP_X = 1.15;
constexpr double DROP_DEPTH_M = 0.05;

d435_sim::World drop_world()
{
  d435_sim::World w = d435_sim::truth_world();
  w.drop_x = DROP_X;
  w.drop_depth_m = DROP_DEPTH_M;
  return w;
}
constexpr double FOOTPRINT_MARGIN_M = 0.10;  // one cell: a lethal mark may lie on the cell that touches the box

// --- the run --------------------------------------------------------------------------------------------------------

enum class Cls : int
{
  free = 0,
  warning = 1,
  obstacle = 2,
  drop = 3
};

struct FrameRecord
{
  double t_s = 0.0;
  double pan_deg = 0.0;
  bool moving = false;     // the node gated the frame
  bool published = false;  // the node answered with the lethal clouds and the warning areas
  int box_pixels = 0;      // BOX pixels in the rendered frame (the box is in view)
  int ramp_pixels = 0;
  int lower_pixels = 0;                         // the floor beyond a drop edge (drop_world)
  std::vector<Eigen::Vector2d> obstacle, drop;  // base_link xy
  json areas = json::array();                   // the warning areas (map == odom frame), polygons in odom
  bool head_moving_state = false;               // the newest /ground/state document says head_moving
};

struct Recording
{
  std::vector<FrameRecord> frames;
  double processed = 0.0;
  double moving_frames = 0.0;
  double stale_frames = 0.0;
};

std::pair<int, int> cell_of(const Eigen::Vector2d & odom_xy)
{
  return {static_cast<int>(std::floor(odom_xy.x() / CELL_M)), static_cast<int>(std::floor(odom_xy.y() / CELL_M))};
}

Eigen::Vector2d cell_centre(const std::pair<int, int> & cell)
{
  return Eigen::Vector2d((cell.first + 0.5) * CELL_M, (cell.second + 0.5) * CELL_M);
}

bool inside_polygon(const json & polygon, const Eigen::Vector2d & p)
{
  bool in = false;
  const size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double ax = polygon[i][0].get<double>(), ay = polygon[i][1].get<double>();
    const double bx = polygon[j][0].get<double>(), by = polygon[j][1].get<double>();
    if ((ay > p.y()) != (by > p.y()) && p.x() < (bx - ax) * (p.y() - ay) / (by - ay) + ax) {
      in = !in;
    }
  }
  return in;
}

// Class of a base_link point's cell in a frame record.
Cls class_in_frame(const FrameRecord & f, const std::pair<int, int> & cell)
{
  const Eigen::Isometry3d fixed_T_base = odom_T_base();
  for (const auto & p : f.drop) {
    if (cell_of((fixed_T_base * Eigen::Vector3d(p.x(), p.y(), 0.0)).head<2>()) == cell) {
      return Cls::drop;
    }
  }
  for (const auto & p : f.obstacle) {
    if (cell_of((fixed_T_base * Eigen::Vector3d(p.x(), p.y(), 0.0)).head<2>()) == cell) {
      return Cls::obstacle;
    }
  }
  const Eigen::Vector2d centre = cell_centre(cell);
  for (const auto & a : f.areas) {
    if (inside_polygon(a["polygon"], centre)) {
      return Cls::warning;
    }
  }
  return Cls::free;
}

// The flicker of a run. The published frames of a run split into settled stretches: a gated frame ends a stretch, and the
// first REACQUIRE_FRAMES frames after it are left out, because the cells aged out during the sweep and need
// temporal.min_frames values again (a re-acquisition, measured separately). Inside a stretch a cell's class is
// free, warning, obstacle or drop per frame, and
//  - a change is a frame whose class differs from the one before; changes per cell per 10 s of settled time is the
//    flicker metric,
//  - a blip is a run of fewer than BLIP_FRAMES equal classes between two other runs: a class that came and went.
constexpr int REACQUIRE_FRAMES = 8;
constexpr int BLIP_FRAMES = 5;

struct Flicker
{
  double worst_changes_per_10s = 0.0;  // the worst cell
  double mean_changes_per_10s = 0.0;   // over the cells that were ever not free
  int blips = 0;                       // all cells
  int active_cells = 0;
  double settled_s = 0.0;
};

// The settled stretches of a run from from_s: lists of published frames.
std::vector<std::vector<const FrameRecord *>> settled_stretches(const Recording & run, double from_s)
{
  std::vector<std::vector<const FrameRecord *>> stretches;
  std::vector<const FrameRecord *> current;
  int since_gate = REACQUIRE_FRAMES;  // frames since the last gated frame
  bool first = true;
  for (const FrameRecord & f : run.frames) {
    if (f.moving || !f.published) {
      if (!current.empty()) {
        stretches.push_back(std::move(current));
        current.clear();
      }
      since_gate = 0;
      first = false;
      continue;
    }
    ++since_gate;
    if (f.t_s < from_s || (!first && since_gate <= REACQUIRE_FRAMES)) {
      continue;
    }
    current.push_back(&f);
  }
  if (!current.empty()) {
    stretches.push_back(std::move(current));
  }
  return stretches;
}

Flicker flicker_of(const Recording & run, double from_s)
{
  const auto stretches = settled_stretches(run, from_s);
  Flicker out;
  const Eigen::Isometry3d fixed_T_base = odom_T_base();
  std::set<std::pair<int, int>> cells;
  for (const auto & stretch : stretches) {
    if (stretch.size() >= 2) {
      out.settled_s += stretch.back()->t_s - stretch.front()->t_s;
    }
    for (const FrameRecord * f : stretch) {
      for (const auto * points : {&f->obstacle, &f->drop}) {
        for (const auto & p : *points) {
          cells.insert(cell_of((fixed_T_base * Eigen::Vector3d(p.x(), p.y(), 0.0)).head<2>()));
        }
      }
      for (const auto & a : f->areas) {
        double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
        for (const auto & v : a["polygon"]) {
          x0 = std::min(x0, v[0].get<double>());
          x1 = std::max(x1, v[0].get<double>());
          y0 = std::min(y0, v[1].get<double>());
          y1 = std::max(y1, v[1].get<double>());
        }
        for (int cx = static_cast<int>(std::floor(x0 / CELL_M)); cx <= static_cast<int>(std::floor(x1 / CELL_M));
             ++cx) {
          for (int cy = static_cast<int>(std::floor(y0 / CELL_M)); cy <= static_cast<int>(std::floor(y1 / CELL_M));
               ++cy) {
            cells.insert({cx, cy});
          }
        }
      }
    }
  }
  if (out.settled_s <= 0.0) {
    return out;
  }
  double sum = 0.0;
  for (const auto & cell : cells) {
    int changes = 0;
    bool ever = false;
    for (const auto & stretch : stretches) {
      std::vector<Cls> classes;
      for (const FrameRecord * f : stretch) {
        classes.push_back(class_in_frame(*f, cell));
        ever = ever || classes.back() != Cls::free;
      }
      // Runs of equal classes.
      std::vector<std::pair<Cls, int>> runs;
      for (const Cls c : classes) {
        if (!runs.empty() && runs.back().first == c) {
          ++runs.back().second;
        } else {
          runs.emplace_back(c, 1);
        }
      }
      changes += static_cast<int>(runs.size()) - 1;
      for (size_t i = 1; i + 1 < runs.size(); ++i) {
        out.blips += runs[i].second < BLIP_FRAMES ? 1 : 0;
      }
    }
    if (!ever) {
      continue;
    }
    const double per_10s = changes * 10.0 / out.settled_s;
    out.worst_changes_per_10s = std::max(out.worst_changes_per_10s, per_10s);
    sum += per_10s;
    ++out.active_cells;
  }
  out.mean_changes_per_10s = out.active_cells > 0 ? sum / out.active_cells : 0.0;
  return out;
}

class D435SweepTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  Config config_for_test(const std::function<void(Config &)> & tweak = nullptr)
  {
    const YAML::Node node = YAML::LoadFile(std::string(GROUND_CONFIG_DIR) + "/ground_estimator.yaml");
    const auto errors = ground_estimator_cpp::validate(node);
    EXPECT_TRUE(errors.empty()) << (errors.empty() ? "" : errors.front());
    Config c = ground_estimator_cpp::from_yaml(node);
    const std::string prefix = "/sw" + std::to_string(index_++);
    c.cloud_topic = prefix + c.cloud_topic;
    c.imu_topic = prefix + "/imu/data";
    c.rgbd_cloud_topic = prefix + "/head_rgbd/points";
    c.output_namespace = prefix + "/ground";
    c.metrics_port = std::nullopt;
    c.colour.enabled = false;
    c.guidance.enabled = false;
    c.warnings.frame = ODOM;
    if (tweak) {
      tweak(c);
    }
    return c;
  }

  static geometry_msgs::msg::TransformStamped transform_of(
    const Eigen::Isometry3d & parent_T_child, const char * parent, const char * child, double t_s)
  {
    geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(parent_T_child);
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.header.stamp = stamp_of(t_s);
    return tf;
  }

  // Start a node on `config` and a TF buffer holding the head along `plan` over [t0 - margin, t0 + duration + margin].
  void start(const Config & config, const PanPlan & plan, double t0, double duration_s)
  {
    config_ = config;
    buffer_ = std::make_shared<tf2_ros::Buffer>(
      std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), tf2::Duration(std::chrono::seconds(300)));
    buffer_->setTransform(transform_of(Eigen::Isometry3d::Identity(), MAP, ODOM, 0.0), "test", true);
    buffer_->setTransform(transform_of(odom_T_base(), ODOM, BASE, 0.0), "test", true);
    for (double t = t0 - TF_MARGIN_S; t <= t0 + duration_s + TF_MARGIN_S; t += TF_STEP_S) {
      const d435_sim::HeadPose pose{plan(t - t0), 30.0};
      buffer_->setTransform(transform_of(d435_sim::base_T_optical(pose), BASE, OPTICAL, t), "test", false);
    }
    node_ = std::make_shared<GroundNode>(config, buffer_);
    probe_ = std::make_shared<rclcpp::Node>("sweep_probe_" + std::to_string(index_));
    cloud_pub_ =
      probe_->create_publisher<sensor_msgs::msg::PointCloud2>(config.cloud_topic, rclcpp::QoS(10).reliable());
    const auto reliable = rclcpp::QoS(100).reliable();
    obstacle_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.obstacle_points(), reliable, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        obstacle_[ns_of(m->header.stamp)] = *m;
      });
    drop_sub_ = probe_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.drop_points(), reliable, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        drop_[ns_of(m->header.stamp)] = *m;
      });
    warnings_sub_ = probe_->create_subscription<std_msgs::msg::String>(
      config.warning_areas(), reliable, [this](std_msgs::msg::String::ConstSharedPtr m) {
        const json doc = json::parse(m->data);
        std::lock_guard<std::mutex> lock(mutex_);
        warnings_[doc["stamp_ns"].get<int64_t>()] = doc;
      });
    state_sub_ = probe_->create_subscription<std_msgs::msg::String>(
      config.state_topic(), rclcpp::QoS(100).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        const json doc = json::parse(m->data);
        std::lock_guard<std::mutex> lock(mutex_);
        last_state_ = doc;
      });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(probe_);
    spinner_ = std::thread([this] { executor_->spin(); });
    ASSERT_TRUE(wait_for([this] {
      return cloud_pub_->get_subscription_count() > 0 && obstacle_sub_->get_publisher_count() > 0 &&
             warnings_sub_->get_publisher_count() > 0;
    }));
  }

  void TearDown() override { tear_down_node(); }

  // Stop the executor and drop the node and the probe, so a test can start a second node.
  void tear_down_node()
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spinner_.joinable()) {
      spinner_.join();
    }
    cloud_pub_.reset();
    obstacle_sub_.reset();
    drop_sub_.reset();
    warnings_sub_.reset();
    state_sub_.reset();
    executor_.reset();
    probe_.reset();
    node_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_.clear();
    drop_.clear();
    warnings_.clear();
    last_state_ = json::object();
  }

  double processed() const
  {
    const std::string text = node_->metrics().render();
    double sum = 0.0;
    size_t at = 0;
    while ((at = text.find("\nground_fits_total{", at)) != std::string::npos) {
      const size_t end = text.find('\n', at + 1);
      const std::string line = text.substr(at + 1, end - at - 1);
      sum += std::stod(line.substr(line.rfind(' ') + 1));
      at = end == std::string::npos ? text.size() : end;
    }
    return sum;
  }

  double metric(const std::string & series) const { return series_value(node_->metrics().render(), series); }

  // Render and publish the frames of a scenario, waiting for the node to process each; returns the per-frame records.
  Recording play(
    const d435_sim::World & world, const PanPlan & plan, double duration_s, double exposure_offset_max_s, uint32_t seed,
    const std::function<void(Config &)> & tweak = nullptr)
  {
    const double t0 = wall_now_s() + 1.0;
    start(config_for_test(tweak), plan, t0, duration_s);
    Recording out;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> offset(0.0, exposure_offset_max_s);
    ros2_common_cpp::DepthEdgeMaskParams mask;  // head_rgbd_cpp's shipped edge_mask (window 3, 1 cm, 2.5 %)
    const int frames = static_cast<int>(duration_s * d435_sim::FRAME_RATE_HZ);
    for (int k = 0; k < frames; ++k) {
      const double t = k / d435_sim::FRAME_RATE_HZ;
      // The exposure lies up to exposure_offset_max_s after the stamp the TF is read at.
      const double exposed = t + offset(rng);
      const d435_sim::Frame frame = d435_sim::render(world, d435_sim::HeadPose{plan(exposed), 30.0}, seed + k);
      const pcl::PointCloud<pcl::PointXYZ> cloud = d435_sim::optical_cloud(frame.depth_mm, &mask);
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.frame_id = OPTICAL;
      msg.header.stamp = stamp_of(t0 + t);
      FrameRecord rec;
      rec.t_s = t;
      rec.pan_deg = plan(t);
      rec.box_pixels = cv::countNonZero(frame.label == d435_sim::BOX);
      rec.ramp_pixels = cv::countNonZero(frame.label == d435_sim::RAMP);
      rec.lower_pixels = cv::countNonZero(frame.label == d435_sim::LOWER);
      const double before = processed();
      const double moving_before = metric("ground_head_moving_frames_total{input=\"cloud\"}");
      cloud_pub_->publish(msg);
      if (!wait_for([&] { return processed() > before; })) {
        ADD_FAILURE() << "frame " << k << " was not processed";
        break;
      }
      rec.moving = metric("ground_head_moving_frames_total{input=\"cloud\"}") > moving_before;
      collect(rec, ns_of(msg.header.stamp));
      out.frames.push_back(std::move(rec));
    }
    out.processed = processed();
    out.moving_frames = metric("ground_head_moving_frames_total{input=\"cloud\"}");
    out.stale_frames = metric("ground_guidance_stale_frames_total");
    return out;
  }

  // Fill the outputs of the frame stamped stamp_ns; a frame the node gated has none.
  void collect(FrameRecord & rec, int64_t stamp_ns)
  {
    const bool arrived = wait_for(
      [&] {
        std::lock_guard<std::mutex> lock(mutex_);
        return obstacle_.count(stamp_ns) > 0 && drop_.count(stamp_ns) > 0 && warnings_.count(stamp_ns) > 0;
      },
      rec.moving ? std::chrono::milliseconds(1) : PUBLISH_WAIT);
    std::lock_guard<std::mutex> lock(mutex_);
    rec.published = arrived;
    if (arrived) {
      fill_points(obstacle_[stamp_ns], rec.obstacle);
      fill_points(drop_[stamp_ns], rec.drop);
      rec.areas = warnings_[stamp_ns]["areas"];
    }
    rec.head_moving_state = last_state_.contains("head_moving") && last_state_["head_moving"].get<bool>();
  }

  static void fill_points(const sensor_msgs::msg::PointCloud2 & msg, std::vector<Eigen::Vector2d> & out)
  {
    if (msg.width * msg.height == 0) {
      return;
    }
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y");
    for (; x != x.end(); ++x, ++y) {
      out.emplace_back(*x, *y);
    }
  }

  static int index_;
  Config config_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<GroundNode> node_;
  std::shared_ptr<rclcpp::Node> probe_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_, drop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr warnings_sub_, state_sub_;
  std::mutex mutex_;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> obstacle_, drop_;
  std::map<int64_t, json> warnings_;
  json last_state_ = json::object();
};

int D435SweepTest::index_ = 0;

// --- what a run is judged on ----------------------------------------------------------------------------------------

constexpr int MIN_VISIBLE_PIXELS = 200;      // an object counts as in view with this many pixels
constexpr int MAX_REACQUIRE_FRAMES = 12;     // frames until the box is lethal again after a sweep (0.8 s at 15 Hz)
constexpr double MAX_CHANGES_PER_10S = 4.0;  // class changes per cell per 10 s of settled time (measured worst 3.3)

struct Verdict
{
  int published = 0;
  int gated = 0;
  int box_visible = 0;  // settled frames (as for the flicker) with the box in view
  int box_lethal = 0;
  int ramp_visible = 0;
  int ramp_warned = 0;
  int board_lethal = 0;
  int ramp_lethal = 0;
  int stray_lethal = 0;  // frames with a lethal point outside the box footprint, all published frames
  int drop_frames = 0;
  int warned_frames_on_empty = 0;
  int state_moving_frames = 0;
  int reacquire_frames_max = 0;  // frames from the first ungated frame after a gate to the box lethal again
  Flicker flicker;
};

bool any_in(const std::vector<Eigen::Vector2d> & points, const Rect & rect, double margin)
{
  return std::any_of(points.begin(), points.end(), [&](const Eigen::Vector2d & p) { return rect.contains(p, margin); });
}

bool warned_at(const FrameRecord & f, const Rect & rect)
{
  const Eigen::Isometry3d fixed_T_base = odom_T_base();
  const Eigen::Vector3d centre =
    fixed_T_base * Eigen::Vector3d((rect.x0 + rect.x1) / 2.0, (rect.y0 + rect.y1) / 2.0, 0.0);
  return std::any_of(
    f.areas.begin(), f.areas.end(), [&](const json & a) { return inside_polygon(a["polygon"], centre.head<2>()); });
}

Verdict judge(const Recording & run, double from_s)
{
  Verdict v;
  for (const FrameRecord & f : run.frames) {
    if (f.t_s < from_s) {
      continue;
    }
    v.gated += f.moving ? 1 : 0;
    v.state_moving_frames += f.head_moving_state ? 1 : 0;
    if (!f.published) {
      continue;
    }
    ++v.published;
    v.board_lethal +=
      any_in(f.obstacle, Rect{BOARD.x0, 0.50, BOARD.y0, BOARD.y1}, 0.0) || any_in(f.drop, BOARD, 0.0) ? 1 : 0;
    v.ramp_lethal += any_in(f.obstacle, RAMP, FOOTPRINT_MARGIN_M) || any_in(f.drop, RAMP, FOOTPRINT_MARGIN_M) ? 1 : 0;
    const bool stray = std::any_of(f.obstacle.begin(), f.obstacle.end(), [](const Eigen::Vector2d & p) {
      return !BOX.contains(p, FOOTPRINT_MARGIN_M);
    });
    v.stray_lethal += stray ? 1 : 0;
    v.drop_frames += f.drop.empty() ? 0 : 1;
    v.warned_frames_on_empty += warned_at(f, EMPTY) ? 1 : 0;
  }
  for (const auto & stretch : settled_stretches(run, from_s)) {
    for (const FrameRecord * f : stretch) {
      if (f->box_pixels >= MIN_VISIBLE_PIXELS) {
        ++v.box_visible;
        v.box_lethal += any_in(f->obstacle, BOX, FOOTPRINT_MARGIN_M) ? 1 : 0;
      }
      if (f->ramp_pixels >= MIN_VISIBLE_PIXELS) {
        ++v.ramp_visible;
        v.ramp_warned += warned_at(*f, RAMP) ? 1 : 0;
      }
    }
  }
  // Re-acquisition: after each gated frame, the ungated frames until the box is lethal again.
  int since_gate = -1;
  for (const FrameRecord & f : run.frames) {
    if (f.moving) {
      since_gate = 0;
      continue;
    }
    if (since_gate < 0 || f.t_s < from_s) {
      continue;
    }
    ++since_gate;
    if (f.published && any_in(f.obstacle, BOX, FOOTPRINT_MARGIN_M)) {
      v.reacquire_frames_max = std::max(v.reacquire_frames_max, since_gate);
      since_gate = -1;
    }
  }
  v.flicker = flicker_of(run, from_s);
  return v;
}

void print(const char * name, const Recording & run, const Verdict & v)
{
  std::cout << "d435 sweep " << name << ": frames " << run.frames.size() << ", published " << v.published << ", gated "
            << v.gated << ", state head_moving " << v.state_moving_frames << ", settled box visible/lethal "
            << v.box_visible << "/" << v.box_lethal << ", ramp visible/warned " << v.ramp_visible << "/"
            << v.ramp_warned << ", board lethal " << v.board_lethal << ", ramp lethal " << v.ramp_lethal
            << ", stray lethal frames " << v.stray_lethal << ", drop frames " << v.drop_frames << ", warned on empty "
            << v.warned_frames_on_empty << ", re-acquire frames max " << v.reacquire_frames_max
            << ", flicker: changes per cell per 10 s worst " << v.flicker.worst_changes_per_10s << " mean "
            << v.flicker.mean_changes_per_10s << ", blips " << v.flicker.blips << ", " << v.flicker.active_cells
            << " cells over " << v.flicker.settled_s << " s settled" << std::endl;
}

// What every scenario with the scene must show on its settled frames, head still or sweeping.
void expect_scene_classes(const Verdict & v)
{
  ASSERT_GT(v.box_visible, 20);
  EXPECT_EQ(v.box_lethal, v.box_visible) << "the 5 cm box is lethal in every settled frame that sees it";
  EXPECT_EQ(v.board_lethal, 0);
  EXPECT_EQ(v.ramp_lethal, 0) << "the 14 deg ramp is a warning, never lethal";
  EXPECT_EQ(v.stray_lethal, 0) << "no lethal mark off the box on the floor";
  EXPECT_EQ(v.drop_frames, 0) << "no drop cell on flat floor";
  ASSERT_GT(v.ramp_visible, 20);
  EXPECT_EQ(v.ramp_warned, v.ramp_visible) << "the ramp is warned in every settled frame that sees it";
}

}  // namespace

TEST_F(D435SweepTest, StaticHeadVotesEveryFrameAndNothingIsGated)
{
  const Recording run = play(d435_sim::truth_world(), static_plan(0.0), 6.0, EXPOSURE_OFFSET_MAX_S, 11);
  const Verdict v = judge(run, WARM_UP_S);
  print("static", run, v);
  EXPECT_EQ(run.moving_frames, 0.0);
  EXPECT_EQ(v.gated, 0);
  EXPECT_EQ(v.state_moving_frames, 0);
  expect_scene_classes(v);
  EXPECT_EQ(v.flicker.blips, 0);
  EXPECT_EQ(v.flicker.worst_changes_per_10s, 0.0);
}

TEST_F(D435SweepTest, EmptyFloorHasNoLethalMarkNoDropAndNoWarningWhileTheHeadSweeps)
{
  d435_sim::World empty;  // the floor only
  const Recording run = play(empty, sweep_plan(), sweep_duration_s(), EXPOSURE_OFFSET_MAX_S, 13);
  const Verdict v = judge(run, WARM_UP_S);
  print("empty floor sweep", run, v);
  ASSERT_GT(v.published, 60);
  EXPECT_GT(v.gated, 40);
  for (const FrameRecord & f : run.frames) {
    EXPECT_TRUE(f.obstacle.empty()) << "lethal mark on empty floor at " << f.t_s << " s";
    EXPECT_TRUE(f.drop.empty()) << "drop on empty floor at " << f.t_s << " s";
    EXPECT_TRUE(f.areas.empty()) << "warning area on empty floor at " << f.t_s << " s";
  }
}

TEST_F(D435SweepTest, SweepFramesAreGatedAndTheHeadMovingFlagFollowsThem)
{
  const PanPlan plan = sweep_plan();
  const Recording run = play(d435_sim::truth_world(), plan, sweep_duration_s(), EXPOSURE_OFFSET_MAX_S, 12);
  const Verdict v = judge(run, WARM_UP_S);
  print("sweep gate", run, v);
  const ground_estimator_cpp::MotionConfig motion = config_.motion;
  int moving_truth = 0, agree = 0;
  for (const FrameRecord & f : run.frames) {
    const double rate = std::fabs(plan(f.t_s) - plan(f.t_s - motion.window_s)) / motion.window_s;
    const bool should = rate > motion.max_head_rate_deg_s;
    moving_truth += should ? 1 : 0;
    agree += should == f.moving ? 1 : 0;
    if (f.moving) {
      EXPECT_FALSE(f.published) << "gated frame at " << f.t_s << " s published";
    }
  }
  EXPECT_GT(moving_truth, 40);  // two sweeps of ~2.1 s at 15 Hz
  // The plan's own rate and the gate's TF rate differ only at the leg ends, a frame or two each.
  EXPECT_GE(agree, static_cast<int>(run.frames.size()) - 10);
  EXPECT_GT(v.state_moving_frames, 20);
  EXPECT_EQ(
    run.moving_frames, static_cast<double>(std::count_if(
                         run.frames.begin(), run.frames.end(), [](const FrameRecord & f) { return f.moving; })));
}

TEST_F(D435SweepTest, ScenesKeepTheirClassesWhileTheHeadSweepsWithAnExposureOffset)
{
  const Recording run = play(d435_sim::truth_world(), sweep_plan(), sweep_duration_s(), EXPOSURE_OFFSET_MAX_S, 14);
  const Verdict v = judge(run, WARM_UP_S);
  print("sweep 0-20 ms", run, v);
  expect_scene_classes(v);
  EXPECT_LE(v.reacquire_frames_max, MAX_REACQUIRE_FRAMES);
  EXPECT_EQ(v.flicker.blips, 0);
  EXPECT_LE(v.flicker.worst_changes_per_10s, MAX_CHANGES_PER_10S);
}

TEST_F(D435SweepTest, ScenesKeepTheirClassesWhileTheHeadSweepsWithoutAnExposureOffset)
{
  const Recording run = play(d435_sim::truth_world(), sweep_plan(), sweep_duration_s(), 0.0, 15);
  const Verdict v = judge(run, WARM_UP_S);
  print("sweep 0 ms", run, v);
  expect_scene_classes(v);
  EXPECT_LE(v.reacquire_frames_max, MAX_REACQUIRE_FRAMES);
  EXPECT_EQ(v.flicker.blips, 0);
  EXPECT_LE(v.flicker.worst_changes_per_10s, MAX_CHANGES_PER_10S);
}

TEST_F(D435SweepTest, WithoutTheGateTheSweepIsWorseOrEqual)
{
  // The same sweep with motion.enabled false: every frame votes. Printed for the record; the gated run must not be
  // worse on the stray lethal marks and blips.
  const PanPlan plan = sweep_plan();
  const Recording gated = play(d435_sim::truth_world(), plan, sweep_duration_s(), EXPOSURE_OFFSET_MAX_S, 16);
  const Verdict vg = judge(gated, WARM_UP_S);
  print("A/B gate on", gated, vg);
  tear_down_node();
  const Recording open = play(
    d435_sim::truth_world(), plan, sweep_duration_s(), EXPOSURE_OFFSET_MAX_S, 16,
    [](Config & c) { c.motion.enabled = false; });
  const Verdict vo = judge(open, WARM_UP_S);
  print("A/B gate off", open, vo);
  EXPECT_EQ(open.moving_frames, 0.0);
  EXPECT_LE(vg.stray_lethal, vo.stray_lethal);
  EXPECT_LE(vg.flicker.blips, vo.flicker.blips);
}

// Task T3 (drop safety): a 5 cm drop beyond the scene is a lethal drop with the shipped config, head still and sweeping.
// Before T3 the plane tilted to catch both floor levels: the drop was lethal in 0 of 60 still frames and the 14 deg ramp
// lethal in 32. The box stays lethal, the ramp a warning and never lethal, and no drop mark lies before the edge. The
// flicker is not bounded here (worst cell about 80 class changes per 10 s): the lower floor at 1.2-2 m clears the 40 mm
// limit by 10 mm, about the D435 depth noise there.
namespace
{

struct DropVerdict
{
  int visible = 0;    // settled frames with the lower floor in view
  int lethal = 0;     // of those, frames with a drop mark beyond the edge
  int misplaced = 0;  // published frames with a drop mark before the edge
};

DropVerdict judge_drop(const Recording & run, double from_s)
{
  DropVerdict d;
  for (const FrameRecord & f : run.frames) {
    if (f.t_s >= from_s && f.published) {
      d.misplaced +=
        std::any_of(
          f.drop.begin(), f.drop.end(), [](const Eigen::Vector2d & p) { return p.x() < DROP_X - FOOTPRINT_MARGIN_M; })
          ? 1
          : 0;
    }
  }
  for (const auto & stretch : settled_stretches(run, from_s)) {
    for (const FrameRecord * f : stretch) {
      if (f->lower_pixels < MIN_VISIBLE_PIXELS) {
        continue;
      }
      ++d.visible;
      d.lethal +=
        std::any_of(f->drop.begin(), f->drop.end(), [](const Eigen::Vector2d & p) { return p.x() >= DROP_X; }) ? 1 : 0;
    }
  }
  return d;
}

void expect_drop_scene(const char * name, const Recording & run, const Verdict & v)
{
  const DropVerdict d = judge_drop(run, WARM_UP_S);
  print(name, run, v);
  std::cout << "d435 sweep " << name << ": drop visible/lethal " << d.visible << "/" << d.lethal << ", misplaced "
            << d.misplaced << std::endl;
  ASSERT_GT(d.visible, 20);
  EXPECT_EQ(d.lethal, d.visible) << "the 5 cm drop is lethal in every settled frame that sees it";
  EXPECT_EQ(d.misplaced, 0) << "no drop mark before the edge";
  ASSERT_GT(v.box_visible, 20);
  EXPECT_EQ(v.box_lethal, v.box_visible);
  EXPECT_EQ(v.board_lethal, 0);
  EXPECT_EQ(v.ramp_lethal, 0) << "the 14 deg ramp is a warning, never lethal";
  EXPECT_EQ(v.stray_lethal, 0);
  ASSERT_GT(v.ramp_visible, 20);
  EXPECT_EQ(v.ramp_warned, v.ramp_visible);
}

}  // namespace

TEST_F(D435SweepTest, FiveCentimetreDropIsLethalWithTheShippedFitHeadStill)
{
  const Recording run = play(drop_world(), static_plan(0.0), 6.0, EXPOSURE_OFFSET_MAX_S, 21);
  expect_drop_scene("drop static", run, judge(run, WARM_UP_S));
}

TEST_F(D435SweepTest, FiveCentimetreDropIsLethalWithTheShippedFitWhileTheHeadSweeps)
{
  const Recording run = play(drop_world(), sweep_plan(), sweep_duration_s(), EXPOSURE_OFFSET_MAX_S, 22);
  expect_drop_scene("drop sweep", run, judge(run, WARM_UP_S));
}
