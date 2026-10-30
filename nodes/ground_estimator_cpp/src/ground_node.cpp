#include "ground_estimator_cpp/ground_node.hpp"

#include <prometheus/counter.h>
#include <prometheus/histogram.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <initializer_list>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <utility>

#include "ground_estimator_cpp/motion.hpp"
#include "ground_estimator_cpp/occlusion.hpp"
#include "ground_estimator_cpp/segmentation.hpp"

namespace ground_estimator_cpp
{

namespace
{

constexpr auto TF_RETRY_PERIOD = std::chrono::milliseconds(5);
constexpr double NS_PER_S = 1e9;
constexpr double QUATERNION_NORM_TOLERANCE = 1e-3;
constexpr double UNKNOWN_COVARIANCE = -1.0;  // sensor_msgs/Imu: orientation not provided
constexpr double MIN_NORMAL_Z = 1e-6;
constexpr double FLOOR_NOISE_KEEP_M = 0.01;  // shadowed points this close to the plane stay (see drop_occluded)
constexpr double SAME_EDGE_M = 1e-9;         // a region copies its cluster's edge line exactly
constexpr std::array<double, 9> FIT_SECONDS_BUCKETS = {0.005, 0.01, 0.02, 0.03, 0.05, 0.1, 0.2, 0.5, 1.0};
constexpr std::array<FitReject, 6> ALL_REJECTS = {FitReject::too_few_points,   FitReject::no_plane,
                                                  FitReject::normal_off_prior, FitReject::offset_off_prior,
                                                  FitReject::too_few_inliers,  FitReject::low_inlier_ratio};
constexpr std::array<const char *, 4> ALL_DROP_REASONS = {
  DROP_REASON_TF, DROP_REASON_NO_PLANE_RGBD, DROP_REASON_NO_MAP_TF, DROP_REASON_BAD_INPUT};
constexpr std::array<const char *, 3> GUIDANCE_DROP_REASONS = {
  DROP_REASON_GUIDANCE_BAD_INPUT, DROP_REASON_GUIDANCE_TF, DROP_REASON_GUIDANCE_NO_PLANE};
constexpr const char * FITS_HELP = "Plane fits of depth clouds, by result (ok or the reject reason)";
constexpr const char * GUIDANCE_FRAMES_HELP =
  "Guidance pairs (upsampled depth, camera info, JPEG), by result (ok or the drop reason)";
constexpr const char * DROPPED_HELP = "Clouds, guidance pairs or warning-area messages dropped, by reason";
constexpr const char * HEAD_MOVING_HELP =
  "Inputs not used for voting because the head moved when they were captured, by input";
constexpr const char * HEAD_RATE_UNKNOWN_HELP =
  "Inputs whose head rate could not be read (no TF before the stamp), counted as still";
constexpr const char * GUIDANCE_STALE_HELP =
  "Depth clouds with a plane for which no guidance pair was within guidance.max_age_s";

struct GaugeSpec
{
  const char * name;
  const char * help;
};

constexpr std::array<GaugeSpec, 14> GAUGES = {{
  {METRIC_INLIER_RATIO, "Inlier ratio of the last valid plane fit"},
  {METRIC_SLOPE, "Floor slope against gravity of the last cloud with a plane and a fresh IMU, degrees"},
  {METRIC_PLANE_VS_BASE, "Angle between the floor normal and base_link +z of the last cloud with a plane, degrees"},
  {METRIC_REGIONS, "Step and drop regions of the last cloud with a plane"},
  {METRIC_WARNING_AREAS, "Warning areas (depth, colour and no_return) of the last cloud with a plane"},
  {METRIC_OBSTACLE_POINTS, "Points of the last obstacle_points cloud"},
  {METRIC_DROP_POINTS, "Points of the last drop_points cloud"},
  {METRIC_TEMPORAL_CELLS, "Cells the temporal median currently holds"},
  {METRIC_TEMPORAL_GATED,
   "Temporal cells unknown in the last cloud because their spread exceeded temporal.max_spread_m"},
  {METRIC_STABLE_LETHAL, "Fixed-frame cells whose stable class is obstacle or drop"},
  {METRIC_STABLE_WARNING, "Fixed-frame cells whose stable class is warning"},
  {METRIC_OCCLUDED_POINTS, "Points of the last depth cloud with a plane dropped by the occlusion shadow gate"},
  {METRIC_MISMATCH_RATIO, "Floor colour mismatch share of the floor band of the last labelled RGB-D cloud"},
  {METRIC_HEAD_RATE, "Camera angular speed in base_link at the last depth cloud, degrees per second"},
}};

int64_t ns_of(const builtin_interfaces::msg::Time & t) { return rclcpp::Time(t).nanoseconds(); }

tf2::TimePoint tf_time(const builtin_interfaces::msg::Time & t)
{
  return tf2::TimePoint(std::chrono::nanoseconds(ns_of(t)));
}

// Whether msg has every named field with the given datatype.
bool has_fields(const sensor_msgs::msg::PointCloud2 & msg, std::initializer_list<const char *> names, uint8_t datatype)
{
  return std::all_of(names.begin(), names.end(), [&](const char * name) {
    return std::any_of(msg.fields.begin(), msg.fields.end(), [&](const sensor_msgs::msg::PointField & f) {
      return f.name == name && f.datatype == datatype;
    });
  });
}

bool has_rgb(const sensor_msgs::msg::PointCloud2 & msg)
{
  return std::any_of(msg.fields.begin(), msg.fields.end(), [](const sensor_msgs::msg::PointField & f) {
    return (f.name == "rgb" || f.name == "rgba") && f.count == 1 &&
           (f.datatype == sensor_msgs::msg::PointField::FLOAT32 || f.datatype == sensor_msgs::msg::PointField::UINT32);
  });
}

// The finite float32 x, y, z points of msg moved by target_T_msg into out (cleared; its capacity is kept).
// @return false when msg has no float32 x, y, z.
bool read_xyz(
  const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Isometry3d & target_T_msg,
  pcl::PointCloud<pcl::PointXYZ> & out)
{
  out.clear();
  if (!has_fields(msg, {"x", "y", "z"}, sensor_msgs::msg::PointField::FLOAT32)) {
    return false;
  }
  if (static_cast<std::size_t>(msg.width) * msg.height == 0 || msg.data.empty()) {
    return true;
  }
  const Eigen::Isometry3f t = target_T_msg.cast<float>();
  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      continue;
    }
    const Eigen::Vector3f p = t * Eigen::Vector3f(*x, *y, *z);
    out.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
  }
  return true;
}

// As read_xyz, with the colour of the rgb (or rgba) field.
bool read_xyzrgb(
  const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Isometry3d & target_T_msg,
  pcl::PointCloud<pcl::PointXYZRGB> & out)
{
  out.clear();
  if (!has_fields(msg, {"x", "y", "z"}, sensor_msgs::msg::PointField::FLOAT32) || !has_rgb(msg)) {
    return false;
  }
  if (static_cast<std::size_t>(msg.width) * msg.height == 0 || msg.data.empty()) {
    return true;
  }
  const Eigen::Isometry3f t = target_T_msg.cast<float>();
  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> r(msg, "r"), g(msg, "g"), b(msg, "b");
  for (; x != x.end(); ++x, ++y, ++z, ++r, ++g, ++b) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      continue;
    }
    const Eigen::Vector3f p = t * Eigen::Vector3f(*x, *y, *z);
    pcl::PointXYZRGB q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    q.r = *r;
    q.g = *g;
    q.b = *b;
    out.push_back(q);
  }
  return true;
}

// Floor z under a base_link xy (0 for a degenerate, vertical plane).
double floor_z(const Plane & plane, const Eigen::Vector2d & xy)
{
  const Eigen::Vector3d & n = plane.normal;
  if (std::fabs(n.z()) < MIN_NORMAL_Z) {
    return 0.0;
  }
  return -(n.x() * xy.x() + n.y() * xy.y() + plane.d) / n.z();
}

// The area with its polygon moved by target_T_source; vertices are lifted onto the source floor plane first.
WarningArea moved(const WarningArea & area, const Plane & source_plane, const Eigen::Isometry3d & target_T_source)
{
  WarningArea out = area;
  for (auto & v : out.polygon) {
    v = (target_T_source * Eigen::Vector3d(v.x(), v.y(), floor_z(source_plane, v))).head<2>();
  }
  return out;
}

// The points of `points` at `indices`, with their heights, into out and out_h (cleared; capacity kept).
void gather(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<float> & heights, const std::vector<int> & indices,
  pcl::PointCloud<pcl::PointXYZ> & out, std::vector<float> & out_h)
{
  out.clear();
  out_h.clear();
  for (const int i : indices) {
    out.push_back(points[static_cast<std::size_t>(i)]);
    out_h.push_back(heights[static_cast<std::size_t>(i)]);
  }
}

// Regions of the step and drop clusters, nearest first, at most max_regions (names stay per kind by distance).
std::vector<Region> all_regions(
  const std::vector<Cluster> & above, const std::vector<Cluster> & below, const EdgesConfig & cfg)
{
  std::vector<Region> regions = regions_from_clusters(above, cfg);
  std::vector<Region> drops = regions_from_clusters(below, cfg);
  regions.insert(regions.end(), std::make_move_iterator(drops.begin()), std::make_move_iterator(drops.end()));
  std::stable_sort(
    regions.begin(), regions.end(), [](const Region & a, const Region & b) { return a.distance_m < b.distance_m; });
  if (regions.size() > static_cast<std::size_t>(cfg.max_regions)) {
    regions.resize(static_cast<std::size_t>(cfg.max_regions));
  }
  return regions;
}

bool is_lethal(CellClass c) { return c == CellClass::obstacle || c == CellClass::drop; }

std::uint32_t float_bits(float v)
{
  std::uint32_t b = 0;
  std::memcpy(&b, &v, sizeof(b));
  return b;
}

// The plane of the source frame expressed in the target frame.
Plane transformed(const Plane & plane, const Eigen::Isometry3d & target_T_source)
{
  Plane out;
  out.normal = target_T_source.linear() * plane.normal;
  out.d = plane.d - out.normal.dot(target_T_source.translation());
  return out;
}

// Whether any point of `points` (base_link) lies in a stable lethal cell.
bool any_stable_lethal(
  const pcl::PointCloud<pcl::PointXYZ> & points, const CellClassFilter & classes,
  const Eigen::Isometry3d & fixed_T_base)
{
  return std::any_of(points.begin(), points.end(), [&](const pcl::PointXYZ & p) {
    return is_lethal(classes.stable_at((fixed_T_base * Eigen::Vector3d(p.x, p.y, p.z)).head<2>()));
  });
}

// The cluster a region was made from (regions_from_clusters copies its kind, edge line and point count), or nullptr.
const Cluster * cluster_of(
  const Region & region, const std::vector<Cluster> & above, const std::vector<Cluster> & below)
{
  for (const auto * group : {&above, &below}) {
    for (const Cluster & c : *group) {
      if (
        c.kind == region.kind && c.points.size() == region.points &&
        std::fabs(c.distance_m - region.distance_m) < SAME_EDGE_M &&
        (c.edge_point - region.point).norm() < SAME_EDGE_M) {
        return &c;
      }
    }
  }
  return nullptr;
}

// Mean of a polygon's vertices (the origin for an empty polygon).
Eigen::Vector2d vertex_mean(const std::vector<Eigen::Vector2d> & polygon)
{
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  for (const auto & v : polygon) {
    sum += v;
  }
  return polygon.empty() ? sum : Eigen::Vector2d(sum / static_cast<double>(polygon.size()));
}

// Even-odd point-in-polygon test.
bool contains(const std::vector<Eigen::Vector2d> & polygon, const Eigen::Vector2d & p)
{
  bool in = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Eigen::Vector2d & a = polygon[i];
    const Eigen::Vector2d & b = polygon[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) && p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x()) {
      in = !in;
    }
  }
  return in;
}

// Whether an area of polygon `next` continues the area of polygon `prev`: equal, or either holds the other's vertex
// centroid (a cell joining or leaving keeps the area's identity).
bool continues(const std::vector<Eigen::Vector2d> & prev, const std::vector<Eigen::Vector2d> & next)
{
  if (prev.empty() || next.empty()) {
    return false;
  }
  return prev == next || contains(prev, vertex_mean(next)) || contains(next, vertex_mean(prev));
}

double seconds_since(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

std::size_t GroundNode::PointBitsHash::operator()(const PointBits & b) const
{
  return (static_cast<std::size_t>(b.x) * 73856093U) ^ (static_cast<std::size_t>(b.y) * 19349663U) ^
         (static_cast<std::size_t>(b.z) * 83492791U);
}

GroundNode::PointBits GroundNode::bits_of(const pcl::PointXYZ & p)
{
  return PointBits{float_bits(p.x), float_bits(p.y), float_bits(p.z)};
}

GroundNode::GroundNode(
  const Config & config, std::shared_ptr<tf2_ros::Buffer> tf_buffer, const rclcpp::NodeOptions & options)
: rclcpp::Node(NODE_NAME, options),
  config_(config),
  tf_buffer_(std::move(tf_buffer)),
  tracker_(config.fit),
  cell_filter_(config.temporal),
  cell_class_(config.hysteresis, config.temporal.cell_size_m),
  raw_heights_(slope_history_config(config.temporal)),
  colour_model_(config.colour),
  guidance_points_(config.temporal.cell_size_m),
  no_return_(config.guidance.no_return, config.temporal.cell_size_m, config.bands.floor_band_m),
  metrics_(NODE_NAME)
{
  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  }

  if (config_.metrics_port) {
    metrics_.serve(static_cast<std::uint16_t>(*config_.metrics_port));
  }
  metrics_.counter(METRIC_FITS, FITS_HELP, {{"result", FIT_RESULT_OK}});
  for (const FitReject reject : ALL_REJECTS) {
    metrics_.counter(METRIC_FITS, FITS_HELP, {{"result", to_string(reject)}});
  }
  for (const char * reason : ALL_DROP_REASONS) {
    metrics_.counter(METRIC_DROPPED, DROPPED_HELP, {{"reason", reason}});
  }
  labelled_ = &metrics_.counter(METRIC_LABELLED, "Labelled RGB-D clouds published");
  metrics_.counter(METRIC_HEAD_RATE_UNKNOWN, HEAD_RATE_UNKNOWN_HELP);
  metrics_.counter(METRIC_HEAD_MOVING, HEAD_MOVING_HELP, {{"input", INPUT_CLOUD}});
  if (config_.colour.enabled) {
    metrics_.counter(METRIC_HEAD_MOVING, HEAD_MOVING_HELP, {{"input", INPUT_RGBD}});
  }
  fit_seconds_ = &metrics_.histogram(
    METRIC_FIT_SECONDS, "Plane fit and segmentation time per depth cloud",
    std::vector<double>(FIT_SECONDS_BUCKETS.begin(), FIT_SECONDS_BUCKETS.end()));
  for (const GaugeSpec & spec : GAUGES) {
    gauges_[spec.name] = &prometheus::BuildGauge().Name(spec.name).Help(spec.help).Register(*metrics_.registry());
  }

  const rclcpp::QoS reliable = rclcpp::QoS(1).reliable();
  const rclcpp::QoS best_effort = rclcpp::QoS(1).best_effort();
  obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(config_.obstacle_points(), reliable);
  drop_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(config_.drop_points(), reliable);
  warnings_pub_ = create_publisher<std_msgs::msg::String>(config_.warning_areas(), reliable);
  state_pub_ =
    create_publisher<std_msgs::msg::String>(config_.state_topic(), rclcpp::QoS(1).reliable().transient_local());

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    config_.cloud_topic, reliable, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m) { on_cloud(m); });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    config_.imu_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::ConstSharedPtr & m) { on_imu(m); });
  if (config_.colour.enabled) {
    labelled_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(config_.labelled_points(), best_effort);
    rgbd_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      config_.rgbd_cloud_topic, best_effort,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m) { on_rgbd(m); });
  }

  if (config_.guidance.enabled) {
    for (const char * result :
         {GUIDANCE_RESULT_OK, GUIDANCE_RESULT_HEAD_MOVING, DROP_REASON_GUIDANCE_BAD_INPUT, DROP_REASON_GUIDANCE_TF,
          DROP_REASON_GUIDANCE_NO_PLANE}) {
      metrics_.counter(METRIC_GUIDANCE_FRAMES, GUIDANCE_FRAMES_HELP, {{"result", result}});
    }
    metrics_.counter(METRIC_HEAD_MOVING, HEAD_MOVING_HELP, {{"input", INPUT_GUIDANCE}});
    metrics_.counter(METRIC_GUIDANCE_STALE, GUIDANCE_STALE_HELP);
    for (const char * reason : GUIDANCE_DROP_REASONS) {
      metrics_.counter(METRIC_DROPPED, DROPPED_HELP, {{"reason", reason}});
    }
    guidance_seconds_ = &metrics_.histogram(
      METRIC_GUIDANCE_SECONDS, "Processing time per guidance pair",
      std::vector<double>(FIT_SECONDS_BUCKETS.begin(), FIT_SECONDS_BUCKETS.end()));
    for (const GaugeSpec & spec :
         {GaugeSpec{
            METRIC_EDGE_POINTS, "Guidance edge points appended to the lethal clouds of the last cloud with a plane"},
          GaugeSpec{METRIC_NO_RETURN_AREAS, "no_return warning areas of the last cloud with a plane"}}) {
      gauges_[spec.name] = &prometheus::BuildGauge().Name(spec.name).Help(spec.help).Register(*metrics_.registry());
    }
    // Best effort, as head_rgbd_cpp publishes the pair; one header per pair, so ExactTime matches it.
    depth_sub_.subscribe(this, config_.guidance.depth_topic, rmw_qos_profile_sensor_data);
    info_sub_.subscribe(this, config_.guidance.info_topic, rmw_qos_profile_sensor_data);
    rgb_sub_.subscribe(this, config_.guidance.rgb_topic, rmw_qos_profile_sensor_data);
    guidance_sync_ = std::make_unique<message_filters::Synchronizer<GuidancePolicy>>(
      GuidancePolicy(GUIDANCE_QUEUE), depth_sub_, info_sub_, rgb_sub_);
    guidance_sync_->registerCallback(
      std::bind(&GroundNode::on_guidance, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      config_.guidance.depth_info_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & m) { on_depth_info(m); });
  }

  state_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / config_.state.rate_hz), [this] { on_state_timer(); });
  RCLCPP_INFO(
    get_logger(), "ground estimator: %s -> %s/{obstacle_points,drop_points,warning_areas,state}%s",
    config_.cloud_topic.c_str(), config_.output_namespace.c_str(),
    config_.colour.enabled ? (", " + config_.rgbd_cloud_topic + " -> labelled_points").c_str() : "");
  if (config_.guidance.enabled) {
    RCLCPP_INFO(
      get_logger(), "guidance: %s + %s + %s, depth camera_info from %s (edge points %s, no_return areas %s)",
      config_.guidance.depth_topic.c_str(), config_.guidance.info_topic.c_str(), config_.guidance.rgb_topic.c_str(),
      config_.guidance.depth_info_topic.c_str(), config_.guidance.edge.enabled ? "on" : "off",
      config_.guidance.no_return.enabled ? "on" : "off");
  }
}

void GroundNode::drop(const char * reason, const std::string & detail)
{
  metrics_.counter(METRIC_DROPPED, DROPPED_HELP, {{"reason", reason}}).Increment();
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), DROP_LOG_THROTTLE_MS, "dropped (%s): %s", reason, detail.c_str());
}

void GroundNode::set_gauge(const char * name, double value) { gauges_.at(name)->Add({}).Set(value); }

bool GroundNode::lookup(
  const std::string & target, const std::string & source, const builtin_interfaces::msg::Time & stamp,
  Eigen::Isometry3d & out)
{
  if (target == source) {
    out = Eigen::Isometry3d::Identity();
    return true;
  }
  // Wait for a late transform only between frames the buffer knows; an unknown frame fails at once.
  const bool known = tf_buffer_->_frameExists(target) && tf_buffer_->_frameExists(source);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(config_.tf_timeout_s);
  while (true) {
    try {
      out = tf2::transformToEigen(tf_buffer_->lookupTransform(target, source, tf_time(stamp)));
      return true;
    } catch (const tf2::TransformException & e) {
      if (!known || std::chrono::steady_clock::now() >= deadline) {
        RCLCPP_DEBUG(get_logger(), "TF %s <- %s failed: %s", target.c_str(), source.c_str(), e.what());
        return false;
      }
      std::this_thread::sleep_for(TF_RETRY_PERIOD);
    }
  }
}

bool GroundNode::head_moving_at(
  const std::string & frame, const Eigen::Isometry3d & base_T_frame_now, const builtin_interfaces::msg::Time & stamp,
  bool gauge)
{
  if (!config_.motion.enabled) {
    return false;
  }
  const double window_s = config_.motion.window_s;
  const tf2::TimePoint before(std::chrono::nanoseconds(ns_of(stamp) - std::llround(window_s * NS_PER_S)));
  double rate_deg_s = 0.0;
  try {
    const Eigen::Isometry3d base_T_frame_before =
      tf2::transformToEigen(tf_buffer_->lookupTransform(config_.base_frame, frame, before));
    rate_deg_s = rotation_rate_deg_s(base_T_frame_before.linear(), base_T_frame_now.linear(), window_s);
  } catch (const tf2::TransformException & e) {
    metrics_.counter(METRIC_HEAD_RATE_UNKNOWN, HEAD_RATE_UNKNOWN_HELP).Increment();
    RCLCPP_DEBUG(get_logger(), "head rate unknown for %s: %s", frame.c_str(), e.what());
    return false;
  }
  if (gauge) {
    set_gauge(METRIC_HEAD_RATE, rate_deg_s);
  }
  return is_moving(rate_deg_s, config_.motion);
}

void GroundNode::on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  if (msg->orientation_covariance[0] == UNKNOWN_COVARIANCE) {
    return;
  }
  const Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  if (!q.coeffs().allFinite() || std::fabs(q.norm() - 1.0) > QUATERNION_NORM_TOLERANCE) {
    return;
  }
  if (!imu_to_base_ || imu_frame_ != msg->header.frame_id) {
    try {
      const auto tf = tf_buffer_->lookupTransform(config_.base_frame, msg->header.frame_id, tf2::TimePointZero);
      imu_to_base_ = Eigen::Quaterniond(tf2::transformToEigen(tf).rotation());
      imu_frame_ = msg->header.frame_id;
    } catch (const tf2::TransformException & e) {
      imu_to_base_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), DROP_LOG_THROTTLE_MS, "IMU not used: no TF %s <- %s: %s",
        config_.base_frame.c_str(), msg->header.frame_id.c_str(), e.what());
      return;
    }
  }
  latest_imu_ = ImuSample{
    gravity_up_in_base(q, *imu_to_base_, config_.imu.level_reference_roll_deg, config_.imu.level_reference_pitch_deg),
    static_cast<double>(ns_of(msg->header.stamp)) / NS_PER_S};
}

void GroundNode::on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const int64_t stamp_ns = ns_of(msg->header.stamp);
  const double stamp_s = static_cast<double>(stamp_ns) / NS_PER_S;
  Eigen::Isometry3d fixed_T_base;
  Eigen::Isometry3d base_T_cloud;
  const bool have_fixed = lookup(config_.fixed_frame, config_.base_frame, msg->header.stamp, fixed_T_base);
  if (!have_fixed || !lookup(config_.base_frame, msg->header.frame_id, msg->header.stamp, base_T_cloud)) {
    drop(
      DROP_REASON_TF, "no TF " + config_.fixed_frame + " <- " + config_.base_frame + " <- " + msg->header.frame_id +
                        " at the cloud stamp");
    have_cloud_ = true;
    tracker_.mark_failed(stamp_s);
    last_reject_.reset();
    last_reason_ = REASON_NO_TF;
    cycle_ = Cycle{};
    // A plane fitted before is still held at this stamp when the robot pose is known.
    publish_state_on_change(
      stamp_ns, have_fixed ? tracker_.current(fixed_T_base, stamp_s) : std::optional<TrackedPlane>());
    return;
  }

  const auto started = std::chrono::steady_clock::now();
  const bool moving = head_moving_at(msg->header.frame_id, base_T_cloud, msg->header.stamp, true);
  if (moving) {
    metrics_.counter(METRIC_HEAD_MOVING, HEAD_MOVING_HELP, {{"input", INPUT_CLOUD}}).Increment();
  }
  head_moving_ = moving;
  if (!read_xyz(*msg, base_T_cloud, base_cloud_)) {
    drop(DROP_REASON_BAD_INPUT, "depth cloud without float32 x, y, z");
    return;
  }
  have_cloud_ = true;
  prior_ = choose_prior(latest_imu_, stamp_s, config_.imu);
  const FitResult fit = fit_plane(base_cloud_, prior_.up, config_.fit, FIT_SEED);
  if (fit.plane) {
    tracker_.update(*fit.plane, fixed_T_base, stamp_s);
    last_inliers_ = fit.inliers;
    last_inlier_ratio_ = fit.inlier_ratio;
    last_reject_.reset();
    metrics_.counter(METRIC_FITS, FITS_HELP, {{"result", FIT_RESULT_OK}}).Increment();
    set_gauge(METRIC_INLIER_RATIO, fit.inlier_ratio);
  } else {
    tracker_.mark_failed(stamp_s);
    last_reject_ = fit.reject.value_or(FitReject::no_plane);
    metrics_.counter(METRIC_FITS, FITS_HELP, {{"result", to_string(*last_reject_)}}).Increment();
  }
  last_reason_ = nullptr;

  const auto tracked = tracker_.current(fixed_T_base, stamp_s);
  if (tracked && moving) {
    // The plane follows the head, the cells and classes keep their state: this frame does not vote or publish.
    fit_seconds_->Observe(seconds_since(started));
  } else if (tracked) {
    process_plane(*msg, *tracked, fixed_T_base, base_T_cloud.translation(), stamp_s, started);
  } else {
    // No valid or held plane: no cloud, no warning areas (no placeholder data); the cell history is stale.
    reset_cells();
    fit_seconds_->Observe(seconds_since(started));
    cycle_ = Cycle{};
  }
  publish_state_on_change(stamp_ns, tracked);
}

void GroundNode::reset_cells()
{
  cell_filter_.reset();
  cell_class_.reset();
  raw_heights_.reset();
  areas_fixed_.clear();
  no_return_.reset();
  no_return_fixed_.clear();
  if (config_.guidance.enabled && config_.guidance.no_return.enabled) {
    set_gauge(METRIC_NO_RETURN_AREAS, 0.0);
  }
  set_gauge(METRIC_TEMPORAL_CELLS, 0.0);
  set_gauge(METRIC_TEMPORAL_GATED, static_cast<double>(cell_filter_.gated_cell_count()));
  set_gauge(METRIC_STABLE_LETHAL, 0.0);
  set_gauge(METRIC_STABLE_WARNING, 0.0);
}

void GroundNode::drop_occluded(const Plane & plane, const Eigen::Vector3d & sensor_base)
{
  const std::size_t n = filtered_cloud_.size();
  floor_reference_.heights.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p = filtered_cloud_[i];
    floor_reference_.heights[i] = static_cast<float>(height_above(plane, Eigen::Vector3d(p.x, p.y, p.z)));
  }
  const double sensor_height_m = height_above(plane, sensor_base);
  const std::vector<bool> occluded =
    occluded_mask(filtered_cloud_, floor_reference_.heights, sensor_base.head<2>(), sensor_height_m, config_.occlusion);
  floor_reference_.points.clear();
  floor_reference_.points.reserve(n);
  std::size_t kept = 0;
  for (std::size_t i = 0; i < n; ++i) {
    // A shadowed point within the depth floor noise is floor whatever hides it: it cannot make a step, ramp or drop,
    // and it keeps the floor level beside an obstacle.
    if (!occluded[i] || std::fabs(floor_reference_.heights[i]) <= FLOOR_NOISE_KEEP_M) {
      floor_reference_.points.push_back(filtered_cloud_[i]);
      floor_reference_.heights[kept++] = floor_reference_.heights[i];
    }
  }
  floor_reference_.heights.resize(kept);
  set_gauge(METRIC_OCCLUDED_POINTS, static_cast<double>(n - kept));
  filtered_cloud_ = floor_reference_.points;
}

void GroundNode::process_plane(
  const sensor_msgs::msg::PointCloud2 & msg, const TrackedPlane & tracked, const Eigen::Isometry3d & fixed_T_base,
  const Eigen::Vector3d & sensor_base, double stamp_s, const std::chrono::steady_clock::time_point & started)
{
  const Plane & plane = tracked.plane;
  const double edge_band_m = config_.traversability.edge_band_m;
  filtered_cloud_ = cell_filter_.apply(base_cloud_, plane, fixed_T_base, stamp_s, &filtered_raw_h_);
  raw_height_of_.clear();
  raw_height_of_.reserve(filtered_cloud_.size());
  for (std::size_t i = 0; i < filtered_cloud_.size(); ++i) {
    raw_height_of_.emplace(bits_of(filtered_cloud_[i]), filtered_raw_h_[i]);
  }
  if (config_.temporal.enabled) {
    raw_heights_.add(base_cloud_, plane, fixed_T_base, stamp_s);
  }
  set_gauge(METRIC_TEMPORAL_CELLS, static_cast<double>(cell_filter_.cell_count()));
  set_gauge(METRIC_TEMPORAL_GATED, static_cast<double>(cell_filter_.gated_cell_count()));
  // The shadow gate reads the cells, not the raw points: a cell's median over the window is stable to a few millimetres
  // where a raw point scatters by centimetres, so a 5 cm box is told from floor noise and its mixed-pixel tail (a
  // stable bias, which no per-point test removes) from the floor behind it.
  drop_occluded(plane, sensor_base);
  const Segmented seg = segment(filtered_cloud_, plane, config_.bands, config_.bands.floor_band_m);
  auto above = extract_clusters(
    seg.above, seg.above_height, "step", plane, config_.edges, edge_band_m, &unclustered_index_, &floor_reference_);
  gather(seg.above, seg.above_height, unclustered_index_, unclustered_above_, unclustered_above_h_);
  auto below = extract_clusters(
    seg.below, seg.below_height, "drop", plane, config_.edges, edge_band_m, &unclustered_index_, &floor_reference_);
  gather(seg.below, seg.below_height, unclustered_index_, unclustered_below_, unclustered_below_h_);
  refine_slopes(above, 1.0, fixed_T_base);
  refine_slopes(below, -1.0, fixed_T_base);
  Traversability rule = apply_rule(
    above, below, unclustered_above_, unclustered_above_h_, unclustered_below_, unclustered_below_h_,
    all_regions(above, below, config_.edges), config_.edges, config_.traversability);
  observe_classes(rule, fixed_T_base, stamp_s);
  fit_seconds_->Observe(seconds_since(started));

  // Lethal clouds: only this frame's points, only those in stable lethal cells (a lethal observation in a cell that is
  // not yet stable is withheld; warning points in a stable lethal cell are lethal), and of those only the lethal marks.
  lethal_marks_of(rule.obstacle, rule.warning_above, plane, fixed_T_base, stable_obstacle_);
  lethal_marks_of(rule.drop, rule.warning_below, plane, fixed_T_base, stable_drop_);
  const int64_t stamp_ns = ns_of(msg.header.stamp);
  append_edge_points(fixed_T_base, stamp_ns);

  cycle_ = Cycle{};
  cycle_.regions = std::move(rule.regions);
  for (Region & r : cycle_.regions) {
    const Cluster * c = cluster_of(r, above, below);
    r.cls = c != nullptr && any_stable_lethal(c->points, cell_class_, fixed_T_base) ? "obstacle" : "warning";
  }
  const Plane plane_fixed = transformed(plane, fixed_T_base);
  areas_fixed_ = stable_areas_fixed();
  const Eigen::Isometry3d base_T_fixed = fixed_T_base.inverse();
  for (const PublishedArea & published : areas_fixed_) {
    cycle_.warnings.push_back(moved(published.area, plane_fixed, base_T_fixed));
  }
  // The no_return areas are cell-aligned and stable on their own (NoReturnTracker hysteresis), published only while
  // the guidance that votes them is fresh.
  no_return_fixed_.clear();
  if (config_.guidance.enabled && config_.guidance.no_return.enabled) {
    if (guidance_fresh(stamp_ns)) {
      no_return_fixed_ = no_return_.areas(Eigen::Isometry3d::Identity());
    }
    for (const WarningArea & area : no_return_fixed_) {
      cycle_.warnings.push_back(moved(area, plane_fixed, base_T_fixed));
    }
    set_gauge(METRIC_NO_RETURN_AREAS, static_cast<double>(no_return_fixed_.size()));
  }
  cycle_.obstacle_points = stable_obstacle_.size();
  cycle_.drop_points = stable_drop_.size();

  // An empty cloud is a real "nothing lethal" observation with a valid plane.
  obstacle_pub_->publish(to_cloud_msg(stable_obstacle_, config_.base_frame, msg.header.stamp));
  drop_pub_->publish(to_cloud_msg(stable_drop_, config_.base_frame, msg.header.stamp));

  set_gauge(METRIC_OBSTACLE_POINTS, static_cast<double>(cycle_.obstacle_points));
  set_gauge(METRIC_DROP_POINTS, static_cast<double>(cycle_.drop_points));
  set_gauge(METRIC_REGIONS, static_cast<double>(cycle_.regions.size()));
  set_gauge(METRIC_WARNING_AREAS, static_cast<double>(cycle_.warnings.size()));
  set_gauge(
    METRIC_STABLE_LETHAL,
    static_cast<double>(cell_class_.count(CellClass::obstacle) + cell_class_.count(CellClass::drop)));
  set_gauge(METRIC_STABLE_WARNING, static_cast<double>(cell_class_.count(CellClass::warning)));
  set_gauge(METRIC_PLANE_VS_BASE, plane_vs_base_deg(plane));
  if (prior_.from_imu) {
    set_gauge(METRIC_SLOPE, slope_vs_gravity(plane, prior_.up, config_.slope).total_deg);
  }

  publish_warning_areas(msg.header.stamp, plane_fixed);
}

void GroundNode::refine_slopes(
  std::vector<Cluster> & clusters, double sign, const Eigen::Isometry3d & fixed_T_base) const
{
  // With the temporal median a cluster's slope comes from one sample per cell: a small sparse ramp has three or four,
  // straddling its foot, sides and top edge, and the fit scatters by several degrees from cloud to cloud. For an
  // inclined cluster its raw points over the slope history, in its cells and their neighbours, give the slope instead
  // (surface_slope.hpp).
  if (!config_.temporal.enabled) {
    return;
  }
  for (Cluster & c : clusters) {
    if (!c.slope_known || c.slope_deg < SLOPE_REFIT_MIN_DEG) {
      continue;
    }
    // A new inclined cluster is fit over the clouds kept so far: fewer clouds scatter more (~1 deg at
    // temporal.min_frames clouds for the sparse 14 deg mat) but a steep incline is lethal from its first frame, where
    // an unknown slope would leave it to its edge step, which a low incline passes.
    const auto slope = surface_slope_deg(raw_heights_.around(c.points, fixed_T_base), sign, config_.bands.floor_band_m);
    c.slope_known = slope.has_value();
    c.slope_deg = slope.value_or(0.0);
  }
}

void GroundNode::observe_classes(const Traversability & rule, const Eigen::Isometry3d & fixed_T_base, double stamp_s)
{
  cell_class_.begin_frame();
  observe_steady(rule.obstacle, CellClass::obstacle, fixed_T_base);
  observe_steady(rule.drop, CellClass::drop, fixed_T_base);
  for (const WarningArea & area : rule.warnings) {
    cell_class_.observe_area(area, fixed_T_base);
  }
  // The colour areas of the latest labelled RGB-D cloud, moved from its base_link to this cloud's base_link through
  // the fixed frame. Every colour cluster is a warning, never lethal.
  if (colour_ && std::fabs(stamp_s - colour_->stamp_s) <= config_.fit.hold_s) {
    const Eigen::Isometry3d base_T_colour = fixed_T_base.inverse() * colour_->fixed_T_base;
    for (const WarningArea & area : colour_->areas) {
      cell_class_.observe_area(moved(area, colour_->plane, base_T_colour), fixed_T_base);
    }
  }
  cell_class_.end_frame();
}

void GroundNode::lethal_marks_of(
  const pcl::PointCloud<pcl::PointXYZ> & lethal, const pcl::PointCloud<pcl::PointXYZ> & warning, const Plane & plane,
  const Eigen::Isometry3d & fixed_T_base, pcl::PointCloud<pcl::PointXYZ> & out)
{
  lethal_candidates_.clear();
  candidate_filtered_h_.clear();
  candidate_raw_h_.clear();
  for (const auto * cloud : {&lethal, &warning}) {
    for (const auto & p : *cloud) {
      const Eigen::Vector3d pb(p.x, p.y, p.z);
      const auto raw = raw_height_of_.find(bits_of(p));
      if (raw == raw_height_of_.end() || !is_lethal(cell_class_.stable_at((fixed_T_base * pb).head<2>()))) {
        continue;  // every rule point comes from filtered_cloud_; one without its own height is no evidence
      }
      lethal_candidates_.push_back(p);
      candidate_filtered_h_.push_back(static_cast<float>(height_above(plane, pb)));
      candidate_raw_h_.push_back(raw->second);
    }
  }
  out = lethal_marks(
    lethal_candidates_, candidate_filtered_h_, candidate_raw_h_, config_.max_step_m(),
    config_.edges.cluster_tolerance_m, config_.traversability.lethal_min_points);
}

void GroundNode::observe_steady(
  const pcl::PointCloud<pcl::PointXYZ> & points, CellClass cls, const Eigen::Isometry3d & fixed_T_base)
{
  const double max_spread_m = LETHAL_SPREAD_FRACTION * config_.temporal.max_spread_m;
  const int quiet_frames = LETHAL_QUIET_WINDOWS * config_.temporal.window_frames;
  steady_lethal_.clear();
  for (const auto & p : points) {
    if (cell_filter_.steady_at((fixed_T_base * Eigen::Vector3d(p.x, p.y, p.z)).head<2>(), max_spread_m, quiet_frames)) {
      steady_lethal_.push_back(p);
    }
  }
  cell_class_.observe_points(steady_lethal_, cls, fixed_T_base);
}

std::vector<GroundNode::PublishedArea> GroundNode::stable_areas_fixed() const
{
  const bool hold = config_.hysteresis.enabled;
  std::vector<PublishedArea> out;
  for (const StableArea & stable : cell_class_.warning_areas()) {
    PublishedArea next;
    next.area.kind = stable.kind;
    next.area.polygon = stable.polygon;
    next.area.height_m = stable.height_m;
    next.area.slope_deg = stable.slope_deg;
    next.area.source = stable.source;
    const auto prev = std::find_if(areas_fixed_.begin(), areas_fixed_.end(), [&](const PublishedArea & p) {
      return p.area.source == next.area.source && continues(p.area.polygon, next.area.polygon);
    });
    if (hold && prev != areas_fixed_.end()) {
      const WarningArea & held = prev->area;
      if (
        next.area.kind == held.kind && within_deadband(held.height_m, next.area.height_m, AREA_HEIGHT_DEADBAND_M) &&
        within_deadband(held.slope_deg, next.area.slope_deg, AREA_SLOPE_DEADBAND_DEG)) {
        next.area.height_m = held.height_m;
        next.area.slope_deg = held.slope_deg;
      }
    }
    out.push_back(std::move(next));
  }
  return out;
}

void GroundNode::publish_warning_areas(const builtin_interfaces::msg::Time & stamp, const Plane & plane_fixed)
{
  Eigen::Isometry3d frame_T_fixed;
  if (!lookup(config_.warnings.frame, config_.fixed_frame, stamp, frame_T_fixed)) {
    drop(
      DROP_REASON_NO_MAP_TF, "no TF " + config_.warnings.frame + " <- " + config_.fixed_frame + " at the cloud stamp");
    return;
  }
  areas_in_frame_.clear();
  for (const PublishedArea & published : areas_fixed_) {
    areas_in_frame_.push_back(moved(published.area, plane_fixed, frame_T_fixed));
  }
  for (const WarningArea & area : no_return_fixed_) {
    areas_in_frame_.push_back(moved(area, plane_fixed, frame_T_fixed));
  }
  std_msgs::msg::String out;
  out.data = to_warning_areas_string(ns_of(stamp), config_.warnings.frame, areas_in_frame_);
  warnings_pub_->publish(out);
}

void GroundNode::on_rgbd(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const int64_t stamp_ns = ns_of(msg->header.stamp);
  const double stamp_s = static_cast<double>(stamp_ns) / NS_PER_S;
  Eigen::Isometry3d fixed_T_base;
  Eigen::Isometry3d base_T_cloud;
  if (
    !lookup(config_.fixed_frame, config_.base_frame, msg->header.stamp, fixed_T_base) ||
    !lookup(config_.base_frame, msg->header.frame_id, msg->header.stamp, base_T_cloud)) {
    drop(DROP_REASON_TF, "no TF to " + config_.base_frame + " for the RGB-D cloud in " + msg->header.frame_id);
    return;
  }
  if (head_moving_at(msg->header.frame_id, base_T_cloud, msg->header.stamp, false)) {
    metrics_.counter(METRIC_HEAD_MOVING, HEAD_MOVING_HELP, {{"input", INPUT_RGBD}}).Increment();
    return;
  }
  const auto tracked = tracker_.current(fixed_T_base, stamp_s);
  if (!tracked) {
    drop(DROP_REASON_NO_PLANE_RGBD, "no plane within fit.hold_s at the RGB-D stamp");
    return;
  }
  if (!read_xyzrgb(*msg, base_T_cloud, rgbd_cloud_)) {
    drop(DROP_REASON_BAD_INPUT, "RGB-D cloud without float32 x, y, z and rgb");
    return;
  }
  const Plane & plane = tracked->plane;
  const LabelStats stats =
    label_cloud(rgbd_cloud_, plane, config_.bands, config_.colour, config_.bands.floor_band_m, colour_model_, labels_);
  mismatch_cloud_.clear();
  mismatch_h_.clear();
  for (const MismatchPoint & m : mismatch_points(rgbd_cloud_, labels_, plane)) {
    mismatch_cloud_.push_back(
      pcl::PointXYZ(static_cast<float>(m.point.x()), static_cast<float>(m.point.y()), static_cast<float>(m.point.z())));
    mismatch_h_.push_back(static_cast<float>(m.height));
  }
  ColourFrame frame;
  frame.areas = colour_warning_areas(mismatch_cloud_, mismatch_h_, plane, config_.edges, config_.traversability);
  frame.plane = plane;
  frame.fixed_T_base = fixed_T_base;
  frame.stamp_s = stamp_s;
  colour_ = std::move(frame);
  colour_inputs_ = ColourInputs{stats, colour_model_.ready(), colour_model_.samples(), stamp_ns};

  labelled_pub_->publish(to_labelled_msg(rgbd_cloud_, labels_, config_.base_frame, msg->header.stamp));
  labelled_->Increment();
  if (stats.mismatch_ratio) {
    set_gauge(METRIC_MISMATCH_RATIO, *stats.mismatch_ratio);
  }
}

void GroundNode::on_depth_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  depth_model_ = camera_model_of(*msg);
  depth_frame_ = msg->header.frame_id;
}

void GroundNode::on_guidance(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info,
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & rgb)
{
  const auto started = std::chrono::steady_clock::now();
  std::string detail;
  const char * reason = use_guidance(*depth, *info, *rgb, detail);
  metrics_
    .counter(
      METRIC_GUIDANCE_FRAMES, GUIDANCE_FRAMES_HELP, {{"result", reason != nullptr ? reason : GUIDANCE_RESULT_OK}})
    .Increment();
  if (reason != nullptr && std::strcmp(reason, GUIDANCE_RESULT_HEAD_MOVING) != 0) {
    drop(reason, detail);
  }
  guidance_seconds_->Observe(seconds_since(started));
}

const char * GroundNode::use_guidance(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & info,
  const sensor_msgs::msg::CompressedImage & rgb, std::string & detail)
{
  // 1. Decode.
  std::string why;
  const std::optional<GuidanceFrame> frame = decode_guidance(depth, info, rgb, &why);
  if (!frame) {
    detail = "guidance pair: " + why;
    return DROP_REASON_GUIDANCE_BAD_INPUT;
  }
  if (!depth_model_) {
    detail = "guidance pair before a usable depth camera_info on " + config_.guidance.depth_info_topic;
    return DROP_REASON_GUIDANCE_BAD_INPUT;
  }
  // 2. TF at the pair stamp: the pair's optical frame (the depth is registered to the RGB) in base_link, base_link in
  // the fixed frame and in the depth camera optical frame.
  const builtin_interfaces::msg::Time & stamp = depth.header.stamp;
  Eigen::Isometry3d base_T_cam;
  Eigen::Isometry3d fixed_T_base;
  Eigen::Isometry3d depth_T_base;
  if (
    !lookup(config_.base_frame, frame->frame_id, stamp, base_T_cam) ||
    !lookup(config_.fixed_frame, config_.base_frame, stamp, fixed_T_base) ||
    !lookup(depth_frame_, config_.base_frame, stamp, depth_T_base)) {
    detail = "no TF " + config_.base_frame + " <- " + frame->frame_id + ", " + config_.fixed_frame + " <- " +
             config_.base_frame + " or " + depth_frame_ + " <- " + config_.base_frame + " at the pair stamp";
    return DROP_REASON_GUIDANCE_TF;
  }
  // The head moved when the pair was captured: its depth and colour sit where the TF at the stamp does not say.
  if (head_moving_at(frame->frame_id, base_T_cam, stamp, false)) {
    metrics_.counter(METRIC_HEAD_MOVING, HEAD_MOVING_HELP, {{"input", INPUT_GUIDANCE}}).Increment();
    return GUIDANCE_RESULT_HEAD_MOVING;
  }
  // 3. The tracked plane at the pair stamp (the rule of the RGB-D cloud: no plane, no use).
  const auto tracked = tracker_.current(fixed_T_base, static_cast<double>(frame->stamp_ns) / NS_PER_S);
  if (!tracked) {
    detail = "no plane within fit.hold_s at the guidance pair stamp";
    return DROP_REASON_GUIDANCE_NO_PLANE;
  }
  // 4. Sample.
  guidance_grid_.ensure(frame->model, config_.guidance.sample_step_px);
  const GuidanceSamples samples =
    sample_guidance(*frame, guidance_grid_, base_T_cam, tracked->plane, *depth_model_, depth_T_base, config_.guidance);
  // 5. Keep the depth samples for the edge points and vote the no-return cells; the cells depth already sees are the
  // occlusion shadows of what it sees, never no-return cells.
  guidance_points_.set(samples, fixed_T_base, frame->stamp_ns, config_.guidance.max_age_s);
  if (config_.guidance.no_return.enabled) {
    no_return_.observe(samples, colour_model_, config_.colour, fixed_T_base, cell_class_.blocking_cells());
  }
  guidance_ns_ = frame->stamp_ns;
  return nullptr;
}

bool GroundNode::guidance_fresh(int64_t stamp_ns) const
{
  return config_.guidance.enabled && guidance_ns_ &&
         std::fabs(static_cast<double>(stamp_ns - *guidance_ns_) / NS_PER_S) <= config_.guidance.max_age_s;
}

void GroundNode::append_edge_points(const Eigen::Isometry3d & fixed_T_base, int64_t stamp_ns)
{
  if (!config_.guidance.enabled) {
    return;
  }
  if (!guidance_fresh(stamp_ns)) {
    metrics_.counter(METRIC_GUIDANCE_STALE, GUIDANCE_STALE_HELP).Increment();
  }
  std::size_t added = 0;
  // The kept points are the last good guidance per cell, each no older than guidance.max_age_s (an empty set gives no
  // edge point).
  if (config_.guidance.edge.enabled) {
    const auto [points, heights] =
      guidance_points_.points_in_base(fixed_T_base.inverse(), stamp_ns, config_.guidance.max_age_s);
    const EdgePoints edge = edge_points(points, heights, stable_obstacle_, stable_drop_, config_.guidance.edge);
    stable_obstacle_ += edge.obstacle;
    stable_drop_ += edge.drop;
    added = edge.obstacle.size() + edge.drop.size();
  }
  set_gauge(METRIC_EDGE_POINTS, static_cast<double>(added));
}

void GroundNode::on_state_timer()
{
  if (!have_cloud_) {
    return;  // no state from no data
  }
  const rclcpp::Time now = get_clock()->now();
  try {
    // The latest robot pose: the transform at now is not yet known when the timer fires.
    const Eigen::Isometry3d fixed_T_base =
      tf2::transformToEigen(tf_buffer_->lookupTransform(config_.fixed_frame, config_.base_frame, tf2::TimePointZero));
    publish_state(
      now.nanoseconds(), tracker_.current(fixed_T_base, static_cast<double>(now.nanoseconds()) / NS_PER_S), nullptr);
  } catch (const tf2::TransformException &) {
    publish_state(now.nanoseconds(), std::nullopt, REASON_NO_TF);
  }
}

void GroundNode::publish_state_on_change(int64_t stamp_ns, const std::optional<TrackedPlane> & plane)
{
  if (!last_state_valid_ || *last_state_valid_ != plane.has_value() || last_state_moving_ != head_moving_) {
    publish_state(stamp_ns, plane, nullptr);
  }
}

void GroundNode::publish_state(
  int64_t stamp_ns, const std::optional<TrackedPlane> & plane, const char * reason_override)
{
  StateInputs in;
  in.stamp_ns = stamp_ns;
  in.frame = config_.base_frame;
  in.prior = prior_;
  in.imu_ok = prior_.from_imu;
  in.plane = plane;
  in.inliers = last_inliers_;
  in.inlier_ratio = last_inlier_ratio_;
  if (reason_override != nullptr) {
    in.reason = reason_override;
  } else {
    in.reject = last_reject_;
    // No reject and no TF problem: the last fit simply aged out (no recent cloud).
    in.reason = last_reason_ != nullptr ? last_reason_ : REASON_NO_CLOUD;
  }
  if (plane) {
    in.plane_vs_base_deg = plane_vs_base_deg(plane->plane);
    if (prior_.from_imu) {
      in.slope = slope_vs_gravity(plane->plane, prior_.up, config_.slope);
    }
    in.obstacle_points = cycle_.obstacle_points;
    in.drop_points = cycle_.drop_points;
    in.regions = cycle_.regions;
    in.warnings = cycle_.warnings;
  }
  in.base_tilt_deg = prior_.from_imu ? base_tilt_deg(prior_.up) : 0.0;
  in.traversability = config_.traversability;
  in.head_moving = head_moving_;
  in.colour = colour_inputs_;
  std_msgs::msg::String out;
  out.data = to_state_string(in);
  state_pub_->publish(out);
  last_state_valid_ = plane.has_value();
  last_state_moving_ = head_moving_;
}

}  // namespace ground_estimator_cpp
