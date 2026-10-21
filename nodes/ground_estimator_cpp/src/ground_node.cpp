#include "ground_estimator_cpp/ground_node.hpp"

#include <prometheus/counter.h>
#include <prometheus/histogram.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <initializer_list>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <utility>

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
constexpr std::array<double, 9> FIT_SECONDS_BUCKETS = {0.005, 0.01, 0.02, 0.03, 0.05, 0.1, 0.2, 0.5, 1.0};
constexpr std::array<FitReject, 6> ALL_REJECTS = {FitReject::too_few_points,   FitReject::no_plane,
                                                  FitReject::normal_off_prior, FitReject::offset_off_prior,
                                                  FitReject::too_few_inliers,  FitReject::low_inlier_ratio};
constexpr std::array<const char *, 4> ALL_DROP_REASONS = {
  DROP_REASON_TF, DROP_REASON_NO_PLANE_RGBD, DROP_REASON_NO_MAP_TF, DROP_REASON_BAD_INPUT};
constexpr const char * FITS_HELP = "Plane fits of ToF clouds, by result (ok or the reject reason)";
constexpr const char * DROPPED_HELP = "Clouds or warning-area messages dropped, by reason";

struct GaugeSpec
{
  const char * name;
  const char * help;
};

constexpr std::array<GaugeSpec, 9> GAUGES = {{
  {METRIC_INLIER_RATIO, "Inlier ratio of the last valid plane fit"},
  {METRIC_SLOPE, "Floor slope against gravity of the last cloud with a plane and a fresh IMU, degrees"},
  {METRIC_PLANE_VS_BASE, "Angle between the floor normal and base_link +z of the last cloud with a plane, degrees"},
  {METRIC_REGIONS, "Step and drop regions of the last cloud with a plane"},
  {METRIC_WARNING_AREAS, "Warning areas (ToF and colour) of the last cloud with a plane"},
  {METRIC_OBSTACLE_POINTS, "Points of the last obstacle_points cloud"},
  {METRIC_DROP_POINTS, "Points of the last drop_points cloud"},
  {METRIC_TEMPORAL_CELLS, "Cells the temporal median currently holds"},
  {METRIC_MISMATCH_RATIO, "Floor colour mismatch share of the floor band of the last labelled RGB-D cloud"},
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

double seconds_since(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

GroundNode::GroundNode(
  const Config & config, std::shared_ptr<tf2_ros::Buffer> tf_buffer, const rclcpp::NodeOptions & options)
: rclcpp::Node(NODE_NAME, options),
  config_(config),
  tf_buffer_(std::move(tf_buffer)),
  tracker_(config.fit),
  cell_filter_(config.temporal),
  colour_model_(config.colour),
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
  fit_seconds_ = &metrics_.histogram(
    METRIC_FIT_SECONDS, "Plane fit and segmentation time per ToF cloud",
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

  state_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / config_.state.rate_hz), [this] { on_state_timer(); });
  RCLCPP_INFO(
    get_logger(), "ground estimator: %s -> %s/{obstacle_points,drop_points,warning_areas,state}%s",
    config_.cloud_topic.c_str(), config_.output_namespace.c_str(),
    config_.colour.enabled ? (", " + config_.rgbd_cloud_topic + " -> labelled_points").c_str() : "");
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
  if (!read_xyz(*msg, base_T_cloud, base_cloud_)) {
    drop(DROP_REASON_BAD_INPUT, "ToF cloud without float32 x, y, z");
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
  if (tracked) {
    process_plane(*msg, *tracked, fixed_T_base, stamp_s, started);
  } else {
    // No valid or held plane: no cloud, no warning areas (no placeholder data); the cell history is stale.
    cell_filter_.reset();
    set_gauge(METRIC_TEMPORAL_CELLS, 0.0);
    fit_seconds_->Observe(seconds_since(started));
    cycle_ = Cycle{};
  }
  publish_state_on_change(stamp_ns, tracked);
}

void GroundNode::process_plane(
  const sensor_msgs::msg::PointCloud2 & msg, const TrackedPlane & tracked, const Eigen::Isometry3d & fixed_T_base,
  double stamp_s, const std::chrono::steady_clock::time_point & started)
{
  const Plane & plane = tracked.plane;
  const double edge_band_m = config_.traversability.edge_band_m;
  filtered_cloud_ = cell_filter_.apply(base_cloud_, plane, fixed_T_base, stamp_s);
  set_gauge(METRIC_TEMPORAL_CELLS, static_cast<double>(cell_filter_.cell_count()));
  const Segmented seg = segment(filtered_cloud_, plane, config_.bands, config_.fit.distance_m);
  const auto above =
    extract_clusters(seg.above, seg.above_height, "step", plane, config_.edges, edge_band_m, &unclustered_index_);
  gather(seg.above, seg.above_height, unclustered_index_, unclustered_above_, unclustered_above_h_);
  const auto below =
    extract_clusters(seg.below, seg.below_height, "drop", plane, config_.edges, edge_band_m, &unclustered_index_);
  gather(seg.below, seg.below_height, unclustered_index_, unclustered_below_, unclustered_below_h_);
  Traversability rule = apply_rule(
    above, below, unclustered_above_, unclustered_above_h_, unclustered_below_, unclustered_below_h_,
    all_regions(above, below, config_.edges), config_.edges, config_.traversability);
  fit_seconds_->Observe(seconds_since(started));

  cycle_ = Cycle{};
  cycle_.warnings = std::move(rule.warnings);
  // The colour areas of the latest labelled RGB-D cloud, moved from its base_link to this cloud's base_link through
  // the fixed frame. Every colour cluster is a warning, never lethal.
  if (colour_ && std::fabs(stamp_s - colour_->stamp_s) <= config_.fit.hold_s) {
    const Eigen::Isometry3d base_T_colour = fixed_T_base.inverse() * colour_->fixed_T_base;
    for (const WarningArea & area : colour_->areas) {
      cycle_.warnings.push_back(moved(area, colour_->plane, base_T_colour));
    }
  }
  cycle_.regions = std::move(rule.regions);
  cycle_.obstacle_points = rule.obstacle.size();
  cycle_.drop_points = rule.drop.size();

  // Lethal points only; an empty cloud is a real "nothing lethal" observation with a valid plane.
  obstacle_pub_->publish(to_cloud_msg(rule.obstacle, config_.base_frame, msg.header.stamp));
  drop_pub_->publish(to_cloud_msg(rule.drop, config_.base_frame, msg.header.stamp));

  set_gauge(METRIC_OBSTACLE_POINTS, static_cast<double>(cycle_.obstacle_points));
  set_gauge(METRIC_DROP_POINTS, static_cast<double>(cycle_.drop_points));
  set_gauge(METRIC_REGIONS, static_cast<double>(cycle_.regions.size()));
  set_gauge(METRIC_WARNING_AREAS, static_cast<double>(cycle_.warnings.size()));
  set_gauge(METRIC_PLANE_VS_BASE, plane_vs_base_deg(plane));
  if (prior_.from_imu) {
    set_gauge(METRIC_SLOPE, slope_vs_gravity(plane, prior_.up, config_.slope).total_deg);
  }

  publish_warning_areas(msg.header.stamp, plane);
}

void GroundNode::publish_warning_areas(const builtin_interfaces::msg::Time & stamp, const Plane & plane)
{
  Eigen::Isometry3d frame_T_base;
  if (!lookup(config_.warnings.frame, config_.base_frame, stamp, frame_T_base)) {
    drop(
      DROP_REASON_NO_MAP_TF, "no TF " + config_.warnings.frame + " <- " + config_.base_frame + " at the cloud stamp");
    return;
  }
  areas_in_frame_.clear();
  for (const WarningArea & area : cycle_.warnings) {
    areas_in_frame_.push_back(moved(area, plane, frame_T_base));
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
    label_cloud(rgbd_cloud_, plane, config_.bands, config_.colour, config_.fit.distance_m, colour_model_, labels_);
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
  if (!last_state_valid_ || *last_state_valid_ != plane.has_value()) {
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
  in.colour = colour_inputs_;
  std_msgs::msg::String out;
  out.data = to_state_string(in);
  state_pub_->publish(out);
  last_state_valid_ = plane.has_value();
}

}  // namespace ground_estimator_cpp
