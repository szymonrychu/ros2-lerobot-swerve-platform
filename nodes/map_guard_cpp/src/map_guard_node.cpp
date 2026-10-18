#include "map_guard_cpp/map_guard_node.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <future>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.h"

namespace map_guard_cpp
{

namespace
{

std::chrono::nanoseconds seconds(double s)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(s));
}

// The reasons as Python prints a list of str: ['mismatch', 'lifted'].
std::string reasons_repr(const std::vector<std::string> & reasons)
{
  std::string out = "[";
  for (const auto & reason : reasons) {
    out += (out.size() > 1 ? ", '" : "'") + reason + "'";
  }
  return out + "]";
}

}  // namespace

rclcpp::QoS sensor_qos() { return rclcpp::QoS(rclcpp::KeepLast(SENSOR_QOS_DEPTH)).best_effort(); }

rclcpp::QoS map_qos() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); }

double monotonic_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

GridMap grid_from_msg(const nav_msgs::msg::OccupancyGrid & msg)
{
  const auto & info = msg.info;
  const auto & q = info.origin.orientation;
  GridMap grid;
  grid.resolution = info.resolution;
  grid.origin_x = info.origin.position.x;
  grid.origin_y = info.origin.position.y;
  grid.origin_yaw = quaternion_yaw({q.x, q.y, q.z, q.w});
  grid.width = static_cast<int>(info.width);
  grid.height = static_cast<int>(info.height);
  grid.cells.assign(msg.data.begin(), msg.data.end());
  return grid;
}

// --- SlamServices ---------------------------------------------------------------------------------------------------

SlamServices::SlamServices(rclcpp::Node & node, const Config & config)
: logger_(node.get_logger()),
  timeout_s_(config.service_timeout_s),
  pause_client_(node.create_client<slam_toolbox::srv::Pause>(config.pause_service)),
  reset_client_(node.create_client<slam_toolbox::srv::Reset>(config.reset_service)),
  serialize_client_(node.create_client<slam_toolbox::srv::SerializePoseGraph>(config.serialize_service)),
  params_client_(node.create_client<rcl_interfaces::srv::GetParameters>(config.parameters_service))
{
}

template <typename ServiceT>
typename ServiceT::Response::SharedPtr SlamServices::call(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client, typename ServiceT::Request::SharedPtr request)
{
  if (!client->wait_for_service(seconds(SERVICE_WAIT_S))) {
    RCLCPP_WARN(logger_, "service %s unavailable", client->get_service_name());
    return nullptr;
  }
  auto future = client->async_send_request(request);
  if (future.wait_for(seconds(timeout_s_)) != std::future_status::ready) {
    client->remove_pending_request(future);
    RCLCPP_WARN(logger_, "service %s timed out after %g s", client->get_service_name(), timeout_s_);
    return nullptr;
  }
  return future.get();
}

std::optional<bool> SlamServices::query_paused()
{
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {PAUSED_PARAMETER};
  const auto response = call<rcl_interfaces::srv::GetParameters>(params_client_, request);
  if (
    !response || response->values.empty() ||
    response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
    return std::nullopt;
  }
  return static_cast<bool>(response->values[0].bool_value);
}

bool SlamServices::toggle_pause()
{
  const auto response =
    call<slam_toolbox::srv::Pause>(pause_client_, std::make_shared<slam_toolbox::srv::Pause::Request>());
  return response && response->status;
}

bool SlamServices::serialize_map(const std::string & filename)
{
  auto request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
  request->filename = filename;
  const auto response = call<slam_toolbox::srv::SerializePoseGraph>(serialize_client_, request);
  return response && response->result == slam_toolbox::srv::SerializePoseGraph::Response::RESULT_SUCCESS;
}

bool SlamServices::reset()
{
  auto request = std::make_shared<slam_toolbox::srv::Reset::Request>();
  request->pause_new_measurements = false;  // the pause state is left as it is
  const auto response = call<slam_toolbox::srv::Reset>(reset_client_, request);
  return response && response->result == slam_toolbox::srv::Reset::Response::RESULT_SUCCESS;
}

// --- MapGuardNode ---------------------------------------------------------------------------------------------------

MapGuardNode::MapGuardNode(const rclcpp::NodeOptions & options) : MapGuardNode(options, load_config_or_exit(), true) {}

MapGuardNode::MapGuardNode(const rclcpp::NodeOptions & options, const Config & config, bool serve_metrics)
: Node(NODE_NAME, options),
  config_(config),
  metrics_(METRICS_NODE),
  guard_metrics_(metrics_),
  detectors_(config_),
  agreement_limiter_(config_.agreement_hz)
{
  if (serve_metrics) {
    std::optional<int> port;
    if (config_.metrics_port) {
      port = static_cast<int>(*config_.metrics_port);
    }
    if (const auto resolved = ros2_common_cpp::resolve_metrics_port(port)) {
      metrics_.serve(*resolved);
    }
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // No spin thread: the /tf subscriptions run on this node's (single-threaded) executor, as rclpy's listener does.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  events_pub_ = create_publisher<std_msgs::msg::String>(config_.events_topic, EVENTS_QOS_DEPTH);
  reset_pub_ = create_publisher<std_msgs::msg::Empty>(config_.reset_topic, EVENTS_QOS_DEPTH);
  notifier_ = std::make_unique<EventNotifier>(
    AgentState(config_.agent_state_url, config_.agent_poll_s, config_.agent_timeout_s),
    [this](const std::string & text) {
      std_msgs::msg::String msg;
      msg.data = text;
      events_pub_->publish(msg);
    },
    config_.dry_run);
  slam_ = std::make_unique<SlamServices>(*this, config_);
  command_pub_ = create_publisher<std_msgs::msg::String>(config_.command_topic, COMMAND_QOS_DEPTH);
  guard_ = std::make_unique<MapGuard>(
    config_, *slam_, *notifier_, [this]() { reset_pub_->publish(std_msgs::msg::Empty()); },
    [this](const std::string & request_id, double since) { request_relocalise(request_id, since); },
    [this](const std::string & line) { RCLCPP_WARN(get_logger(), "%s", line.c_str()); }, guard_metrics_);

  using std::placeholders::_1;
  subscriptions_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
    config_.map_topic, map_qos(), std::bind(&MapGuardNode::on_map, this, _1)));
  subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
    config_.localised_topic, map_qos(), std::bind(&MapGuardNode::on_localised, this, _1)));
  subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
    config_.result_topic, rclcpp::QoS(COMMAND_QOS_DEPTH), std::bind(&MapGuardNode::on_result, this, _1)));
  subscriptions_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
    config_.scan_topic, sensor_qos(),
    [this](std::shared_ptr<const rclcpp::SerializedMessage> serialized) { on_scan(serialized); }));
  subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>(
    config_.imu_topic, sensor_qos(), std::bind(&MapGuardNode::on_imu, this, _1)));
  subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
    config_.wheel_odom_topic, sensor_qos(), std::bind(&MapGuardNode::on_wheel_odom, this, _1)));
  subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
    config_.lidar_odom_topic, sensor_qos(), std::bind(&MapGuardNode::on_lidar_odom, this, _1)));
  subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>(
    config_.cmd_vel_topic, sensor_qos(), std::bind(&MapGuardNode::on_cmd, this, _1)));

  RCLCPP_INFO(
    get_logger(), "map_guard: dry_run=%s, reasons=%s, scan=%s, map=%s", config_.dry_run ? "True" : "False",
    reasons_repr(config_.reasons).c_str(), config_.scan_topic.c_str(), config_.map_topic.c_str());
  worker_ = std::thread([this]() { run_guard(); });
}

MapGuardNode::~MapGuardNode()
{
  {
    std::lock_guard<std::mutex> lock(stop_lock_);
    stop_ = true;
  }
  stop_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void MapGuardNode::on_map(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0f) {
    return;
  }
  auto prepared = std::make_shared<const PreparedMap>(prepare_map(
    grid_from_msg(*msg), static_cast<int>(config_.occupied_threshold), static_cast<int>(config_.hit_radius_cells)));
  std::lock_guard<std::mutex> lock(lock_);
  prepared_ = std::move(prepared);
}

void MapGuardNode::on_localised(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(lock_);
  held_ = !msg->data;
  if (held_) {
    pending_.clear();  // samples scored against the not yet localised map are dropped
  }
}

void MapGuardNode::request_relocalise(const std::string & request_id, double since)
{
  RCLCPP_WARN(
    get_logger(), "map_guard: requesting relocalise %s on %s", request_id.c_str(), config_.command_topic.c_str());
  std_msgs::msg::String msg;
  msg.data = relocalise_command(request_id, since);
  command_pub_->publish(msg);
}

void MapGuardNode::on_result(const std_msgs::msg::String::ConstSharedPtr & msg)
{
  const auto result = parse_result(msg->data);
  if (!result) {
    return;
  }
  // The guard worker thread owns the state machine (its service calls block): it applies the answer on its next tick.
  std::lock_guard<std::mutex> lock(lock_);
  results_.push_back(*result);
}

void MapGuardNode::on_scan(const std::shared_ptr<const rclcpp::SerializedMessage> & serialized)
{
  // Only the scans the rate limit lets through are deserialized, transformed and scored.
  if (!agreement_limiter_.ready(monotonic_s())) {
    return;
  }
  std::shared_ptr<const PreparedMap> prepared;
  {
    std::lock_guard<std::mutex> lock(lock_);
    prepared = prepared_;
  }
  if (!prepared) {
    return;
  }
  sensor_msgs::msg::LaserScan scan;
  scan_serialization_.deserialize_message(serialized.get(), &scan);
  geometry_msgs::msg::Transform tf;
  try {
    tf = tf_buffer_->lookupTransform(config_.map_frame, scan.header.frame_id, tf2::TimePointZero).transform;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_DEBUG(
      get_logger(), "no %s <- %s transform: %s", config_.map_frame.c_str(), scan.header.frame_id.c_str(), exc.what());
    return;
  }
  const Points points = scan_endpoints(
    scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max, config_.max_range_m);
  const auto & t = tf.translation;
  const auto & r = tf.rotation;
  const auto result = agreement(
    transform_points(points, {t.x, t.y, t.z}, {r.x, r.y, r.z, r.w}), prepared.get(),
    static_cast<int>(config_.min_points));
  if (!result) {
    return;
  }
  std::lock_guard<std::mutex> lock(lock_);
  if (held_) {
    return;
  }
  pending_.emplace_back(result->ratio, prepared->known_cells);
}

void MapGuardNode::on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  const auto & q = msg->orientation;
  if (msg->orientation_covariance[0] == -1.0 || (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0)) {
    return;  // no orientation in this sample
  }
  const auto & w = msg->angular_velocity;
  const auto & a = msg->linear_acceleration;
  std::lock_guard<std::mutex> lock(lock_);
  detectors_.on_imu(monotonic_s(), {q.x, q.y, q.z, q.w}, {w.x, w.y, w.z}, {a.x, a.y, a.z});
}

void MapGuardNode::on_wheel_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const auto & twist = msg->twist.twist;
  std::lock_guard<std::mutex> lock(lock_);
  detectors_.on_wheel_odom(monotonic_s(), std::hypot(twist.linear.x, twist.linear.y), twist.angular.z);
}

void MapGuardNode::on_lidar_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const auto & position = msg->pose.pose.position;
  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  const PlanarStamp current{position.x, position.y, stamp};
  const auto & twist = msg->twist.twist;
  const double speed = odom_speed(twist.linear.x, twist.linear.y, lidar_pose_, current);
  lidar_pose_ = current;
  std::lock_guard<std::mutex> lock(lock_);
  detectors_.on_lidar_odom(monotonic_s(), speed);
}

void MapGuardNode::on_cmd(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(lock_);
  detectors_.on_cmd(monotonic_s(), std::hypot(msg->linear.x, msg->linear.y), msg->angular.z);
}

void MapGuardNode::run_guard()
{
  const auto period = seconds(1.0 / config_.tick_hz);
  while (rclcpp::ok()) {
    double now = 0.0;
    DisturbanceReport report;
    std::vector<MapGuard::Sample> samples;
    std::vector<RelocaliseResult> results;
    bool held = false;
    {
      std::lock_guard<std::mutex> lock(lock_);
      held = held_;
      now = monotonic_s();
      report = detectors_.evaluate(now);
      samples.swap(pending_);
      results.swap(results_);
    }
    try {
      guard_->set_hold(held);
      for (const auto & result : results) {
        guard_->on_relocalise_result(result.request_id, result.ok, result.state, result.discarded_pois);
      }
      guard_->tick(now, report, samples);
    } catch (const std::exception & exc) {  // keep guarding after an unexpected error
      RCLCPP_ERROR(get_logger(), "map_guard tick failed: %s", exc.what());
    }
    state_ = guard_->state();
    std::unique_lock<std::mutex> lock(stop_lock_);
    if (stop_cv_.wait_for(lock, period, [this]() { return stop_; })) {
      return;
    }
  }
}

}  // namespace map_guard_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(map_guard_cpp::MapGuardNode)
