#include "head_aim_cpp/head_aim_node.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace head_aim_cpp
{

namespace
{

constexpr int SKIP_LOG_THROTTLE_MS = 5000;
constexpr std::size_t COMMAND_QUEUE = 10;
constexpr std::size_t INPUT_QUEUE = 5;

// QoS of a ROS 2 action's status topic (rcl_action status default): reliable, transient local, depth 1, so the
// latest status arrives on subscribe.
rclcpp::QoS goal_status_qos() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); }

// Best effort matches both reliable and best-effort publishers.
rclcpp::QoS input_qos(std::size_t depth) { return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort(); }

tf2::Duration to_tf2(double seconds) { return tf2::durationFromSec(seconds); }

}  // namespace

HeadAimNode::HeadAimNode(const Config & cfg, const rclcpp::NodeOptions & options)
: rclcpp::Node(NODE_NAME, options),
  cfg_(cfg),
  gate_(cfg),
  plan_received_(0, 0, get_clock()->get_clock_type()),
  cmd_vel_received_(0, 0, get_clock()->get_clock_type()),
  camera_info_received_(0, 0, get_clock()->get_clock_type())
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    cfg_.plan_topic, input_qos(INPUT_QUEUE), std::bind(&HeadAimNode::on_plan, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cfg_.cmd_vel_topic, input_qos(INPUT_QUEUE), std::bind(&HeadAimNode::on_cmd_vel, this, std::placeholders::_1));
  status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
    cfg_.goal_status_topic, goal_status_qos(), std::bind(&HeadAimNode::on_goal_status, this, std::placeholders::_1));
  head_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    cfg_.head_command_topic, input_qos(COMMAND_QUEUE),
    std::bind(&HeadAimNode::on_head_command, this, std::placeholders::_1));
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    cfg_.camera_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&HeadAimNode::on_camera_info, this, std::placeholders::_1));
  command_pub_ = create_publisher<sensor_msgs::msg::JointState>(cfg_.head_command_topic, COMMAND_QUEUE);

  const auto period = std::chrono::duration<double>(1.0 / cfg_.rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&HeadAimNode::tick, this));
  RCLCPP_INFO(
    get_logger(), "aiming %s from %s at %.1f Hz while a goal on %s is active", cfg_.head_command_topic.c_str(),
    cfg_.plan_topic.c_str(), cfg_.rate_hz, cfg_.goal_status_topic.c_str());
}

void HeadAimNode::on_plan(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  plan_ = msg;
  plan_received_ = now();
}

void HeadAimNode::on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  cmd_vel_ = msg;
  cmd_vel_received_ = now();
}

void HeadAimNode::on_goal_status(action_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
{
  std::vector<int8_t> statuses;
  statuses.reserve(msg->status_list.size());
  for (const auto & s : msg->status_list) {
    statuses.push_back(s.status);
  }
  gate_.on_goal_status(any_goal_active(statuses), now().seconds());
}

void HeadAimNode::on_head_command(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  gate_.on_foreign_command(msg->header.frame_id, now().seconds());
}

void HeadAimNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  const double fy = msg->k[4];
  if (msg->height == 0 || !(fy > 0.0)) {
    skip("camera_info without height or fy");
    return;
  }
  vfov_rad_ = vertical_fov(static_cast<double>(msg->height), fy);
  camera_info_received_ = now();
}

bool HeadAimNode::fresh(const rclcpp::Time & received) const
{
  return received.nanoseconds() > 0 && (now() - received).seconds() <= cfg_.input_timeout_s;
}

void HeadAimNode::skip(const std::string & reason)
{
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), SKIP_LOG_THROTTLE_MS, "not aiming: %s", reason.c_str());
}

std::optional<Aim> HeadAimNode::target()
{
  if (!gate_.goal_active()) {
    return std::nullopt;
  }
  if (!vfov_rad_ || !fresh(camera_info_received_)) {
    skip("no fresh camera_info on " + cfg_.camera_info_topic);
    return std::nullopt;
  }
  if (!plan_ || !fresh(plan_received_)) {
    skip("no fresh path on " + cfg_.plan_topic);
    return std::nullopt;
  }
  geometry_msgs::msg::TransformStamped path_tf;
  geometry_msgs::msg::TransformStamped head_tf;
  try {
    const tf2::Duration timeout = to_tf2(cfg_.tf_timeout_s);
    path_tf = tf_buffer_->lookupTransform(cfg_.base_frame, plan_->header.frame_id, tf2::TimePointZero, timeout);
    head_tf = tf_buffer_->lookupTransform(cfg_.base_frame, cfg_.head_frame, tf2::TimePointZero, timeout);
  } catch (const tf2::TransformException & e) {
    skip(std::string("TF: ") + e.what());
    return std::nullopt;
  }
  tf2::Transform to_base;
  tf2::fromMsg(path_tf.transform, to_base);
  std::vector<Point2> path;
  path.reserve(plan_->poses.size());
  for (const auto & pose : plan_->poses) {
    const tf2::Vector3 p = to_base * tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    path.push_back({p.x(), p.y()});
  }
  Point2 velocity;
  if (cmd_vel_ && fresh(cmd_vel_received_)) {
    velocity = {cmd_vel_->linear.x, cmd_vel_->linear.y};
  }
  const auto direction = travel_direction(path, velocity, cfg_.lookahead_m, cfg_.min_travel_speed_mps);
  if (!direction) {
    skip("no travel direction (path shorter than 2 points and robot not moving)");
    return std::nullopt;
  }
  const Point3 head{head_tf.transform.translation.x, head_tf.transform.translation.y, head_tf.transform.translation.z};
  const Point2 look = look_point(path, *direction, cfg_.footprint, cfg_.lookahead_m);
  return aim(look, head, leading_edge_point(*direction, cfg_.footprint), *vfov_rad_, cfg_);
}

void HeadAimNode::tick()
{
  const auto aim_target = target();
  const auto command = gate_.decide(aim_target, now().seconds());
  if (!command) {
    return;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.header.frame_id = cfg_.source_tag;
  msg.name = {PAN_JOINT, TILT_JOINT};
  msg.position = {command->pan, command->tilt};
  command_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "%s pan %.1f deg tilt %.1f deg", command->home ? "home" : "aim", command->pan * 180.0 / M_PI,
    command->tilt * 180.0 / M_PI);
}

}  // namespace head_aim_cpp
