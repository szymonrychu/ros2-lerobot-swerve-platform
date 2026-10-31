// HeadAimNode: the ROS wiring of the head aim. While a NavigateToPose goal is active, a timer at rate_hz transforms the
// latest local path into base_frame, reads the head origin from TF, runs the aim geometry (aim.hpp) and the
// step-and-settle / yield gate (gate.hpp), and publishes head_command_topic tagged source_tag. A missing or stale path,
// a failed TF lookup or no camera_info: a throttled log line and nothing published (no placeholder data).
#ifndef HEAD_AIM_CPP__HEAD_AIM_NODE_HPP_
#define HEAD_AIM_CPP__HEAD_AIM_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "head_aim_cpp/aim.hpp"
#include "head_aim_cpp/config.hpp"
#include "head_aim_cpp/gate.hpp"

namespace head_aim_cpp
{

constexpr const char * NODE_NAME = "head_aim";
constexpr const char * PAN_JOINT = "head_pan";
constexpr const char * TILT_JOINT = "head_tilt";

class HeadAimNode : public rclcpp::Node
{
public:
  // @param cfg the validated config (Config).
  // @param options node options (rclcpp::NodeOptions).
  explicit HeadAimNode(const Config & cfg, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_plan(nav_msgs::msg::Path::ConstSharedPtr msg);
  void on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void on_goal_status(action_msgs::msg::GoalStatusArray::ConstSharedPtr msg);
  void on_head_command(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void tick();

  // The aim for the latest inputs, nullopt (with a throttled log line naming why) when one is missing or stale.
  // @return the target (Aim) or nullopt.
  std::optional<Aim> target();

  // Log a skipped cycle, throttled.
  // @param reason why nothing was aimed (std::string).
  void skip(const std::string & reason);

  bool fresh(const rclcpp::Time & received) const;

  Config cfg_;
  AimGate gate_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path::ConstSharedPtr plan_;
  rclcpp::Time plan_received_;
  geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_;
  rclcpp::Time cmd_vel_received_;
  std::optional<double> vfov_rad_;
  rclcpp::Time camera_info_received_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr head_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace head_aim_cpp

#endif  // HEAD_AIM_CPP__HEAD_AIM_NODE_HPP_
