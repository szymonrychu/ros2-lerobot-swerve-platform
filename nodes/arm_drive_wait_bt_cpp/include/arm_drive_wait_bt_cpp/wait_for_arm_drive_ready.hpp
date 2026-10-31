// WaitForArmDriveReady: a Nav2 BT action node that holds a NavigateToPose goal until mcp_server lifted the arm to its
// drive pose (or skipped the lift, or the drive pose is disabled), at most timeout_s; it never fails the goal.
#ifndef ARM_DRIVE_WAIT_BT_CPP__WAIT_FOR_ARM_DRIVE_READY_HPP_
#define ARM_DRIVE_WAIT_BT_CPP__WAIT_FOR_ARM_DRIVE_READY_HPP_

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "arm_drive_wait_bt_cpp/readiness.hpp"

namespace arm_drive_wait_bt_cpp
{

constexpr const char * DEFAULT_TOPIC = "/arm/drive_pose/state";
constexpr double DEFAULT_TIMEOUT_S = 8.0;

// StatefulActionNode (it returns RUNNING while it waits). Ports: topic (the state topic), timeout_s (the longest wait).
// It uses the rclcpp node of the blackboard entry "node" (bt_navigator's), subscribes reliable + transient_local
// (depth 1) on its own callback group that no executor spins, takes the received messages on every tick, keeps the
// last state with its receipt time (steady clock) for readiness.hpp's freshness rule, and logs the outcome once per
// goal.
class WaitForArmDriveReady : public BT::StatefulActionNode
{
public:
  // name: the tree node name; config: the BT node configuration (ports, blackboard with "node").
  WaitForArmDriveReady(const std::string & name, const BT::NodeConfig & config);

  // The ports: topic (default /arm/drive_pose/state), timeout_s (default 8.0).
  static BT::PortsList providedPorts();

  // Start of a goal's wait: notes the start, returns SUCCESS unless a fresh state waits, else RUNNING.
  BT::NodeStatus onStart() override;

  // A tick while waiting: SUCCESS on a ready state or the timeout, else RUNNING.
  BT::NodeStatus onRunning() override;

  // The goal was cancelled or preempted while waiting: nothing to undo.
  void onHalted() override;

private:
  // Take the received states and decide (shared by onStart and onRunning).
  BT::NodeStatus check();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::optional<DrivePoseState> latest_;
  std::chrono::steady_clock::time_point received_;
  double timeout_s_{DEFAULT_TIMEOUT_S};
  std::chrono::steady_clock::time_point start_;
};

}  // namespace arm_drive_wait_bt_cpp

#endif  // ARM_DRIVE_WAIT_BT_CPP__WAIT_FOR_ARM_DRIVE_READY_HPP_
