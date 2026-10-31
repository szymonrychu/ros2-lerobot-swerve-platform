#include "arm_drive_wait_bt_cpp/wait_for_arm_drive_ready.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "arm_drive_wait_bt_cpp/readiness.hpp"

namespace arm_drive_wait_bt_cpp
{

WaitForArmDriveReady::WaitForArmDriveReady(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  // A callback group no executor spins: check() takes the messages itself, so a goal sees every state already in
  // the reader (including the latched one), whatever ran before.
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  std::string topic = DEFAULT_TOPIC;
  getInput("topic", topic);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node_->create_subscription<std_msgs::msg::String>(
    topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), [](const std_msgs::msg::String::SharedPtr) {},
    options);
}

BT::PortsList WaitForArmDriveReady::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", DEFAULT_TOPIC, "drive pose state topic (mcp_server, latched JSON)"),
    BT::InputPort<double>("timeout_s", DEFAULT_TIMEOUT_S, "longest wait (s); the goal drives on after it")};
}

BT::NodeStatus WaitForArmDriveReady::onStart()
{
  timeout_s_ = DEFAULT_TIMEOUT_S;
  getInput("timeout_s", timeout_s_);
  start_ = std::chrono::steady_clock::now();
  return check();
}

BT::NodeStatus WaitForArmDriveReady::onRunning() { return check(); }

void WaitForArmDriveReady::onHalted() {}

BT::NodeStatus WaitForArmDriveReady::check()
{
  const auto now = std::chrono::steady_clock::now();
  std_msgs::msg::String msg;
  rclcpp::MessageInfo info;
  while (subscription_->take(msg, info)) {
    latest_ = parse_state(msg.data);
    received_ = now;
  }
  const double elapsed_s = std::chrono::duration<double>(now - start_).count();
  const double age_s = std::chrono::duration<double>(now - received_).count();
  const Readiness readiness = evaluate(latest_, age_s, elapsed_s, timeout_s_);
  if (!readiness.ready) {
    return BT::NodeStatus::RUNNING;
  }
  if (readiness.reason == TIMEOUT) {
    RCLCPP_WARN(
      node_->get_logger(), "WaitForArmDriveReady: arm drive pose still %s after %.1f s; driving on",
      latest_->state.c_str(), elapsed_s);
  } else if (readiness.reason == NO_FRESH_STATE) {
    RCLCPP_WARN(
      node_->get_logger(), "WaitForArmDriveReady: no arm drive pose state within %.1f s (mcp_server down?); driving on",
      FRESH_S);
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "WaitForArmDriveReady: arm drive pose %s after %.1f s", readiness.reason.c_str(), elapsed_s);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace arm_drive_wait_bt_cpp

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<arm_drive_wait_bt_cpp::WaitForArmDriveReady>("WaitForArmDriveReady");
}
