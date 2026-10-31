// WaitForArmDriveReady in a BT factory, loaded from the built plugin library (BT_REGISTER_NODES): a one-node tree is
// ticked while a test publisher sends /arm/drive_pose/state messages; RUNNING until a ready state stamped after the
// start arrives, then SUCCESS; SUCCESS after timeout_s without one; never FAILURE. The repo NavigateToPose tree
// (nodes/nav2_bringup/behavior_trees/navigate_to_pose_arm_wait.xml) loads into a factory with the nav2 built-in BT
// plugins and this one, and is the stock Jazzy tree behind the wait.
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/utils/shared_library.h>
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <nav2_behavior_tree/plugins_list.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using std::chrono::milliseconds;
using std::chrono::steady_clock;

constexpr const char * TEST_TOPIC = "/test/arm/drive_pose/state";
constexpr milliseconds TICK_PERIOD{20};
constexpr milliseconds SETTLE{300};
constexpr double LONG_TIMEOUT_S = 30.0;
constexpr double SHORT_TIMEOUT_S = 0.5;
constexpr int64_t NS_PER_S = 1000000000;
constexpr const char * STOCK_BT = "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml";
constexpr const char * RECOVERY_OPEN = "<RecoveryNode number_of_retries=\"6\" name=\"NavigateRecovery\">";
constexpr const char * RECOVERY_TAG = "<RecoveryNode ";
constexpr const char * RECOVERY_CLOSE = "</RecoveryNode>";

int64_t wall_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
    .count();
}

std::string state_json(const std::string & state, int64_t stamp_ns)
{
  return "{\"state\":\"" + state + "\",\"reason\":\"\",\"enabled\":true,\"stamp_ns\":" + std::to_string(stamp_ns) + "}";
}

std::string one_node_tree(double timeout_s)
{
  return "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
         "<WaitForArmDriveReady topic=\"" +
         std::string(TEST_TOPIC) + "\" timeout_s=\"" + std::to_string(timeout_s) + "\"/></BehaviorTree></root>";
}

// The library names of the nav2 built-in BT plugins, as bt_navigator loads them before plugin_lib_names.
std::vector<std::string> nav2_builtin_plugins()
{
  std::vector<std::string> libs;
  std::stringstream list(nav2::details::BT_BUILTIN_PLUGINS);
  std::string lib;
  while (std::getline(list, lib, ';')) {
    libs.push_back(lib);
  }
  return libs;
}

void register_nav2_builtin_plugins(BT::BehaviorTreeFactory & factory)
{
  for (const auto & lib : nav2_builtin_plugins()) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(lib));
  }
}

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

// The NavigateRecovery RecoveryNode of a tree file up to its closing tag, each line trimmed of its indentation.
std::string navigate_recovery(const std::string & text)
{
  const auto open = text.find(RECOVERY_OPEN);
  std::string block;
  if (open != std::string::npos) {
    std::stringstream lines(text.substr(open));
    std::string line;
    int depth = 0;
    while (std::getline(lines, line)) {
      const auto first = line.find_first_not_of(' ');
      if (first == std::string::npos) {
        continue;
      }
      const std::string trimmed = line.substr(first);
      block += trimmed + "\n";
      if (trimmed.rfind(RECOVERY_TAG, 0) == 0) {
        ++depth;
      } else if (trimmed == RECOVERY_CLOSE && --depth == 0) {
        break;
      }
    }
  }
  return block;
}

class WaitForArmDriveReadyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("bt_navigator_under_test");
    probe_ = std::make_shared<rclcpp::Node>("arm_drive_state_probe");
    pub_ = probe_->create_publisher<std_msgs::msg::String>(
      TEST_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    factory_.registerFromPlugin(ARM_DRIVE_WAIT_BT_LIBRARY);
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
  }

  void publish(const std::string & data)
  {
    std_msgs::msg::String msg;
    msg.data = data;
    pub_->publish(msg);
  }

  // Tick for `duration`, failing on any status other than RUNNING.
  void expect_running_for(BT::Tree & tree, milliseconds duration)
  {
    const auto end = steady_clock::now() + duration;
    while (steady_clock::now() < end) {
      ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
      std::this_thread::sleep_for(TICK_PERIOD);
    }
  }

  // Tick until the tree is no longer RUNNING (at most `limit`); returns the status and the elapsed seconds.
  std::pair<BT::NodeStatus, double> tick_until_done(BT::Tree & tree, milliseconds limit)
  {
    const auto start = steady_clock::now();
    BT::NodeStatus status = tree.tickOnce();
    while (status == BT::NodeStatus::RUNNING && steady_clock::now() - start < limit) {
      std::this_thread::sleep_for(TICK_PERIOD);
      status = tree.tickOnce();
    }
    return {status, std::chrono::duration<double>(steady_clock::now() - start).count()};
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

}  // namespace

TEST_F(WaitForArmDriveReadyTest, RunningUntilLiftedThenSuccess)
{
  auto tree = factory_.createTreeFromText(one_node_tree(LONG_TIMEOUT_S), blackboard_);
  expect_running_for(tree, SETTLE);
  publish(state_json("lifting", wall_ns()));
  expect_running_for(tree, SETTLE);
  publish(state_json("lifted", wall_ns()));
  const auto [status, elapsed_s] = tick_until_done(tree, milliseconds(5000));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_LT(elapsed_s, 2.0);
}

TEST_F(WaitForArmDriveReadyTest, SkippedAndDisabledAreReady)
{
  for (const std::string state : {"skipped: torque off", "disabled"}) {
    auto tree = factory_.createTreeFromText(one_node_tree(LONG_TIMEOUT_S), blackboard_);
    expect_running_for(tree, SETTLE);
    publish(state_json(state, wall_ns()));
    EXPECT_EQ(tick_until_done(tree, milliseconds(5000)).first, BT::NodeStatus::SUCCESS) << state;
  }
}

TEST_F(WaitForArmDriveReadyTest, ALatchedStateFromBeforeTheStartDoesNotCount)
{
  publish(state_json("lifted", wall_ns() - NS_PER_S));
  auto tree = factory_.createTreeFromText(one_node_tree(LONG_TIMEOUT_S), blackboard_);
  expect_running_for(tree, SETTLE);
  publish("not json");
  expect_running_for(tree, SETTLE);
  publish(state_json("lifted", wall_ns()));
  EXPECT_EQ(tick_until_done(tree, milliseconds(5000)).first, BT::NodeStatus::SUCCESS);
}

TEST_F(WaitForArmDriveReadyTest, SuccessAfterTheTimeoutWithoutAMessage)
{
  auto tree = factory_.createTreeFromText(one_node_tree(SHORT_TIMEOUT_S), blackboard_);
  const auto [status, elapsed_s] = tick_until_done(tree, milliseconds(5000));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_GE(elapsed_s, SHORT_TIMEOUT_S - 0.05);
  EXPECT_LT(elapsed_s, SHORT_TIMEOUT_S + 1.0);
}

TEST_F(WaitForArmDriveReadyTest, EveryGoalWaitsAgain)
{
  auto tree = factory_.createTreeFromText(one_node_tree(LONG_TIMEOUT_S), blackboard_);
  expect_running_for(tree, SETTLE);
  publish(state_json("lifted", wall_ns()));
  ASSERT_EQ(tick_until_done(tree, milliseconds(5000)).first, BT::NodeStatus::SUCCESS);
  // The next goal ticks the same tree again: the old lifted stamp predates this start.
  std::this_thread::sleep_for(milliseconds(10));
  expect_running_for(tree, SETTLE);
  publish(state_json("lifted", wall_ns()));
  EXPECT_EQ(tick_until_done(tree, milliseconds(5000)).first, BT::NodeStatus::SUCCESS);
}

TEST_F(WaitForArmDriveReadyTest, DefaultPorts)
{
  auto tree = factory_.createTreeFromText(
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
    "<WaitForArmDriveReady/></BehaviorTree></root>",
    blackboard_);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  auto default_pub = probe_->create_publisher<std_msgs::msg::String>(
    "/arm/drive_pose/state", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  std_msgs::msg::String msg;
  msg.data = state_json("disabled", wall_ns());
  default_pub->publish(msg);
  EXPECT_EQ(tick_until_done(tree, milliseconds(5000)).first, BT::NodeStatus::SUCCESS);
}

TEST(NavigateToPoseArmWaitTree, LoadsWithTheNav2PluginsAndThisNode)
{
  BT::BehaviorTreeFactory factory;
  register_nav2_builtin_plugins(factory);
  factory.registerFromPlugin(ARM_DRIVE_WAIT_BT_LIBRARY);
  ASSERT_NO_THROW(factory.registerBehaviorTreeFromFile(ARM_WAIT_BT_XML));
  const auto trees = factory.registeredBehaviorTrees();
  EXPECT_EQ(trees, std::vector<std::string>{"MainTree"});
}

TEST(NavigateToPoseArmWaitTree, DoesNotLoadWithoutThisNode)
{
  BT::BehaviorTreeFactory factory;
  register_nav2_builtin_plugins(factory);
  EXPECT_ANY_THROW(factory.registerBehaviorTreeFromFile(ARM_WAIT_BT_XML));
}

TEST(NavigateToPoseArmWaitTree, IsTheStockJazzyTreeBehindTheWait)
{
  const std::string stock =
    read_file(ament_index_cpp::get_package_share_directory("nav2_bt_navigator") + "/behavior_trees/" + STOCK_BT);
  const std::string repo = read_file(ARM_WAIT_BT_XML);
  const std::string stock_recovery = navigate_recovery(stock);
  ASSERT_FALSE(stock_recovery.empty());
  EXPECT_EQ(navigate_recovery(repo), stock_recovery);
  EXPECT_NE(repo.find("<Sequence>\n      <WaitForArmDriveReady"), std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
