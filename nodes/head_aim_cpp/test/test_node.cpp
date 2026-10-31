// HeadAimNode in-process: a probe node publishes the path, goal status, camera_info and foreign head commands, a
// static TF broadcaster provides odom -> base_link and base_link -> head_link (0.15, 0.10, 0.35), and the head commands
// tagged head_aim are collected and compared with the aim geometry of aim.hpp.
#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <thread>
#include <vector>

#include "head_aim_cpp/aim.hpp"
#include "head_aim_cpp/config.hpp"
#include "head_aim_cpp/head_aim_node.hpp"

namespace
{

using head_aim_cpp::Config;
using head_aim_cpp::Point2;
using head_aim_cpp::Point3;
using sensor_msgs::msg::JointState;
using std::chrono::milliseconds;

constexpr const char * ODOM = "odom";
constexpr const char * BASE = "base_link";
constexpr const char * HEAD = "head_link";
const Point3 HEAD_ORIGIN{0.15, 0.10, 0.35};
constexpr uint32_t IMAGE_HEIGHT = 480;
constexpr double FY = 433.0;
constexpr double PATH_STEP_M = 0.05;
constexpr double PATH_LENGTH_M = 2.0;
constexpr double TOL = 1e-6;
constexpr milliseconds INPUT_PERIOD{50};
constexpr double RATE_HZ = 20.0;

// The probe side of the test: inputs to the node and the collected head commands.
class Probe
{
public:
  explicit Probe(const Config & cfg)
  : node_(std::make_shared<rclcpp::Node>("head_aim_probe")),
    tf_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_))
  {
    plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(cfg.plan_topic, 5);
    info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(cfg.camera_info_topic, 5);
    status_pub_ = node_->create_publisher<action_msgs::msg::GoalStatusArray>(
      cfg.goal_status_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    command_pub_ = node_->create_publisher<JointState>(cfg.head_command_topic, 10);
    command_sub_ = node_->create_subscription<JointState>(cfg.head_command_topic, 10, [this](JointState::SharedPtr m) {
      if (m->header.frame_id == "head_aim") {
        commands.push_back(*m);
      }
    });
  }

  // Static transforms: odom -> base_link at yaw (rad), and base_link -> head_link when with_head.
  void publish_tf(double yaw, bool with_head)
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    geometry_msgs::msg::TransformStamped odom_base;
    odom_base.header.stamp = node_->now();
    odom_base.header.frame_id = ODOM;
    odom_base.child_frame_id = BASE;
    odom_base.transform.translation.x = 1.0;
    odom_base.transform.translation.y = 2.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom_base.transform.rotation.x = q.x();
    odom_base.transform.rotation.y = q.y();
    odom_base.transform.rotation.z = q.z();
    odom_base.transform.rotation.w = q.w();
    transforms.push_back(odom_base);
    if (with_head) {
      geometry_msgs::msg::TransformStamped base_head;
      base_head.header.stamp = node_->now();
      base_head.header.frame_id = BASE;
      base_head.child_frame_id = HEAD;
      base_head.transform.translation.x = HEAD_ORIGIN.x;
      base_head.transform.translation.y = HEAD_ORIGIN.y;
      base_head.transform.translation.z = HEAD_ORIGIN.z;
      base_head.transform.rotation.w = 1.0;
      transforms.push_back(base_head);
    }
    tf_->sendTransform(transforms);
  }

  void publish_status(int8_t status)
  {
    action_msgs::msg::GoalStatusArray msg;
    action_msgs::msg::GoalStatus s;
    s.status = status;
    msg.status_list.push_back(s);
    status_pub_->publish(msg);
  }

  void publish_foreign_command(const std::string & source)
  {
    JointState msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = source;
    msg.name = {"head_pan", "head_tilt"};
    msg.position = {0.3, 0.2};
    command_pub_->publish(msg);
  }

  // The periodic inputs: the path (in frame) and, when with_info, the camera_info.
  void publish_inputs(const std::string & frame, const std::vector<Point2> & points, bool with_info)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = frame;
    for (const Point2 & p : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    plan_pub_->publish(path);
    if (with_info) {
      sensor_msgs::msg::CameraInfo info;
      info.header.stamp = node_->now();
      info.height = IMAGE_HEIGHT;
      info.width = 640;
      info.k = {FY, 0.0, 320.0, 0.0, FY, 240.0, 0.0, 0.0, 1.0};
      info_pub_->publish(info);
    }
  }

  rclcpp::Node::SharedPtr node() const { return node_; }

  std::vector<JointState> commands;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub_;
  rclcpp::Publisher<JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<JointState>::SharedPtr command_sub_;
};

// A straight path from origin along bearing (rad).
std::vector<Point2> straight(const Point2 & origin, double bearing)
{
  std::vector<Point2> path;
  for (double s = 0.0; s <= PATH_LENGTH_M + 1e-9; s += PATH_STEP_M) {
    path.push_back({origin.x + s * std::cos(bearing), origin.y + s * std::sin(bearing)});
  }
  return path;
}

// The aim aim.hpp gives for a path already in base_link.
head_aim_cpp::Aim expected_aim(const std::vector<Point2> & base_path, const Config & cfg)
{
  const auto dir = head_aim_cpp::travel_direction(base_path, {0.0, 0.0}, cfg.lookahead_m, cfg.min_travel_speed_mps);
  EXPECT_TRUE(dir);
  const Point2 look = head_aim_cpp::look_point(base_path, *dir, cfg.footprint, cfg.lookahead_m);
  return head_aim_cpp::aim(
    look, HEAD_ORIGIN, head_aim_cpp::leading_edge_point(*dir, cfg.footprint),
    head_aim_cpp::vertical_fov(IMAGE_HEIGHT, FY), cfg);
}

class HeadAimNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void start(const Config & cfg)
  {
    cfg_ = cfg;
    probe_ = std::make_unique<Probe>(cfg_);
    node_ = std::make_shared<head_aim_cpp::HeadAimNode>(cfg_);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(probe_->node());
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->remove_node(node_);
      executor_->remove_node(probe_->node());
    }
  }

  // Spin for duration, publishing the inputs every INPUT_PERIOD; stops early once until() holds.
  void run(
    milliseconds duration, const std::string & frame, const std::vector<Point2> & path, bool with_info,
    const std::function<bool()> & until = [] { return false; })
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    auto next_input = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      if (std::chrono::steady_clock::now() >= next_input) {
        probe_->publish_inputs(frame, path, with_info);
        next_input += INPUT_PERIOD;
      }
      executor_->spin_some(milliseconds(10));
      if (until()) {
        return;
      }
      std::this_thread::sleep_for(milliseconds(5));
    }
  }

  static Config fast_config()
  {
    Config cfg;
    cfg.rate_hz = RATE_HZ;
    return cfg;
  }

  Config cfg_;
  std::unique_ptr<Probe> probe_;
  std::shared_ptr<head_aim_cpp::HeadAimNode> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

constexpr int8_t EXECUTING = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
constexpr int8_t SUCCEEDED = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;

}  // namespace

TEST_F(HeadAimNodeTest, ExecutingGoalWithForwardPathPublishesOneTaggedCommand)
{
  start(fast_config());
  probe_->publish_tf(0.0, true);
  probe_->publish_status(EXECUTING);
  const auto path = straight({0.0, 0.0}, 0.0);
  run(milliseconds(5000), BASE, path, true, [this] { return !probe_->commands.empty(); });
  ASSERT_EQ(probe_->commands.size(), 1u);
  run(milliseconds(1500), BASE, path, true);  // an unchanged target is not re-sent
  ASSERT_EQ(probe_->commands.size(), 1u);
  const JointState & cmd = probe_->commands.front();
  EXPECT_EQ(cmd.header.frame_id, "head_aim");
  ASSERT_EQ(cmd.name, (std::vector<std::string>{"head_pan", "head_tilt"}));
  const auto expected = expected_aim(path, cfg_);
  EXPECT_NEAR(cmd.position[0], expected.pan, TOL);
  EXPECT_NEAR(cmd.position[1], expected.tilt, TOL);
  EXPECT_LT(cmd.position[0], 0.0);  // straight ahead, seen from the head origin 0.10 m left
}

TEST_F(HeadAimNodeTest, NoHeadTransformPublishesNothing)
{
  start(fast_config());
  probe_->publish_tf(0.0, false);
  probe_->publish_status(EXECUTING);
  run(milliseconds(1500), BASE, straight({0.0, 0.0}, 0.0), true);
  EXPECT_TRUE(probe_->commands.empty());
}

TEST_F(HeadAimNodeTest, NoCameraInfoPublishesNothing)
{
  start(fast_config());
  probe_->publish_tf(0.0, true);
  probe_->publish_status(EXECUTING);
  run(milliseconds(1500), BASE, straight({0.0, 0.0}, 0.0), false);
  EXPECT_TRUE(probe_->commands.empty());
}

TEST_F(HeadAimNodeTest, NoGoalPublishesNothing)
{
  start(fast_config());
  probe_->publish_tf(0.0, true);
  run(milliseconds(1500), BASE, straight({0.0, 0.0}, 0.0), true);
  EXPECT_TRUE(probe_->commands.empty());
}

TEST_F(HeadAimNodeTest, WebUiCommandSuppressesForTheYieldWindow)
{
  Config cfg = fast_config();
  cfg.yield_s = 2.0;
  start(cfg);
  probe_->publish_tf(0.0, true);
  const auto path = straight({0.0, 0.0}, 0.0);
  run(milliseconds(300), BASE, path, true);
  probe_->publish_foreign_command("web_ui");
  run(milliseconds(200), BASE, path, true);
  probe_->publish_status(EXECUTING);
  run(milliseconds(1200), BASE, path, true);
  EXPECT_TRUE(probe_->commands.empty());
  run(milliseconds(3000), BASE, path, true, [this] { return !probe_->commands.empty(); });
  EXPECT_EQ(probe_->commands.size(), 1u);
}

TEST_F(HeadAimNodeTest, GoalSucceededSendsOneHomeCommand)
{
  start(fast_config());
  probe_->publish_tf(0.0, true);
  probe_->publish_status(EXECUTING);
  const auto path = straight({0.0, 0.0}, 0.0);
  run(milliseconds(5000), BASE, path, true, [this] { return !probe_->commands.empty(); });
  ASSERT_EQ(probe_->commands.size(), 1u);
  probe_->publish_status(SUCCEEDED);
  run(milliseconds(3000), BASE, path, true, [this] { return probe_->commands.size() >= 2; });
  run(milliseconds(1000), BASE, path, true);
  ASSERT_EQ(probe_->commands.size(), 2u);
  EXPECT_NEAR(probe_->commands[1].position[0], 0.0, TOL);
  EXPECT_NEAR(probe_->commands[1].position[1], 0.5236, TOL);
}

TEST_F(HeadAimNodeTest, PathInOdomIsTransformedIntoBaseLink)
{
  // The robot is yawed +90 deg in odom at (1, 2); a path along odom bearing 135 deg lies at bearing 45 deg in
  // base_link, so the head pans left by the base_link bearing, not the odom one.
  start(fast_config());
  probe_->publish_tf(M_PI / 2.0, true);
  probe_->publish_status(EXECUTING);
  const auto odom_path = straight({1.0, 2.0}, 135.0 * M_PI / 180.0);
  run(milliseconds(5000), ODOM, odom_path, true, [this] { return !probe_->commands.empty(); });
  ASSERT_EQ(probe_->commands.size(), 1u);
  const auto base_path = straight({0.0, 0.0}, 45.0 * M_PI / 180.0);
  const auto expected = expected_aim(base_path, cfg_);
  EXPECT_NEAR(probe_->commands.front().position[0], expected.pan, 1e-4);
  EXPECT_NEAR(probe_->commands.front().position[1], expected.tilt, 1e-4);
  EXPECT_GT(probe_->commands.front().position[0], 30.0 * M_PI / 180.0);
}
