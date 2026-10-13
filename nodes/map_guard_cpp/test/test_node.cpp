// Port of nodes/map_guard/tests/test_node.py (grid conversion, slam_toolbox service calls with the pause parameter
// read, unavailable and timed-out services, result codes) against a fake slam_toolbox node, plus an end-to-end run
// of MapGuardNode: a latched map, a static TF and scans give an agreement ratio, odometry of a carried robot moves the
// guard to DISTURBED, and in dry run no slam_toolbox service is ever called.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "map_guard_cpp/map_guard_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace mg = map_guard_cpp;
using namespace std::chrono_literals;

namespace
{

// slam_toolbox stand-in: paused_new_measurements parameter (served by the node's parameter services), the Pause
// toggle, Reset and SerializePoseGraph with configurable answers and an optional delay.
class FakeSlamToolbox : public rclcpp::Node
{
public:
  explicit FakeSlamToolbox(const std::string & name = "slam_toolbox") : Node(name)
  {
    declare_parameter(mg::PAUSED_PARAMETER, false);
    pause_ = create_service<slam_toolbox::srv::Pause>(
      "/" + name + "/pause_new_measurements",
      [this](
        const slam_toolbox::srv::Pause::Request::SharedPtr, slam_toolbox::srv::Pause::Response::SharedPtr response) {
        ++pause_calls;
        std::this_thread::sleep_for(delay);
        if (toggle_works) {
          set_parameter(rclcpp::Parameter(mg::PAUSED_PARAMETER, !get_parameter(mg::PAUSED_PARAMETER).as_bool()));
        }
        response->status = toggle_status;
      });
    reset_ = create_service<slam_toolbox::srv::Reset>(
      "/" + name + "/reset", [this](
                               const slam_toolbox::srv::Reset::Request::SharedPtr request,
                               slam_toolbox::srv::Reset::Response::SharedPtr response) {
        ++reset_calls;
        reset_pause_flag = request->pause_new_measurements;
        response->result = reset_result;
      });
    serialize_ = create_service<slam_toolbox::srv::SerializePoseGraph>(
      "/" + name + "/serialize_map", [this](
                                       const slam_toolbox::srv::SerializePoseGraph::Request::SharedPtr request,
                                       slam_toolbox::srv::SerializePoseGraph::Response::SharedPtr response) {
        ++serialize_calls;
        serialized = request->filename;
        response->result = serialize_result;
      });
  }

  std::chrono::milliseconds delay{0};
  bool toggle_works = true;
  bool toggle_status = true;
  std::uint8_t reset_result = slam_toolbox::srv::Reset::Response::RESULT_SUCCESS;
  std::uint8_t serialize_result = slam_toolbox::srv::SerializePoseGraph::Response::RESULT_SUCCESS;
  std::atomic<int> pause_calls{0};
  std::atomic<int> reset_calls{0};
  std::atomic<int> serialize_calls{0};
  bool reset_pause_flag = true;
  std::string serialized;

private:
  rclcpp::Service<slam_toolbox::srv::Pause>::SharedPtr pause_;
  rclcpp::Service<slam_toolbox::srv::Reset>::SharedPtr reset_;
  rclcpp::Service<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serialize_;
};

class RosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    client_node_ = std::make_shared<rclcpp::Node>("map_guard_test_client");
    fake_ = std::make_shared<FakeSlamToolbox>();
    executor_->add_node(client_node_);
    executor_->add_node(fake_);
    spinner_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    spinner_.join();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<FakeSlamToolbox> fake_;
  std::thread spinner_;
};

// A 4 m x 4 m square room (walls at |x| = 2 or |y| = 2) on a 0.05 m grid with its origin at (-3, -3).
nav_msgs::msg::OccupancyGrid room_map()
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.resolution = 0.05f;
  msg.info.width = 120;
  msg.info.height = 120;
  msg.info.origin.position.x = -3.0;
  msg.info.origin.position.y = -3.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(120 * 120, -1);
  for (int row = 0; row < 120; ++row) {
    for (int col = 0; col < 120; ++col) {
      const double x = -3.0 + (col + 0.5) * 0.05;
      const double y = -3.0 + (row + 0.5) * 0.05;
      if (std::abs(x) < 2.0 && std::abs(y) < 2.0) {
        msg.data[row * 120 + col] = 0;
      }
      if (
        (std::abs(std::abs(x) - 2.0) < 0.025 + 1e-9 && std::abs(y) <= 2.05) ||
        (std::abs(std::abs(y) - 2.0) < 0.025 + 1e-9 && std::abs(x) <= 2.05)) {
        msg.data[row * 120 + col] = 100;
      }
    }
  }
  return msg;
}

// Scan of the room from the origin (laser frame = map frame).
sensor_msgs::msg::LaserScan room_scan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = static_cast<float>(-M_PI);
  scan.angle_increment = static_cast<float>(2 * M_PI / 360);
  scan.range_min = 0.15f;
  scan.range_max = 12.0f;
  for (int i = 0; i < 360; ++i) {
    const double a = scan.angle_min + scan.angle_increment * i;
    const double t = 2.0 / std::max(std::abs(std::cos(a)), std::abs(std::sin(a)));
    scan.ranges.push_back(static_cast<float>(t));
  }
  return scan;
}

double gauge(ros2_common_cpp::Metrics & metrics, const std::string & name)
{
  for (const auto & family : metrics.registry()->Collect()) {
    if (family.name == name && !family.metric.empty()) {
      return family.metric[0].gauge.value;
    }
  }
  return -1.0;
}

template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout = 5000ms)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return pred();
}

}  // namespace

TEST(Node, GridFromMsg)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = 0.05f;
  msg.info.width = 3;
  msg.info.height = 2;
  msg.info.origin.position.x = 1.5;
  msg.info.origin.position.y = -2.0;
  msg.info.origin.orientation.z = std::sin(0.3);
  msg.info.origin.orientation.w = std::cos(0.3);
  msg.data = {-1, 0, 100, 50, -1, 65};
  const auto grid = mg::grid_from_msg(msg);
  EXPECT_FLOAT_EQ(grid.resolution, 0.05f);
  EXPECT_EQ(grid.width, 3);
  EXPECT_EQ(grid.height, 2);
  EXPECT_EQ(grid.origin_x, 1.5);
  EXPECT_EQ(grid.origin_y, -2.0);
  EXPECT_NEAR(grid.origin_yaw, 0.6, 1e-12);
  EXPECT_EQ(grid.cells, (std::vector<std::int8_t>{-1, 0, 100, 50, -1, 65}));
}

TEST(Node, QosMatchesThePythonNode)
{
  const auto sensor = mg::sensor_qos().get_rmw_qos_profile();
  EXPECT_EQ(sensor.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(sensor.depth, 5u);
  const auto map = mg::map_qos().get_rmw_qos_profile();
  EXPECT_EQ(map.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(map.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  EXPECT_EQ(map.depth, 1u);
}

TEST_F(RosTest, QueryPausedReadsTheParameter)
{
  mg::Config config;
  mg::SlamServices slam(*client_node_, config);
  EXPECT_EQ(slam.query_paused(), std::optional<bool>(false));
  fake_->set_parameter(rclcpp::Parameter(mg::PAUSED_PARAMETER, true));
  EXPECT_EQ(slam.query_paused(), std::optional<bool>(true));
}

TEST_F(RosTest, QueryPausedOfAnUnknownParameterTypeIsNone)
{
  mg::Config config;
  config.parameters_service = "/map_guard_test_client/get_parameters";  // has no paused_new_measurements
  mg::SlamServices slam(*client_node_, config);
  EXPECT_FALSE(slam.query_paused());
}

TEST_F(RosTest, TogglePauseChecksStatusAndTheParameterFollows)
{
  mg::Config config;
  mg::SlamServices slam(*client_node_, config);
  EXPECT_TRUE(slam.toggle_pause());
  EXPECT_EQ(slam.query_paused(), std::optional<bool>(true));
  fake_->toggle_status = false;
  EXPECT_FALSE(slam.toggle_pause());
}

TEST_F(RosTest, SerializeAndResetCheckResultCodes)
{
  mg::Config config;
  mg::SlamServices slam(*client_node_, config);
  EXPECT_TRUE(slam.serialize_map("/var/lib/ros2/maps/b"));
  EXPECT_EQ(fake_->serialized, "/var/lib/ros2/maps/b");
  EXPECT_TRUE(slam.reset());
  EXPECT_FALSE(fake_->reset_pause_flag);  // the pause state is left as it is
  fake_->serialize_result = slam_toolbox::srv::SerializePoseGraph::Response::RESULT_FAILED_TO_WRITE_FILE;
  fake_->reset_result = 1;
  EXPECT_FALSE(slam.serialize_map("/x"));
  EXPECT_FALSE(slam.reset());
}

TEST_F(RosTest, UnavailableServiceReturnsNoneOrFalse)
{
  mg::Config config;
  config.pause_service = "/nobody/pause";
  config.reset_service = "/nobody/reset";
  config.serialize_service = "/nobody/serialize";
  config.parameters_service = "/nobody/get_parameters";
  mg::SlamServices slam(*client_node_, config);
  EXPECT_FALSE(slam.query_paused());
  EXPECT_FALSE(slam.toggle_pause());
  EXPECT_FALSE(slam.serialize_map("/x"));
  EXPECT_FALSE(slam.reset());
}

TEST_F(RosTest, TimedOutCallReturnsFalse)
{
  mg::Config config;
  config.service_timeout_s = 0.2;
  // The fake answers from the executor thread, so its delay must not block it: give it its own executor.
  executor_->remove_node(fake_);
  auto fake_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  fake_executor->add_node(fake_);
  std::thread fake_spinner([fake_executor]() { fake_executor->spin(); });
  fake_->delay = 600ms;
  mg::SlamServices slam(*client_node_, config);
  EXPECT_FALSE(slam.toggle_pause());
  fake_executor->cancel();
  fake_spinner.join();
}

TEST_F(RosTest, NodeScoresScansAndDetectsACarriedRobotInDryRun)
{
  mg::Config config;
  config.dry_run = true;
  config.min_map_cells_known = 0;
  config.agent_state_url = "http://127.0.0.1:1/api/state";  // unreachable: no events
  auto node = std::make_shared<mg::MapGuardNode>(rclcpp::NodeOptions(), config, false);
  auto io = std::make_shared<rclcpp::Node>("map_guard_test_io");
  executor_->add_node(node);
  executor_->add_node(io);

  tf2_ros::StaticTransformBroadcaster tf(io);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = io->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "laser";
  transform.transform.rotation.w = 1.0;
  tf.sendTransform(transform);
  auto map_pub = io->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", mg::map_qos());
  auto scan_pub = io->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", mg::sensor_qos());
  auto wheel_pub = io->create_publisher<nav_msgs::msg::Odometry>("/odom", mg::sensor_qos());
  auto lidar_pub = io->create_publisher<nav_msgs::msg::Odometry>("/odom_rf2o", mg::sensor_qos());
  auto events = std::make_shared<std::atomic<int>>(0);
  auto events_sub = io->create_subscription<std_msgs::msg::String>(
    "/robot_events", 10, [events](std_msgs::msg::String::ConstSharedPtr) { ++*events; });
  map_pub->publish(room_map());

  const auto scan = room_scan();
  EXPECT_TRUE(wait_for([&]() {
    scan_pub->publish(scan);
    return gauge(node->metrics(), "map_guard_agreement_ratio") > 0.95;
  }));
  EXPECT_EQ(node->state(), mg::State::OK);

  // Carried: lidar odometry moves at 0.2 m/s while the wheels report still, for more than carried_s.
  nav_msgs::msg::Odometry lidar;
  lidar.twist.twist.linear.x = 0.2;
  nav_msgs::msg::Odometry wheel;
  EXPECT_TRUE(wait_for([&]() {
    lidar.header.stamp = io->now();
    lidar_pub->publish(lidar);
    wheel_pub->publish(wheel);
    return node->state() == mg::State::DISTURBED;
  }));
  EXPECT_EQ(fake_->pause_calls.load(), 0);  // dry run: no slam_toolbox call
  EXPECT_EQ(events->load(), 0);             // agent unreachable: no event
  executor_->remove_node(node);
  executor_->remove_node(io);
}
