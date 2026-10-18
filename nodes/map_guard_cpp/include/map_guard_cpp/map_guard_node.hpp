// ROS wiring of the map guard: subscriptions, TF, slam_toolbox service clients and the guard worker thread.
// Port of nodes/map_guard/map_guard/node.py. All logic lives in the ROS-free core library; this only converts
// messages and runs the state machine on a worker thread so the blocking service calls never stall the executor.
//
// Differences from the Python node that cut its CPU cost, with the same behaviour:
// - one single-threaded executor (component container or the standalone map_guard_node) instead of three threads;
// - the scan subscription takes the serialized message and deserializes only the scans the agreement_hz rate limit
//   lets through; the TF lookup and the endpoint conversion happen only for those;
// - prepare_map (the dilated masks) runs once per /map message, as in Python, in plain loops instead of numpy.
#ifndef MAP_GUARD_CPP__MAP_GUARD_NODE_HPP_
#define MAP_GUARD_CPP__MAP_GUARD_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "map_guard_cpp/agent.hpp"
#include "map_guard_cpp/agreement.hpp"
#include "map_guard_cpp/config.hpp"
#include "map_guard_cpp/detectors.hpp"
#include "map_guard_cpp/guard.hpp"
#include "map_guard_cpp/timing.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_common_cpp/metrics.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/srv/pause.hpp"
#include "slam_toolbox/srv/reset.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace map_guard_cpp
{

// ROS node name (the same as the Python node: only one of them runs).
constexpr const char * NODE_NAME = "map_guard";
// robot_node_info{node} of this node (its ros2_nodes name).
constexpr const char * METRICS_NODE = "map_guard_cpp";
constexpr const char * PAUSED_PARAMETER = "paused_new_measurements";
// Wait for a slam_toolbox service to appear before calling it, s.
constexpr double SERVICE_WAIT_S = 1.0;
constexpr std::size_t SENSOR_QOS_DEPTH = 5;
constexpr std::size_t EVENTS_QOS_DEPTH = 10;
// The map_manager command/result topics are reliable, keep last 10 (a command or answer must not be lost).
constexpr std::size_t COMMAND_QOS_DEPTH = 10;

// Sensor topics: best effort, keep last 5.
rclcpp::QoS sensor_qos();
// slam_toolbox publishes /map reliable + transient_local with depth 1: match it to receive the latched map.
rclcpp::QoS map_qos();

// The relocalise command JSON for map_manager (discard_on_fail: map_manager clears the live POIs on failure).
std::string relocalise_command(const std::string & request_id, double since);

// The parts of a map_manager result the guard acts on.
struct RelocaliseResult
{
  std::string request_id;
  bool ok = false;
  std::string state;
  int discarded_pois = 0;
};

// Parse a map_manager result message; nullopt when the text is not a valid result (JSON object with request_id and
// ok; data.state and data.discarded_pois are optional).
std::optional<RelocaliseResult> parse_result(const std::string & text);

// Seconds of the steady (monotonic) clock.
double monotonic_s();

// An OccupancyGrid as a GridMap (origin yaw from the origin orientation).
GridMap grid_from_msg(const nav_msgs::msg::OccupancyGrid & msg);

// Blocking slam_toolbox service calls (SlamPort), made from the guard worker thread while the executor spins.
class SlamServices : public SlamPort
{
public:
  SlamServices(rclcpp::Node & node, const Config & config);

  std::optional<bool> query_paused() override;
  bool toggle_pause() override;
  bool serialize_map(const std::string & filename) override;
  bool reset() override;

private:
  // Call a service and wait for the response; nullptr when it is unavailable or timed out.
  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client, typename ServiceT::Request::SharedPtr request);

  rclcpp::Logger logger_;
  double timeout_s_;
  rclcpp::Client<slam_toolbox::srv::Pause>::SharedPtr pause_client_;
  rclcpp::Client<slam_toolbox::srv::Reset>::SharedPtr reset_client_;
  rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serialize_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr params_client_;
};

// Subscriptions feeding the detectors and the agreement score; the guard runs on a worker thread at tick_hz.
class MapGuardNode : public rclcpp::Node
{
public:
  // Loads the config from MAP_GUARD_CONFIG (exit 78 when invalid) and serves the metrics.
  explicit MapGuardNode(const rclcpp::NodeOptions & options);
  // Uses the given config; metrics are served only when serve_metrics is true.
  MapGuardNode(const rclcpp::NodeOptions & options, const Config & config, bool serve_metrics);
  ~MapGuardNode() override;

  ros2_common_cpp::Metrics & metrics() { return metrics_; }
  // Guard state after the latest tick.
  State state() const { return state_.load(); }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void on_scan(const std::shared_ptr<const rclcpp::SerializedMessage> & serialized);
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void on_wheel_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void on_lidar_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void on_cmd(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);
  void on_localised(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void on_result(const std_msgs::msg::String::ConstSharedPtr & msg);
  void request_relocalise(const std::string & request_id, double since);
  void run_guard();

  Config config_;
  ros2_common_cpp::Metrics metrics_;
  GuardMetrics guard_metrics_;
  std::mutex lock_;
  Disturbances detectors_;
  std::shared_ptr<const PreparedMap> prepared_;
  std::vector<MapGuard::Sample> pending_;
  bool held_ = false;  // not localised yet, from the localised topic; guarded by lock_
  RateLimiter agreement_limiter_;
  std::optional<PlanarStamp> lidar_pose_;
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serialization_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  std::unique_ptr<EventNotifier> notifier_;
  std::unique_ptr<SlamServices> slam_;
  std::unique_ptr<MapGuard> guard_;
  std::vector<RelocaliseResult> results_;  // map_manager answers waiting for the worker thread; guarded by lock_
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::mutex stop_lock_;
  std::condition_variable stop_cv_;
  bool stop_ = false;
  std::atomic<State> state_{State::OK};
  std::thread worker_;
};

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__MAP_GUARD_NODE_HPP_
