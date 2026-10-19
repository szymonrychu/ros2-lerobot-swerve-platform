// ROS wiring of the 3D map manager (D9): JSON commands and map_guard resets in, RTAB-Map Empty services out, a result
// per command and a status line every status_period_s. The JSON contracts live in command_json.hpp and status.hpp;
// this file only connects them to rclcpp.
//
// Services are called on a worker thread (one action at a time) so the blocking waits never stall the executor.
// Subscribes no large stream: camera_info and Info are tiny, mapGraph carries only poses.
#ifndef MAP3D_MANAGER_CPP__MAP3D_MANAGER_NODE_HPP_
#define MAP3D_MANAGER_CPP__MAP3D_MANAGER_NODE_HPP_

#include <atomic>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "map3d_manager_cpp/command_json.hpp"
#include "map3d_manager_cpp/config.hpp"
#include "map3d_manager_cpp/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/info.hpp"
#include "rtabmap_msgs/msg/map_graph.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

namespace map3d_manager_cpp
{

constexpr const char * NODE_NAME = "map3d_manager";
constexpr const char * LIVE_DB_NAME = "live.db";
constexpr const char * SAVED_DB_NAME = "saved.db";
constexpr const char * DB_SUFFIX = ".db";
constexpr const char * TMP_SUFFIX = ".tmp";
constexpr std::size_t COMMAND_QOS_DEPTH = 10;

// Wall-clock seconds since the epoch (the clock of every stamp in the status).
double wall_s();

// Yaw (rad) of a quaternion.
double yaw_of(double x, double y, double z, double w);

// Copy src to dst atomically: into dst + ".tmp", then rename over dst.
// @return empty on success, else the errno text
std::string copy_atomically(const std::filesystem::path & src, const std::filesystem::path & dst);

class Map3dManagerNode : public rclcpp::Node
{
public:
  explicit Map3dManagerNode(const Config & config, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Map3dManagerNode() override;

  // True while an action runs.
  bool busy() const { return busy_; }

private:
  struct Outcome
  {
    Status status = Status::OK;
    std::string message;
  };

  void on_command(const std_msgs::msg::String & msg);
  void on_map_reset();
  void on_graph(const rtabmap_msgs::msg::MapGraph & msg);
  void publish_status();
  void publish_result(const Result & result);
  // Start `work` on the worker thread unless an action runs; true when started.
  bool start_action(std::function<void()> work);
  Result run_command(const Command & command);
  Outcome call_service(const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr & client, const std::string & name);
  Outcome do_reset();
  Outcome do_save(const Command & command, Result & result);
  void note_error(const Outcome & outcome);

  Config config_;
  std::filesystem::path database_dir_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapGraph>::SharedPtr graph_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr backup_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resume_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::atomic<bool> busy_{false};
  std::thread worker_;

  std::mutex lock_;  // guards the status inputs below
  StatusInputs inputs_;
};

}  // namespace map3d_manager_cpp

#endif  // MAP3D_MANAGER_CPP__MAP3D_MANAGER_NODE_HPP_
