// ROS wiring of the 3D map manager (see the header).
#include "map3d_manager_cpp/map3d_manager_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <system_error>
#include <utility>
#include <variant>

namespace map3d_manager_cpp
{

namespace
{

// A save name that is refused: "live" would overwrite the running database.
constexpr const char * RESERVED_NAME = "live";

rclcpp::QoS command_qos() { return rclcpp::QoS(COMMAND_QOS_DEPTH).reliable(); }

rclcpp::QoS sample_qos() { return rclcpp::QoS(1).best_effort(); }

}  // namespace

double wall_s() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }

double yaw_of(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

std::string copy_atomically(const std::filesystem::path & src, const std::filesystem::path & dst)
{
  std::filesystem::path tmp = dst;
  tmp += TMP_SUFFIX;
  std::error_code ec;
  std::filesystem::copy_file(src, tmp, std::filesystem::copy_options::overwrite_existing, ec);
  if (!ec) {
    std::filesystem::rename(tmp, dst, ec);
  }
  if (ec) {
    std::error_code ignored;
    std::filesystem::remove(tmp, ignored);
    return src.string() + " -> " + dst.string() + ": " + ec.message();
  }
  return "";
}

Map3dManagerNode::Map3dManagerNode(const Config & config, const rclcpp::NodeOptions & options)
: rclcpp::Node(NODE_NAME, options), config_(config), database_dir_(config.database_dir)
{
  const std::string ns = config_.rtabmap_namespace;
  result_pub_ = create_publisher<std_msgs::msg::String>(config_.result_topic, command_qos());
  status_pub_ = create_publisher<std_msgs::msg::String>(config_.status_topic, command_qos());
  command_sub_ = create_subscription<std_msgs::msg::String>(
    config_.command_topic, command_qos(), [this](const std_msgs::msg::String & msg) { on_command(msg); });
  reset_sub_ = create_subscription<std_msgs::msg::Empty>(
    config_.map_reset_topic, command_qos(), [this](const std_msgs::msg::Empty &) { on_map_reset(); });
  graph_sub_ = create_subscription<rtabmap_msgs::msg::MapGraph>(
    ns + "/mapGraph", sample_qos(), [this](const rtabmap_msgs::msg::MapGraph & msg) { on_graph(msg); });
  info_sub_ =
    create_subscription<rtabmap_msgs::msg::Info>(ns + "/info", sample_qos(), [this](const rtabmap_msgs::msg::Info &) {
      std::lock_guard<std::mutex> guard(lock_);
      inputs_.info_stamp_s = wall_s();
    });
  rgb_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    config_.rgb_info_topic, sample_qos(), [this](const sensor_msgs::msg::CameraInfo &) {
      std::lock_guard<std::mutex> guard(lock_);
      inputs_.rgb_info_stamp_s = wall_s();
    });
  depth_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    config_.depth_info_topic, sample_qos(), [this](const sensor_msgs::msg::CameraInfo &) {
      std::lock_guard<std::mutex> guard(lock_);
      inputs_.depth_info_stamp_s = wall_s();
    });
  backup_ = create_client<std_srvs::srv::Empty>(ns + "/backup");
  reset_ = create_client<std_srvs::srv::Empty>(ns + "/reset");
  pause_ = create_client<std_srvs::srv::Empty>(ns + "/pause");
  resume_ = create_client<std_srvs::srv::Empty>(ns + "/resume");
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(config_.status_period_s)),
    [this]() { publish_status(); });
}

Map3dManagerNode::~Map3dManagerNode()
{
  if (worker_.joinable()) {
    worker_.join();
  }
}

void Map3dManagerNode::publish_result(const Result & result)
{
  std_msgs::msg::String msg;
  msg.data = to_json(result);
  result_pub_->publish(msg);
}

void Map3dManagerNode::on_graph(const rtabmap_msgs::msg::MapGraph & msg)
{
  GraphSample g;
  g.stamp_s = wall_s();
  g.nodes = static_cast<int>(msg.poses.size());
  const auto & t = msg.map_to_odom.translation;
  const auto & q = msg.map_to_odom.rotation;
  g.x = t.x;
  g.y = t.y;
  g.z = t.z;
  g.yaw_rad = yaw_of(q.x, q.y, q.z, q.w);
  std::lock_guard<std::mutex> guard(lock_);
  inputs_.graph = g;
}

bool Map3dManagerNode::start_action(std::function<void()> work)
{
  if (busy_.exchange(true)) {
    return false;
  }
  if (worker_.joinable()) {
    worker_.join();  // the previous action cleared busy_ as its last step
  }
  worker_ = std::thread([this, work = std::move(work)]() {
    work();
    busy_ = false;
  });
  return true;
}

void Map3dManagerNode::on_command(const std_msgs::msg::String & msg)
{
  const auto parsed = parse_command(msg.data);
  if (const auto * error = std::get_if<ParseError>(&parsed)) {
    if (!error->request_id) {
      RCLCPP_WARN(get_logger(), "dropping unusable command: %s", error->message.c_str());
      return;
    }
    Result result;
    result.request_id = *error->request_id;
    result.status = Status::INVALID;
    result.message = error->message;
    publish_result(result);
    return;
  }
  const Command command = std::get<Command>(parsed);
  if (command.action == Action::SAVE && command.name == RESERVED_NAME) {
    Result result;
    result.request_id = command.request_id;
    result.action = action_name(command.action);
    result.status = Status::INVALID;
    result.message = std::string("name '") + RESERVED_NAME + "' is the running database";
    publish_result(result);
    return;
  }
  if (!start_action([this, command]() { publish_result(run_command(command)); })) {
    Result result;
    result.request_id = command.request_id;
    result.action = action_name(command.action);
    result.status = Status::BUSY;
    result.message = "another 3D map action is running";
    publish_result(result);
  }
}

void Map3dManagerNode::on_map_reset()
{
  const bool started = start_action([this]() {
    RCLCPP_WARN(get_logger(), "map_reset received: resetting the 3D map");
    const Outcome outcome = do_reset();
    if (outcome.status != Status::OK) {
      RCLCPP_ERROR(get_logger(), "3D map reset after map_reset failed: %s", outcome.message.c_str());
      note_error(outcome);
    }
  });
  if (!started) {
    RCLCPP_WARN(get_logger(), "map_reset received while another 3D map action runs; not reset");
  }
}

void Map3dManagerNode::note_error(const Outcome & outcome)
{
  std::lock_guard<std::mutex> guard(lock_);
  inputs_.last_error = outcome.message;
  inputs_.last_error_stamp_s = wall_s();
}

Map3dManagerNode::Outcome Map3dManagerNode::call_service(
  const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr & client, const std::string & name)
{
  const auto timeout = std::chrono::duration<double>(config_.service_timeout_s);
  if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout))) {
    return {Status::TIMEOUT, "service " + config_.rtabmap_namespace + "/" + name + " not available"};
  }
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  if (future.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(future.request_id);
    return {Status::TIMEOUT, "service " + config_.rtabmap_namespace + "/" + name + " did not answer"};
  }
  return {Status::OK, ""};
}

Map3dManagerNode::Outcome Map3dManagerNode::do_reset() { return call_service(reset_, "reset"); }

Map3dManagerNode::Outcome Map3dManagerNode::do_save(const Command & command, Result & result)
{
  Outcome outcome = call_service(backup_, "backup");
  if (outcome.status != Status::OK) {
    return outcome;
  }
  const auto live = database_dir_ / LIVE_DB_NAME;
  const auto saved = database_dir_ / SAVED_DB_NAME;
  const auto named = database_dir_ / (command.name + DB_SUFFIX);
  for (const auto & target : {saved, named}) {
    const std::string error = copy_atomically(live, target);
    if (!error.empty()) {
      return {Status::ERROR, error};
    }
  }
  std::error_code ec;
  const auto size = std::filesystem::file_size(saved, ec);
  if (ec) {
    return {Status::ERROR, saved.string() + ": " + ec.message()};
  }
  result.db_path = saved.string();
  result.db_size_mb = static_cast<double>(size) / BYTES_PER_MB;
  {
    std::lock_guard<std::mutex> guard(lock_);
    inputs_.last_save = LastSave{command.name, wall_s()};
  }
  return {Status::OK, "saved " + command.name};
}

Result Map3dManagerNode::run_command(const Command & command)
{
  Result result;
  result.request_id = command.request_id;
  result.action = action_name(command.action);
  Outcome outcome;
  switch (command.action) {
    case Action::SAVE:
      outcome = do_save(command, result);
      break;
    case Action::RESET:
      outcome = do_reset();
      if (outcome.status == Status::OK) {
        outcome.message = "3D map reset";
      }
      break;
    case Action::PAUSE:
    case Action::RESUME: {
      const bool pause = command.action == Action::PAUSE;
      outcome = call_service(pause ? pause_ : resume_, pause ? "pause" : "resume");
      if (outcome.status == Status::OK) {
        outcome.message = pause ? "3D mapping paused" : "3D mapping resumed";
        std::lock_guard<std::mutex> guard(lock_);
        inputs_.paused = pause;
      }
      break;
    }
  }
  result.status = outcome.status;
  result.message = outcome.message;
  if (outcome.status != Status::OK) {
    result.db_path.reset();
    result.db_size_mb.reset();
    note_error(outcome);
    RCLCPP_ERROR(
      get_logger(), "%s %s failed: %s", result.action.c_str(), command.request_id.c_str(), outcome.message.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "%s %s ok", result.action.c_str(), command.request_id.c_str());
  }
  return result;
}

void Map3dManagerNode::publish_status()
{
  StatusInputs inputs;
  {
    std::lock_guard<std::mutex> guard(lock_);
    inputs = inputs_;
  }
  inputs.now_s = wall_s();
  std::error_code ec;
  const auto size = std::filesystem::file_size(database_dir_ / LIVE_DB_NAME, ec);
  if (!ec) {
    inputs.db_size_bytes = size;
  }
  inputs.memory_bytes = read_cgroup_memory_bytes(config_.rtabmap_unit);
  std_msgs::msg::String msg;
  msg.data = to_json(build_status(inputs, config_));
  status_pub_->publish(msg);
}

}  // namespace map3d_manager_cpp
