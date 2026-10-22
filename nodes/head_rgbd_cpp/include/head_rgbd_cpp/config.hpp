// Configuration of the head RGB-D fusion node: /etc/ros2/head_rgbd_cpp/config.yaml (HEAD_RGBD_CONFIG). Extra keys are
// forbidden at every level and any error exits 78 (ros2_common_cpp::EXIT_CONFIG). config/head_rgbd.yaml documents
// every key with its meaning and unit.
#ifndef HEAD_RGBD_CPP__CONFIG_HPP_
#define HEAD_RGBD_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>

#include "ros2_common_cpp/config.hpp"

namespace head_rgbd_cpp
{

using ros2_common_cpp::Errors;

constexpr const char * ENV_CONFIG_PATH_KEY = "HEAD_RGBD_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/head_rgbd_cpp/config.yaml";
constexpr long DEFAULT_METRICS_PORT = 19117;  // NOLINT(runtime/int)

// Output image size relative to the RGB image.
struct OutputConfig
{
  int divisor = 2;  // 1 or 2
};

// ToF-RGB pairing.
struct SyncConfig
{
  double max_pair_dt_s = 0.06;  // 0 < v <= 0.2
  int queue_size = 10;          // >= 2
};

// Motion compensation of the capture offset between the two stamps.
struct MotionConfig
{
  bool compensate = true;
  std::string fixed_frame = "odom";
  double max_angular_rate_deg_s = 10.0;  // > 0
  double max_linear_rate_m_s = 0.3;      // > 0
  double rate_window_s = 0.05;           // > 0
  double tf_timeout_s = 0.05;            // >= 0
};

// Flying-pixel filter of the ToF depth.
struct FilterConfig
{
  double jump_rel = 0.05;        // 0 < v < 1
  double jump_abs_mm = 30.0;     // >= 0
  int min_valid_neighbours = 3;  // 0..8
};

// Valid depth range.
struct DepthConfig
{
  double min_m = 0.1;
  double max_m = 4.0;  // > min_m
};

// Guided-filter upsampling to the RGB resolution.
struct UpsamplingConfig
{
  bool enabled = true;
  int radius = 4;            // 1..16
  double eps = 0.001;        // > 0
  double min_support = 0.5;  // 0 < v <= 1
};

// Coloured point cloud.
struct CloudConfig
{
  bool enabled = true;
  double rate_hz = 1.0;  // 0 < v <= 10
};

// Lazy subscription.
struct LazyConfig
{
  double idle_unsubscribe_s = 5.0;  // >= 0
};

// Every setting of the node.
struct Config
{
  std::string tof_depth_topic = "/tof/depth/image_raw";
  std::string rgb_image_topic = "/head_camera/mapping/image_raw/compressed";
  std::string rgb_info_topic = "/head_camera/mapping/camera_info";
  std::string output_namespace = "/head_rgbd";
  std::string tof_calibration_file = "/etc/ros2/head_rgbd_cpp/tof_calibration.yaml";
  OutputConfig output;
  SyncConfig sync;
  MotionConfig motion;
  FilterConfig filter;
  DepthConfig depth;
  UpsamplingConfig upsampling;
  CloudConfig cloud;
  LazyConfig lazy;
  std::optional<long> metrics_port = DEFAULT_METRICS_PORT;  // NOLINT(runtime/int); nullopt disables metrics

  // Output topics derived from output_namespace.
  std::string depth_image() const { return output_namespace + "/depth/image_raw"; }
  std::string depth_info() const { return output_namespace + "/depth/camera_info"; }
  std::string rgb_image() const { return output_namespace + "/rgb/image_raw/compressed"; }
  std::string rgb_camera_info() const { return output_namespace + "/rgb/camera_info"; }
  std::string points() const { return output_namespace + "/points"; }
  std::string depth_upsampled_image() const { return output_namespace + "/depth_upsampled/image_raw"; }
  std::string depth_upsampled_info() const { return output_namespace + "/depth_upsampled/camera_info"; }
};

// Every error of a YAML mapping as "<key>: <problem>" lines (nested keys as "sync.queue_size"); empty when valid.
// @param node the parsed config root.
// @return the error lines.
Errors validate(const YAML::Node & node);

// The Config of a YAML mapping, defaults for missing keys. Invalid values keep their defaults; call validate first.
// @param node the parsed config root.
// @return the config.
Config from_yaml(const YAML::Node & node);

// Load HEAD_RGBD_CONFIG (default DEFAULT_CONFIG_PATH); on a missing file or any error log every error to stderr and
// exit 78.
// @return the validated config.
Config load_config_or_exit();

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__CONFIG_HPP_
