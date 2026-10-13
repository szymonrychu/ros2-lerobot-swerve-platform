// stereo_depth_cpp config: the C++ mirror of nodes/stereo_depth/stereo_depth/config.py (DepthConfig). Same file
// (STEREO_DEPTH_CONFIG, default /etc/ros2/stereo_depth/config.yaml), same keys, defaults and rejections
// (extra keys forbidden, ge/gt/le bounds, multiple of 16, odd block size, p1 < p2, min_depth_m < max_depth_m,
// non-empty stripped topics and frame_id). An invalid config exits 78 (ros2_common_cpp::EXIT_CONFIG).
#ifndef STEREO_DEPTH_CPP__CONFIG_HPP_
#define STEREO_DEPTH_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <vector>

#include "ros2_common_cpp/config.hpp"

namespace stereo_depth_cpp
{

constexpr const char * CONFIG_ENV_VAR = "STEREO_DEPTH_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/stereo_depth/config.yaml";
constexpr const char * DEFAULT_CALIBRATION_DIR = "/var/lib/ros2/camera_calibration";
constexpr const char * DEFAULT_FRAME_ID = "stereo_left_optical_frame";
constexpr int DEFAULT_METRICS_PORT = 19113;
constexpr int DISPARITY_MULTIPLE = 16;
constexpr int MAX_BLOCK_SIZE = 11;
constexpr int MAX_PRE_FILTER_CAP = 63;
constexpr double MAX_SYNC_SLOP_S = 0.1;
constexpr double MAX_RATE_HZ = 30.0;
constexpr int MAX_MATCHER_THREADS = 4;
constexpr int MAX_PORT = 65535;

// Every DepthConfig field with the Python defaults; see config.py for the meaning of each.
struct DepthConfig
{
  std::string left_image_topic = "/overview_camera/image_raw";
  std::string right_image_topic = "/stereo/right/image_raw";
  std::string depth_topic = "/stereo/depth/image_raw";
  std::string camera_info_topic = "/stereo/camera_info";
  bool publish_disparity = false;
  std::string disparity_topic = "/stereo/disparity";
  std::string calibration_dir = DEFAULT_CALIBRATION_DIR;
  double calibration_check_s = 5.0;
  int num_disparities = 48;
  int block_size = 5;
  int p1 = 200;
  int p2 = 800;
  int uniqueness_ratio = 10;
  int speckle_window_size = 100;
  int speckle_range = 2;
  int disp12_max_diff = 1;
  int pre_filter_cap = 31;
  double min_depth_m = 0.3;
  double max_depth_m = 4.0;
  double sync_slop_s = 0.002;
  double max_rate_hz = 5.0;
  int matcher_threads = 2;
  std::string frame_id = DEFAULT_FRAME_ID;
  std::optional<int> metrics_port = DEFAULT_METRICS_PORT;
};

// Every key of DepthConfig, in the Python field order (validate forbids every other key).
const std::vector<std::string> & config_keys();

// Every error of a config mapping (empty = valid): unknown keys, wrong types, bounds and relations.
ros2_common_cpp::Errors validate(const YAML::Node & node);

// The DepthConfig of a mapping that validate accepted (topics and frame_id stripped; missing keys keep defaults).
DepthConfig from_yaml(const YAML::Node & node);

// Result of load_config: the config (defaults when invalid) and every error.
struct ConfigLoad
{
  DepthConfig config;
  ros2_common_cpp::Errors errors;
};

// Read and validate one YAML file; an empty file gives the defaults. Never exits.
ConfigLoad load_config(const std::string & path);

// The node entry point: STEREO_DEPTH_CONFIG (default DEFAULT_CONFIG_PATH), every error to stderr, exit 78 on error.
DepthConfig load_config_or_exit();

}  // namespace stereo_depth_cpp

#endif  // STEREO_DEPTH_CPP__CONFIG_HPP_
