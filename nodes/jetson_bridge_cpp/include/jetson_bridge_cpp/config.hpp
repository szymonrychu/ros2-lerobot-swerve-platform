// jetson_bridge_cpp config, and the one definition on the Pi of the Jetson topic contract (names; QoS is in
// bridge_node.hpp). File: JETSON_BRIDGE_CONFIG (default /etc/ros2/jetson_bridge_cpp/config.yaml); unknown keys are
// rejected and an invalid config exits 78 (ros2_common_cpp::EXIT_CONFIG).
#ifndef JETSON_BRIDGE_CPP__CONFIG_HPP_
#define JETSON_BRIDGE_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ros2_common_cpp/config.hpp"

namespace jetson_bridge_cpp
{

constexpr const char * CONFIG_ENV_VAR = "JETSON_BRIDGE_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/jetson_bridge_cpp/config.yaml";
constexpr const char * DEFAULT_ENDPOINT = "tcp://10.77.0.2:5550";
constexpr int MAX_PORT = 65535;
constexpr int DEFAULT_METRICS_PORT = 19114;

// The topic contract, republished from the Jetson.
constexpr const char * DEFAULT_DEPTH_TOPIC = "/stereo/depth/image_raw";
constexpr const char * DEFAULT_CAMERA_INFO_TOPIC = "/stereo/camera_info";
constexpr const char * DEFAULT_FUSED_TOPIC = "/stereo/depth/fused";
constexpr const char * DEFAULT_RGB_TOPIC = "/overview_camera/image_raw/compressed";
constexpr const char * DEFAULT_RGB_INFO_TOPIC = "/overview_camera/camera_info";

struct BridgeConfig
{
  std::string endpoint = DEFAULT_ENDPOINT;
  std::string depth_topic = DEFAULT_DEPTH_TOPIC;
  std::string camera_info_topic = DEFAULT_CAMERA_INFO_TOPIC;
  std::string fused_topic = DEFAULT_FUSED_TOPIC;
  std::string rgb_topic = DEFAULT_RGB_TOPIC;
  std::string rgb_info_topic = DEFAULT_RGB_INFO_TOPIC;
  int64_t max_age_ms = 500;
  int64_t absent_after_ms = 2000;
  std::optional<int> metrics_port = DEFAULT_METRICS_PORT;  // null in the file disables the exporter
};

const std::vector<std::string> & config_keys();

// Every error of a config mapping (empty = valid).
ros2_common_cpp::Errors validate(const YAML::Node & node);

// The config of a mapping that validate accepted (missing keys keep defaults; strings stripped).
BridgeConfig from_yaml(const YAML::Node & node);

struct ConfigLoad
{
  BridgeConfig config;
  ros2_common_cpp::Errors errors;
};

// Read and validate one YAML file (an empty file gives the defaults). Never exits.
ConfigLoad load_config(const std::string & path);

// The node entry point: JETSON_BRIDGE_CONFIG (default DEFAULT_CONFIG_PATH), every error to stderr, exit 78 on error.
BridgeConfig load_config_or_exit();

}  // namespace jetson_bridge_cpp

#endif  // JETSON_BRIDGE_CPP__CONFIG_HPP_
