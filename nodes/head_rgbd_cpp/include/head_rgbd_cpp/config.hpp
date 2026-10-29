// Configuration of the head RGB-D adapter node: /etc/ros2/head_rgbd_cpp/config.yaml (HEAD_RGBD_CONFIG). Extra keys are
// forbidden at every level and any error exits 78 (ros2_common_cpp::EXIT_CONFIG). config/head_rgbd.yaml documents
// every key with its meaning and unit.
#ifndef HEAD_RGBD_CPP__CONFIG_HPP_
#define HEAD_RGBD_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>

#include "ros2_common_cpp/config.hpp"
#include "ros2_common_cpp/depth_mask.hpp"

namespace head_rgbd_cpp
{

using ros2_common_cpp::Errors;

constexpr const char * ENV_CONFIG_PATH_KEY = "HEAD_RGBD_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/head_rgbd_cpp/config.yaml";
constexpr long DEFAULT_METRICS_PORT = 19117;  // NOLINT(runtime/int)
// Widest colour-to-depth stamp spread of an approximate pair: half a frame period of the 15 Hz D435, so a colour frame
// can never pair with the depth of the next frame.
constexpr double MAX_PAIR_INTERVAL_S = 0.5 / 15.0;

// Output sets: depth size, set rate and RGB JPEG quality.
struct OutputConfig
{
  int divisor = 2;        // depth size = colour size / divisor; 1 or 2
  double rate_hz = 2.0;   // published sets per second, 0 < v <= 15
  int jpeg_quality = 80;  // 1..100
};

// Valid depth range; depth outside it is published as 0 (no depth).
struct DepthConfig
{
  double min_m = 0.15;
  double max_m = 4.0;  // > min_m
};

// Coloured point cloud.
struct CloudConfig
{
  bool enabled = true;
  double rate_hz = 1.0;  // 0 < v <= 10
};

// Depth edge ("veil") mask applied to every depth the node publishes (ros2_common_cpp::depth_mask): window_px odd
// 3..9, abs_m >= 0, 0 <= rel <= 1 (not both 0), grow_px 0..2.
struct EdgeMaskConfig : ros2_common_cpp::DepthEdgeMaskParams
{
  bool enabled = true;
};

// Full-rate filtered copy of the full-FOV D435 depth: every frame of depth_topic, masked, on
// <output_namespace>/image_raw with the frame's own header, and the latest camera_info of info_topic (same size) on
// <output_namespace>/camera_info with that header. The input of d435_obstacle_cloud.
struct FilteredDepthConfig
{
  bool enabled = true;
  std::string depth_topic = "/head/d435/depth/image_rect_raw";
  std::string info_topic = "/head/d435/depth/camera_info";
  std::string output_namespace = "/head/d435/depth_filtered";

  std::string image() const { return output_namespace + "/image_raw"; }
  std::string camera_info() const { return output_namespace + "/camera_info"; }
};

enum class PairingPolicy
{
  EXACT,
  APPROXIMATE
};

// How the aligned depth pairs with the colour frame: message_filters ExactTime (equal stamps) or ApproximateTime
// (stamps at most max_interval_s apart, 0 < v <= MAX_PAIR_INTERVAL_S).
struct PairingConfig
{
  PairingPolicy policy = PairingPolicy::EXACT;
  double max_interval_s = 0.02;
};

// Lazy subscription.
struct LazyConfig
{
  double idle_unsubscribe_s = 5.0;  // >= 0
};

// Every setting of the node.
struct Config
{
  std::string depth_topic = "/head/d435/aligned_depth_to_color/image_raw";
  std::string rgb_image_topic = "/head/d435/color/image_raw";
  std::string rgb_info_topic = "/head/d435/color/camera_info";
  std::string output_namespace = "/head_rgbd";
  OutputConfig output;
  DepthConfig depth;
  CloudConfig cloud;
  LazyConfig lazy;
  EdgeMaskConfig edge_mask;
  FilteredDepthConfig filtered_depth;
  PairingConfig pairing;
  std::optional<long> metrics_port = DEFAULT_METRICS_PORT;  // NOLINT(runtime/int); nullopt disables metrics

  // Output topics derived from output_namespace.
  std::string depth_image() const { return output_namespace + "/depth/image_raw"; }
  std::string depth_info() const { return output_namespace + "/depth/camera_info"; }
  std::string rgb_image() const { return output_namespace + "/rgb/image_raw/compressed"; }
  std::string rgb_camera_info() const { return output_namespace + "/rgb/camera_info"; }
  std::string points() const { return output_namespace + "/points"; }
};

// Every error of a YAML mapping as "<key>: <problem>" lines (nested keys as "output.rate_hz"); empty when valid.
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
