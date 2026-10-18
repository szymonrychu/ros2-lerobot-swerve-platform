// Configuration of the head camera node: /etc/ros2/head_camera_cpp/config.yaml (HEAD_CAMERA_CONFIG). Extra keys are
// forbidden at every level and any error exits 78 (ros2_common_cpp::EXIT_CONFIG). See README.md for the meaning of
// each key.
#ifndef HEAD_CAMERA_CPP__CONFIG_HPP_
#define HEAD_CAMERA_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "ros2_common_cpp/config.hpp"

namespace head_camera_cpp
{

using ros2_common_cpp::Errors;

constexpr const char * ENV_CONFIG_PATH_KEY = "HEAD_CAMERA_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/head_camera_cpp/config.yaml";
constexpr long DEFAULT_METRICS_PORT = 19116;  // NOLINT(runtime/int)

// The zoom still capture: full-resolution autofocused crops on request.
struct ZoomConfig
{
  std::array<int, 2> still_size{4608, 2592};  // full sensor mode of the still (width, height)
  std::vector<int> factors{2, 4, 8};          // allowed zoom factors, a non-empty subset of 2, 4, 8
  int default_max_px = 1024;                  // longest side of a zoom image when the request names none
  int min_max_px = 256;
  int max_max_px = 1024;
  int jpeg_quality = 85;  // 1..100
  double af_timeout_s = 2.5;
  double switch_timeout_s = 1.5;
  int settle_frames = 2;
  int still_buffers = 2;
  double watchdog_s = 6.0;  // above af_timeout_s + switch_timeout_s
  double restore_retry_delay_s = 0.5;
  double af_window_fraction = 0.125;  // 0 < f <= 1
};

// The 2 Hz small-JPEG mapping stream (RTAB-Map, RGB-D fusion).
struct MappingConfig
{
  bool enabled = false;
  bool lock_focus = true;  // runtime parameter mapping.lock_focus: lock the lens at lens_position while subscribed
  double rate_hz = 2.0;    // 0 < r <= 10
  int width = 576;         // even, <= the stream width, the stream aspect within 1 px
  int height = 324;
  int jpeg_quality = 80;  // 1..100
};

// Every setting of the node.
struct Config
{
  std::string camera_id = "/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a";
  std::string frame_id = "head_camera_optical_frame";
  std::string topic_namespace = "/head_camera";
  std::array<int, 2> sensor_mode{2304, 1296};
  int width = 1152;
  int height = 648;
  long frame_duration_us = 33333;      // NOLINT(runtime/int)
  std::string af_mode = "continuous";  // continuous | auto | manual
  double lens_position = 1.0;          // dioptres >= 0; the focus-lock position and the one of af_mode manual
  int jpeg_quality = 70;               // 1..100
  std::optional<std::string> camera_info_file;
  std::optional<long> metrics_port = DEFAULT_METRICS_PORT;  // NOLINT(runtime/int); nullopt disables metrics
  ZoomConfig zoom;
  MappingConfig mapping;

  // Topics derived from topic_namespace.
  std::string image_raw() const { return topic_namespace + "/image_raw"; }
  std::string image_raw_compressed() const { return topic_namespace + "/image_raw/compressed"; }
  std::string camera_info() const { return topic_namespace + "/camera_info"; }
  std::string mapping_image_topic() const { return topic_namespace + "/mapping/image_raw/compressed"; }
  std::string mapping_camera_info_topic() const { return topic_namespace + "/mapping/camera_info"; }
  std::string zoom_command() const { return topic_namespace + "/zoom/command"; }
  std::string zoom_image() const { return topic_namespace + "/zoom/image"; }
  std::string zoom_result() const { return topic_namespace + "/zoom/result"; }
};

// Every error of a YAML mapping as "<key>: <problem>" lines (nested keys as "zoom.<key>" or "mapping.<key>"); empty when valid.
// @param node the parsed config root.
// @return the error lines.
Errors validate(const YAML::Node & node);

// The Config of a YAML mapping, defaults for missing keys. Invalid values keep their defaults; call validate first.
// @param node the parsed config root.
// @return the config.
Config from_yaml(const YAML::Node & node);

// Load HEAD_CAMERA_CONFIG (default DEFAULT_CONFIG_PATH); on a missing file or any error log every error to stderr
// and exit 78.
// @return the validated config.
Config load_config_or_exit();

}  // namespace head_camera_cpp

#endif  // HEAD_CAMERA_CPP__CONFIG_HPP_
