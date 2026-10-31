#pragma once

// The ROS wiring of the head camera (design D2, D3, D4, D13): one rclcpp component that publishes the 30 fps stream
// and answers zoom requests. Serves G1, G2, U1, U8, U9.
//
// Threads: the camera's callback thread copies each video frame into a latest-frame slot and returns; the video
// worker publishes it (an older unpublished frame is dropped, never queued); the zoom worker runs one zoom at a
// time; the executor thread only parses commands and answers busy/invalid.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "head_camera_cpp/camera_backend.hpp"
#include "head_camera_cpp/config.hpp"
#include "head_camera_cpp/mapping_stream.hpp"
#include "head_camera_cpp/zoom_controller.hpp"
#include "head_camera_cpp/zoom_json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_common_cpp/metrics.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace head_camera_cpp
{

constexpr const char * NODE_NAME = "head_camera";
constexpr const char * METRICS_NODE = "head_camera_cpp";
constexpr const char * COMPRESSED_FORMAT = "bgr8; jpeg compressed bgr8";
constexpr const char * ZOOM_IMAGE_FORMAT = "jpeg";
constexpr int START_RETRY_PERIOD_MS = 2000;
constexpr double FPS_WINDOW_S = 1.0;
constexpr int FOCUS_POLL_PERIOD_MS = 100;
constexpr const char * LOCK_FOCUS_PARAMETER = "mapping.lock_focus";

/**
 * Load a ROS camera_calibration YAML and scale its intrinsics to an image size.
 *
 * @param path The YAML file (image_width, image_height, camera_matrix, distortion_*, rectification_matrix,
 *   projection_matrix).
 * @param width Image width the camera_info describes.
 * @param height Image height the camera_info describes.
 * @param frame_id The header frame_id.
 * @return The camera_info with fx, cx (K[0], K[2], P[0], P[2]) scaled by width / image_width and fy, cy by
 *   height / image_height; the header stamp is unset.
 * @throws std::runtime_error when the file is missing or malformed.
 */
sensor_msgs::msg::CameraInfo load_camera_info(
  const std::string & path, int width, int height, const std::string & frame_id);

class HeadCameraNode : public rclcpp::Node
{
public:
  /// Loads the config (exit 78 when invalid) and drives the head camera through libcamera.
  explicit HeadCameraNode(const rclcpp::NodeOptions & options);

  /// For tests: the given config, camera and clock.
  HeadCameraNode(
    const Config & config, std::unique_ptr<CameraBackend> backend, std::unique_ptr<Clock> clock,
    const rclcpp::NodeOptions & options);

  ~HeadCameraNode() override;

  /// @return The node's metrics registry.
  ros2_common_cpp::Metrics & metrics() { return metrics_; }

private:
  /// One video frame owned by the node.
  struct Frame
  {
    std::vector<uint8_t> bgr;
    int stride = 0;
    Size size{0, 0};
    int64_t stamp_ns = 0;
  };

  void init();
  /// The zoom controller, worker thread, metrics, publishers and command subscription (only with zoom.enabled).
  void init_zoom();
  void on_video_frame(const VideoFrame & frame);
  void video_loop();
  void publish_frame(const Frame & frame);
  void on_command(const std_msgs::msg::String & msg);
  void zoom_loop();
  void publish_zoom(const ZoomOutcome & outcome);
  void publish_result(const ZoomResult & result);
  void publish_mapping(const Frame & frame, const std_msgs::msg::Header & header);
  bool focus_lock_desired() const;
  void apply_focus_lock();
  void update_focus_lock();
  void retry_start();
  void record_capture(Clock::TimePoint now);
  void expire_capture_fps();
  int64_t to_ros_ns(int64_t steady_ns) const;

  Config config_;
  std::unique_ptr<CameraBackend> backend_;
  std::unique_ptr<Clock> clock_;
  ros2_common_cpp::Metrics metrics_;
  std::unique_ptr<ZoomController> zoom_;  // null with zoom.enabled false

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr zoom_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zoom_result_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr fps_timer_;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;

  // The 2 Hz mapping stream (only when config_.mapping.enabled) and its focus lock.
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mapping_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mapping_info_pub_;
  std::optional<sensor_msgs::msg::CameraInfo> mapping_info_;  // only with a loaded calibration
  std::optional<MappingDecimator> mapping_decimator_;         // video thread only
  rclcpp::TimerBase::SharedPtr focus_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::atomic<bool> lock_focus_param_{true};     // runtime parameter mapping.lock_focus
  std::atomic<bool> focus_lock_applied_{false};  // what the backend was last told
  prometheus::Counter * published_mapping_image_ = nullptr;
  prometheus::Counter * published_mapping_info_ = nullptr;

  prometheus::Counter * frames_captured_ = nullptr;
  prometheus::Counter * published_raw_ = nullptr;
  prometheus::Counter * published_compressed_ = nullptr;
  prometheus::Counter * published_info_ = nullptr;
  prometheus::Gauge * capture_fps_ = nullptr;
  prometheus::Histogram * encode_seconds_ = nullptr;

  // Backend access: held by the zoom worker for the whole request, tried by the start retry.
  std::mutex backend_mutex_;
  std::atomic<bool> video_running_{false};

  // Latest-frame slot.
  std::mutex frame_mutex_;
  std::condition_variable frame_ready_;
  Frame slot_;  // swapped with the worker's frame so both keep their buffers
  bool slot_full_ = false;
  bool stopping_ = false;
  std::thread video_thread_;

  // Capture rate over FPS_WINDOW_S, measured in the camera callback (so frames the slot drops still count) and
  // zeroed by fps_timer_ when the stream stops.
  std::mutex fps_mutex_;
  bool fps_window_open_ = false;
  Clock::TimePoint fps_window_start_;
  Clock::TimePoint last_capture_;
  int fps_window_intervals_ = 0;

  // Pending zoom request (claimed through zoom_->try_begin()).
  std::mutex zoom_mutex_;
  std::condition_variable zoom_ready_;
  std::optional<ZoomRequest> zoom_pending_;
  bool zoom_stopping_ = false;
  std::thread zoom_thread_;
};

}  // namespace head_camera_cpp
