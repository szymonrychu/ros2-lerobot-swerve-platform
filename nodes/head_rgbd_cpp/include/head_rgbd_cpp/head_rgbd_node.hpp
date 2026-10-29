// The head RGB-D adapter node (design D1): pairs the D435 aligned depth with the colour image (FramePairer, exact or
// approximate stamps), throttles the sets to output.rate_hz, masks the depth edges, downscales the depth,
// JPEG-encodes the colour and publishes the /head_rgbd contract (every message of a set with the colour header), only
// while an output has subscribers and only from real, paired inputs with a camera_info. A second, independent stage
// masks every frame of the full-FOV depth and republishes it with its camera_info on filtered_depth.output_namespace
// (the obstacle cloud input), again only while that output has subscribers.
#ifndef HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_
#define HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "head_rgbd_cpp/cloud.hpp"
#include "head_rgbd_cpp/config.hpp"
#include "head_rgbd_cpp/pairing.hpp"
#include "ros2_common_cpp/metrics.hpp"

namespace head_rgbd_cpp
{

constexpr const char * NODE_NAME = "head_rgbd";
// Period of the lazy-subscription check.
constexpr std::chrono::milliseconds LAZY_CHECK_PERIOD{1000};
// Throttle of the drop logs, milliseconds.
constexpr int64_t DROP_LOG_THROTTLE_MS = 5000;
// Pairing queue, frames per stream (a third of a second at the D435's 15 Hz).
constexpr uint32_t PAIR_QUEUE_SIZE = 5;
// Input queue of the full-rate filtered depth stage, frames.
constexpr size_t FILTERED_QUEUE_SIZE = 2;
// Format of the published colour: compressed_image_transport's "<original encoding>; jpeg compressed bgr8".
constexpr const char * COMPRESSED_FORMAT = "bgr8; jpeg compressed bgr8";

// Drop reason labels of head_rgbd_dropped_total{reason}.
constexpr const char * DROP_REASON_BAD_INPUT = "bad_input";
constexpr const char * DROP_REASON_SIZE_MISMATCH = "size_mismatch";
constexpr const char * DROP_REASON_NO_CAMERA_INFO = "no_camera_info";
constexpr const char * DROP_REASON_UNPAIRED = "unpaired";
// Values of the stage label of head_rgbd_edge_mask_removed_ratio{stage}.
constexpr const char * STAGE_ALIGNED = "aligned";
constexpr const char * STAGE_FILTERED = "filtered";

// Downscale a 16UC1 depth (mm) by an integer divisor without interpolating across missing depth: an output pixel takes
// the first pixel of its divisor x divisor source block, in row-major order from the top-left one (the nearest-neighbour
// sample), whose depth lies in [range.min_m, range.max_m]; 0 when none does. Divisor 1 only applies the range.
// @param depth_mm CV_16UC1 depth in millimetres.
// @param divisor 1 or more; the output is depth_mm.cols / divisor x depth_mm.rows / divisor (partial blocks dropped).
// @param range the valid depth range.
// @return the CV_16UC1 output depth, 0 for no depth.
cv::Mat downscale_depth(const cv::Mat & depth_mm, int divisor, const DepthConfig & range);

// The camera_info of an image downscaled by divisor with downscale_depth: width and height divided (floor), the first
// two rows of K and P divided (the output pixel u samples the source pixel divisor * u), D, R, distortion model and
// header kept.
// @param info the camera_info of the source image.
// @param divisor 1 or more.
// @return the camera_info of the downscaled image.
sensor_msgs::msg::CameraInfo scale_camera_info(const sensor_msgs::msg::CameraInfo & info, int divisor);

// Accepts stamps on a fixed grid of 1 / rate_hz, so a faster stream is thinned to rate_hz on average: the first stamp
// is accepted, then the first stamp at or after the next grid point. A stamp before the last accepted one (clock jump,
// bag replay loop) restarts the grid there, as does a gap longer than one period.
class SetThrottle
{
public:
  // @param rate_hz accepted stamps per second, > 0.
  explicit SetThrottle(double rate_hz);

  // @param stamp_ns stamp of the set in nanoseconds.
  // @return true when this set is due (and remembers it).
  bool due(int64_t stamp_ns);

private:
  int64_t period_ns_;
  std::optional<int64_t> last_ns_;
  int64_t next_ns_ = 0;
};

class HeadRgbdNode : public rclcpp::Node
{
public:
  // @param config the validated settings.
  // @param options node options.
  explicit HeadRgbdNode(const Config & config, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // @return the node's metrics registry.
  ros2_common_cpp::Metrics & metrics() { return metrics_; }
  // @return true while the input subscriptions of the /head_rgbd sets exist.
  bool inputs_subscribed() const { return depth_sub_ != nullptr; }
  // @return true while the input subscriptions of the filtered depth stage exist.
  bool filtered_inputs_subscribed() const { return filtered_depth_sub_ != nullptr; }

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  void check_subscribers();
  // One lazy stage: subscribes while has_subscribers, releases idle_unsubscribe_s after the last subscriber left.
  void check_stage(
    bool has_subscribers, bool subscribed, std::optional<rclcpp::Time> & idle_since,
    const std::function<void()> & subscribe, const std::function<void()> & unsubscribe);
  void subscribe_inputs();
  void unsubscribe_inputs();
  void subscribe_filtered_inputs();
  void unsubscribe_filtered_inputs();
  bool has_output_subscribers() const;
  bool has_filtered_subscribers() const;
  void on_pair(const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & color);
  void on_unpaired(Stream stream, int64_t stamp_ns);
  void drop(const char * reason, const std::string & detail);
  // @return the reason the set cannot be published (and fills detail), nullptr when it can.
  const char * reject_reason(const Image & depth, const Image & color, std::string & detail) const;
  void publish_set(const Image & depth, const Image & color);
  // Masks one full-FOV depth frame and publishes it with the latest camera_info (or counts why not).
  void on_filtered_depth(const Image::ConstSharedPtr & depth);
  void drop_filtered(const char * reason, const std::string & detail);
  // Applies the edge mask (when enabled) to depth_mm in place and sets the removed-ratio gauge of the stage.
  void mask_depth(cv::Mat & depth_mm, prometheus::Gauge & removed_ratio) const;

  Config config_;
  SetThrottle set_throttle_;
  RateDecimator cloud_decimator_;
  CameraInfo::ConstSharedPtr latest_info_;

  rclcpp::Publisher<Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  std::unique_ptr<FramePairer> pairer_;
  rclcpp::TimerBase::SharedPtr lazy_timer_;
  std::optional<rclcpp::Time> idle_since_;

  rclcpp::Publisher<Image>::SharedPtr filtered_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr filtered_info_pub_;
  rclcpp::Subscription<Image>::SharedPtr filtered_depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr filtered_info_sub_;
  CameraInfo::ConstSharedPtr latest_depth_info_;
  std::optional<rclcpp::Time> filtered_idle_since_;

  ros2_common_cpp::Metrics metrics_;
  prometheus::Counter * published_ = nullptr;
  prometheus::Histogram * process_ = nullptr;
  prometheus::Gauge * valid_ratio_ = nullptr;
  prometheus::Counter * depth_frames_ = nullptr;
  prometheus::Counter * colour_frames_ = nullptr;
  prometheus::Counter * pairs_ = nullptr;
  prometheus::Histogram * pair_delta_ = nullptr;
  prometheus::Gauge * aligned_removed_ = nullptr;
  prometheus::Gauge * filtered_removed_ = nullptr;
  prometheus::Counter * filtered_published_ = nullptr;
  prometheus::Histogram * filtered_process_ = nullptr;
};

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_
