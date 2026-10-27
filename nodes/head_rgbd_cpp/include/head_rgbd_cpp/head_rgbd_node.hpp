// The head RGB-D adapter node (design D1): pairs the D435 aligned depth with the colour image by exact stamp,
// throttles the sets to output.rate_hz, downscales the depth, JPEG-encodes the colour and publishes the /head_rgbd
// contract (every message of a set with the colour header), only while an output has subscribers and only from real,
// exactly paired inputs with a camera_info.
#ifndef HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_
#define HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
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
#include "ros2_common_cpp/metrics.hpp"

namespace head_rgbd_cpp
{

constexpr const char * NODE_NAME = "head_rgbd";
// Period of the lazy-subscription check.
constexpr std::chrono::milliseconds LAZY_CHECK_PERIOD{1000};
// Throttle of the drop logs, milliseconds.
constexpr int64_t DROP_LOG_THROTTLE_MS = 5000;
// Exact-time pairing queue, stamps (a third of a second at the D435's 15 Hz).
constexpr uint32_t PAIR_QUEUE_SIZE = 5;
// Format of the published colour: compressed_image_transport's "<original encoding>; jpeg compressed bgr8".
constexpr const char * COMPRESSED_FORMAT = "bgr8; jpeg compressed bgr8";

// Drop reason labels of head_rgbd_dropped_total{reason}.
constexpr const char * DROP_REASON_BAD_INPUT = "bad_input";
constexpr const char * DROP_REASON_SIZE_MISMATCH = "size_mismatch";
constexpr const char * DROP_REASON_NO_CAMERA_INFO = "no_camera_info";
constexpr const char * DROP_REASON_UNPAIRED = "unpaired";

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
  // @return true while the input subscriptions exist.
  bool inputs_subscribed() const { return depth_sub_ != nullptr; }

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ExactTime<Image, Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void check_subscribers();
  void subscribe_inputs();
  void unsubscribe_inputs();
  bool has_output_subscribers() const;
  void on_pair(const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & color);
  void on_unpaired(const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & color);
  void drop(const char * reason, const std::string & detail);
  // @return the reason the set cannot be published (and fills detail), nullptr when it can.
  const char * reject_reason(const Image & depth, const Image & color, std::string & detail) const;
  void publish_set(const Image & depth, const Image & color);

  Config config_;
  SetThrottle set_throttle_;
  RateDecimator cloud_decimator_;
  CameraInfo::ConstSharedPtr latest_info_;

  rclcpp::Publisher<Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

  std::shared_ptr<message_filters::Subscriber<Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> color_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::TimerBase::SharedPtr lazy_timer_;
  std::optional<rclcpp::Time> idle_since_;

  ros2_common_cpp::Metrics metrics_;
  prometheus::Counter * published_ = nullptr;
  prometheus::Histogram * process_ = nullptr;
  prometheus::Gauge * valid_ratio_ = nullptr;
};

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_
