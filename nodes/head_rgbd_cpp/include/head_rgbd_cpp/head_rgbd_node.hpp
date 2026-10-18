// The head RGB-D fusion node (design D1, D2, D5, D6): registers the ToF depth into the Camera Module 3 mapping stream
// and publishes the aligned pair under output_namespace, only while an output has subscribers and only from real,
// synchronised inputs with valid TF.
#ifndef HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_
#define HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "head_rgbd_cpp/cloud.hpp"
#include "head_rgbd_cpp/config.hpp"
#include "head_rgbd_cpp/registration.hpp"
#include "ros2_common_cpp/metrics.hpp"

namespace head_rgbd_cpp
{

constexpr const char * NODE_NAME = "head_rgbd";
// Period of the lazy-subscription check.
constexpr std::chrono::milliseconds LAZY_CHECK_PERIOD{1000};
// Throttle of the drop logs, milliseconds.
constexpr int64_t DROP_LOG_THROTTLE_MS = 5000;

// Drop reason labels of head_rgbd_pairs_dropped_total{reason} not defined with the motion gate (motion.hpp has
// pair_dt and motion).
constexpr const char * DROP_REASON_NO_CALIBRATION = "no_calibration";
constexpr const char * DROP_REASON_SIZE_MISMATCH = "size_mismatch";
constexpr const char * DROP_REASON_NO_CAMERA_INFO = "no_camera_info";
constexpr const char * DROP_REASON_NO_TF = "no_tf";
constexpr const char * DROP_REASON_BAD_INPUT = "bad_input";

class HeadRgbdNode : public rclcpp::Node
{
public:
  // @param config the validated settings.
  // @param tf_buffer the TF buffer to look transforms up in; when null the node owns a buffer fed by a TF listener.
  // @param options node options.
  explicit HeadRgbdNode(
    const Config & config, std::shared_ptr<tf2_ros::Buffer> tf_buffer = nullptr,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // @return the node's metrics registry.
  ros2_common_cpp::Metrics & metrics() { return metrics_; }
  // @return true when the ToF calibration loaded and is valid.
  bool has_calibration() const { return tof_rays_ != nullptr; }
  // @return true while the input subscriptions exist.
  bool inputs_subscribed() const { return tof_sub_ != nullptr; }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void load_calibration();
  void check_subscribers();
  void subscribe_inputs();
  void unsubscribe_inputs();
  bool has_output_subscribers() const;
  void on_set(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & rgb,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);
  void drop(const char * reason, const std::string & detail);
  bool lookup(
    const std::string & target, const rclcpp::Time & target_time, const std::string & source,
    const rclcpp::Time & source_time, const std::string & fixed, Eigen::Isometry3d & out);
  void publish_pair(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & rgb,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info, const cv::Mat & registered,
    const PinholeModel & out_model);

  Config config_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<PinholeModel> tof_model_;
  std::unique_ptr<RayTable> tof_rays_;
  RateDecimator cloud_decimator_;
  bool upsampling_ = false;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr up_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr up_info_pub_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> tof_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> rgb_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> info_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::TimerBase::SharedPtr lazy_timer_;
  std::optional<rclcpp::Time> idle_since_;

  ros2_common_cpp::Metrics metrics_;
  prometheus::Counter * published_ = nullptr;
  prometheus::Histogram * pair_dt_ = nullptr;
  prometheus::Histogram * process_ = nullptr;
  prometheus::Gauge * valid_ratio_ = nullptr;
};

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__HEAD_RGBD_NODE_HPP_
