// ROS2 node: synced left/right camera images -> StereoSGBM -> 16UC1 depth (mm) + camera_info, the rclcpp port of
// nodes/stereo_depth/stereo_depth/node.py with the same contract:
// - inputs left_image_topic and right_image_topic (sensor_msgs/Image, sensor-data QoS: best effort, keep last 5),
//   paired by message_filters ApproximateTime (queue 5) whose pairs never span more than sync_slop_s;
// - outputs depth_topic (16UC1, millimetres, 0 = invalid), camera_info_topic (the rectified left camera) and, when
//   publish_disparity, disparity_topic (stereo_msgs/DisparityImage), all reliable keep last 5, under one header:
//   the left image's stamp and frame_id;
// - every gate, drop reason and metric of StereoDepthCore (pipeline.hpp); nothing is published without a valid
//   calibration and a decoded, size-checked pair with at least one valid depth pixel;
// - the calibration files are re-checked every calibration_check_s (loaded when they appear, reloaded on change).
// Registered as the rclcpp_components component stereo_depth_cpp::StereoDepthNode; the stereo_depth_node executable
// runs it standalone. An invalid config exits 78 before the node starts.
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <string>

#include "ros2_common_cpp/metrics.hpp"
#include "stereo_depth_cpp/config.hpp"
#include "stereo_depth_cpp/depth.hpp"
#include "stereo_depth_cpp/pipeline.hpp"

namespace stereo_depth_cpp
{

namespace
{

constexpr std::size_t PUBLISH_QOS_DEPTH = 5;
constexpr std::uint32_t SYNC_QUEUE_SIZE = 5;
constexpr int LOG_THROTTLE_MS = 10000;
constexpr const char * DEPTH_ENCODING = "16UC1";
constexpr const char * DISPARITY_ENCODING = "32FC1";
constexpr const char * RECTIFIED_DISTORTION_MODEL = "plumb_bob";
constexpr std::size_t RECTIFIED_DISTORTION_SIZE = 5;

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using stereo_msgs::msg::DisparityImage;

// Image.data of a continuous single-channel cv::Mat (host byte order; is_bigendian 0 on the robot's arm64).
void fill_image(Image & msg, const std_msgs::msg::Header & header, const cv::Mat & image, const char * encoding)
{
  msg.header = header;
  msg.height = static_cast<std::uint32_t>(image.rows);
  msg.width = static_cast<std::uint32_t>(image.cols);
  msg.encoding = encoding;
  msg.is_bigendian = 0;
  msg.step = static_cast<std::uint32_t>(image.cols * image.elemSize());
  const cv::Mat packed = image.isContinuous() ? image : image.clone();
  msg.data.resize(packed.total() * packed.elemSize());
  std::memcpy(msg.data.data(), packed.data, msg.data.size());
}

ImageView view_of(const Image & msg)
{
  return {
    msg.encoding,
    msg.width,
    msg.height,
    msg.step,
    msg.data.data(),
    msg.data.size(),
    rclcpp::Time(msg.header.stamp).nanoseconds()};
}

}  // namespace

class StereoDepthNode : public rclcpp::Node
{
public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  explicit StereoDepthNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node(NODE_NAME, options),
    config_(load_config_or_exit()),
    metrics_(std::make_unique<ros2_common_cpp::Metrics>(NODE_NAME)),
    core_(std::make_unique<StereoDepthCore>(config_, *metrics_))
  {
    if (const auto port = ros2_common_cpp::resolve_metrics_port(config_.metrics_port)) {
      metrics_->serve(*port);
    }
    const rclcpp::QoS publish_qos(PUBLISH_QOS_DEPTH);
    depth_pub_ = create_publisher<Image>(config_.depth_topic, publish_qos);
    info_pub_ = create_publisher<CameraInfo>(config_.camera_info_topic, publish_qos);
    if (config_.publish_disparity) {
      disparity_pub_ = create_publisher<DisparityImage>(config_.disparity_topic, publish_qos);
    }
    left_sub_.subscribe(this, config_.left_image_topic, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, config_.right_image_topic, rmw_qos_profile_sensor_data);
    sync_ =
      std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(SYNC_QUEUE_SIZE), left_sub_, right_sub_);
    sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(config_.sync_slop_s));
    sync_->registerCallback(std::bind(&StereoDepthNode::on_pair, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(config_.calibration_check_s), std::bind(&StereoDepthNode::check_calibration, this));
    check_calibration();
    RCLCPP_INFO(
      get_logger(),
      "stereo_depth: %s + %s -> %s, num_disparities=%d block_size=%d max_rate_hz=%g sync_slop_s=%g "
      "calibration_dir=%s",
      config_.left_image_topic.c_str(), config_.right_image_topic.c_str(), config_.depth_topic.c_str(),
      config_.num_disparities, config_.block_size, config_.max_rate_hz, config_.sync_slop_s,
      config_.calibration_dir.c_str());
  }

private:
  void check_calibration()
  {
    const CalibrationCheck check = core_->check_calibration();
    if (!check.changed) {
      return;
    }
    if (check.loaded) {
      RCLCPP_INFO(get_logger(), "%s", check.message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", check.message.c_str());
    }
  }

  void on_pair(const Image::ConstSharedPtr & left, const Image::ConstSharedPtr & right)
  {
    const double now_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const PairResult result = core_->process(view_of(*left), view_of(*right), now_s);
    if (!result.reason.empty()) {
      if (!result.message.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS, "%s", result.message.c_str());
      }
      return;
    }
    publish(*left, result);
  }

  void publish(const Image & left, const PairResult & result)
  {
    const StereoCalibration & calibration = *core_->calibration();
    std_msgs::msg::Header header;
    header.stamp = left.header.stamp;
    header.frame_id = config_.frame_id;

    auto depth = std::make_unique<Image>();
    fill_image(*depth, header, result.depth_mm, DEPTH_ENCODING);
    depth_pub_->publish(std::move(depth));

    auto info = std::make_unique<CameraInfo>();
    const auto & p = calibration.left.p;
    info->header = header;
    info->width = static_cast<std::uint32_t>(calibration.width());
    info->height = static_cast<std::uint32_t>(calibration.height());
    info->distortion_model = RECTIFIED_DISTORTION_MODEL;
    info->d.assign(RECTIFIED_DISTORTION_SIZE, 0.0);
    info->k = {p[0], p[1], p[2], p[4], p[5], p[6], 0.0, 0.0, 1.0};
    info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::copy(p.begin(), p.end(), info->p.begin());
    info_pub_->publish(std::move(info));

    if (disparity_pub_) {
      auto disparity = std::make_unique<DisparityImage>();
      disparity->header = header;
      fill_image(disparity->image, header, result.disparity, DISPARITY_ENCODING);
      disparity->f = static_cast<float>(calibration.fx());
      disparity->t = static_cast<float>(calibration.baseline_m());
      disparity->valid_window.width = static_cast<std::uint32_t>(result.disparity.cols);
      disparity->valid_window.height = static_cast<std::uint32_t>(result.disparity.rows);
      disparity->min_disparity = static_cast<float>(MIN_DISPARITY);
      disparity->max_disparity = static_cast<float>(MIN_DISPARITY + config_.num_disparities - 1);
      disparity->delta_d = static_cast<float>(1.0 / DISPARITY_SCALE);
      disparity_pub_->publish(std::move(disparity));
    }
  }

  DepthConfig config_;
  std::unique_ptr<ros2_common_cpp::Metrics> metrics_;
  std::unique_ptr<StereoDepthCore> core_;
  rclcpp::Publisher<Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<DisparityImage>::SharedPtr disparity_pub_;
  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace stereo_depth_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_depth_cpp::StereoDepthNode)
