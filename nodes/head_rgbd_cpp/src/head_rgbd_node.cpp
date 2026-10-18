#include "head_rgbd_cpp/head_rgbd_node.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <array>
#include <camera_calibration_parsers/parse.hpp>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <utility>
#include <vector>

#include "head_rgbd_cpp/motion.hpp"
#include "head_rgbd_cpp/tof_filter.hpp"
#include "head_rgbd_cpp/upsample.hpp"

namespace head_rgbd_cpp
{

namespace
{
constexpr const char * DEPTH_ENCODING = "16UC1";
constexpr int PLUMB_BOB_SIZE = 5;
constexpr auto TF_RETRY_PERIOD = std::chrono::milliseconds(5);
constexpr std::array<double, 8> PAIR_DT_BUCKETS = {0.005, 0.01, 0.02, 0.03, 0.04, 0.06, 0.1, 0.2};
constexpr std::array<double, 8> PROCESS_BUCKETS = {0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0};
constexpr std::array<const char *, 7> ALL_DROP_REASONS = {
  DROP_REASON_NO_CALIBRATION, DROP_REASON_SIZE_MISMATCH, DROP_REASON_NO_CAMERA_INFO, DROP_REASON_PAIR_DT,
  DROP_REASON_MOTION,         DROP_REASON_NO_TF,         DROP_REASON_BAD_INPUT};

tf2::TimePoint to_tf_time(const rclcpp::Time & t) { return tf2::TimePoint(std::chrono::nanoseconds(t.nanoseconds())); }

// The plumb_bob model of a camera_info; nullopt when it holds no usable calibration (zero or negative focal length,
// no size, or distortion coefficients that are not plumb_bob's).
std::optional<PinholeModel> model_of(const sensor_msgs::msg::CameraInfo & info)
{
  const bool zero_k = std::all_of(info.k.begin(), info.k.end(), [](double v) { return v == 0.0; });
  if (zero_k || info.width == 0 || info.height == 0 || !(info.k[0] > 0.0) || !(info.k[4] > 0.0)) {
    return std::nullopt;
  }
  if (info.d.size() > static_cast<size_t>(PLUMB_BOB_SIZE)) {
    return std::nullopt;
  }
  PinholeModel m;
  m.K = cv::Matx33d(info.k[0], info.k[1], info.k[2], info.k[3], info.k[4], info.k[5], info.k[6], info.k[7], info.k[8]);
  m.D = cv::Vec<double, PLUMB_BOB_SIZE>::zeros();
  for (size_t i = 0; i < info.d.size(); ++i) {
    m.D[static_cast<int>(i)] = info.d[i];
  }
  m.width = static_cast<int>(info.width);
  m.height = static_cast<int>(info.height);
  return m;
}
}  // namespace

HeadRgbdNode::HeadRgbdNode(
  const Config & config, std::shared_ptr<tf2_ros::Buffer> tf_buffer, const rclcpp::NodeOptions & options)
: rclcpp::Node(NODE_NAME, options),
  config_(config),
  tf_buffer_(std::move(tf_buffer)),
  cloud_decimator_(config.cloud.rate_hz),
  metrics_(NODE_NAME)
{
  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  }

  if (config_.metrics_port) {
    metrics_.serve(static_cast<std::uint16_t>(*config_.metrics_port));
  }
  published_ = &metrics_.counter("head_rgbd_pairs_published_total", "Registered RGB-D pairs published");
  for (const char * reason : ALL_DROP_REASONS) {
    metrics_.counter("head_rgbd_pairs_dropped_total", "Synchronised sets dropped, by reason", {{"reason", reason}});
  }
  pair_dt_ = &metrics_.histogram(
    "head_rgbd_pair_dt_seconds", "Absolute stamp difference of the ToF-RGB pairs",
    std::vector<double>(PAIR_DT_BUCKETS.begin(), PAIR_DT_BUCKETS.end()));
  process_ = &metrics_.histogram(
    "head_rgbd_process_seconds", "Processing time of one synchronised set",
    std::vector<double>(PROCESS_BUCKETS.begin(), PROCESS_BUCKETS.end()));
  valid_ratio_ = &metrics_.gauge("head_rgbd_valid_ratio", "Fraction of registered depth pixels holding a depth");

  load_calibration();

  const rclcpp::QoS qos = rclcpp::QoS(1).best_effort();
  depth_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.depth_image(), qos);
  depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.depth_info(), qos);
  rgb_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(config_.rgb_image(), qos);
  rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.rgb_camera_info(), qos);
  if (config_.cloud.enabled) {
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(config_.points(), qos);
  }
  if (config_.upsampling.enabled) {
    if (config_.output.divisor == UPSAMPLE_FACTOR) {
      up_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.depth_upsampled_image(), qos);
      up_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.depth_upsampled_info(), qos);
      upsampling_ = true;
    } else {
      RCLCPP_ERROR(
        get_logger(), "upsampling.enabled needs output.divisor == %d (got %d): upsampled topics are not published",
        UPSAMPLE_FACTOR, config_.output.divisor);
    }
  }

  lazy_timer_ = create_wall_timer(LAZY_CHECK_PERIOD, [this] { check_subscribers(); });
}

void HeadRgbdNode::load_calibration()
{
  sensor_msgs::msg::CameraInfo info;
  std::string name;
  if (!camera_calibration_parsers::readCalibration(config_.tof_calibration_file, name, info)) {
    RCLCPP_ERROR(
      get_logger(), "cannot read the ToF calibration '%s': publishing nothing", config_.tof_calibration_file.c_str());
    return;
  }
  const auto model = model_of(info);
  if (!model) {
    RCLCPP_ERROR(
      get_logger(), "the ToF calibration '%s' is invalid (size, K or D): publishing nothing",
      config_.tof_calibration_file.c_str());
    return;
  }
  tof_model_ = model;
  tof_rays_ = std::make_unique<RayTable>(*model);
  RCLCPP_INFO(
    get_logger(), "ToF calibration %dx%d loaded from %s", model->width, model->height,
    config_.tof_calibration_file.c_str());
}

bool HeadRgbdNode::has_output_subscribers() const
{
  size_t count = depth_pub_->get_subscription_count() + depth_info_pub_->get_subscription_count() +
                 rgb_pub_->get_subscription_count() + rgb_info_pub_->get_subscription_count();
  if (points_pub_) {
    count += points_pub_->get_subscription_count();
  }
  if (upsampling_) {
    count += up_pub_->get_subscription_count() + up_info_pub_->get_subscription_count();
  }
  return count > 0;
}

void HeadRgbdNode::check_subscribers()
{
  if (has_output_subscribers()) {
    idle_since_.reset();
    if (!inputs_subscribed()) {
      subscribe_inputs();
    }
    return;
  }
  if (!inputs_subscribed()) {
    return;
  }
  const rclcpp::Time now = get_clock()->now();
  if (!idle_since_) {
    idle_since_ = now;
  }
  if ((now - *idle_since_).seconds() >= config_.lazy.idle_unsubscribe_s) {
    unsubscribe_inputs();
    idle_since_.reset();
  }
}

void HeadRgbdNode::subscribe_inputs()
{
  const rmw_qos_profile_t qos = rclcpp::QoS(config_.sync.queue_size).best_effort().get_rmw_qos_profile();
  tof_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, config_.tof_depth_topic, qos);
  rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
    this, config_.rgb_image_topic, qos);
  info_sub_ =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, config_.rgb_info_topic, qos);
  sync_ = std::make_shared<Synchronizer>(SyncPolicy(config_.sync.queue_size), *tof_sub_, *rgb_sub_, *info_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(config_.sync.max_pair_dt_s));
  sync_->registerCallback(&HeadRgbdNode::on_set, this);
  RCLCPP_INFO(get_logger(), "output subscribers present: subscribed to the inputs");
}

void HeadRgbdNode::unsubscribe_inputs()
{
  sync_.reset();
  tof_sub_.reset();
  rgb_sub_.reset();
  info_sub_.reset();
  RCLCPP_INFO(get_logger(), "no output subscribers: released the inputs");
}

void HeadRgbdNode::drop(const char * reason, const std::string & detail)
{
  metrics_.counter("head_rgbd_pairs_dropped_total", "Synchronised sets dropped, by reason", {{"reason", reason}})
    .Increment();
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), DROP_LOG_THROTTLE_MS, "dropped a set (%s): %s", reason, detail.c_str());
}

bool HeadRgbdNode::lookup(
  const std::string & target, const rclcpp::Time & target_time, const std::string & source,
  const rclcpp::Time & source_time, const std::string & fixed, Eigen::Isometry3d & out)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(config_.motion.tf_timeout_s);
  while (true) {
    try {
      const auto tf = fixed.empty() ? tf_buffer_->lookupTransform(target, source, to_tf_time(source_time))
                                    : tf_buffer_->lookupTransform(
                                        target, to_tf_time(target_time), source, to_tf_time(source_time), fixed);
      out = tf2::transformToEigen(tf);
      return true;
    } catch (const tf2::TransformException & e) {
      if (std::chrono::steady_clock::now() >= deadline) {
        RCLCPP_DEBUG(get_logger(), "TF lookup %s <- %s failed: %s", target.c_str(), source.c_str(), e.what());
        return false;
      }
      std::this_thread::sleep_for(TF_RETRY_PERIOD);
    }
  }
}

void HeadRgbdNode::on_set(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth, const sensor_msgs::msg::CompressedImage::ConstSharedPtr & rgb,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  const auto started = std::chrono::steady_clock::now();
  struct Timer
  {
    std::chrono::steady_clock::time_point start;
    prometheus::Histogram * h;
    ~Timer() { h->Observe(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count()); }
  } timer{started, process_};

  const rclcpp::Time t_tof(depth->header.stamp);
  const rclcpp::Time t_rgb(rgb->header.stamp);
  pair_dt_->Observe(std::abs((t_rgb - t_tof).seconds()));

  if (!tof_rays_) {
    drop(DROP_REASON_NO_CALIBRATION, "no valid ToF calibration loaded");
    return;
  }
  if (depth->encoding != DEPTH_ENCODING) {
    drop(DROP_REASON_BAD_INPUT, "ToF encoding " + depth->encoding + ", expected " + DEPTH_ENCODING);
    return;
  }
  if (static_cast<int>(depth->width) != tof_model_->width || static_cast<int>(depth->height) != tof_model_->height) {
    drop(
      DROP_REASON_SIZE_MISMATCH, std::to_string(depth->width) + "x" + std::to_string(depth->height) +
                                   " ToF image, calibration is " + std::to_string(tof_model_->width) + "x" +
                                   std::to_string(tof_model_->height));
    return;
  }
  if (depth->step < depth->width * 2 || depth->data.size() < static_cast<size_t>(depth->step) * depth->height) {
    drop(DROP_REASON_BAD_INPUT, "ToF image data shorter than its size");
    return;
  }
  const auto rgb_model = model_of(*info);
  if (!rgb_model) {
    drop(DROP_REASON_NO_CAMERA_INFO, "RGB camera_info holds no calibration");
    return;
  }
  const int out_w = rgb_model->width / config_.output.divisor;
  const int out_h = rgb_model->height / config_.output.divisor;
  PinholeModel out_model;
  try {
    out_model = scale_model(*rgb_model, out_w, out_h);
  } catch (const std::invalid_argument & e) {
    drop(DROP_REASON_NO_CAMERA_INFO, e.what());
    return;
  }

  const MotionConfig & motion = config_.motion;
  const int64_t t_rgb_ns = t_rgb.nanoseconds();
  const int64_t t_tof_ns = t_tof.nanoseconds();
  if (evaluate_pair(t_rgb_ns, t_tof_ns, MotionRate{}, config_.sync, motion) == PairDecision::pair_dt) {
    drop(DROP_REASON_PAIR_DT, "ToF and RGB stamps too far apart");
    return;
  }

  const std::string & rgb_frame = rgb->header.frame_id;
  const std::string & tof_frame = depth->header.frame_id;
  const rclcpp::Time t_before = t_tof - rclcpp::Duration::from_seconds(motion.rate_window_s);
  Eigen::Isometry3d fixed_T_tof_before;
  Eigen::Isometry3d fixed_T_tof;
  Eigen::Isometry3d rgb_T_tof_static = Eigen::Isometry3d::Identity();
  if (
    !lookup(motion.fixed_frame, t_before, tof_frame, t_before, "", fixed_T_tof_before) ||
    !lookup(motion.fixed_frame, t_tof, tof_frame, t_tof, "", fixed_T_tof)) {
    drop(DROP_REASON_NO_TF, motion.fixed_frame + " <- " + tof_frame + " not available at the ToF stamp");
    return;
  }
  const MotionRate rate = frame_rate(fixed_T_tof_before, fixed_T_tof, motion.rate_window_s);
  if (evaluate_pair(t_rgb_ns, t_tof_ns, rate, config_.sync, motion) == PairDecision::motion) {
    drop(
      DROP_REASON_MOTION, "ToF frame moves at " + std::to_string(rate.angular_deg_s) + " deg/s, " +
                            std::to_string(rate.linear_m_s) + " m/s");
    return;
  }

  Eigen::Isometry3d rgb_T_tof;
  if (motion.compensate) {
    if (!lookup(rgb_frame, t_rgb, tof_frame, t_tof, motion.fixed_frame, rgb_T_tof)) {
      drop(DROP_REASON_NO_TF, rgb_frame + " <- " + tof_frame + " not available through " + motion.fixed_frame);
      return;
    }
  } else if (!lookup(rgb_frame, t_rgb, tof_frame, t_rgb, "", rgb_T_tof_static)) {
    drop(DROP_REASON_NO_TF, rgb_frame + " <- " + tof_frame + " static transform not available");
    return;
  } else {
    rgb_T_tof = rgb_T_tof_static;
  }

  const cv::Mat raw(depth->height, depth->width, CV_16UC1, const_cast<uint8_t *>(depth->data.data()), depth->step);
  const cv::Mat filtered = filter_jump_edges(raw, config_.filter);
  const cv::Mat registered = register_depth(filtered, *tof_rays_, rgb_T_tof, out_model, config_.depth);
  valid_ratio_->Set(static_cast<double>(cv::countNonZero(registered)) / static_cast<double>(registered.total()));

  publish_pair(rgb, info, registered, out_model);
}

void HeadRgbdNode::publish_pair(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & rgb,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info, const cv::Mat & registered, const PinholeModel & out_model)
{
  sensor_msgs::msg::Image depth_msg;
  depth_msg.header = rgb->header;
  depth_msg.height = registered.rows;
  depth_msg.width = registered.cols;
  depth_msg.encoding = DEPTH_ENCODING;
  depth_msg.is_bigendian = false;
  depth_msg.step = registered.cols * sizeof(uint16_t);
  depth_msg.data.assign(registered.data, registered.data + static_cast<size_t>(depth_msg.step) * registered.rows);

  sensor_msgs::msg::CameraInfo depth_info;
  depth_info.header = rgb->header;
  depth_info.width = registered.cols;
  depth_info.height = registered.rows;
  depth_info.distortion_model = info->distortion_model;
  depth_info.d = info->d;
  depth_info.r = info->r;
  depth_info.k = {out_model.K(0, 0), out_model.K(0, 1), out_model.K(0, 2), out_model.K(1, 0), out_model.K(1, 1),
                  out_model.K(1, 2), out_model.K(2, 0), out_model.K(2, 1), out_model.K(2, 2)};
  depth_info.p = {
    out_model.K(0, 0),
    out_model.K(0, 1),
    out_model.K(0, 2),
    0.0,
    out_model.K(1, 0),
    out_model.K(1, 1),
    out_model.K(1, 2),
    0.0,
    0.0,
    0.0,
    1.0,
    0.0};

  std::optional<sensor_msgs::msg::Image> up_msg;
  if (upsampling_) {
    const cv::Mat guide =
      decode_rgb_for_depth(rgb->data, static_cast<int>(info->width), static_cast<int>(info->height));
    try {
      if (guide.empty()) {
        throw std::invalid_argument("the RGB JPEG does not decode");
      }
      const cv::Mat up = guided_upsample(registered, guide, config_.upsampling);
      up_msg.emplace();
      up_msg->header = rgb->header;
      up_msg->height = up.rows;
      up_msg->width = up.cols;
      up_msg->encoding = DEPTH_ENCODING;
      up_msg->step = up.cols * sizeof(uint16_t);
      up_msg->data.assign(up.data, up.data + static_cast<size_t>(up_msg->step) * up.rows);
    } catch (const std::invalid_argument & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), DROP_LOG_THROTTLE_MS, "no upsampled depth: %s", e.what());
    }
  }

  depth_pub_->publish(depth_msg);
  depth_info_pub_->publish(depth_info);
  rgb_pub_->publish(*rgb);
  rgb_info_pub_->publish(*info);
  if (up_msg) {
    up_pub_->publish(*up_msg);
    up_info_pub_->publish(*info);
  }
  published_->Increment();

  if (
    points_pub_ && points_pub_->get_subscription_count() > 0 &&
    cloud_decimator_.due(rclcpp::Time(rgb->header.stamp).nanoseconds())) {
    const cv::Mat colour = decode_rgb_for_depth(rgb->data, registered.cols, registered.rows);
    const auto cloud = colour.empty() ? nullptr : build_cloud(depth_msg, depth_info, colour);
    if (cloud) {
      points_pub_->publish(*cloud);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), DROP_LOG_THROTTLE_MS, "no point cloud: RGB or depth not usable");
    }
  }
}

}  // namespace head_rgbd_cpp
