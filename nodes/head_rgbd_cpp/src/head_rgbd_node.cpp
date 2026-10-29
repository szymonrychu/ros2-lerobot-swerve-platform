#include "head_rgbd_cpp/head_rgbd_node.hpp"

#include <array>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <utility>
#include <vector>

namespace head_rgbd_cpp
{

namespace
{
namespace enc = sensor_msgs::image_encodings;

constexpr double MM_PER_M = 1000.0;
constexpr size_t DEPTH_BYTES_PER_PX = sizeof(uint16_t);
constexpr size_t COLOR_BYTES_PER_PX = 3;
constexpr size_t PROJECTION_ROWS_SCALED = 2;  // the fx/cx and fy/cy rows of K and P
constexpr size_t K_COLS = 3;
constexpr size_t P_COLS = 4;
constexpr std::array<double, 8> PROCESS_BUCKETS = {0.002, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5};
// Colour minus depth stamp of a pair candidate, s: the 0 bucket (+-0.5 ms) holds exact pairs, a few ms is a stamp
// mismatch approximate pairing fixes, about a frame period (66.7 ms at 15 Hz) is a lost frame.
constexpr std::array<double, 12> PAIR_DELTA_BUCKETS = {-0.05,  -0.02, -0.01, -0.005, -0.002, -0.0005,
                                                       0.0005, 0.002, 0.005, 0.01,   0.02,   0.05};
constexpr std::array<const char *, 4> ALL_DROP_REASONS = {
  DROP_REASON_BAD_INPUT, DROP_REASON_SIZE_MISMATCH, DROP_REASON_NO_CAMERA_INFO, DROP_REASON_UNPAIRED};
constexpr std::array<const char *, 2> FILTERED_DROP_REASONS = {DROP_REASON_BAD_INPUT, DROP_REASON_NO_CAMERA_INFO};
constexpr const char * DROPPED_SERIES = "head_rgbd_dropped_total";
constexpr const char * DROPPED_HELP = "Input frames or sets dropped, by reason (unpaired: per frame)";
constexpr const char * FILTERED_DROPPED_SERIES = "head_rgbd_filtered_depth_dropped_total";
constexpr const char * FILTERED_DROPPED_HELP = "Full-FOV depth frames not republished, by reason";
constexpr const char * FRAMES_SERIES = "head_rgbd_frames_total";
constexpr const char * FRAMES_HELP = "Input frames offered to the pairing, by stream";
constexpr const char * REMOVED_SERIES = "head_rgbd_edge_mask_removed_ratio";
constexpr const char * REMOVED_HELP = "Fraction of the valid depth pixels of the last frame removed by the edge mask";

std::string size_of(uint32_t width, uint32_t height) { return std::to_string(width) + "x" + std::to_string(height); }

// True when the image data holds height rows of step bytes, each at least width * bytes_per_px long.
bool data_fits(const sensor_msgs::msg::Image & image, size_t bytes_per_px)
{
  return image.width > 0 && image.height > 0 && image.step >= image.width * bytes_per_px &&
         image.data.size() >= static_cast<size_t>(image.step) * image.height;
}

// The camera matrix holds a usable pinhole calibration: positive focal lengths and a homogeneous last row.
bool has_camera_matrix(const sensor_msgs::msg::CameraInfo & info)
{
  return info.k[0] > 0.0 && info.k[4] > 0.0 && info.k[8] != 0.0;
}
}  // namespace

cv::Mat downscale_depth(const cv::Mat & depth_mm, int divisor, const DepthConfig & range)
{
  const double min_mm = range.min_m * MM_PER_M;
  const double max_mm = range.max_m * MM_PER_M;
  const auto valid = [min_mm, max_mm](uint16_t mm) { return mm != 0 && mm >= min_mm && mm <= max_mm; };
  cv::Mat out(depth_mm.rows / divisor, depth_mm.cols / divisor, CV_16UC1, cv::Scalar(0));
  for (int v = 0; v < out.rows; ++v) {
    auto * dst = out.ptr<uint16_t>(v);
    for (int u = 0; u < out.cols; ++u) {
      for (int k = 0; k < divisor * divisor; ++k) {
        const uint16_t mm = depth_mm.ptr<uint16_t>(v * divisor + k / divisor)[u * divisor + k % divisor];
        if (valid(mm)) {
          dst[u] = mm;
          break;
        }
      }
    }
  }
  return out;
}

sensor_msgs::msg::CameraInfo scale_camera_info(const sensor_msgs::msg::CameraInfo & info, int divisor)
{
  sensor_msgs::msg::CameraInfo out = info;
  const auto d = static_cast<uint32_t>(divisor);
  out.width = info.width / d;
  out.height = info.height / d;
  for (size_t i = 0; i < PROJECTION_ROWS_SCALED * K_COLS; ++i) {
    out.k[i] = info.k[i] / divisor;
  }
  for (size_t i = 0; i < PROJECTION_ROWS_SCALED * P_COLS; ++i) {
    out.p[i] = info.p[i] / divisor;
  }
  out.roi.x_offset = info.roi.x_offset / d;
  out.roi.y_offset = info.roi.y_offset / d;
  out.roi.width = info.roi.width / d;
  out.roi.height = info.roi.height / d;
  return out;
}

SetThrottle::SetThrottle(double rate_hz) : period_ns_(static_cast<int64_t>(std::llround(NS_PER_S / rate_hz))) {}

bool SetThrottle::due(int64_t stamp_ns)
{
  if (last_ns_ && stamp_ns >= *last_ns_ && stamp_ns < next_ns_) {
    return false;
  }
  const bool on_grid = last_ns_ && stamp_ns >= *last_ns_ && stamp_ns - next_ns_ < period_ns_;
  next_ns_ = on_grid ? next_ns_ + period_ns_ : stamp_ns + period_ns_;
  last_ns_ = stamp_ns;
  return true;
}

HeadRgbdNode::HeadRgbdNode(const Config & config, const rclcpp::NodeOptions & options)
: rclcpp::Node(NODE_NAME, options),
  config_(config),
  set_throttle_(config.output.rate_hz),
  cloud_decimator_(config.cloud.rate_hz),
  metrics_(NODE_NAME)
{
  if (config_.metrics_port) {
    metrics_.serve(static_cast<std::uint16_t>(*config_.metrics_port));
  }
  published_ = &metrics_.counter("head_rgbd_sets_published_total", "RGB-D sets published");
  for (const char * reason : ALL_DROP_REASONS) {
    metrics_.counter(DROPPED_SERIES, DROPPED_HELP, {{"reason", reason}});
  }
  process_ = &metrics_.histogram(
    "head_rgbd_process_seconds", "Processing time of one published set",
    std::vector<double>(PROCESS_BUCKETS.begin(), PROCESS_BUCKETS.end()));
  valid_ratio_ = &metrics_.gauge("head_rgbd_valid_ratio", "Fraction of published depth pixels holding a depth");
  depth_frames_ = &metrics_.counter(FRAMES_SERIES, FRAMES_HELP, {{"stream", "depth"}});
  colour_frames_ = &metrics_.counter(FRAMES_SERIES, FRAMES_HELP, {{"stream", "colour"}});
  pairs_ = &metrics_.counter("head_rgbd_pairs_total", "Depth and colour pairs formed (before the set throttle)");
  pair_delta_ = &metrics_.histogram(
    "head_rgbd_pair_stamp_delta_seconds",
    "Colour minus depth stamp of each pair, and of each unpaired colour frame to the nearest depth frame",
    std::vector<double>(PAIR_DELTA_BUCKETS.begin(), PAIR_DELTA_BUCKETS.end()));
  aligned_removed_ = &metrics_.gauge(REMOVED_SERIES, REMOVED_HELP, {{"stage", STAGE_ALIGNED}});

  const rclcpp::QoS qos = rclcpp::QoS(1).best_effort();
  depth_pub_ = create_publisher<Image>(config_.depth_image(), qos);
  depth_info_pub_ = create_publisher<CameraInfo>(config_.depth_info(), qos);
  rgb_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(config_.rgb_image(), qos);
  rgb_info_pub_ = create_publisher<CameraInfo>(config_.rgb_camera_info(), qos);
  if (config_.cloud.enabled) {
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(config_.points(), qos);
  }
  if (config_.filtered_depth.enabled) {
    // Reliable: d435_obstacle_cloud (point_cloud_xyz, qos 0) subscribes with the system default, reliable; best-effort
    // subscribers match it too.
    const rclcpp::QoS filtered_qos = rclcpp::QoS(1).reliable();
    filtered_pub_ = create_publisher<Image>(config_.filtered_depth.image(), filtered_qos);
    filtered_info_pub_ = create_publisher<CameraInfo>(config_.filtered_depth.camera_info(), filtered_qos);
    filtered_published_ =
      &metrics_.counter("head_rgbd_filtered_depth_published_total", "Full-FOV depth frames republished masked");
    for (const char * reason : FILTERED_DROP_REASONS) {
      metrics_.counter(FILTERED_DROPPED_SERIES, FILTERED_DROPPED_HELP, {{"reason", reason}});
    }
    filtered_process_ = &metrics_.histogram(
      "head_rgbd_filtered_depth_process_seconds", "Processing time of one full-FOV depth frame",
      std::vector<double>(PROCESS_BUCKETS.begin(), PROCESS_BUCKETS.end()));
    filtered_removed_ = &metrics_.gauge(REMOVED_SERIES, REMOVED_HELP, {{"stage", STAGE_FILTERED}});
  }

  lazy_timer_ = create_wall_timer(LAZY_CHECK_PERIOD, [this] { check_subscribers(); });
}

bool HeadRgbdNode::has_output_subscribers() const
{
  size_t count = depth_pub_->get_subscription_count() + depth_info_pub_->get_subscription_count() +
                 rgb_pub_->get_subscription_count() + rgb_info_pub_->get_subscription_count();
  if (points_pub_) {
    count += points_pub_->get_subscription_count();
  }
  return count > 0;
}

bool HeadRgbdNode::has_filtered_subscribers() const
{
  return filtered_pub_ && filtered_pub_->get_subscription_count() + filtered_info_pub_->get_subscription_count() > 0;
}

void HeadRgbdNode::check_subscribers()
{
  check_stage(
    has_output_subscribers(), inputs_subscribed(), idle_since_, [this] { subscribe_inputs(); },
    [this] { unsubscribe_inputs(); });
  check_stage(
    has_filtered_subscribers(), filtered_inputs_subscribed(), filtered_idle_since_,
    [this] { subscribe_filtered_inputs(); }, [this] { unsubscribe_filtered_inputs(); });
}

void HeadRgbdNode::check_stage(
  bool has_subscribers, bool subscribed, std::optional<rclcpp::Time> & idle_since,
  const std::function<void()> & subscribe, const std::function<void()> & unsubscribe)
{
  if (has_subscribers) {
    idle_since.reset();
    if (!subscribed) {
      subscribe();
    }
    return;
  }
  if (!subscribed) {
    return;
  }
  const rclcpp::Time now = get_clock()->now();
  if (!idle_since) {
    idle_since = now;
  }
  if ((now - *idle_since).seconds() >= config_.lazy.idle_unsubscribe_s) {
    unsubscribe();
    idle_since.reset();
  }
}

void HeadRgbdNode::subscribe_inputs()
{
  pairer_ = std::make_unique<FramePairer>(
    config_.pairing, PAIR_QUEUE_SIZE,
    [this](const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & color) { on_pair(depth, color); },
    [this](Stream stream, int64_t stamp_ns) { on_unpaired(stream, stamp_ns); },
    [this](double delta_s) { pair_delta_->Observe(delta_s); });
  const rclcpp::QoS qos = rclcpp::QoS(PAIR_QUEUE_SIZE).best_effort();
  depth_sub_ = create_subscription<Image>(config_.depth_topic, qos, [this](Image::ConstSharedPtr depth) {
    depth_frames_->Increment();
    pairer_->add_depth(depth);
  });
  color_sub_ = create_subscription<Image>(config_.rgb_image_topic, qos, [this](Image::ConstSharedPtr color) {
    colour_frames_->Increment();
    pairer_->add_colour(color);
  });
  info_sub_ = create_subscription<CameraInfo>(
    config_.rgb_info_topic, rclcpp::QoS(1).best_effort(),
    [this](CameraInfo::ConstSharedPtr info) { latest_info_ = std::move(info); });
  RCLCPP_INFO(get_logger(), "output subscribers present: subscribed to the inputs");
}

void HeadRgbdNode::unsubscribe_inputs()
{
  depth_sub_.reset();
  color_sub_.reset();
  info_sub_.reset();
  pairer_.reset();
  latest_info_.reset();
  RCLCPP_INFO(get_logger(), "no output subscribers: released the inputs");
}

void HeadRgbdNode::subscribe_filtered_inputs()
{
  const rclcpp::QoS qos = rclcpp::QoS(FILTERED_QUEUE_SIZE).best_effort();
  filtered_depth_sub_ = create_subscription<Image>(
    config_.filtered_depth.depth_topic, qos, [this](Image::ConstSharedPtr depth) { on_filtered_depth(depth); });
  filtered_info_sub_ = create_subscription<CameraInfo>(
    config_.filtered_depth.info_topic, rclcpp::QoS(1).best_effort(),
    [this](CameraInfo::ConstSharedPtr info) { latest_depth_info_ = std::move(info); });
  RCLCPP_INFO(get_logger(), "filtered depth subscribers present: subscribed to the full-FOV depth");
}

void HeadRgbdNode::unsubscribe_filtered_inputs()
{
  filtered_depth_sub_.reset();
  filtered_info_sub_.reset();
  latest_depth_info_.reset();
  RCLCPP_INFO(get_logger(), "no filtered depth subscribers: released the full-FOV depth");
}

void HeadRgbdNode::drop(const char * reason, const std::string & detail)
{
  metrics_.counter(DROPPED_SERIES, DROPPED_HELP, {{"reason", reason}}).Increment();
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), DROP_LOG_THROTTLE_MS, "dropped a set (%s): %s", reason, detail.c_str());
}

void HeadRgbdNode::on_unpaired(Stream stream, int64_t stamp_ns)
{
  drop(
    DROP_REASON_UNPAIRED, std::string(stream == Stream::DEPTH ? "depth" : "colour") + " frame at " +
                            std::to_string(static_cast<double>(stamp_ns) / NS_PER_S) + " s has no partner");
}

void HeadRgbdNode::drop_filtered(const char * reason, const std::string & detail)
{
  metrics_.counter(FILTERED_DROPPED_SERIES, FILTERED_DROPPED_HELP, {{"reason", reason}}).Increment();
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), DROP_LOG_THROTTLE_MS, "filtered depth not published (%s): %s", reason, detail.c_str());
}

void HeadRgbdNode::mask_depth(cv::Mat & depth_mm, prometheus::Gauge & removed_ratio) const
{
  if (!config_.edge_mask.enabled) {
    return;
  }
  const int valid = cv::countNonZero(depth_mm);
  const int removed = ros2_common_cpp::apply_depth_edge_mask(depth_mm, config_.edge_mask);
  removed_ratio.Set(valid > 0 ? static_cast<double>(removed) / valid : 0.0);
}

void HeadRgbdNode::on_filtered_depth(const Image::ConstSharedPtr & depth)
{
  const auto started = std::chrono::steady_clock::now();
  if (depth->encoding != enc::TYPE_16UC1 || depth->is_bigendian || !data_fits(*depth, DEPTH_BYTES_PER_PX)) {
    drop_filtered(DROP_REASON_BAD_INPUT, "depth encoding " + depth->encoding + " or data size, expected 16UC1");
    return;
  }
  if (!latest_depth_info_) {
    drop_filtered(DROP_REASON_NO_CAMERA_INFO, "no camera_info received on " + config_.filtered_depth.info_topic);
    return;
  }
  if (latest_depth_info_->width != depth->width || latest_depth_info_->height != depth->height) {
    drop_filtered(
      DROP_REASON_NO_CAMERA_INFO, "camera_info is " + size_of(latest_depth_info_->width, latest_depth_info_->height) +
                                    ", depth " + size_of(depth->width, depth->height));
    return;
  }
  if (!has_camera_matrix(*latest_depth_info_)) {
    drop_filtered(DROP_REASON_NO_CAMERA_INFO, "camera_info holds no camera matrix (zero or non-positive K)");
    return;
  }

  Image out;
  out.header = depth->header;
  out.height = depth->height;
  out.width = depth->width;
  out.encoding = enc::TYPE_16UC1;
  out.is_bigendian = false;
  out.step = static_cast<uint32_t>(depth->width * DEPTH_BYTES_PER_PX);
  cv::Mat filtered(static_cast<int>(out.height), static_cast<int>(out.width), CV_16UC1);
  cv::Mat(
    static_cast<int>(depth->height), static_cast<int>(depth->width), CV_16UC1,
    const_cast<uint8_t *>(depth->data.data()), depth->step)
    .copyTo(filtered);
  mask_depth(filtered, *filtered_removed_);
  out.data.assign(filtered.data, filtered.data + static_cast<size_t>(out.step) * out.height);

  CameraInfo info = *latest_depth_info_;
  info.header = depth->header;
  filtered_pub_->publish(out);
  filtered_info_pub_->publish(info);
  filtered_published_->Increment();
  filtered_process_->Observe(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
}

const char * HeadRgbdNode::reject_reason(const Image & depth, const Image & color, std::string & detail) const
{
  if (depth.encoding != enc::TYPE_16UC1 || depth.is_bigendian) {
    detail = "depth encoding " + depth.encoding + (depth.is_bigendian ? " big-endian" : "") + ", expected 16UC1";
    return DROP_REASON_BAD_INPUT;
  }
  if (!data_fits(depth, DEPTH_BYTES_PER_PX)) {
    detail = "depth data does not hold its " + size_of(depth.width, depth.height) + " size";
    return DROP_REASON_BAD_INPUT;
  }
  if (color.encoding != enc::RGB8 && color.encoding != enc::BGR8) {
    detail = "colour encoding " + color.encoding + ", expected rgb8 or bgr8";
    return DROP_REASON_BAD_INPUT;
  }
  if (!data_fits(color, COLOR_BYTES_PER_PX)) {
    detail = "colour data does not hold its " + size_of(color.width, color.height) + " size";
    return DROP_REASON_BAD_INPUT;
  }
  if (depth.width != color.width || depth.height != color.height) {
    detail = size_of(depth.width, depth.height) + " depth, " + size_of(color.width, color.height) + " colour";
    return DROP_REASON_SIZE_MISMATCH;
  }
  const auto divisor = static_cast<uint32_t>(config_.output.divisor);
  if (depth.width < divisor || depth.height < divisor) {
    detail = size_of(depth.width, depth.height) + " is smaller than the divisor";
    return DROP_REASON_BAD_INPUT;
  }
  if (!latest_info_) {
    detail = "no camera_info received on " + config_.rgb_info_topic;
    return DROP_REASON_NO_CAMERA_INFO;
  }
  if (latest_info_->width != color.width || latest_info_->height != color.height) {
    detail = "camera_info is " + size_of(latest_info_->width, latest_info_->height) + ", colour " +
             size_of(color.width, color.height);
    return DROP_REASON_NO_CAMERA_INFO;
  }
  if (!has_camera_matrix(*latest_info_)) {
    detail = "camera_info holds no camera matrix (zero or non-positive K)";
    return DROP_REASON_NO_CAMERA_INFO;
  }
  return nullptr;
}

void HeadRgbdNode::on_pair(const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & color)
{
  pairs_->Increment();
  std::string detail;
  if (const char * reason = reject_reason(*depth, *color, detail)) {
    drop(reason, detail);
    return;
  }
  if (!set_throttle_.due(rclcpp::Time(color->header.stamp).nanoseconds())) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  publish_set(*depth, *color);
  process_->Observe(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
}

void HeadRgbdNode::publish_set(const Image & depth, const Image & color)
{
  cv::Mat masked;
  cv::Mat(
    static_cast<int>(depth.height), static_cast<int>(depth.width), CV_16UC1, const_cast<uint8_t *>(depth.data.data()),
    depth.step)
    .copyTo(masked);
  mask_depth(masked, *aligned_removed_);
  const cv::Mat scaled = downscale_depth(masked, config_.output.divisor, config_.depth);
  valid_ratio_->Set(static_cast<double>(cv::countNonZero(scaled)) / static_cast<double>(scaled.total()));

  Image depth_msg;
  depth_msg.header = color.header;
  depth_msg.height = static_cast<uint32_t>(scaled.rows);
  depth_msg.width = static_cast<uint32_t>(scaled.cols);
  depth_msg.encoding = enc::TYPE_16UC1;
  depth_msg.is_bigendian = false;
  depth_msg.step = static_cast<uint32_t>(scaled.cols * DEPTH_BYTES_PER_PX);
  depth_msg.data.assign(scaled.data, scaled.data + static_cast<size_t>(depth_msg.step) * scaled.rows);

  CameraInfo rgb_info = *latest_info_;
  rgb_info.header = color.header;
  CameraInfo depth_info = scale_camera_info(rgb_info, config_.output.divisor);

  // The JPEG and the cloud colours are BGR; an rgb8 input is converted.
  const cv::Mat bgr = cv_bridge::toCvCopy(color, enc::BGR8)->image;
  sensor_msgs::msg::CompressedImage rgb_msg;
  rgb_msg.header = color.header;
  rgb_msg.format = COMPRESSED_FORMAT;
  cv::imencode(".jpg", bgr, rgb_msg.data, {cv::IMWRITE_JPEG_QUALITY, config_.output.jpeg_quality});

  depth_pub_->publish(depth_msg);
  depth_info_pub_->publish(depth_info);
  rgb_pub_->publish(rgb_msg);
  rgb_info_pub_->publish(rgb_info);
  published_->Increment();

  if (
    points_pub_ && points_pub_->get_subscription_count() > 0 &&
    cloud_decimator_.due(rclcpp::Time(color.header.stamp).nanoseconds())) {
    cv::Mat colour = bgr;
    if (colour.cols != scaled.cols || colour.rows != scaled.rows) {
      cv::resize(bgr, colour, scaled.size(), 0, 0, cv::INTER_AREA);
    }
    const auto cloud = build_cloud(depth_msg, depth_info, colour);
    if (cloud) {
      points_pub_->publish(*cloud);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), DROP_LOG_THROTTLE_MS, "no point cloud: depth not usable");
    }
  }
}

}  // namespace head_rgbd_cpp
