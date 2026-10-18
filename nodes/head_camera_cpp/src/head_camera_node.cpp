#include "head_camera_cpp/head_camera_node.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <utility>
#include <variant>

#include "head_camera_cpp/encode.hpp"
#include "head_camera_cpp/libcamera_backend.hpp"
#include "head_camera_cpp/mapping_stream.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace head_camera_cpp
{

namespace
{

constexpr int IMAGE_QOS_DEPTH = 1;
constexpr int COMMAND_QOS_DEPTH = 10;
constexpr int ZOOM_IMAGE_QOS_DEPTH = 2;
constexpr int64_t NS_PER_S = 1'000'000'000;
constexpr const char * ZOOM_REQUESTS_HELP = "Zoom requests, by result status";
constexpr const char * ZOOM_PHASE_HELP = "Duration of a zoom phase";
const std::vector<double> ENCODE_BUCKETS{0.002, 0.005, 0.01, 0.02, 0.033, 0.05, 0.1, 0.2};
const std::vector<double> PHASE_BUCKETS{0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0};
const std::array<ZoomStatus, 6> ALL_STATUSES{ZoomStatus::Ok,      ZoomStatus::AfFailed,    ZoomStatus::Busy,
                                             ZoomStatus::Timeout, ZoomStatus::CameraError, ZoomStatus::Invalid};

rclcpp::QoS reliable_qos(int depth) { return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable(); }

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

// The "data" of calibration matrix `key`, validated for its element count.
std::vector<double> matrix(const YAML::Node & root, const std::string & key, size_t size)
{
  const YAML::Node node = root[key];
  if (!node || !node["data"]) {
    throw std::runtime_error(key + " missing");
  }
  auto data = node["data"].as<std::vector<double>>();
  if (data.size() != size) {
    throw std::runtime_error(key + " must have " + std::to_string(size) + " values");
  }
  return data;
}

builtin_interfaces::msg::Time to_time_msg(int64_t ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / NS_PER_S);
  stamp.nanosec = static_cast<uint32_t>(ns % NS_PER_S);
  return stamp;
}

}  // namespace

sensor_msgs::msg::CameraInfo load_camera_info(
  const std::string & path, int width, int height, const std::string & frame_id)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("cannot read " + path + ": " + e.what());
  }
  if (!root["image_width"] || !root["image_height"]) {
    throw std::runtime_error(path + ": image_width and image_height are required");
  }
  CameraIntrinsics calibrated;
  calibrated.width = root["image_width"].as<int>();
  calibrated.height = root["image_height"].as<int>();

  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.distortion_model = root["distortion_model"] ? root["distortion_model"].as<std::string>() : "plumb_bob";
  if (root["distortion_coefficients"] && root["distortion_coefficients"]["data"]) {
    calibrated.d = root["distortion_coefficients"]["data"].as<std::vector<double>>();
  }
  const auto k = matrix(root, "camera_matrix", 9);
  std::copy(k.begin(), k.end(), calibrated.k.begin());
  const auto p = matrix(root, "projection_matrix", 12);
  std::copy(p.begin(), p.end(), calibrated.p.begin());
  if (root["rectification_matrix"]) {
    const auto r = matrix(root, "rectification_matrix", 9);
    std::copy(r.begin(), r.end(), calibrated.r.begin());
  }
  const CameraIntrinsics scaled = scale_intrinsics(calibrated, width, height);
  info.width = static_cast<uint32_t>(scaled.width);
  info.height = static_cast<uint32_t>(scaled.height);
  info.d = scaled.d;
  info.k = scaled.k;
  info.r = scaled.r;
  info.p = scaled.p;
  return info;
}

HeadCameraNode::HeadCameraNode(const rclcpp::NodeOptions & options)
: Node(NODE_NAME, options),
  config_(load_config_or_exit()),
  clock_(std::make_unique<SteadyClock>()),
  metrics_(METRICS_NODE)
{
  const auto logger = get_logger();
  backend_ = std::make_unique<LibcameraBackend>(config_, [logger](LogLevel level, const std::string & message) {
    switch (level) {
      case LogLevel::Info:
        RCLCPP_INFO(logger, "%s", message.c_str());
        break;
      case LogLevel::Warn:
        RCLCPP_WARN(logger, "%s", message.c_str());
        break;
      case LogLevel::Error:
        RCLCPP_ERROR(logger, "%s", message.c_str());
        break;
    }
  });
  init();
}

HeadCameraNode::HeadCameraNode(
  const Config & config, std::unique_ptr<CameraBackend> backend, std::unique_ptr<Clock> clock,
  const rclcpp::NodeOptions & options)
: Node(NODE_NAME, options),
  config_(config),
  backend_(std::move(backend)),
  clock_(std::move(clock)),
  metrics_(METRICS_NODE)
{
  init();
}

void HeadCameraNode::init()
{
  if (config_.metrics_port) {
    metrics_.serve(static_cast<std::uint16_t>(*config_.metrics_port));
  }
  frames_captured_ = &metrics_.counter("head_camera_frames_captured_total", "Video frames delivered by the camera");
  const std::string published_help = "Frames published, by topic";
  published_raw_ = &metrics_.counter("head_camera_frames_published_total", published_help, {{"topic", "image_raw"}});
  published_compressed_ =
    &metrics_.counter("head_camera_frames_published_total", published_help, {{"topic", "image_raw_compressed"}});
  published_info_ = &metrics_.counter("head_camera_frames_published_total", published_help, {{"topic", "camera_info"}});
  capture_fps_ = &metrics_.gauge("head_camera_capture_fps", "Video frames per second delivered by the camera");
  encode_seconds_ =
    &metrics_.histogram("head_camera_encode_seconds", "Time to JPEG-encode one video frame", ENCODE_BUCKETS);
  for (const auto status : ALL_STATUSES) {
    metrics_.counter("head_camera_zoom_requests_total", ZOOM_REQUESTS_HELP, {{"status", to_string(status)}});
  }
  for (const char * phase : {"af", "switch", "capture", "restore"}) {
    metrics_.histogram("head_camera_zoom_phase_seconds", ZOOM_PHASE_HELP, PHASE_BUCKETS, {{"phase", phase}});
  }

  zoom_ = std::make_unique<ZoomController>(*backend_, *clock_, config_.zoom);

  image_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.image_raw(), reliable_qos(IMAGE_QOS_DEPTH));
  compressed_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>(config_.image_raw_compressed(), reliable_qos(IMAGE_QOS_DEPTH));
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(config_.camera_info(), reliable_qos(IMAGE_QOS_DEPTH));
  zoom_image_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>(config_.zoom_image(), reliable_qos(ZOOM_IMAGE_QOS_DEPTH));
  zoom_result_pub_ = create_publisher<std_msgs::msg::String>(config_.zoom_result(), reliable_qos(COMMAND_QOS_DEPTH));

  if (config_.camera_info_file) {
    try {
      camera_info_ = load_camera_info(*config_.camera_info_file, config_.width, config_.height, config_.frame_id);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "camera_info not published: %s", e.what());
    }
  } else {
    RCLCPP_INFO(get_logger(), "no camera_info_file configured: camera_info is not published");
  }

  video_thread_ = std::thread([this] { video_loop(); });
  zoom_thread_ = std::thread([this] { zoom_loop(); });

  backend_->set_video_callback([this](const VideoFrame & frame) { on_video_frame(frame); });
  command_sub_ = create_subscription<std_msgs::msg::String>(
    config_.zoom_command(), reliable_qos(COMMAND_QOS_DEPTH),
    [this](const std_msgs::msg::String & msg) { on_command(msg); });

  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    video_running_ = backend_->start_video();
  }
  if (!video_running_) {
    RCLCPP_ERROR(get_logger(), "camera did not start; retrying every %d ms", START_RETRY_PERIOD_MS);
  }
  retry_timer_ = create_wall_timer(std::chrono::milliseconds(START_RETRY_PERIOD_MS), [this] { retry_start(); });
  fps_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { expire_capture_fps(); });
}

HeadCameraNode::~HeadCameraNode()
{
  retry_timer_.reset();
  fps_timer_.reset();
  backend_->set_video_callback({});
  {
    std::lock_guard<std::mutex> lock(zoom_mutex_);
    zoom_stopping_ = true;
  }
  zoom_ready_.notify_all();
  zoom_thread_.join();
  {
    std::lock_guard<std::mutex> lock(backend_mutex_);
    backend_->stop();
  }
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    stopping_ = true;
  }
  frame_ready_.notify_all();
  video_thread_.join();
}

int64_t HeadCameraNode::to_ros_ns(int64_t steady_ns) const
{
  return steady_ns + (get_clock()->now().nanoseconds() - steady_now_ns());
}

void HeadCameraNode::retry_start()
{
  if (video_running_) {
    return;
  }
  std::unique_lock<std::mutex> lock(backend_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;  // a zoom is running and restores the stream itself
  }
  video_running_ = backend_->start_video();
  if (video_running_) {
    RCLCPP_INFO(get_logger(), "camera started");
  } else {
    RCLCPP_ERROR(get_logger(), "camera did not start; retrying in %d ms", START_RETRY_PERIOD_MS);
  }
}

void HeadCameraNode::on_video_frame(const VideoFrame & frame)
{
  frames_captured_->Increment();
  record_capture(clock_->now());
  const size_t bytes = static_cast<size_t>(frame.stride) * static_cast<size_t>(frame.size.height);
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    slot_.bgr.assign(frame.bgr, frame.bgr + bytes);  // an unpublished older frame is overwritten
    slot_.stride = frame.stride;
    slot_.size = frame.size;
    slot_.stamp_ns = frame.stamp_ns;
    slot_full_ = true;
  }
  frame_ready_.notify_one();
}

void HeadCameraNode::record_capture(Clock::TimePoint now)
{
  std::lock_guard<std::mutex> lock(fps_mutex_);
  last_capture_ = now;
  if (!fps_window_open_) {
    // The first frame only opens the window: n frames span n - 1 intervals.
    fps_window_open_ = true;
    fps_window_start_ = now;
    fps_window_intervals_ = 0;
    return;
  }
  ++fps_window_intervals_;
  const double window_s = std::chrono::duration<double>(now - fps_window_start_).count();
  if (window_s >= FPS_WINDOW_S) {
    capture_fps_->Set(fps_window_intervals_ / window_s);
    fps_window_start_ = now;
    fps_window_intervals_ = 0;
  }
}

void HeadCameraNode::expire_capture_fps()
{
  std::lock_guard<std::mutex> lock(fps_mutex_);
  if (fps_window_open_ && clock_->now() - last_capture_ > std::chrono::duration<double>(2 * FPS_WINDOW_S)) {
    capture_fps_->Set(0.0);
    fps_window_open_ = false;
  }
}

void HeadCameraNode::video_loop()
{
  Frame work;
  while (true) {
    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      frame_ready_.wait(lock, [this] { return slot_full_ || stopping_; });
      if (stopping_) {
        return;
      }
      std::swap(slot_, work);
      slot_full_ = false;
    }
    try {
      publish_frame(work);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "frame not published: %s", e.what());
    }
  }
}

void HeadCameraNode::publish_frame(const Frame & frame)
{
  std_msgs::msg::Header header;
  header.stamp = to_time_msg(to_ros_ns(frame.stamp_ns));
  header.frame_id = config_.frame_id;

  if (image_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = header;
    msg->height = static_cast<uint32_t>(frame.size.height);
    msg->width = static_cast<uint32_t>(frame.size.width);
    msg->encoding = "bgr8";
    msg->is_bigendian = false;
    msg->step = static_cast<uint32_t>(frame.size.width * 3);
    msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
    for (int row = 0; row < frame.size.height; ++row) {
      const uint8_t * src = frame.bgr.data() + static_cast<size_t>(row) * static_cast<size_t>(frame.stride);
      std::copy(src, src + msg->step, msg->data.begin() + static_cast<std::ptrdiff_t>(row) * msg->step);
    }
    image_pub_->publish(std::move(msg));
    published_raw_->Increment();
  }

  if (compressed_pub_->get_subscription_count() > 0) {
    const auto start = std::chrono::steady_clock::now();
    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header = header;
    msg->format = COMPRESSED_FORMAT;
    msg->data =
      encode_bgr_jpeg(frame.bgr.data(), frame.size.width, frame.size.height, frame.stride, config_.jpeg_quality);
    encode_seconds_->Observe(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    compressed_pub_->publish(std::move(msg));
    published_compressed_->Increment();
  }

  if (camera_info_) {
    auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(*camera_info_);
    info->header = header;
    info_pub_->publish(std::move(info));
    published_info_->Increment();
  }
}

void HeadCameraNode::publish_result(const ZoomResult & result)
{
  metrics_.counter("head_camera_zoom_requests_total", ZOOM_REQUESTS_HELP, {{"status", to_string(result.status)}})
    .Increment();
  std_msgs::msg::String msg;
  msg.data = to_json(result);
  zoom_result_pub_->publish(msg);
}

void HeadCameraNode::on_command(const std_msgs::msg::String & msg)
{
  const auto parsed = parse_request(msg.data, config_.zoom);
  if (const auto * error = std::get_if<ParseError>(&parsed)) {
    if (error->request_id) {
      RCLCPP_WARN(get_logger(), "invalid zoom request %s: %s", error->request_id->c_str(), error->message.c_str());
      publish_result(invalid_result(*error));
    } else {
      RCLCPP_WARN(get_logger(), "zoom command dropped: %s", error->message.c_str());
    }
    return;
  }
  const auto & request = std::get<ZoomRequest>(parsed);
  if (!zoom_->try_begin()) {
    publish_result(ZoomController::busy_result(request));
    return;
  }
  {
    std::lock_guard<std::mutex> lock(zoom_mutex_);
    zoom_pending_ = request;
  }
  zoom_ready_.notify_one();
}

void HeadCameraNode::zoom_loop()
{
  while (true) {
    ZoomRequest request;
    {
      std::unique_lock<std::mutex> lock(zoom_mutex_);
      zoom_ready_.wait(lock, [this] { return zoom_pending_.has_value() || zoom_stopping_; });
      if (!zoom_pending_) {
        return;
      }
      request = *zoom_pending_;
      zoom_pending_.reset();
    }
    ZoomOutcome outcome;
    {
      std::lock_guard<std::mutex> lock(backend_mutex_);
      outcome = zoom_->run_claimed(request);
    }
    try {
      publish_zoom(outcome);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "zoom result not published: %s", e.what());
    }
  }
}

void HeadCameraNode::publish_zoom(const ZoomOutcome & outcome)
{
  ZoomResult result = outcome.result;
  const auto & phase = result.phase_ms;
  const std::array<std::pair<const char *, double>, 4> phases{
    {{"af", phase.af}, {"switch", phase.switch_mode}, {"capture", phase.capture}, {"restore", phase.restore}}};
  for (const auto & [name, ms] : phases) {
    if (ms > 0.0) {
      metrics_.histogram("head_camera_zoom_phase_seconds", ZOOM_PHASE_HELP, PHASE_BUCKETS, {{"phase", name}})
        .Observe(ms / 1000.0);
    }
  }
  if (carries_image(result.status) && !outcome.jpeg.empty()) {
    // The still's sensor stamp in ROS time; the result carries the same value so consumers can pair the two.
    result.stamp_ns = to_ros_ns(result.stamp_ns);
    sensor_msgs::msg::CompressedImage image;
    image.header.stamp = to_time_msg(result.stamp_ns);
    image.header.frame_id = config_.frame_id;
    image.format = ZOOM_IMAGE_FORMAT;
    image.data = outcome.jpeg;
    zoom_image_pub_->publish(image);
  }
  publish_result(result);
}

}  // namespace head_camera_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(head_camera_cpp::HeadCameraNode)
