// Depth wire frames -> the ROS messages the robot already consumes, and the staleness policy that decides whether a
// frame may be published at all. Pure logic: no sockets, no node, no clock (the caller passes the Pi's time in).
#ifndef JETSON_BRIDGE_CPP__CONVERT_HPP_
#define JETSON_BRIDGE_CPP__CONVERT_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <mutex>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "ros2_common_cpp/depth_wire.hpp"

namespace jetson_bridge_cpp
{

// Contract frame ids, used when a header carries none.
constexpr const char * kStereoLeftFrame = "stereo_left_optical_frame";
constexpr const char * kOverviewFrame = "overview_camera_optical_frame";
constexpr const char * kDepthEncoding = "16UC1";
constexpr const char * kJpegFormat = "jpeg";

constexpr int64_t kDefaultMaxAgeMs = 500;
constexpr int64_t kDefaultAbsentAfterMs = 2000;
constexpr int64_t kDefaultMaxFutureMs = 50;

// The Jetson's CLOCK_REALTIME capture time as a ROS stamp (floor division: nanosec is never negative).
builtin_interfaces::msg::Time stamp_from_realtime_ns(int64_t realtime_ns);

// 16UC1 depth image, millimetres, 0 = invalid, little-endian, step 2*width. header.kind must be kDepth or kFused and
// depth_mm.size() must equal width*height (else std::invalid_argument).
sensor_msgs::msg::Image to_depth_image(
  const ros2_common_cpp::depth_wire::Header & header, const std::vector<uint16_t> & depth_mm);

// kRectified: the rectified left camera (K from P, zero distortion, identity R, P as sent), for /stereo/camera_info.
// kRaw: the unrectified left camera (K, D as sent, identity R, P = K with a zero translation column), for
// /overview_camera/camera_info. Both carry the frame, stamp and size of the frame they accompany.
enum class CameraModel
{
  kRectified,
  kRaw
};

// nullopt unless header.calibrated is set: an uncalibrated Jetson sends zeros, which must never reach a consumer.
std::optional<sensor_msgs::msg::CameraInfo> to_camera_info(
  const ros2_common_cpp::depth_wire::Header & header, CameraModel model);

// JPEG overview image, format "jpeg", bytes unchanged. header.kind must be kRgb and jpeg non-empty (else
// std::invalid_argument).
sensor_msgs::msg::CompressedImage to_compressed(
  const ros2_common_cpp::depth_wire::Header & header, const std::vector<uint8_t> & jpeg);

enum class Verdict
{
  kAccept,
  kTooOld,
  kFromFuture
};

// Decides whether a frame may be republished. A frame is rejected when its capture stamp is older than max_age_ms
// against the Pi's clock, or ahead of it by more than max_future_ms (chrony keeps both clocks in step). The Jetson
// is absent when nothing arrived (accepted or not) for absent_after_ms, or ever. Rejected frames are only counted.
// Thread safe: the subscriber thread calls accept, the timer thread calls absent.
class StalenessPolicy
{
public:
  explicit StalenessPolicy(
    int64_t max_age_ms = kDefaultMaxAgeMs, int64_t absent_after_ms = kDefaultAbsentAfterMs,
    int64_t max_future_ms = kDefaultMaxFutureMs);

  // now_ns is the Pi's CLOCK_REALTIME in nanoseconds. Records the arrival and counts the outcome.
  Verdict accept(const ros2_common_cpp::depth_wire::Header & header, int64_t now_ns);
  bool absent(int64_t now_ns) const;

  uint64_t accepted() const;
  uint64_t rejected_too_old() const;
  uint64_t rejected_from_future() const;

private:
  const int64_t max_age_ns_;
  const int64_t absent_after_ns_;
  const int64_t max_future_ns_;
  mutable std::mutex mutex_;
  std::optional<int64_t> last_arrival_ns_;
  uint64_t accepted_ = 0;
  uint64_t rejected_too_old_ = 0;
  uint64_t rejected_from_future_ = 0;
};

}  // namespace jetson_bridge_cpp

#endif  // JETSON_BRIDGE_CPP__CONVERT_HPP_
