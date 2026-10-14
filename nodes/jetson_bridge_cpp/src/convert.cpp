#include "jetson_bridge_cpp/convert.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace jetson_bridge_cpp
{
namespace wire = ros2_common_cpp::depth_wire;

namespace
{
constexpr int64_t kNsPerS = 1000000000LL;
constexpr int64_t kNsPerMs = 1000000LL;
constexpr size_t kBytesPerPixel = 2;

std::string frame_or(const wire::Header & header, const char * fallback)
{
  return header.frame_id.empty() ? std::string(fallback) : header.frame_id;
}

std_msgs::msg::Header ros_header(const wire::Header & header, const char * fallback_frame)
{
  std_msgs::msg::Header out;
  out.stamp = stamp_from_realtime_ns(header.capture_realtime_ns);
  out.frame_id = frame_or(header, fallback_frame);
  return out;
}
}  // namespace

builtin_interfaces::msg::Time stamp_from_realtime_ns(int64_t realtime_ns)
{
  int64_t sec = realtime_ns / kNsPerS;
  int64_t nsec = realtime_ns % kNsPerS;
  if (nsec < 0) {
    nsec += kNsPerS;
    --sec;
  }
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(sec);
  t.nanosec = static_cast<uint32_t>(nsec);
  return t;
}

sensor_msgs::msg::Image to_depth_image(const wire::Header & header, const std::vector<uint16_t> & depth_mm)
{
  if (header.kind != wire::Kind::kDepth && header.kind != wire::Kind::kFused) {
    throw std::invalid_argument("to_depth_image: header kind is not depth or fused");
  }
  if (depth_mm.size() != static_cast<size_t>(header.width) * header.height) {
    throw std::invalid_argument("to_depth_image: pixel count does not match width*height");
  }
  sensor_msgs::msg::Image img;
  img.header = ros_header(header, kStereoLeftFrame);
  img.width = header.width;
  img.height = header.height;
  img.encoding = kDepthEncoding;
  img.is_bigendian = 0;
  img.step = header.width * kBytesPerPixel;
  img.data.resize(depth_mm.size() * kBytesPerPixel);
  for (size_t i = 0; i < depth_mm.size(); ++i) {
    img.data[2 * i] = static_cast<uint8_t>(depth_mm[i] & 0xFFU);
    img.data[2 * i + 1] = static_cast<uint8_t>(depth_mm[i] >> 8U);
  }
  return img;
}

std::optional<sensor_msgs::msg::CameraInfo> to_camera_info(const wire::Header & header, CameraModel model)
{
  if (!header.calibrated) {
    return std::nullopt;
  }
  const bool rectified = model == CameraModel::kRectified;
  sensor_msgs::msg::CameraInfo info;
  info.header = ros_header(header, rectified ? kStereoLeftFrame : kOverviewFrame);
  info.width = header.width;
  info.height = header.height;
  info.distortion_model = "plumb_bob";
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  if (rectified) {
    const auto & p = header.p;
    info.d.assign(5, 0.0);
    info.k = {p[0], p[1], p[2], p[4], p[5], p[6], 0.0, 0.0, 1.0};
    std::copy(p.begin(), p.end(), info.p.begin());
  } else {
    const auto & k = header.k;
    info.d.assign(header.d.begin(), header.d.end());
    std::copy(k.begin(), k.end(), info.k.begin());
    info.p = {k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0};
  }
  return info;
}

sensor_msgs::msg::CompressedImage to_compressed(const wire::Header & header, const std::vector<uint8_t> & jpeg)
{
  if (header.kind != wire::Kind::kRgb) {
    throw std::invalid_argument("to_compressed: header kind is not rgb");
  }
  if (jpeg.empty()) {
    throw std::invalid_argument("to_compressed: empty JPEG");
  }
  sensor_msgs::msg::CompressedImage msg;
  msg.header = ros_header(header, kOverviewFrame);
  msg.format = kJpegFormat;
  msg.data = jpeg;
  return msg;
}

StalenessPolicy::StalenessPolicy(int64_t max_age_ms, int64_t absent_after_ms, int64_t max_future_ms)
: max_age_ns_(max_age_ms * kNsPerMs),
  absent_after_ns_(absent_after_ms * kNsPerMs),
  max_future_ns_(max_future_ms * kNsPerMs)
{
}

Verdict StalenessPolicy::accept(const wire::Header & header, int64_t now_ns)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  last_arrival_ns_ = now_ns;
  const int64_t age_ns = now_ns - header.capture_realtime_ns;
  if (age_ns > max_age_ns_) {
    ++rejected_too_old_;
    return Verdict::kTooOld;
  }
  if (-age_ns > max_future_ns_) {
    ++rejected_from_future_;
    return Verdict::kFromFuture;
  }
  ++accepted_;
  return Verdict::kAccept;
}

bool StalenessPolicy::absent(int64_t now_ns) const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return !last_arrival_ns_.has_value() || now_ns - *last_arrival_ns_ > absent_after_ns_;
}

uint64_t StalenessPolicy::accepted() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return accepted_;
}

uint64_t StalenessPolicy::rejected_too_old() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return rejected_too_old_;
}

uint64_t StalenessPolicy::rejected_from_future() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return rejected_from_future_;
}

}  // namespace jetson_bridge_cpp
