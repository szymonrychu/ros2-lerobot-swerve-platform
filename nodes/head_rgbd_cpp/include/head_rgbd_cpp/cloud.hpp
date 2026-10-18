// Coloured point cloud of the registered RGB-D pair (design D9): rate decimation of the pairs, the RGB decode at the
// depth size and the XYZRGB cloud built with depth_image_proc's conversions and image_geometry.
#ifndef HEAD_RGBD_CPP__CLOUD_HPP_
#define HEAD_RGBD_CPP__CLOUD_HPP_

#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace head_rgbd_cpp
{

constexpr double NS_PER_S = 1e9;

// Accepts a stamp when none was accepted yet, or at least 1e9 / rate_hz ns passed since the last accepted one. A stamp
// before the last accepted one (clock jump, bag replay loop) resets the decimator and is accepted.
class RateDecimator
{
public:
  // rate_hz: accepted stamps per second, > 0.
  explicit RateDecimator(double rate_hz);

  // stamp_ns: stamp of the pair in nanoseconds. Returns true when this pair is due (and remembers it).
  bool due(int64_t stamp_ns);

private:
  int64_t period_ns_;
  std::optional<int64_t> last_ns_;
};

// Decodes a JPEG to BGR at width x height: IMREAD_REDUCED_COLOR_2 when that gives the target size, else a full decode
// resized with INTER_AREA. Returns an empty Mat when the JPEG is corrupt.
cv::Mat decode_rgb_for_depth(const std::vector<uint8_t> & jpeg, int width, int height);

// Unorganised XYZRGB cloud (x, y, z float metres, rgb packed) in the depth frame and stamp, from a 16UC1 depth in mm,
// its camera_info and a BGR8 image of the same size. Pixels with depth 0 are dropped, so is_dense is true. Returns
// nullptr when rgb is empty or not CV_8UC3, its size differs from the depth, or depth is not 16UC1 / inconsistent:
// colour is never made up.
sensor_msgs::msg::PointCloud2::SharedPtr build_cloud(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & depth_info, const cv::Mat & rgb);

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__CLOUD_HPP_
