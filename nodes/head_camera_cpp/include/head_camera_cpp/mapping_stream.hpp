// The core of the 2 Hz mapping stream: decimation of the 30 fps frames, the downscale and the matching intrinsics.
// No ROS, no libcamera.
#ifndef HEAD_CAMERA_CPP__MAPPING_STREAM_HPP_
#define HEAD_CAMERA_CPP__MAPPING_STREAM_HPP_

#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace head_camera_cpp
{

// A pinhole calibration at one image size (the fields of a sensor_msgs/CameraInfo without the ROS types).
struct CameraIntrinsics
{
  int width = 0;
  int height = 0;
  // distortion coefficients
  std::vector<double> d;
  // camera matrix, row-major 3x3
  std::array<double, 9> k{0, 0, 0, 0, 0, 0, 0, 0, 0};
  // rectification matrix
  std::array<double, 9> r{1, 0, 0, 0, 1, 0, 0, 0, 1};
  // projection matrix, row-major 3x4
  std::array<double, 12> p{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

// Accepts frames at no more than rate_hz.
class MappingDecimator
{
public:
  // @param rate_hz output rate in Hz, > 0.
  explicit MappingDecimator(double rate_hz);

  // Whether a frame is accepted. The first frame, and every frame at least 1e9 / rate_hz ns after the last accepted
  // one; a stamp before the last accepted one resets the decimator and is accepted.
  // @param stamp_ns frame timestamp in ns.
  // @return true when the frame is due.
  bool due(int64_t stamp_ns);

private:
  int64_t period_ns_;
  std::optional<int64_t> last_ns_;
};

// Downscale a BGR frame with INTER_AREA.
// @param bgr source image.
// @param width target width in px.
// @param height target height in px.
// @return the resized image.
cv::Mat resize_for_mapping(const cv::Mat & bgr, int width, int height);

// The intrinsics of a downscaled stream: K and P scaled by width / calibrated.width (x row) and
// height / calibrated.height (y row), distortion and rectification unchanged.
// @param calibrated the loaded calibration.
// @param width target width in px.
// @param height target height in px.
// @return the scaled intrinsics.
// @throws std::invalid_argument when a size is not positive or the aspect differs by more than 1 px.
CameraIntrinsics scale_intrinsics(const CameraIntrinsics & calibrated, int width, int height);

}  // namespace head_camera_cpp

#endif  // HEAD_CAMERA_CPP__MAPPING_STREAM_HPP_
