#include "head_rgbd_cpp/cloud.hpp"

#include <cmath>
#include <cstring>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "depth_image_proc/conversions.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace head_rgbd_cpp
{

namespace
{
constexpr int RGB_RED_OFFSET = 2;  // BGR8
constexpr int RGB_GREEN_OFFSET = 1;
constexpr int RGB_BLUE_OFFSET = 0;
constexpr int RGB_COLOR_STEP = 3;
constexpr int REDUCED_DIVISOR = 2;
}  // namespace

RateDecimator::RateDecimator(double rate_hz) : period_ns_(static_cast<int64_t>(std::llround(NS_PER_S / rate_hz))) {}

bool RateDecimator::due(int64_t stamp_ns)
{
  if (last_ns_ && stamp_ns >= *last_ns_ && stamp_ns - *last_ns_ < period_ns_) {
    return false;
  }
  last_ns_ = stamp_ns;
  return true;
}

cv::Mat decode_rgb_for_depth(const std::vector<uint8_t> & jpeg, int width, int height)
{
  if (jpeg.empty()) {
    return cv::Mat();
  }
  const cv::Mat encoded(1, static_cast<int>(jpeg.size()), CV_8UC1, const_cast<uint8_t *>(jpeg.data()));
  cv::Mat image;
  try {
    image = cv::imdecode(encoded, cv::IMREAD_REDUCED_COLOR_2);
  } catch (const cv::Exception &) {
    return cv::Mat();
  }
  if (image.empty()) {
    return cv::Mat();
  }
  if (image.cols == width && image.rows == height) {
    return image;
  }
  // The target is not half the JPEG size: decode at full size and resample.
  try {
    image = cv::imdecode(encoded, cv::IMREAD_COLOR);
  } catch (const cv::Exception &) {
    return cv::Mat();
  }
  if (image.empty() || (image.cols == width && image.rows == height)) {
    return image;
  }
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(width, height), 0, 0, cv::INTER_AREA);
  return resized;
}

sensor_msgs::msg::PointCloud2::SharedPtr build_cloud(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & depth_info, const cv::Mat & rgb)
{
  if (rgb.empty() || rgb.type() != CV_8UC3 || depth.encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
    return nullptr;
  }
  if (
    rgb.cols != static_cast<int>(depth.width) || rgb.rows != static_cast<int>(depth.height) || depth.width == 0 ||
    depth.height == 0 || depth.step < depth.width * sizeof(uint16_t) || depth.data.size() < depth.step * depth.height) {
    return nullptr;
  }
  image_geometry::PinholeCameraModel model;
  if (!model.fromCameraInfo(depth_info)) {
    return nullptr;
  }

  const auto depth_ptr = std::make_shared<const sensor_msgs::msg::Image>(depth);
  auto rgb_msg = std::make_shared<sensor_msgs::msg::Image>();
  rgb_msg->header = depth.header;
  rgb_msg->width = depth.width;
  rgb_msg->height = depth.height;
  rgb_msg->encoding = sensor_msgs::image_encodings::BGR8;
  rgb_msg->step = depth.width * RGB_COLOR_STEP;
  rgb_msg->data.resize(static_cast<size_t>(rgb_msg->step) * depth.height);
  for (int v = 0; v < rgb.rows; ++v) {
    std::memcpy(&rgb_msg->data[static_cast<size_t>(v) * rgb_msg->step], rgb.ptr(v), rgb_msg->step);
  }

  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header = depth.header;
  cloud->height = depth.height;
  cloud->width = depth.width;
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(static_cast<size_t>(depth.width) * depth.height);

  depth_image_proc::convertDepth<uint16_t>(depth_ptr, cloud, model);
  depth_image_proc::convertRgb(rgb_msg, cloud, RGB_RED_OFFSET, RGB_GREEN_OFFSET, RGB_BLUE_OFFSET, RGB_COLOR_STEP);

  // convertDepth marks depth 0 as NaN; compact the valid points to the front.
  const size_t point_step = cloud->point_step;
  const size_t total = static_cast<size_t>(cloud->width) * cloud->height;
  size_t kept = 0;
  for (size_t i = 0; i < total; ++i) {
    uint8_t * point = &cloud->data[i * point_step];
    float z;
    std::memcpy(&z, point + 2 * sizeof(float), sizeof(float));
    if (!std::isfinite(z)) {
      continue;
    }
    if (kept != i) {
      std::memcpy(&cloud->data[kept * point_step], point, point_step);
    }
    ++kept;
  }
  cloud->data.resize(kept * point_step);
  cloud->height = 1;
  cloud->width = static_cast<uint32_t>(kept);
  cloud->row_step = cloud->width * cloud->point_step;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace head_rgbd_cpp
