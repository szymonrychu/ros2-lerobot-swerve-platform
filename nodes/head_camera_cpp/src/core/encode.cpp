#include "head_camera_cpp/encode.hpp"

#include <cstring>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace head_camera_cpp
{

namespace
{

constexpr int MIN_QUALITY = 1;
constexpr int MAX_QUALITY = 100;
constexpr int BGR_CHANNELS = 3;
constexpr int CHROMA_SUBSAMPLING = 2;
const char * const JPEG_EXTENSION = ".jpg";

void check_quality(int quality)
{
  if (quality < MIN_QUALITY || quality > MAX_QUALITY) {
    throw std::invalid_argument("JPEG quality must be in 1..100");
  }
}

std::vector<uint8_t> encode_mat(const cv::Mat & bgr, int quality)
{
  std::vector<uint8_t> bytes;
  if (!cv::imencode(JPEG_EXTENSION, bgr, bytes, {cv::IMWRITE_JPEG_QUALITY, quality})) {
    throw std::runtime_error("JPEG encoding failed");
  }
  return bytes;
}

}  // namespace

std::vector<uint8_t> encode_bgr_jpeg(const uint8_t * data, int width, int height, int stride, int quality)
{
  if (data == nullptr) {
    throw std::invalid_argument("frame data is null");
  }
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("frame size must be positive");
  }
  if (stride < BGR_CHANNELS * width) {
    throw std::invalid_argument("stride is smaller than 3 * width");
  }
  check_quality(quality);
  const cv::Mat frame(height, width, CV_8UC3, const_cast<uint8_t *>(data), static_cast<size_t>(stride));
  return encode_mat(frame, quality);
}

std::vector<uint8_t> crop_yuv420_jpeg(const Yuv420View & frame, Rect region, Size out, int quality)
{
  if (frame.y == nullptr || frame.u == nullptr || frame.v == nullptr) {
    throw std::invalid_argument("frame plane is null");
  }
  if (
    frame.size.width <= 0 || frame.size.height <= 0 || frame.y_stride < frame.size.width ||
    frame.uv_stride < frame.size.width / CHROMA_SUBSAMPLING) {
    throw std::invalid_argument("invalid frame geometry");
  }
  if (
    region.width <= 0 || region.height <= 0 || region.x < 0 || region.y < 0 ||
    region.x + region.width > frame.size.width || region.y + region.height > frame.size.height) {
    throw std::invalid_argument("region is outside the frame");
  }
  if (
    region.x % CHROMA_SUBSAMPLING != 0 || region.y % CHROMA_SUBSAMPLING != 0 ||
    region.width % CHROMA_SUBSAMPLING != 0 || region.height % CHROMA_SUBSAMPLING != 0) {
    throw std::invalid_argument("region values must be even");
  }
  if (out.width <= 0 || out.height <= 0 || out.width > region.width || out.height > region.height) {
    throw std::invalid_argument("output size must be positive and not larger than the region");
  }
  check_quality(quality);

  // Sub-plane copy into one contiguous I420 buffer: Y rows, then the U plane, then the V plane.
  const int cw = region.width / CHROMA_SUBSAMPLING;
  const int ch = region.height / CHROMA_SUBSAMPLING;
  const int cx = region.x / CHROMA_SUBSAMPLING;
  const int cy = region.y / CHROMA_SUBSAMPLING;
  cv::Mat i420(region.height + ch, region.width, CV_8UC1);
  uint8_t * dst = i420.data;
  for (int row = 0; row < region.height; ++row) {
    std::memcpy(dst, frame.y + static_cast<size_t>(region.y + row) * frame.y_stride + region.x, region.width);
    dst += region.width;
  }
  for (const uint8_t * plane : {frame.u, frame.v}) {
    for (int row = 0; row < ch; ++row) {
      std::memcpy(dst, plane + static_cast<size_t>(cy + row) * frame.uv_stride + cx, cw);
      dst += cw;
    }
  }

  cv::Mat bgr;
  cv::cvtColor(i420, bgr, cv::COLOR_YUV2BGR_I420);
  if (out.width != region.width || out.height != region.height) {
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(out.width, out.height), 0, 0, cv::INTER_AREA);
    bgr = resized;
  }
  return encode_mat(bgr, quality);
}

}  // namespace head_camera_cpp
