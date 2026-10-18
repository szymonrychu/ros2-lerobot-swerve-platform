#include <gtest/gtest.h>

#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

#include "head_camera_cpp/crop.hpp"
#include "head_camera_cpp/encode.hpp"

namespace
{

using head_camera_cpp::crop_yuv420_jpeg;
using head_camera_cpp::encode_bgr_jpeg;
using head_camera_cpp::Rect;
using head_camera_cpp::Size;
using head_camera_cpp::Yuv420View;

constexpr int WIDTH = 64;
constexpr int HEIGHT = 48;
constexpr int QUALITY = 80;
constexpr int PAD = 16;

/// Deterministic textured BGR frame.
cv::Mat textured_frame(int width, int height)
{
  cv::Mat frame(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      frame.at<cv::Vec3b>(y, x) = cv::Vec3b(
        static_cast<uint8_t>((x * 37 + y * 11) % 256), static_cast<uint8_t>((x * y) % 256),
        static_cast<uint8_t>((x * 7 + y * 53) % 256));
    }
  }
  return frame;
}

/// Planar I420 frame with four flat quadrants: top-left red, top-right green, bottom-left blue, bottom-right white.
struct Yuv420Frame
{
  int width;
  int height;
  std::vector<uint8_t> y;
  std::vector<uint8_t> u;
  std::vector<uint8_t> v;

  Yuv420View view() const { return Yuv420View{y.data(), u.data(), v.data(), width, width / 2, Size{width, height}}; }
};

Yuv420Frame quadrant_frame(int width, int height)
{
  const cv::Scalar colours[2][2] = {{{0, 0, 255}, {0, 255, 0}}, {{255, 0, 0}, {255, 255, 255}}};  // BGR
  cv::Mat bgr(height, width, CV_8UC3);
  for (int qy = 0; qy < 2; ++qy) {
    for (int qx = 0; qx < 2; ++qx) {
      bgr(cv::Rect(qx * width / 2, qy * height / 2, width / 2, height / 2)).setTo(colours[qy][qx]);
    }
  }
  cv::Mat i420;
  cv::cvtColor(bgr, i420, cv::COLOR_BGR2YUV_I420);
  const size_t y_size = static_cast<size_t>(width) * height;
  const size_t c_size = y_size / 4;
  Yuv420Frame out{width, height, {}, {}, {}};
  out.y.assign(i420.data, i420.data + y_size);
  out.u.assign(i420.data + y_size, i420.data + y_size + c_size);
  out.v.assign(i420.data + y_size + c_size, i420.data + y_size + 2 * c_size);
  return out;
}

cv::Scalar mean_bgr(const std::vector<uint8_t> & jpeg)
{
  const cv::Mat image = cv::imdecode(jpeg, cv::IMREAD_COLOR);
  return cv::mean(image);
}

}  // namespace

TEST(EncodeBgrJpeg, MatchesImencodeAtSameQuality)
{
  const cv::Mat frame = textured_frame(WIDTH, HEIGHT);
  std::vector<uint8_t> expected;
  cv::imencode(".jpg", frame, expected, {cv::IMWRITE_JPEG_QUALITY, QUALITY});
  EXPECT_EQ(encode_bgr_jpeg(frame.data, WIDTH, HEIGHT, WIDTH * 3, QUALITY), expected);
}

TEST(EncodeBgrJpeg, LowerQualityIsSmaller)
{
  const cv::Mat frame = textured_frame(WIDTH, HEIGHT);
  const auto low = encode_bgr_jpeg(frame.data, WIDTH, HEIGHT, WIDTH * 3, 30);
  const auto high = encode_bgr_jpeg(frame.data, WIDTH, HEIGHT, WIDTH * 3, 90);
  EXPECT_LT(low.size(), high.size());
}

TEST(EncodeBgrJpeg, PaddedStrideGivesSameImage)
{
  const cv::Mat frame = textured_frame(WIDTH, HEIGHT);
  const int stride = WIDTH * 3 + PAD;
  std::vector<uint8_t> padded(static_cast<size_t>(stride) * HEIGHT, 0xAB);
  for (int y = 0; y < HEIGHT; ++y) {
    std::copy(frame.ptr(y), frame.ptr(y) + WIDTH * 3, padded.begin() + static_cast<long>(y) * stride);
  }
  EXPECT_EQ(
    encode_bgr_jpeg(padded.data(), WIDTH, HEIGHT, stride, QUALITY),
    encode_bgr_jpeg(frame.data, WIDTH, HEIGHT, WIDTH * 3, QUALITY));
}

TEST(EncodeBgrJpeg, InvalidInputThrows)
{
  const cv::Mat frame = textured_frame(WIDTH, HEIGHT);
  const uint8_t * d = frame.data;
  EXPECT_THROW(encode_bgr_jpeg(nullptr, WIDTH, HEIGHT, WIDTH * 3, QUALITY), std::invalid_argument);
  EXPECT_THROW(encode_bgr_jpeg(d, 0, HEIGHT, WIDTH * 3, QUALITY), std::invalid_argument);
  EXPECT_THROW(encode_bgr_jpeg(d, WIDTH, -1, WIDTH * 3, QUALITY), std::invalid_argument);
  EXPECT_THROW(encode_bgr_jpeg(d, WIDTH, HEIGHT, WIDTH * 3 - 1, QUALITY), std::invalid_argument);
  EXPECT_THROW(encode_bgr_jpeg(d, WIDTH, HEIGHT, WIDTH * 3, 0), std::invalid_argument);
  EXPECT_THROW(encode_bgr_jpeg(d, WIDTH, HEIGHT, WIDTH * 3, 101), std::invalid_argument);
}

TEST(CropYuv420Jpeg, CropAroundQuadrantHasExpectedSizeAndColour)
{
  const Yuv420Frame frame = quadrant_frame(WIDTH, HEIGHT);
  // Inside the top-right (green) quadrant.
  const auto jpeg = crop_yuv420_jpeg(frame.view(), Rect{36, 4, 20, 16}, Size{20, 16}, QUALITY);
  const cv::Mat image = cv::imdecode(jpeg, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.cols, 20);
  EXPECT_EQ(image.rows, 16);
  const cv::Scalar m = mean_bgr(jpeg);
  EXPECT_GT(m[1], 200);
  EXPECT_LT(m[0], 50);
  EXPECT_LT(m[2], 50);
}

TEST(CropYuv420Jpeg, DownscaledCropHasRequestedSize)
{
  const Yuv420Frame frame = quadrant_frame(WIDTH, HEIGHT);
  const auto jpeg = crop_yuv420_jpeg(frame.view(), Rect{0, 24, 32, 24}, Size{16, 12}, QUALITY);
  const cv::Mat image = cv::imdecode(jpeg, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.cols, 16);
  EXPECT_EQ(image.rows, 12);
  const cv::Scalar m = mean_bgr(jpeg);
  EXPECT_GT(m[0], 200);  // blue quadrant
  EXPECT_LT(m[1], 50);
}

TEST(CropYuv420Jpeg, InvalidInputThrows)
{
  const Yuv420Frame frame = quadrant_frame(WIDTH, HEIGHT);
  const Yuv420View v = frame.view();
  const Rect ok{8, 8, 16, 16};
  const Size out{16, 16};
  EXPECT_NO_THROW(crop_yuv420_jpeg(v, ok, out, QUALITY));
  EXPECT_THROW(crop_yuv420_jpeg(v, Rect{56, 8, 16, 16}, out, QUALITY), std::invalid_argument);  // right outside
  EXPECT_THROW(crop_yuv420_jpeg(v, Rect{8, 40, 16, 16}, out, QUALITY), std::invalid_argument);  // bottom outside
  EXPECT_THROW(crop_yuv420_jpeg(v, Rect{-2, 8, 16, 16}, out, QUALITY), std::invalid_argument);
  EXPECT_THROW(crop_yuv420_jpeg(v, Rect{8, 8, 0, 16}, out, QUALITY), std::invalid_argument);
  EXPECT_THROW(crop_yuv420_jpeg(v, Rect{9, 8, 16, 16}, out, QUALITY), std::invalid_argument);  // odd x
  EXPECT_THROW(crop_yuv420_jpeg(v, Rect{8, 9, 16, 16}, out, QUALITY), std::invalid_argument);  // odd y
  EXPECT_THROW(crop_yuv420_jpeg(v, Rect{8, 8, 15, 16}, out, QUALITY), std::invalid_argument);  // odd width
  EXPECT_THROW(crop_yuv420_jpeg(v, ok, Size{18, 16}, QUALITY), std::invalid_argument);         // upscale
  EXPECT_THROW(crop_yuv420_jpeg(v, ok, Size{16, 0}, QUALITY), std::invalid_argument);
  EXPECT_THROW(crop_yuv420_jpeg(v, ok, out, 0), std::invalid_argument);
  EXPECT_THROW(crop_yuv420_jpeg(v, ok, out, 101), std::invalid_argument);
  Yuv420View null_plane = v;
  null_plane.u = nullptr;
  EXPECT_THROW(crop_yuv420_jpeg(null_plane, ok, out, QUALITY), std::invalid_argument);
}
