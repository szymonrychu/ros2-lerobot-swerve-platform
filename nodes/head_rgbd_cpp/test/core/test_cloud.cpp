#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "head_rgbd_cpp/cloud.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

constexpr int WIDTH = 288;
constexpr int HEIGHT = 162;
constexpr double FX = 200.0;
constexpr double FY = 205.0;
constexpr double CX = 143.5;
constexpr double CY = 80.5;
constexpr uint16_t PLANE_MM = 1000;
constexpr double TOLERANCE_M = 1e-3;
constexpr int64_t NS_PER_S = 1000000000;

sensor_msgs::msg::CameraInfo make_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = WIDTH;
  info.height = HEIGHT;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

sensor_msgs::msg::Image make_plane_depth()
{
  sensor_msgs::msg::Image depth;
  depth.header.frame_id = "head_camera_optical_frame";
  depth.header.stamp.sec = 7;
  depth.width = WIDTH;
  depth.height = HEIGHT;
  depth.encoding = "16UC1";
  depth.step = WIDTH * sizeof(uint16_t);
  depth.data.resize(static_cast<size_t>(depth.step) * HEIGHT);
  auto * values = reinterpret_cast<uint16_t *>(depth.data.data());
  for (int i = 0; i < WIDTH * HEIGHT; ++i) {
    values[i] = PLANE_MM;
  }
  return depth;
}

// Colour of pixel (u, v), unique enough to catch a mix-up of rows, columns and channels (BGR order).
cv::Vec3b colour_at(int u, int v) { return cv::Vec3b(u % 256, v % 256, (u + v) % 256); }

cv::Mat make_rgb()
{
  cv::Mat rgb(HEIGHT, WIDTH, CV_8UC3);
  for (int v = 0; v < HEIGHT; ++v) {
    for (int u = 0; u < WIDTH; ++u) {
      rgb.at<cv::Vec3b>(v, u) = colour_at(u, v);
    }
  }
  return rgb;
}

std::vector<uint8_t> encode_jpeg(int width, int height)
{
  const cv::Mat image(height, width, CV_8UC3, cv::Scalar(30, 120, 200));
  std::vector<uint8_t> jpeg;
  cv::imencode(".jpg", image, jpeg);
  return jpeg;
}

}  // namespace

TEST(Cloud, PlaneAtOneMetreBackProjectsWithColours)
{
  const auto cloud = head_rgbd_cpp::build_cloud(make_plane_depth(), make_info(), make_rgb());
  ASSERT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->header.frame_id, "head_camera_optical_frame");
  EXPECT_EQ(cloud->header.stamp.sec, 7);
  EXPECT_EQ(cloud->height, 1U);
  EXPECT_EQ(cloud->width, static_cast<uint32_t>(WIDTH * HEIGHT));
  EXPECT_TRUE(cloud->is_dense);

  sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> r(*cloud, "r");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> g(*cloud, "g");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> b(*cloud, "b");
  for (int v = 0; v < HEIGHT; ++v) {
    for (int u = 0; u < WIDTH; ++u, ++x, ++y, ++z, ++r, ++g, ++b) {
      ASSERT_NEAR(*z, 1.0, TOLERANCE_M);
      ASSERT_NEAR(*x, (u - CX) / FX, TOLERANCE_M);
      ASSERT_NEAR(*y, (v - CY) / FY, TOLERANCE_M);
      const cv::Vec3b expected = colour_at(u, v);
      ASSERT_EQ(*b, expected[0]);
      ASSERT_EQ(*g, expected[1]);
      ASSERT_EQ(*r, expected[2]);
    }
  }
}

TEST(Cloud, ZeroDepthPixelsProduceNoPoints)
{
  auto depth = make_plane_depth();
  auto * values = reinterpret_cast<uint16_t *>(depth.data.data());
  constexpr int ZEROED_ROWS = 10;
  for (int i = 0; i < ZEROED_ROWS * WIDTH; ++i) {
    values[i] = 0;
  }
  const auto cloud = head_rgbd_cpp::build_cloud(depth, make_info(), make_rgb());
  ASSERT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->width, static_cast<uint32_t>((HEIGHT - ZEROED_ROWS) * WIDTH));
  EXPECT_EQ(cloud->data.size(), static_cast<size_t>(cloud->width) * cloud->point_step);
  EXPECT_TRUE(cloud->is_dense);
  sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
  for (; z != z.end(); ++z) {
    ASSERT_NEAR(*z, 1.0, TOLERANCE_M);
  }
}

TEST(Cloud, EmptyOrMismatchedRgbGivesNoCloud)
{
  EXPECT_EQ(head_rgbd_cpp::build_cloud(make_plane_depth(), make_info(), cv::Mat()), nullptr);
  EXPECT_EQ(head_rgbd_cpp::build_cloud(make_plane_depth(), make_info(), cv::Mat(HEIGHT, WIDTH / 2, CV_8UC3)), nullptr);
}

TEST(Cloud, CorruptJpegGivesEmptyMatAndNoCloud)
{
  const std::vector<uint8_t> garbage(256, 0x5a);
  const cv::Mat decoded = head_rgbd_cpp::decode_rgb_for_depth(garbage, WIDTH, HEIGHT);
  EXPECT_TRUE(decoded.empty());
  EXPECT_EQ(head_rgbd_cpp::build_cloud(make_plane_depth(), make_info(), decoded), nullptr);
  EXPECT_TRUE(head_rgbd_cpp::decode_rgb_for_depth({}, WIDTH, HEIGHT).empty());
}

TEST(Cloud, ReducedDecodeOfHalfSizeTarget)
{
  const cv::Mat decoded = head_rgbd_cpp::decode_rgb_for_depth(encode_jpeg(2 * WIDTH, 2 * HEIGHT), WIDTH, HEIGHT);
  EXPECT_EQ(decoded.cols, WIDTH);
  EXPECT_EQ(decoded.rows, HEIGHT);
  EXPECT_EQ(decoded.type(), CV_8UC3);
}

TEST(Cloud, OtherTargetSizesAreResized)
{
  const cv::Mat decoded =
    head_rgbd_cpp::decode_rgb_for_depth(encode_jpeg(2 * WIDTH, 2 * HEIGHT), WIDTH / 2, HEIGHT / 2);
  EXPECT_EQ(decoded.cols, WIDTH / 2);
  EXPECT_EQ(decoded.rows, HEIGHT / 2);
  const cv::Mat same = head_rgbd_cpp::decode_rgb_for_depth(encode_jpeg(WIDTH, HEIGHT), WIDTH, HEIGHT);
  EXPECT_EQ(same.cols, WIDTH);
  EXPECT_EQ(same.rows, HEIGHT);
}

TEST(RateDecimator, OneHertzAcceptsOneOfTwoPairsAtTwoHertz)
{
  head_rgbd_cpp::RateDecimator decimator(1.0);
  constexpr int SECONDS = 10;
  constexpr int PAIRS_PER_SECOND = 2;
  int accepted = 0;
  for (int i = 0; i < SECONDS * PAIRS_PER_SECOND; ++i) {
    if (decimator.due(i * (NS_PER_S / PAIRS_PER_SECOND))) {
      ++accepted;
    }
  }
  EXPECT_NEAR(accepted, SECONDS, 1);
}

TEST(RateDecimator, FirstStampAcceptedAndBackwardsStampResets)
{
  head_rgbd_cpp::RateDecimator decimator(1.0);
  EXPECT_TRUE(decimator.due(5 * NS_PER_S));
  EXPECT_FALSE(decimator.due(5 * NS_PER_S + NS_PER_S / 2));
  EXPECT_TRUE(decimator.due(6 * NS_PER_S));
  EXPECT_TRUE(decimator.due(NS_PER_S));
  EXPECT_FALSE(decimator.due(NS_PER_S + 1));
}
