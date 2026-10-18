#include <gtest/gtest.h>

#include <cstdint>
#include <opencv2/core.hpp>

#include "head_camera_cpp/mapping_stream.hpp"

namespace
{

using head_camera_cpp::CameraIntrinsics;
using head_camera_cpp::MappingDecimator;

constexpr int64_t NS_PER_S = 1000000000;
constexpr int64_t FRAME_NS_30FPS = NS_PER_S / 30;

CameraIntrinsics stream_intrinsics()
{
  CameraIntrinsics c;
  c.width = 1152;
  c.height = 648;
  c.k = {800.0, 0.0, 576.0, 0.0, 810.0, 324.0, 0.0, 0.0, 1.0};
  c.p = {800.0, 0.0, 576.0, 0.0, 0.0, 810.0, 324.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  c.d = {0.1, -0.2, 0.001, 0.002, 0.0};
  return c;
}

}  // namespace

TEST(MappingDecimator, AcceptsTwoFramesPerSecondFrom30FpsInput)
{
  MappingDecimator decimator(2.0);
  int accepted = 0;
  const int64_t base = 5 * NS_PER_S;
  for (int64_t i = 0; i < 300; ++i) {
    if (decimator.due(base + i * FRAME_NS_30FPS)) {
      ++accepted;
    }
  }
  EXPECT_NEAR(accepted, 20, 1);
}

TEST(MappingDecimator, FirstFrameIsDue)
{
  MappingDecimator decimator(2.0);
  EXPECT_TRUE(decimator.due(123));
  EXPECT_FALSE(decimator.due(123 + NS_PER_S / 4));
  EXPECT_TRUE(decimator.due(123 + NS_PER_S / 2));
}

TEST(MappingDecimator, ResetsOnABackwardsStamp)
{
  MappingDecimator decimator(2.0);
  EXPECT_TRUE(decimator.due(100 * NS_PER_S));
  EXPECT_FALSE(decimator.due(100 * NS_PER_S + 1000));
  EXPECT_TRUE(decimator.due(10 * NS_PER_S));
  EXPECT_FALSE(decimator.due(10 * NS_PER_S + 1000));
  EXPECT_TRUE(decimator.due(10 * NS_PER_S + NS_PER_S / 2));
}

TEST(MappingStream, ResizeProducesTheRequestedSize)
{
  const cv::Mat bgr(648, 1152, CV_8UC3, cv::Scalar(10, 20, 30));
  const cv::Mat out = head_camera_cpp::resize_for_mapping(bgr, 576, 324);
  EXPECT_EQ(out.cols, 576);
  EXPECT_EQ(out.rows, 324);
  EXPECT_EQ(out.type(), CV_8UC3);
  EXPECT_EQ(out.at<cv::Vec3b>(100, 100), cv::Vec3b(10, 20, 30));
}

TEST(MappingStream, IntrinsicsHalveWhenTheSizeHalves)
{
  const CameraIntrinsics out = head_camera_cpp::scale_intrinsics(stream_intrinsics(), 576, 324);
  EXPECT_EQ(out.width, 576);
  EXPECT_EQ(out.height, 324);
  EXPECT_DOUBLE_EQ(out.k[0], 400.0);
  EXPECT_DOUBLE_EQ(out.k[2], 288.0);
  EXPECT_DOUBLE_EQ(out.k[4], 405.0);
  EXPECT_DOUBLE_EQ(out.k[5], 162.0);
  EXPECT_DOUBLE_EQ(out.k[8], 1.0);
  EXPECT_DOUBLE_EQ(out.p[0], 400.0);
  EXPECT_DOUBLE_EQ(out.p[2], 288.0);
  EXPECT_DOUBLE_EQ(out.p[5], 405.0);
  EXPECT_DOUBLE_EQ(out.p[6], 162.0);
  EXPECT_EQ(out.d, stream_intrinsics().d);
}

TEST(MappingStream, IntrinsicsAspectMismatchThrows)
{
  EXPECT_THROW(head_camera_cpp::scale_intrinsics(stream_intrinsics(), 576, 400), std::invalid_argument);
  EXPECT_THROW(head_camera_cpp::scale_intrinsics(stream_intrinsics(), 0, 0), std::invalid_argument);
}
