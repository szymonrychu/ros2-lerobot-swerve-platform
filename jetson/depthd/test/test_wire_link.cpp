// Proves the shared wire codec (shared/ros2_common_cpp/src/depth_wire.cpp) compiles with this toolchain (gcc 8)
// and round-trips a depth message.
#include <gtest/gtest.h>

#include <numeric>

#include "ros2_common_cpp/depth_wire.hpp"

namespace dw = ros2_common_cpp::depth_wire;

TEST(WireLink, DepthMessageRoundTrips)
{
  dw::Header h;
  h.kind = dw::Kind::kDepth;
  h.sequence = 7;
  h.capture_realtime_ns = 1700000000123456789LL;
  h.width = 8;
  h.height = 4;
  h.frame_id = "stereo_left_optical_frame";
  h.baseline_m = 0.06;
  h.calibrated = true;
  std::vector<uint16_t> depth(32);
  std::iota(depth.begin(), depth.end(), static_cast<uint16_t>(200));

  const dw::Frames frames = dw::encode_depth(h, depth, 1);
  EXPECT_EQ(frames.topic, "depth");
  EXPECT_EQ(frames.header.size(), dw::kHeaderSize);

  const dw::Decoded out = dw::decode(frames);
  ASSERT_TRUE(out.ok()) << out.error;
  EXPECT_EQ(out.header.sequence, 7U);
  EXPECT_EQ(out.header.frame_id, "stereo_left_optical_frame");
  EXPECT_EQ(out.header.width, 8U);
  EXPECT_TRUE(out.header.calibrated);
  EXPECT_EQ(out.depth_mm, depth);
}
