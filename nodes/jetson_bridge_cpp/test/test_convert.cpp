// Conversion of depth wire frames to the ROS contracts, and the staleness policy.
#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "jetson_bridge_cpp/convert.hpp"

namespace wire = ros2_common_cpp::depth_wire;
using jetson_bridge_cpp::StalenessPolicy;
using jetson_bridge_cpp::Verdict;

namespace
{
constexpr int64_t kMs = 1000000;
constexpr int64_t kNow = 1760000000LL * 1000000000LL + 123456789LL;  // the Pi's CLOCK_REALTIME

wire::Header calibrated_header(wire::Kind kind)
{
  wire::Header h;
  h.kind = kind;
  h.sequence = 42;
  h.capture_realtime_ns = kNow - 100 * kMs;
  h.width = 4;
  h.height = 2;
  h.frame_id = kind == wire::Kind::kRgb ? "overview_camera_optical_frame" : "stereo_left_optical_frame";
  h.k = {500.0, 0.0, 320.5, 0.0, 510.0, 240.5, 0.0, 0.0, 1.0};
  h.d = {-0.1, 0.01, 0.001, -0.002, 0.0003};
  h.p = {400.0, 0.0, 300.5, 0.0, 0.0, 410.0, 200.5, 0.0, 0.0, 0.0, 1.0, 0.0};
  h.baseline_m = 0.06;
  h.calibrated = true;
  return h;
}
}  // namespace

TEST(StampFromRealtimeNs, SplitsSecondsAndNanoseconds)
{
  const auto t = jetson_bridge_cpp::stamp_from_realtime_ns(1760000000LL * 1000000000LL + 123456789LL);
  EXPECT_EQ(t.sec, 1760000000);
  EXPECT_EQ(t.nanosec, 123456789U);
}

TEST(StampFromRealtimeNs, ExactSecondAndZero)
{
  const auto t = jetson_bridge_cpp::stamp_from_realtime_ns(5LL * 1000000000LL);
  EXPECT_EQ(t.sec, 5);
  EXPECT_EQ(t.nanosec, 0U);
  const auto z = jetson_bridge_cpp::stamp_from_realtime_ns(0);
  EXPECT_EQ(z.sec, 0);
  EXPECT_EQ(z.nanosec, 0U);
}

TEST(StampFromRealtimeNs, NegativeUsesFloorSoNanosecIsNeverNegative)
{
  const auto t = jetson_bridge_cpp::stamp_from_realtime_ns(-1);
  EXPECT_EQ(t.sec, -1);
  EXPECT_EQ(t.nanosec, 999999999U);
}

TEST(ToDepthImage, FieldByField)
{
  const auto h = calibrated_header(wire::Kind::kDepth);
  const std::vector<uint16_t> mm = {0, 1, 255, 256, 1000, 4000, 65535, 0x1234};
  const auto img = jetson_bridge_cpp::to_depth_image(h, mm);
  EXPECT_EQ(img.header.frame_id, "stereo_left_optical_frame");
  EXPECT_EQ(img.header.stamp.sec, 1760000000);
  EXPECT_EQ(img.header.stamp.nanosec, 23456789U);
  EXPECT_EQ(img.width, 4U);
  EXPECT_EQ(img.height, 2U);
  EXPECT_EQ(img.encoding, "16UC1");
  EXPECT_EQ(img.is_bigendian, 0);
  EXPECT_EQ(img.step, 8U);
  ASSERT_EQ(img.data.size(), 16U);
  // little-endian, 0 stays 0 (invalid)
  EXPECT_EQ(img.data[0], 0);
  EXPECT_EQ(img.data[1], 0);
  EXPECT_EQ(img.data[2], 1);
  EXPECT_EQ(img.data[3], 0);
  EXPECT_EQ(img.data[6], 0);
  EXPECT_EQ(img.data[7], 1);
  EXPECT_EQ(img.data[12], 0xFF);
  EXPECT_EQ(img.data[13], 0xFF);
  EXPECT_EQ(img.data[14], 0x34);
  EXPECT_EQ(img.data[15], 0x12);
}

TEST(ToDepthImage, FusedKindIsAccepted)
{
  const auto h = calibrated_header(wire::Kind::kFused);
  const auto img = jetson_bridge_cpp::to_depth_image(h, std::vector<uint16_t>(8, 7));
  EXPECT_EQ(img.encoding, "16UC1");
}

TEST(ToDepthImage, SizeMismatchThrows)
{
  const auto h = calibrated_header(wire::Kind::kDepth);
  EXPECT_THROW(jetson_bridge_cpp::to_depth_image(h, std::vector<uint16_t>(7, 1)), std::invalid_argument);
}

TEST(ToDepthImage, RgbKindThrows)
{
  const auto h = calibrated_header(wire::Kind::kRgb);
  EXPECT_THROW(jetson_bridge_cpp::to_depth_image(h, std::vector<uint16_t>(8, 1)), std::invalid_argument);
}

TEST(ToDepthImage, EmptyFrameIdFallsBackToTheContractFrame)
{
  auto h = calibrated_header(wire::Kind::kDepth);
  h.frame_id.clear();
  EXPECT_EQ(
    jetson_bridge_cpp::to_depth_image(h, std::vector<uint16_t>(8, 1)).header.frame_id, "stereo_left_optical_frame");
}

TEST(ToCameraInfo, RectifiedCarriesPAndTheDepthHeader)
{
  const auto h = calibrated_header(wire::Kind::kDepth);
  const auto info = jetson_bridge_cpp::to_camera_info(h, jetson_bridge_cpp::CameraModel::kRectified);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->header.frame_id, "stereo_left_optical_frame");
  EXPECT_EQ(info->header.stamp.sec, 1760000000);
  EXPECT_EQ(info->width, 4U);
  EXPECT_EQ(info->height, 2U);
  EXPECT_EQ(info->distortion_model, "plumb_bob");
  ASSERT_EQ(info->d.size(), 5U);
  for (double d : info->d) {
    EXPECT_EQ(d, 0.0);
  }
  const double k[9] = {400.0, 0.0, 300.5, 0.0, 410.0, 200.5, 0.0, 0.0, 1.0};
  const double r[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  for (size_t i = 0; i < 9; ++i) {
    EXPECT_EQ(info->k[i], k[i]) << i;
    EXPECT_EQ(info->r[i], r[i]) << i;
  }
  for (size_t i = 0; i < 12; ++i) {
    EXPECT_EQ(info->p[i], h.p[i]) << i;
  }
}

TEST(ToCameraInfo, RawCarriesTheUnrectifiedKAndD)
{
  const auto h = calibrated_header(wire::Kind::kRgb);
  const auto info = jetson_bridge_cpp::to_camera_info(h, jetson_bridge_cpp::CameraModel::kRaw);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->header.frame_id, "overview_camera_optical_frame");
  EXPECT_EQ(info->header.stamp.sec, 1760000000);
  EXPECT_EQ(info->distortion_model, "plumb_bob");
  ASSERT_EQ(info->d.size(), 5U);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(info->d[i], h.d[i]) << i;
  }
  for (size_t i = 0; i < 9; ++i) {
    EXPECT_EQ(info->k[i], h.k[i]) << i;
  }
  const double r[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  for (size_t i = 0; i < 9; ++i) {
    EXPECT_EQ(info->r[i], r[i]) << i;
  }
  // P of the unrectified camera is K padded with a zero translation column
  const double p[12] = {500.0, 0.0, 320.5, 0.0, 0.0, 510.0, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};
  for (size_t i = 0; i < 12; ++i) {
    EXPECT_EQ(info->p[i], p[i]) << i;
  }
}

TEST(ToCameraInfo, UncalibratedProducesNone)
{
  auto h = calibrated_header(wire::Kind::kDepth);
  h.calibrated = false;
  EXPECT_FALSE(jetson_bridge_cpp::to_camera_info(h, jetson_bridge_cpp::CameraModel::kRectified).has_value());
  EXPECT_FALSE(jetson_bridge_cpp::to_camera_info(h, jetson_bridge_cpp::CameraModel::kRaw).has_value());
}

TEST(ToCompressed, FieldByField)
{
  const auto h = calibrated_header(wire::Kind::kRgb);
  const std::vector<uint8_t> jpeg = {0xFF, 0xD8, 0x01, 0x02, 0xFF, 0xD9};
  const auto msg = jetson_bridge_cpp::to_compressed(h, jpeg);
  EXPECT_EQ(msg.header.frame_id, "overview_camera_optical_frame");
  EXPECT_EQ(msg.header.stamp.sec, 1760000000);
  EXPECT_EQ(msg.format, "jpeg");
  EXPECT_EQ(msg.data, jpeg);
}

TEST(ToCompressed, EmptyFrameIdFallsBackToTheContractFrame)
{
  auto h = calibrated_header(wire::Kind::kRgb);
  h.frame_id.clear();
  EXPECT_EQ(jetson_bridge_cpp::to_compressed(h, {0xFF, 0xD8}).header.frame_id, "overview_camera_optical_frame");
}

TEST(ToCompressed, DepthKindOrEmptyJpegThrows)
{
  EXPECT_THROW(jetson_bridge_cpp::to_compressed(calibrated_header(wire::Kind::kDepth), {0xFF}), std::invalid_argument);
  EXPECT_THROW(jetson_bridge_cpp::to_compressed(calibrated_header(wire::Kind::kRgb), {}), std::invalid_argument);
}

TEST(StalenessPolicy, DefaultsAreTheDesignValues)
{
  StalenessPolicy p;
  auto h = calibrated_header(wire::Kind::kDepth);
  h.capture_realtime_ns = kNow - 500 * kMs;
  EXPECT_EQ(p.accept(h, kNow), Verdict::kAccept);
  h.capture_realtime_ns = kNow - 500 * kMs - 1;
  EXPECT_EQ(p.accept(h, kNow), Verdict::kTooOld);
  h.capture_realtime_ns = kNow + 50 * kMs;
  EXPECT_EQ(p.accept(h, kNow), Verdict::kAccept);
  h.capture_realtime_ns = kNow + 50 * kMs + 1;
  EXPECT_EQ(p.accept(h, kNow), Verdict::kFromFuture);
}

TEST(StalenessPolicy, FreshFrameIsAccepted)
{
  StalenessPolicy p(500, 2000);
  auto h = calibrated_header(wire::Kind::kDepth);
  h.capture_realtime_ns = kNow - 10 * kMs;
  EXPECT_EQ(p.accept(h, kNow), Verdict::kAccept);
}

TEST(StalenessPolicy, MaxAgeIsConfigurable)
{
  StalenessPolicy p(100, 2000);
  auto h = calibrated_header(wire::Kind::kDepth);
  h.capture_realtime_ns = kNow - 101 * kMs;
  EXPECT_EQ(p.accept(h, kNow), Verdict::kTooOld);
  h.capture_realtime_ns = kNow - 100 * kMs;
  EXPECT_EQ(p.accept(h, kNow), Verdict::kAccept);
}

TEST(StalenessPolicy, CountsRejectsPerReason)
{
  StalenessPolicy p;
  auto h = calibrated_header(wire::Kind::kDepth);
  h.capture_realtime_ns = kNow - 900 * kMs;
  p.accept(h, kNow);
  p.accept(h, kNow);
  h.capture_realtime_ns = kNow + 200 * kMs;
  p.accept(h, kNow);
  h.capture_realtime_ns = kNow;
  p.accept(h, kNow);
  EXPECT_EQ(p.rejected_too_old(), 2U);
  EXPECT_EQ(p.rejected_from_future(), 1U);
  EXPECT_EQ(p.accepted(), 1U);
}

TEST(StalenessPolicy, AbsentBeforeAnyArrival)
{
  StalenessPolicy p;
  EXPECT_TRUE(p.absent(kNow));
}

TEST(StalenessPolicy, AbsentAfterAbsentAfterMsWithoutArrival)
{
  StalenessPolicy p;
  auto h = calibrated_header(wire::Kind::kDepth);
  h.capture_realtime_ns = kNow;
  ASSERT_EQ(p.accept(h, kNow), Verdict::kAccept);
  EXPECT_FALSE(p.absent(kNow));
  EXPECT_FALSE(p.absent(kNow + 2000 * kMs));
  EXPECT_TRUE(p.absent(kNow + 2000 * kMs + 1));
}

TEST(StalenessPolicy, ARejectedFrameStillCountsAsArrival)
{
  StalenessPolicy p;
  auto h = calibrated_header(wire::Kind::kDepth);
  h.capture_realtime_ns = kNow - 900 * kMs;
  ASSERT_EQ(p.accept(h, kNow), Verdict::kTooOld);
  EXPECT_FALSE(p.absent(kNow + 1000 * kMs));
  EXPECT_TRUE(p.absent(kNow + 2001 * kMs));
}

TEST(StalenessPolicy, AbsentAfterIsConfigurable)
{
  StalenessPolicy p(500, 300);
  auto h = calibrated_header(wire::Kind::kDepth);
  h.capture_realtime_ns = kNow;
  p.accept(h, kNow);
  EXPECT_FALSE(p.absent(kNow + 300 * kMs));
  EXPECT_TRUE(p.absent(kNow + 301 * kMs));
}
