// Depth wire format: header round trip, depth payload, rgb passthrough and every rejection.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "ros2_common_cpp/depth_wire.hpp"

namespace dw = ros2_common_cpp::depth_wire;

namespace
{

dw::Header full_header(dw::Kind kind, uint32_t w, uint32_t h)
{
  dw::Header x;
  x.kind = kind;
  x.sequence = 0x0102030405060708ULL;
  x.capture_realtime_ns = 1760000000123456789LL;
  x.right_minus_left_ns = -2500000;
  x.width = w;
  x.height = h;
  x.frame_id = "stereo_left_optical_frame";
  for (size_t i = 0; i < x.k.size(); ++i) {x.k[i] = 100.5 + static_cast<double>(i) / 3.0;}
  for (size_t i = 0; i < x.d.size(); ++i) {x.d[i] = -0.25 * static_cast<double>(i) + 1e-7;}
  for (size_t i = 0; i < x.p.size(); ++i) {x.p[i] = 7.125 * static_cast<double>(i) - 3.0;}
  x.baseline_m = 0.0603;
  x.calibrated = true;
  x.pipeline_fps = 14.5F;
  x.stage_capture_ms = 1.5F;
  x.stage_rectify_ms = 2.25F;
  x.stage_match_ms = 20.5F;
  x.stage_post_ms = 4.75F;
  x.stage_mono_ms = 30.0F;
  x.stage_encode_ms = 3.125F;
  x.pairs_dropped_total = 12345678901ULL;
  x.gpu_temp_c = 47.5F;
  x.nvgpu_errors_total = 9;
  return x;
}

std::vector<uint16_t> smooth_image(uint32_t w, uint32_t h)
{
  std::vector<uint16_t> v(static_cast<size_t>(w) * h);
  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      v[static_cast<size_t>(y) * w + x] = static_cast<uint16_t>(1000 + x + y);
    }
  }
  return v;
}

void expect_equal(const dw::Header & a, const dw::Header & b)
{
  EXPECT_EQ(a.kind, b.kind);
  EXPECT_EQ(a.sequence, b.sequence);
  EXPECT_EQ(a.capture_realtime_ns, b.capture_realtime_ns);
  EXPECT_EQ(a.right_minus_left_ns, b.right_minus_left_ns);
  EXPECT_EQ(a.width, b.width);
  EXPECT_EQ(a.height, b.height);
  EXPECT_EQ(a.frame_id, b.frame_id);
  EXPECT_EQ(a.k, b.k);
  EXPECT_EQ(a.d, b.d);
  EXPECT_EQ(a.p, b.p);
  EXPECT_EQ(a.baseline_m, b.baseline_m);
  EXPECT_EQ(a.calibrated, b.calibrated);
  EXPECT_EQ(a.pipeline_fps, b.pipeline_fps);
  EXPECT_EQ(a.stage_capture_ms, b.stage_capture_ms);
  EXPECT_EQ(a.stage_rectify_ms, b.stage_rectify_ms);
  EXPECT_EQ(a.stage_match_ms, b.stage_match_ms);
  EXPECT_EQ(a.stage_post_ms, b.stage_post_ms);
  EXPECT_EQ(a.stage_mono_ms, b.stage_mono_ms);
  EXPECT_EQ(a.stage_encode_ms, b.stage_encode_ms);
  EXPECT_EQ(a.pairs_dropped_total, b.pairs_dropped_total);
  EXPECT_EQ(a.gpu_temp_c, b.gpu_temp_c);
  EXPECT_EQ(a.nvgpu_errors_total, b.nvgpu_errors_total);
}

}  // namespace

TEST(DepthWire, HeaderRoundTripsEveryField)
{
  const dw::Header in = full_header(dw::Kind::kFused, 640, 480);
  const auto bytes = dw::encode_header(in);
  ASSERT_EQ(bytes.size(), dw::kHeaderSize);
  EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 4), "DPW1");
  dw::Header out;
  ASSERT_EQ(dw::decode_header(bytes.data(), bytes.size(), out), "");
  expect_equal(in, out);
}

TEST(DepthWire, HeaderIsLittleEndian)
{
  dw::Header h;
  h.sequence = 0x0102030405060708ULL;
  const auto bytes = dw::encode_header(h);
  // magic 4, version 2, kind 1, reserved 1, then the sequence
  EXPECT_EQ(bytes[4], 1);
  EXPECT_EQ(bytes[5], 0);
  EXPECT_EQ(bytes[8], 0x08);
  EXPECT_EQ(bytes[15], 0x01);
}

TEST(DepthWire, FrameIdOf63BytesRoundTripsAndLongerThrows)
{
  dw::Header h;
  h.frame_id = std::string(63, 'x');
  dw::Header out;
  const auto bytes = dw::encode_header(h);
  ASSERT_EQ(dw::decode_header(bytes.data(), bytes.size(), out), "");
  EXPECT_EQ(out.frame_id, h.frame_id);
  h.frame_id = std::string(64, 'x');
  EXPECT_THROW(dw::encode_header(h), std::invalid_argument);
}

TEST(DepthWire, DepthRoundTripPreservesZeroAndMax)
{
  auto img = smooth_image(16, 8);
  img[0] = 0;
  img[1] = 65535;
  img[127] = 65535;
  img[64] = 0;
  const dw::Header in = full_header(dw::Kind::kDepth, 16, 8);
  const auto frames = dw::encode_depth(in, img);
  EXPECT_EQ(frames.topic, "depth");
  const auto out = dw::decode(frames);
  ASSERT_TRUE(out.ok()) << out.error;
  EXPECT_EQ(out.depth_mm, img);
  expect_equal(in, out.header);
}

TEST(DepthWire, FusedUsesItsOwnTopic)
{
  const auto img = smooth_image(8, 8);
  const auto frames = dw::encode_depth(full_header(dw::Kind::kFused, 8, 8), img);
  EXPECT_EQ(frames.topic, "fused");
  EXPECT_TRUE(dw::decode(frames).ok());
}

TEST(DepthWire, PayloadIsSmallerThanRawForASmoothImage)
{
  const auto img = smooth_image(640, 480);
  const auto frames = dw::encode_depth(full_header(dw::Kind::kDepth, 640, 480), img);
  EXPECT_LT(frames.payload.size(), img.size() * 2);
}

TEST(DepthWire, RgbCarriesTheJpegBytesUnchanged)
{
  std::vector<uint8_t> jpeg = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00, 0xFF, 0xD9};
  const dw::Header in = full_header(dw::Kind::kRgb, 640, 480);
  const auto frames = dw::encode_rgb(in, jpeg);
  EXPECT_EQ(frames.topic, "rgb");
  EXPECT_EQ(frames.payload, jpeg);
  const auto out = dw::decode(frames);
  ASSERT_TRUE(out.ok()) << out.error;
  EXPECT_EQ(out.jpeg, jpeg);
  expect_equal(in, out.header);
}

TEST(DepthWire, RejectsWrongMagic)
{
  auto f = dw::encode_depth(full_header(dw::Kind::kDepth, 4, 4), smooth_image(4, 4));
  f.header[0] = 'X';
  const auto out = dw::decode(f);
  EXPECT_FALSE(out.ok());
  EXPECT_EQ(out.header.width, 0U);
  EXPECT_TRUE(out.depth_mm.empty());
}

TEST(DepthWire, RejectsUnknownVersion)
{
  auto f = dw::encode_depth(full_header(dw::Kind::kDepth, 4, 4), smooth_image(4, 4));
  f.header[4] = 2;
  EXPECT_FALSE(dw::decode(f).ok());
}

TEST(DepthWire, RejectsTruncatedHeaderAtEveryLength)
{
  auto f = dw::encode_depth(full_header(dw::Kind::kDepth, 4, 4), smooth_image(4, 4));
  for (size_t n = 0; n < f.header.size(); ++n) {
    const auto out = dw::decode(f.topic, f.header.data(), n, f.payload.data(), f.payload.size());
    EXPECT_FALSE(out.ok()) << "length " << n;
    EXPECT_EQ(out.header.sequence, 0U);
  }
}

TEST(DepthWire, RejectsDimensionsThatDoNotMatchThePayload)
{
  auto f = dw::encode_depth(full_header(dw::Kind::kDepth, 4, 4), smooth_image(4, 4));
  dw::Header wrong = full_header(dw::Kind::kDepth, 5, 4);
  f.header = dw::encode_header(wrong);
  EXPECT_FALSE(dw::decode(f).ok());
}

TEST(DepthWire, RejectsGarbageDepthPayload)
{
  auto f = dw::encode_depth(full_header(dw::Kind::kDepth, 4, 4), smooth_image(4, 4));
  f.payload.assign(10, 0x42);
  EXPECT_FALSE(dw::decode(f).ok());
}

TEST(DepthWire, RejectsUnknownTopic)
{
  auto f = dw::encode_depth(full_header(dw::Kind::kDepth, 4, 4), smooth_image(4, 4));
  f.topic = "lidar";
  EXPECT_FALSE(dw::decode(f).ok());
}

TEST(DepthWire, RejectsTopicThatDisagreesWithHeaderKind)
{
  auto f = dw::encode_depth(full_header(dw::Kind::kDepth, 4, 4), smooth_image(4, 4));
  f.topic = "rgb";
  EXPECT_FALSE(dw::decode(f).ok());
}
