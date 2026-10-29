// FramePairer on synthetic 15 Hz stamp streams, without ROS spinning: exact and approximate pairing with the colour
// and depth stamps equal, 3 ms apart and with lost frames. Every input frame ends up exactly once as paired or
// unpaired (or still pending), and the stamp-delta callback sees each pair candidate.
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "head_rgbd_cpp/pairing.hpp"

namespace
{

using head_rgbd_cpp::FramePairer;
using head_rgbd_cpp::PairingConfig;
using head_rgbd_cpp::PairingPolicy;
using head_rgbd_cpp::Stream;
using Image = sensor_msgs::msg::Image;

constexpr int64_t T0_NS = 1'000'000'000'000;
constexpr int64_t PERIOD_NS = 66'666'667;  // 15 Hz
constexpr int64_t OFFSET_3MS_NS = 3'000'000;
constexpr int64_t OFFSET_30MS_NS = 30'000'000;
constexpr uint32_t QUEUE = 5;
constexpr int FRAMES = 30;
constexpr double NS_TO_S = 1e-9;
const std::set<int> LOST = {4, 9, 14, 19, 24};

struct Recorder
{
  std::vector<std::pair<int64_t, int64_t>> pairs;  // (depth, colour) stamps
  std::vector<std::pair<Stream, int64_t>> unpaired;
  std::vector<double> deltas;
};

int64_t ns_of(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1'000'000'000 + t.nanosec;
}

Image::ConstSharedPtr frame(int64_t stamp_ns)
{
  auto m = std::make_shared<Image>();
  m->header.stamp.sec = static_cast<int32_t>(stamp_ns / 1'000'000'000);
  m->header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000);
  return m;
}

std::unique_ptr<FramePairer> make(Recorder & r, PairingPolicy policy, double max_interval_s = 0.02)
{
  PairingConfig config;
  config.policy = policy;
  config.max_interval_s = max_interval_s;
  return std::make_unique<FramePairer>(
    config, QUEUE,
    [&r](const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & colour) {
      r.pairs.emplace_back(ns_of(depth->header.stamp), ns_of(colour->header.stamp));
    },
    [&r](Stream stream, int64_t stamp_ns) { r.unpaired.emplace_back(stream, stamp_ns); },
    [&r](double delta_s) { r.deltas.push_back(delta_s); });
}

int64_t colour_ns(int i) { return T0_NS + i * PERIOD_NS; }

// Colour frame i at colour_ns(i), then the depth frame of the same index depth_offset_ns later (realsense2_camera
// publishes the aligned depth after the colour), skipping the lost colour frames.
void feed(FramePairer & pairer, int64_t depth_offset_ns, const std::set<int> & lost_colour = {})
{
  for (int i = 0; i < FRAMES; ++i) {
    if (!lost_colour.count(i)) {
      pairer.add_colour(frame(colour_ns(i)));
    }
    pairer.add_depth(frame(colour_ns(i) + depth_offset_ns));
  }
}

// Every input frame is paired, unpaired or pending, exactly once.
void expect_each_frame_once(const Recorder & r, const FramePairer & pairer, size_t frames)
{
  std::set<std::pair<Stream, int64_t>> seen;
  for (const auto & [depth, colour] : r.pairs) {
    EXPECT_TRUE(seen.insert({Stream::DEPTH, depth}).second);
    EXPECT_TRUE(seen.insert({Stream::COLOUR, colour}).second);
  }
  for (const auto & u : r.unpaired) {
    EXPECT_TRUE(seen.insert(u).second) << "counted twice: " << u.second;
  }
  EXPECT_EQ(seen.size() + pairer.pending(), frames);
}

}  // namespace

TEST(FramePairer, ExactPairsEqualStamps)
{
  Recorder r;
  auto pairer = make(r, PairingPolicy::EXACT);
  feed(*pairer, 0);
  ASSERT_EQ(r.pairs.size(), static_cast<size_t>(FRAMES));
  for (int i = 0; i < FRAMES; ++i) {
    EXPECT_EQ(r.pairs[i].first, colour_ns(i));
    EXPECT_EQ(r.pairs[i].second, colour_ns(i));
  }
  EXPECT_TRUE(r.unpaired.empty());
  EXPECT_EQ(pairer->pending(), 0u);
  ASSERT_EQ(r.deltas.size(), static_cast<size_t>(FRAMES));
  for (double d : r.deltas) {
    EXPECT_DOUBLE_EQ(d, 0.0);
  }
}

TEST(FramePairer, ExactCountsEachFrameOfAShiftedStreamOnceAsUnpaired)
{
  Recorder r;
  auto pairer = make(r, PairingPolicy::EXACT);
  feed(*pairer, OFFSET_3MS_NS);
  EXPECT_TRUE(r.pairs.empty());
  expect_each_frame_once(r, *pairer, 2 * FRAMES);
  EXPECT_LE(pairer->pending(), 2 * QUEUE);
  // The near miss shows in the histogram: one colour-minus-nearest-depth delta per unpaired colour frame.
  size_t colour_unpaired = 0;
  for (const auto & u : r.unpaired) {
    colour_unpaired += u.first == Stream::COLOUR ? 1 : 0;
  }
  EXPECT_GT(colour_unpaired, 0u);
  ASSERT_EQ(r.deltas.size(), colour_unpaired);
  for (double d : r.deltas) {
    EXPECT_NEAR(d, -OFFSET_3MS_NS * NS_TO_S, 1e-9);
  }
}

TEST(FramePairer, ApproximatePairsAShiftedStream)
{
  Recorder r;
  auto pairer = make(r, PairingPolicy::APPROXIMATE);
  feed(*pairer, OFFSET_3MS_NS);
  // The approximate policy holds the newest set until a later frame shows it is the best one.
  EXPECT_GE(r.pairs.size(), static_cast<size_t>(FRAMES - 2));
  for (const auto & [depth, colour] : r.pairs) {
    EXPECT_EQ(depth - colour, OFFSET_3MS_NS);
  }
  EXPECT_TRUE(r.unpaired.empty());
  expect_each_frame_once(r, *pairer, 2 * FRAMES);
  ASSERT_EQ(r.deltas.size(), r.pairs.size());
  for (double d : r.deltas) {
    EXPECT_NEAR(d, -OFFSET_3MS_NS * NS_TO_S, 1e-9);
  }
}

TEST(FramePairer, ApproximateNeverPairsBeyondTheMaxInterval)
{
  Recorder r;
  auto pairer = make(r, PairingPolicy::APPROXIMATE, 0.02);
  feed(*pairer, OFFSET_30MS_NS);
  EXPECT_TRUE(r.pairs.empty());
  EXPECT_FALSE(r.unpaired.empty());
  expect_each_frame_once(r, *pairer, 2 * FRAMES);
}

TEST(FramePairer, ExactCountsTheDepthOfALostColourFrameOnce)
{
  Recorder r;
  auto pairer = make(r, PairingPolicy::EXACT);
  feed(*pairer, 0, LOST);
  EXPECT_EQ(r.pairs.size(), FRAMES - LOST.size());
  std::vector<std::pair<Stream, int64_t>> expected;
  for (int i : LOST) {
    expected.emplace_back(Stream::DEPTH, colour_ns(i));
  }
  EXPECT_EQ(r.unpaired, expected);
  EXPECT_EQ(pairer->pending(), 0u);
  expect_each_frame_once(r, *pairer, 2 * FRAMES - LOST.size());
  EXPECT_EQ(r.deltas.size(), r.pairs.size());
}

TEST(FramePairer, ApproximateCountsTheDepthOfALostColourFrameOnce)
{
  Recorder r;
  auto pairer = make(r, PairingPolicy::APPROXIMATE);
  feed(*pairer, OFFSET_3MS_NS, LOST);
  EXPECT_GE(r.pairs.size(), FRAMES - LOST.size() - 2);
  std::set<int64_t> lost_depth;
  for (int i : LOST) {
    lost_depth.insert(colour_ns(i) + OFFSET_3MS_NS);
  }
  ASSERT_EQ(r.unpaired.size(), LOST.size());
  for (const auto & [stream, stamp] : r.unpaired) {
    EXPECT_EQ(stream, Stream::DEPTH);
    EXPECT_TRUE(lost_depth.count(stamp)) << stamp;
  }
  expect_each_frame_once(r, *pairer, 2 * FRAMES - LOST.size());
}
