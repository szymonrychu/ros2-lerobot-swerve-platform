#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <random>
#include <set>
#include <vector>

#include "depthd/pairing.hpp"

using depthd::Frame;
using depthd::SkewHistogram;
using depthd::StereoPair;
using depthd::StereoPairer;

namespace
{
constexpr int64_t kMs = 1000000;
constexpr int64_t kPeriod = 33333333;  // 30 fps

Frame frame(int64_t t_ns, uint64_t seq)
{
  Frame f;
  f.capture_realtime_ns = t_ns;
  f.sequence = seq;
  return f;
}
}  // namespace

TEST(Pairing, InWindowFramesPair)
{
  StereoPairer p(8.0);
  EXPECT_FALSE(p.push_left(frame(1000 * kMs, 1)));
  const auto pair = p.push_right(frame(1000 * kMs + 5 * kMs, 7));
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->capture_realtime_ns, 1000 * kMs);
  EXPECT_EQ(pair->right_minus_left_ns, 5 * kMs);
  EXPECT_EQ(pair->left_sequence, 1u);
  EXPECT_EQ(pair->right_sequence, 7u);
  EXPECT_EQ(p.queued_left(), 0u);
  EXPECT_EQ(p.queued_right(), 0u);
  EXPECT_EQ(p.stats().pairs_emitted, 1u);
}

TEST(Pairing, NegativeSkewInWindowPairsAndKeepsLeftStamp)
{
  StereoPairer p(8.0);
  EXPECT_FALSE(p.push_right(frame(500 * kMs, 1)));
  const auto pair = p.push_left(frame(507 * kMs, 1));
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->capture_realtime_ns, 507 * kMs);
  EXPECT_EQ(pair->right_minus_left_ns, -7 * kMs);
}

TEST(Pairing, WindowBoundaryIsInclusive)
{
  StereoPairer p(8.0);
  p.push_left(frame(0, 1));
  EXPECT_TRUE(p.push_right(frame(8 * kMs, 1)));
}

TEST(Pairing, OutOfWindowDropsTheEarlierFrameAsSkew)
{
  StereoPairer p(8.0);
  EXPECT_FALSE(p.push_left(frame(1000 * kMs, 1)));
  EXPECT_FALSE(p.push_right(frame(1012 * kMs, 1)));  // 12 ms apart: no pair
  const auto s = p.stats();
  EXPECT_EQ(s.pairs_emitted, 0u);
  EXPECT_EQ(s.dropped_skew, 1u);
  EXPECT_EQ(s.dropped_orphan, 0u);
  EXPECT_EQ(p.queued_left(), 0u);   // the earlier (left) frame can never pair: dropped
  EXPECT_EQ(p.queued_right(), 1u);  // the right frame waits for the next left
  // The next left frame is 1033.3 ms, 21 ms after the waiting right: dropped too, right was the earlier one.
  EXPECT_FALSE(p.push_left(frame(1000 * kMs + kPeriod, 2)));
  EXPECT_EQ(p.stats().dropped_skew, 2u);
}

TEST(Pairing, OrphanIsDroppedWhenItsCounterpartWasMissed)
{
  StereoPairer p(8.0);
  // The right camera missed the frame matching left #1: left #1 and #2 queue, then right matches #2.
  EXPECT_FALSE(p.push_left(frame(0, 1)));
  EXPECT_FALSE(p.push_left(frame(kPeriod, 2)));
  const auto pair = p.push_right(frame(kPeriod + 2 * kMs, 1));
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->left_sequence, 2u);
  const auto s = p.stats();
  EXPECT_EQ(s.dropped_orphan, 1u);
  EXPECT_EQ(s.dropped_skew, 0u);
  EXPECT_EQ(p.queued_left(), 0u);
}

TEST(Pairing, StaleArrivalIsDroppedAsOrphan)
{
  StereoPairer p(8.0);
  p.push_left(frame(100 * kMs, 1));
  ASSERT_TRUE(p.push_right(frame(101 * kMs, 1)));
  // A late left frame older than the one already paired can never be paired again.
  EXPECT_FALSE(p.push_left(frame(99 * kMs, 0)));
  EXPECT_EQ(p.queued_left(), 0u);
  EXPECT_EQ(p.stats().dropped_orphan, 1u);
}

TEST(Pairing, NeverReusesAFrame)
{
  StereoPairer p(8.0);
  p.push_left(frame(0, 1));
  ASSERT_TRUE(p.push_right(frame(1 * kMs, 1)));
  // A second right frame close to the same left must not pair with it again.
  EXPECT_FALSE(p.push_right(frame(2 * kMs, 2)));
  EXPECT_EQ(p.stats().pairs_emitted, 1u);
  EXPECT_EQ(p.queued_right(), 1u);
}

TEST(Pairing, OrderingWithJitterPairsEveryFrameOnceInOrder)
{
  StereoPairer p(8.0);
  std::mt19937 rng(42);
  std::uniform_int_distribution<int64_t> jitter(-2 * kMs, 2 * kMs);
  std::bernoulli_distribution right_first(0.5);
  const int64_t offset = 3 * kMs;  // the right sensor runs 3 ms behind
  std::vector<StereoPair> pairs;
  for (uint64_t i = 1; i <= 300; ++i) {
    const auto l = frame(static_cast<int64_t>(i) * kPeriod + jitter(rng), i);
    const auto r = frame(static_cast<int64_t>(i) * kPeriod + offset + jitter(rng), i);
    std::optional<StereoPair> got;
    if (right_first(rng)) {
      got = p.push_right(r);
      if (!got) got = p.push_left(l);
    } else {
      got = p.push_left(l);
      if (!got) got = p.push_right(r);
    }
    if (got) pairs.push_back(*got);
  }
  ASSERT_EQ(pairs.size(), 300u);
  std::set<uint64_t> lefts;
  std::set<uint64_t> rights;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    EXPECT_EQ(pairs[i].left_sequence, pairs[i].right_sequence);
    EXPECT_LE(std::abs(pairs[i].right_minus_left_ns), 8 * kMs);
    if (i > 0) {
      EXPECT_GT(pairs[i].capture_realtime_ns, pairs[i - 1].capture_realtime_ns);
    }
    lefts.insert(pairs[i].left_sequence);
    rights.insert(pairs[i].right_sequence);
  }
  EXPECT_EQ(lefts.size(), 300u);
  EXPECT_EQ(rights.size(), 300u);
  const auto s = p.stats();
  EXPECT_EQ(s.dropped_skew + s.dropped_orphan, 0u);
}

TEST(Pairing, ArrivalOrderWithinASideIsSortedByTimestamp)
{
  StereoPairer p(8.0);
  // Two left frames arrive out of order; the right frame matches the older one.
  EXPECT_FALSE(p.push_left(frame(2 * kPeriod, 2)));
  EXPECT_FALSE(p.push_left(frame(kPeriod, 1)));
  const auto pair = p.push_right(frame(kPeriod + kMs, 1));
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->left_sequence, 1u);
  EXPECT_EQ(p.queued_left(), 1u);
}

TEST(Pairing, QueuesAreBoundedToThreeFramesPerSide)
{
  StereoPairer p(8.0);
  for (uint64_t i = 0; i < 10; ++i) {
    EXPECT_FALSE(p.push_left(frame(static_cast<int64_t>(i) * kPeriod, i)));
    EXPECT_LE(p.queued_left(), StereoPairer::kMaxQueue);
  }
  EXPECT_EQ(p.queued_left(), 3u);
  EXPECT_EQ(p.stats().dropped_orphan, 7u);
  // The right side is bounded too.
  StereoPairer q(8.0);
  for (uint64_t i = 0; i < 5; ++i) q.push_right(frame(static_cast<int64_t>(i) * kPeriod, i));
  EXPECT_EQ(q.queued_right(), 3u);
  // The newest frames are the ones kept: a right frame next to #9 pairs.
  const auto pair = p.push_right(frame(9 * kPeriod + kMs, 9));
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->left_sequence, 9u);
}

TEST(Pairing, ResetDropsQueuedFramesAsOrphans)
{
  StereoPairer p(8.0);
  p.push_left(frame(0, 1));
  p.push_left(frame(kPeriod, 2));
  p.reset();
  EXPECT_EQ(p.queued_left(), 0u);
  EXPECT_EQ(p.stats().dropped_orphan, 2u);
  EXPECT_FALSE(p.push_right(frame(kPeriod, 1)));
}

TEST(Pairing, PairCarriesTheGpuImages)
{
  StereoPairer p(8.0);
  Frame l = frame(0, 1);
  Frame r = frame(kMs, 1);
  l.image = cv::cuda::GpuMat(4, 8, CV_8UC1, cv::Scalar(10));
  r.image = cv::cuda::GpuMat(4, 8, CV_8UC1, cv::Scalar(20));
  const void * left_data = l.image.data;
  p.push_left(l);
  const auto pair = p.push_right(r);
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->left.data, left_data);  // no copy
  cv::Mat host;
  pair->right.download(host);
  EXPECT_EQ(host.at<uint8_t>(0, 0), 20);
}

TEST(SkewHistogramTest, BucketsAre1_2_4_8_16_33Ms)
{
  EXPECT_EQ(SkewHistogram::kBoundsMs, (std::array<double, 6>{{1, 2, 4, 8, 16, 33}}));
  SkewHistogram h;
  for (double ms : {0.5, 1.0, 1.5, 3.0, 7.9, 8.0, 12.0, 20.0, 33.0, 40.0}) h.record(ms);
  EXPECT_EQ(h.counts, (std::array<uint64_t, 7>{{2, 1, 1, 2, 1, 2, 1}}));
  EXPECT_EQ(h.cumulative(), (std::array<uint64_t, 7>{{2, 3, 4, 6, 7, 9, 10}}));
  EXPECT_EQ(h.count, 10u);
  EXPECT_NEAR(h.sum_ms, 126.9, 1e-9);
}

TEST(SkewHistogramTest, PairerRecordsEmittedAndSkewDroppedCandidates)
{
  StereoPairer p(8.0);
  p.push_left(frame(0, 1));
  p.push_right(frame(3 * kMs, 1));  // emitted, 3 ms
  p.push_left(frame(kPeriod, 2));
  p.push_right(frame(kPeriod + 12 * kMs, 2));  // skew drop, 12 ms
  const auto h = p.stats().skew;
  EXPECT_EQ(h.count, 2u);
  EXPECT_EQ(h.counts[2], 1u);  // (2, 4]
  EXPECT_EQ(h.counts[4], 1u);  // (8, 16]
}
