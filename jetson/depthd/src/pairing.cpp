#include "depthd/pairing.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace depthd
{

void SkewHistogram::record(double skew_ms)
{
  const double v = std::abs(skew_ms);
  std::size_t i = 0;
  while (i < kBuckets && v > kBoundsMs[i]) ++i;
  ++counts[i];
  ++count;
  sum_ms += v;
}

std::array<uint64_t, SkewHistogram::kBuckets + 1> SkewHistogram::cumulative() const
{
  std::array<uint64_t, kBuckets + 1> out{};
  uint64_t running = 0;
  for (std::size_t i = 0; i < out.size(); ++i) {
    running += counts[i];
    out[i] = running;
  }
  return out;
}

StereoPairer::StereoPairer(double pair_window_ms) : window_ns_(static_cast<int64_t>(std::llround(pair_window_ms * 1e6)))
{
}

void StereoPairer::set_skew_observer(SkewObserver observer) { skew_observer_ = std::move(observer); }

std::optional<StereoPair> StereoPairer::push_left(Frame frame) { return push(Side::left, std::move(frame)); }

std::optional<StereoPair> StereoPairer::push_right(Frame frame) { return push(Side::right, std::move(frame)); }

std::optional<StereoPair> StereoPairer::push(Side side, Frame frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & queue = side == Side::left ? left_ : right_;
  const auto & consumed = side == Side::left ? left_consumed_ns_ : right_consumed_ns_;
  if (consumed && frame.capture_realtime_ns <= *consumed) {
    ++stats_.dropped_orphan;  // older than a frame already paired or dropped: it can never pair
    return std::nullopt;
  }
  const auto pos = std::upper_bound(
    queue.begin(), queue.end(), frame.capture_realtime_ns,
    [](int64_t t, const Frame & f) { return t < f.capture_realtime_ns; });
  queue.insert(pos, std::move(frame));
  while (queue.size() > kMaxQueue) {
    auto & mark = side == Side::left ? left_consumed_ns_ : right_consumed_ns_;
    mark = queue.front().capture_realtime_ns;
    queue.pop_front();
    ++stats_.dropped_orphan;
  }
  return match();
}

// Repeatedly judge the earlier of the two queue heads E against the other head O (its nearest counterpart: everything
// else on O's side is later still):
//   - E's own successor is nearer to O  -> E's counterpart was never captured: drop E as an orphan;
//   - |O - E| <= window                 -> emit the pair (with window < half a frame period it is the nearest);
//   - otherwise                         -> E can never pair: drop it for skew.
std::optional<StereoPair> StereoPairer::match()
{
  while (!left_.empty() && !right_.empty()) {
    const bool left_earlier = left_.front().capture_realtime_ns <= right_.front().capture_realtime_ns;
    auto & early = left_earlier ? left_ : right_;
    auto & other = left_earlier ? right_ : left_;
    auto & early_mark = left_earlier ? left_consumed_ns_ : right_consumed_ns_;
    const int64_t e = early.front().capture_realtime_ns;
    const int64_t o = other.front().capture_realtime_ns;
    const int64_t gap = o - e;  // >= 0
    if (early.size() > 1 && std::llabs(early[1].capture_realtime_ns - o) < gap) {
      early_mark = e;
      early.pop_front();
      ++stats_.dropped_orphan;
      continue;
    }
    const double gap_ms = static_cast<double>(gap) / 1e6;
    stats_.skew.record(gap_ms);
    if (skew_observer_) skew_observer_(gap_ms);
    if (gap > window_ns_) {
      early_mark = e;
      early.pop_front();
      ++stats_.dropped_skew;
      continue;
    }
    Frame l = std::move(left_.front());
    Frame r = std::move(right_.front());
    left_.pop_front();
    right_.pop_front();
    left_consumed_ns_ = l.capture_realtime_ns;
    right_consumed_ns_ = r.capture_realtime_ns;
    ++stats_.pairs_emitted;
    StereoPair pair;
    pair.left = std::move(l.image);
    pair.right = std::move(r.image);
    pair.capture_realtime_ns = l.capture_realtime_ns;
    pair.right_minus_left_ns = r.capture_realtime_ns - l.capture_realtime_ns;
    pair.left_sequence = l.sequence;
    pair.right_sequence = r.sequence;
    return pair;
  }
  return std::nullopt;
}

void StereoPairer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.dropped_orphan += left_.size() + right_.size();
  if (!left_.empty()) left_consumed_ns_ = left_.back().capture_realtime_ns;
  if (!right_.empty()) right_consumed_ns_ = right_.back().capture_realtime_ns;
  left_.clear();
  right_.clear();
}

PairingStats StereoPairer::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

std::size_t StereoPairer::queued_left() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return left_.size();
}

std::size_t StereoPairer::queued_right() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return right_.size();
}

}  // namespace depthd
