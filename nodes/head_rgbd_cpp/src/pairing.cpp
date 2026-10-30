#include "head_rgbd_cpp/pairing.hpp"

#include <algorithm>
#include <cstdlib>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <utility>

namespace head_rgbd_cpp
{

namespace
{
constexpr double S_PER_NS = 1e-9;

int64_t stamp_ns(const sensor_msgs::msg::Image & image) { return rclcpp::Time(image.header.stamp).nanoseconds(); }
}  // namespace

FramePairer::FramePairer(
  const PairingConfig & config, uint32_t queue_size, PairCallback on_pair, UnpairedCallback on_unpaired,
  DeltaCallback on_delta)
: queue_size_(queue_size),
  on_pair_(std::move(on_pair)),
  on_unpaired_(std::move(on_unpaired)),
  on_delta_(std::move(on_delta))
{
  if (config.policy == PairingPolicy::APPROXIMATE) {
    message_filters::sync_policies::ApproximateTime<Image, Image> policy(queue_size);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(config.max_interval_s));
    approximate_ = std::make_unique<ApproximateSync>(policy);
    approximate_->registerCallback(&FramePairer::on_synced, this);
  } else {
    exact_ = std::make_unique<ExactSync>(message_filters::sync_policies::ExactTime<Image, Image>(queue_size));
    exact_->registerCallback(&FramePairer::on_synced, this);
  }
}

void FramePairer::add_depth(const Image::ConstSharedPtr & depth)
{
  const int64_t stamp = stamp_ns(*depth);
  pending_depth_.push_back(stamp);
  recent_depth_.push_back(stamp);
  while (recent_depth_.size() > 2 * static_cast<size_t>(queue_size_)) {
    recent_depth_.pop_front();
  }
  if (exact_) {
    exact_->add<0>(depth);
  } else {
    approximate_->add<0>(depth);
  }
  cap(Stream::DEPTH);
}

void FramePairer::add_colour(const Image::ConstSharedPtr & colour)
{
  pending_colour_.push_back(stamp_ns(*colour));
  if (exact_) {
    exact_->add<1>(colour);
  } else {
    approximate_->add<1>(colour);
  }
  cap(Stream::COLOUR);
}

void FramePairer::on_synced(const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & colour)
{
  const int64_t depth_ns = stamp_ns(*depth);
  const int64_t colour_ns = stamp_ns(*colour);
  settle(Stream::DEPTH, depth_ns);
  settle(Stream::COLOUR, colour_ns);
  on_delta_(static_cast<double>(colour_ns - depth_ns) * S_PER_NS);
  on_pair_(depth, colour);
}

void FramePairer::settle(Stream stream, int64_t paired_ns)
{
  std::deque<int64_t> & pending = stream == Stream::DEPTH ? pending_depth_ : pending_colour_;
  std::deque<int64_t> older;
  std::deque<int64_t> kept;
  bool removed = false;
  for (const int64_t stamp : pending) {
    if (stamp == paired_ns && !removed) {
      removed = true;
    } else if (stamp < paired_ns) {
      older.push_back(stamp);
    } else {
      kept.push_back(stamp);
    }
  }
  pending = std::move(kept);
  for (const int64_t stamp : older) {
    report_unpaired(stream, stamp);
  }
}

void FramePairer::cap(Stream stream)
{
  std::deque<int64_t> & pending = stream == Stream::DEPTH ? pending_depth_ : pending_colour_;
  while (pending.size() > queue_size_) {
    const int64_t oldest = pending.front();
    pending.pop_front();
    report_unpaired(stream, oldest);
  }
}

void FramePairer::report_unpaired(Stream stream, int64_t stamp_ns)
{
  if (stream == Stream::COLOUR && !recent_depth_.empty()) {
    const auto nearest = std::min_element(recent_depth_.begin(), recent_depth_.end(), [stamp_ns](int64_t a, int64_t b) {
      return std::llabs(a - stamp_ns) < std::llabs(b - stamp_ns);
    });
    on_delta_(static_cast<double>(stamp_ns - *nearest) * S_PER_NS);
  }
  on_unpaired_(stream, stamp_ns);
}

}  // namespace head_rgbd_cpp
