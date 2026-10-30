// Pairs the D435 aligned depth with the colour frames with message_filters (design D3 of the task: the library's
// ExactTime or ApproximateTime policy, fed directly through Synchronizer::add, no subscriber of its own) and accounts
// for every input frame exactly once: paired, or unpaired once it can no longer pair (a newer pair passed it, or more
// than queue_size newer frames of its stream arrived), or still pending. Each pair candidate feeds the stamp-delta
// callback: a pair with its colour-minus-depth stamp, an unpaired colour frame with the delta to the nearest depth
// frame seen, so a live histogram tells a few-millisecond stamp mismatch from a lost frame.
#ifndef HEAD_RGBD_CPP__PAIRING_HPP_
#define HEAD_RGBD_CPP__PAIRING_HPP_

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "head_rgbd_cpp/config.hpp"

namespace head_rgbd_cpp
{

enum class Stream
{
  DEPTH,
  COLOUR
};

class FramePairer
{
public:
  using Image = sensor_msgs::msg::Image;
  using PairCallback = std::function<void(const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & colour)>;
  using UnpairedCallback = std::function<void(Stream stream, int64_t stamp_ns)>;
  using DeltaCallback = std::function<void(double colour_minus_depth_s)>;

  // @param config the pairing policy and, for approximate, the widest stamp spread of a pair.
  // @param queue_size frames per stream the synchronizer holds (>= 2).
  // @param on_pair called with every pair, oldest first.
  // @param on_unpaired called once for every frame that will never pair.
  // @param on_delta called once per pair candidate: each pair and each unpaired colour frame with a depth frame seen.
  FramePairer(
    const PairingConfig & config, uint32_t queue_size, PairCallback on_pair, UnpairedCallback on_unpaired,
    DeltaCallback on_delta);

  // @param depth an aligned depth frame.
  void add_depth(const Image::ConstSharedPtr & depth);
  // @param colour a colour frame.
  void add_colour(const Image::ConstSharedPtr & colour);

  // @return the frames neither paired nor counted unpaired yet.
  size_t pending() const { return pending_depth_.size() + pending_colour_.size(); }

private:
  using ExactSync = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<Image, Image>>;
  using ApproximateSync = message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<Image, Image>>;

  void on_synced(const Image::ConstSharedPtr & depth, const Image::ConstSharedPtr & colour);
  // Removes paired_ns from pending and counts every older pending frame of the stream as unpaired.
  void settle(Stream stream, int64_t paired_ns);
  // Counts the oldest pending frames beyond queue_size as unpaired.
  void cap(Stream stream);
  void report_unpaired(Stream stream, int64_t stamp_ns);

  uint32_t queue_size_;
  PairCallback on_pair_;
  UnpairedCallback on_unpaired_;
  DeltaCallback on_delta_;
  std::unique_ptr<ExactSync> exact_;
  std::unique_ptr<ApproximateSync> approximate_;
  std::deque<int64_t> pending_depth_;
  std::deque<int64_t> pending_colour_;
  std::deque<int64_t> recent_depth_;  // stamps of the last 2 x queue_size depth frames, paired or not
};

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__PAIRING_HPP_
