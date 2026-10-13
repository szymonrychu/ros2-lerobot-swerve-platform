// The per-pair logic of the stereo_depth node without rclcpp, so it is unit-tested: image decoding, the rate cap and
// the gates of StereoDepthNode.on_pair in nodes/stereo_depth/stereo_depth/node.py, in the same order:
//   skew > sync_slop_s -> "skew"; no valid calibration -> "no_calibration"; before the next rate slot (less
//   RATE_JITTER_FRACTION of a period) -> "rate_limit"; undecodable image -> "bad_image"; left, right and calibration
//   sizes differ -> "size_mismatch"; no valid depth pixel -> "no_valid_pixels"; otherwise depth is published.
// Every drop is counted in stereo_depth_frames_dropped_total{reason}; nothing is ever returned for publishing without
// a valid calibration and a decoded, size-checked pair. The metric names and buckets are the Python node's.
#ifndef STEREO_DEPTH_CPP__PIPELINE_HPP_
#define STEREO_DEPTH_CPP__PIPELINE_HPP_

#include <cstddef>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

#include "ros2_common_cpp/metrics.hpp"
#include "stereo_depth_cpp/calibration.hpp"
#include "stereo_depth_cpp/config.hpp"

namespace stereo_depth_cpp
{

constexpr const char * NODE_NAME = "stereo_depth";
constexpr double RATE_JITTER_FRACTION = 0.1;  // a pair this much of a period early still takes the slot
constexpr double NS_PER_S = 1e9;

constexpr const char * REASON_SKEW = "skew";
constexpr const char * REASON_NO_CALIBRATION = "no_calibration";
constexpr const char * REASON_RATE_LIMIT = "rate_limit";
constexpr const char * REASON_BAD_IMAGE = "bad_image";
constexpr const char * REASON_SIZE_MISMATCH = "size_mismatch";
constexpr const char * REASON_NO_VALID_PIXELS = "no_valid_pixels";

constexpr const char * METRIC_PUBLISHED = "stereo_depth_frames_published_total";
constexpr const char * METRIC_DROPPED = "stereo_depth_frames_dropped_total";
constexpr const char * METRIC_MATCHER_SECONDS = "stereo_depth_matcher_seconds";
constexpr const char * METRIC_PAIR_SKEW_SECONDS = "stereo_depth_pair_skew_seconds";
constexpr const char * METRIC_CALIBRATION_LOADED = "stereo_depth_calibration_loaded";

const std::vector<double> & matcher_buckets();
const std::vector<double> & skew_buckets();

// A camera Image without the message type: the fields image_to_grey and the gates need.
struct ImageView
{
  std::string encoding;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  const std::uint8_t * data = nullptr;
  std::size_t size = 0;
  std::int64_t stamp_ns = 0;
};

// Decode an 8-bit mono8, bgr8, rgb8, bgra8 or rgba8 image (rows may be padded) into a CV_8U grey cv::Mat; nullopt
// for an empty, unsupported or truncated image.
std::optional<cv::Mat> image_to_grey(const ImageView & image);

// Upper bound of the publish rate: a pair takes the next slot unless it arrives more than RATE_JITTER_FRACTION of a
// period before it; the slot then moves one period past max(slot, now).
class RateCap
{
public:
  explicit RateCap(double max_rate_hz);
  bool admit(double now_s);

private:
  double period_s_;
  double next_due_s_;
};

// What process returned: reason empty means depth_mm (CV_16U, mm, 0 = invalid) and disparity (CV_32F, px) are
// ready to publish under the left stamp; otherwise the pair was dropped and counted for reason, message says why.
struct PairResult
{
  std::string reason;
  std::string message;
  cv::Mat depth_mm;
  cv::Mat disparity;
};

// Result of check_calibration: changed is false when the files are unchanged since the last check.
struct CalibrationCheck
{
  bool changed = false;
  bool loaded = false;
  std::string message;
};

class StereoDepthCore
{
public:
  StereoDepthCore(const DepthConfig & config, ros2_common_cpp::Metrics & metrics);

  // Load the calibration when its files appeared or changed since the last check; drop it when invalid.
  CalibrationCheck check_calibration();

  // One synced pair through every gate, the rectification, the matcher and the depth conversion.
  PairResult process(const ImageView & left, const ImageView & right, double now_s);

  const std::optional<StereoCalibration> & calibration() const { return calibration_; }
  const DepthConfig & config() const { return config_; }

private:
  PairResult drop(const std::string & reason, const std::string & message);

  DepthConfig config_;
  ros2_common_cpp::Metrics & metrics_;
  prometheus::Counter & published_;
  prometheus::Histogram & matcher_seconds_;
  prometheus::Histogram & pair_skew_seconds_;
  prometheus::Gauge & calibration_loaded_;
  cv::Ptr<cv::StereoSGBM> matcher_;
  RateCap rate_cap_;
  std::optional<StereoCalibration> calibration_;
  RectificationMaps maps_;
  std::string reason_ = "not loaded yet";
  std::optional<CalibrationSignature> signature_;
};

}  // namespace stereo_depth_cpp

#endif  // STEREO_DEPTH_CPP__PIPELINE_HPP_
