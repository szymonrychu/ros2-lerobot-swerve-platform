// GPU rectification: remap maps are built once when the calibration loads and rectification is one
// cv::cuda::remap (linear) per camera. A missing or invalid calibration leaves the rectifier `uncalibrated`
// (reason kept for /health) and rectify() produces nothing, so no depth is computed without a checked calibration.
#ifndef DEPTHD_RECTIFY_HPP_
#define DEPTHD_RECTIFY_HPP_

#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <string>

#include "depthd/calibration.hpp"

namespace depthd
{

// An immutable calibrated state: the scaled calibration and its GPU maps.
class Rectification
{
public:
  explicit Rectification(StereoCalibration calibration);

  const StereoCalibration & calibration() const { return cal_; }
  const cv::Mat & left_P() const { return cal_.P1; }  // rectified left projection (camera_info P)
  const cv::Mat & left_K() const { return cal_.K1; }  // unrectified left K (camera_info K)
  const cv::Mat & left_D() const { return cal_.D1; }  // unrectified left D (camera_info D)
  const cv::Mat & Q() const { return cal_.Q; }
  double baseline_m() const { return cal_.baseline_m; }
  cv::Size size() const { return cal_.size; }

  // Rectifies CV_8UC1 images of size() into left_out/right_out (created when needed).
  void rectify(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right, cv::cuda::GpuMat & left_out,
               cv::cuda::GpuMat & right_out, cv::cuda::Stream & stream = cv::cuda::Stream::Null()) const;

private:
  StereoCalibration cal_;
  cv::cuda::GpuMat left_x_, left_y_, right_x_, right_y_;  // CV_32FC1 maps
};

class Rectifier
{
public:
  // Loads `path` for a processing size of `size`; failure leaves the rectifier uncalibrated.
  Rectifier(std::string path, double max_rms_px, cv::Size size);

  // Reloads the file and swaps the maps atomically. An invalid or missing file makes the rectifier uncalibrated
  // (the file on disk is what depthd must not trust). Returns calibrated().
  bool reload();

  bool calibrated() const { return snapshot() != nullptr; }
  std::string state() const { return calibrated() ? "calibrated" : "uncalibrated"; }
  std::string reason() const;  // empty when calibrated

  // The current calibrated state, or null; hold it for a consistent K, P, baseline and maps.
  std::shared_ptr<const Rectification> snapshot() const;

  // Rectifies a pair. Returns false and writes nothing when uncalibrated or when an input is not CV_8UC1 of the
  // processing size.
  bool rectify(const cv::cuda::GpuMat & left, const cv::cuda::GpuMat & right, cv::cuda::GpuMat & left_out,
               cv::cuda::GpuMat & right_out, cv::cuda::Stream & stream = cv::cuda::Stream::Null()) const;

private:
  struct State
  {
    std::shared_ptr<const Rectification> model;
    std::string reason;
  };
  std::shared_ptr<const State> state_ptr() const;

  std::string path_;
  double max_rms_px_;
  cv::Size size_;
  std::mutex reload_mutex_;
  std::shared_ptr<const State> state_;  // accessed with std::atomic_load / atomic_store
};

}  // namespace depthd

#endif  // DEPTHD_RECTIFY_HPP_
