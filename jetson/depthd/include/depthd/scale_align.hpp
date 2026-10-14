// Monocular depth alignment (design D5): the network's relative inverse depth has an unknown scale and shift, so
// every mono frame is fitted to the stereo frame it was run on, and only the pixels stereo left invalid are filled.
//
//   stereo inverse depth y = 1000 / depth_mm  (1/m, valid stereo pixels only: LR-checked and post-processed, != 0)
//   model                y ~= scale * mono + shift
//
// The fit is RANSAC over two-point hypotheses (scale > 0 only) on at most max_samples evenly strided valid pixels,
// then a least-squares refit on the consensus set. An inlier has |scale * mono + shift - y| <= inlier_rel_tol * y.
// Fewer than min_inliers inliers (or too few valid stereo pixels, or a scene without depth variation to scale
// against) is no fit, and no fit means no fused frame: nothing is guessed.
#ifndef DEPTHD_SCALE_ALIGN_HPP_
#define DEPTHD_SCALE_ALIGN_HPP_

#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace depthd
{

struct AlignParams
{
  int min_inliers = 500;          // fewer inliers: no fused frame
  int iterations = 64;            // RANSAC hypotheses
  double inlier_rel_tol = 0.08;   // relative inverse-depth residual of an inlier
  int max_samples = 20000;        // valid stereo pixels used by the fit (evenly strided)
  double min_y_spread = 0.02;     // relative spread (p90 - p10) / median of y below which the scale is unobservable
  double min_depth_m = 0.2;       // filled depths outside [min, max] stay 0 (invalid), as stereo does
  double max_depth_m = 6.0;
  uint32_t seed = 0x5eed;
};

struct ScaleShift
{
  bool ok = false;
  double scale = 0.0;
  double shift = 0.0;
  int inliers = 0;
  int samples = 0;
  std::string reason;  // why ok is false
};

// Fits target ~= scale * mono + shift over paired samples (same length). Non-finite pairs are ignored.
ScaleShift fit_scale_shift(const std::vector<float> & mono, const std::vector<float> & target, const AlignParams & p);

struct FillResult
{
  ScaleShift fit;
  std::size_t holes = 0;   // stereo-invalid pixels (0)
  std::size_t filled = 0;  // holes that received an in-range aligned mono depth
  double fill_ratio() const { return holes ? static_cast<double>(filled) / static_cast<double>(holes) : 0.0; }
};

// mono_inv: CV_32FC1 relative inverse depth; stereo_mm: CV_16UC1 depth in mm, 0 = invalid; same size. On a fit,
// `fused` (resized to the image) is stereo_mm where stereo is valid, byte for byte, and the aligned mono depth in mm
// where stereo is 0 (0 where that is out of range or not finite). Without a fit `fused` is left untouched.
FillResult align_and_fill(
  const cv::Mat & mono_inv, const cv::Mat & stereo_mm, const AlignParams & p, std::vector<uint16_t> & fused);

}  // namespace depthd

#endif  // DEPTHD_SCALE_ALIGN_HPP_
