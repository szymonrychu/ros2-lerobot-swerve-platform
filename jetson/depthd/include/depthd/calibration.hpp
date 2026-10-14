// Stereo calibration file: the one definition that the calibration tool (T20, Python cv2.FileStorage) writes and
// depthd reads. The file is OpenCV FileStorage YAML at calibration.path (default /var/lib/depthd/stereo.yaml). It
// mirrors the R, T, Q, rms of the Pi-era stereo_depth/calibration.py and adds the per-camera intrinsics and the
// rectification outputs of cv::stereoRectify (alpha 0), so depthd builds its remap maps without solving anything.
//
// Keys (every key is required; a matrix is an `!!opencv-matrix` with rows, cols, dt: d, data):
//
//   key            type    shape  meaning
//   image_width    int     -      width of the images the calibration was recorded at, pixels
//   image_height   int     -      height of those images, pixels
//   rms            real    -      stereo reprojection RMS from cv::stereoCalibrate, pixels
//   baseline_m     real    -      camera separation, metres; must be 0.03..0.12 and equal |T| within 1 %
//   K_left         double  3x3    left camera matrix [fx 0 cx; 0 fy cy; 0 0 1], unrectified
//   D_left         double  1x5    left distortion (k1 k2 p1 p2 k3); 1x8 (rational model) is also accepted
//   K_right        double  3x3    right camera matrix, unrectified
//   D_right        double  1x5    right distortion (1x8 also accepted)
//   R              double  3x3    rotation of the right camera relative to the left (cv::stereoCalibrate)
//   T              double  3x1    translation of the right camera relative to the left, metres (T[0] < 0)
//   R1, R2         double  3x3    rectification rotations of the left and right camera (cv::stereoRectify)
//   P1, P2         double  3x4    rectified projections; P2(0,3) = -fx * baseline_m (cv::stereoRectify)
//   Q              double  4x4    disparity-to-depth matrix (cv::stereoRectify)
//
// Validation (load_calibration, first failure wins, the reason names the key):
//   a missing key or a wrong matrix shape; a non-finite value; a non-positive focal length (K and P entries
//   (0,0) and (1,1)); rms above calibration.max_rms_px; baseline_m outside 0.03-0.12 m or differing from |T| by
//   more than 1 %; an image aspect ratio differing from the processing size's by more than 1 %.
//
// A valid file recorded at another resolution with the same aspect is scaled to the processing size: K and P rows 0
// and 1, and Q column 3, scale by processing_width / image_width (x terms and the focal length) and
// processing_height / image_height (y terms); D, R, T, R1, R2 and baseline_m do not change.
#ifndef DEPTHD_CALIBRATION_HPP_
#define DEPTHD_CALIBRATION_HPP_

#include <opencv2/core.hpp>
#include <optional>
#include <string>

namespace depthd
{

constexpr double kMinBaselineM = 0.03;
constexpr double kMaxBaselineM = 0.12;
constexpr double kBaselineTranslationTolerance = 0.01;  // |baseline_m - |T|| / |T|
constexpr double kAspectTolerance = 0.01;

// A calibration; every matrix is CV_64F. After load_calibration, size is the processing size.
struct StereoCalibration
{
  cv::Size size;
  double rms = 0.0;
  double baseline_m = 0.0;
  cv::Mat K1, D1, K2, D2, R, T, R1, R2, P1, P2, Q;  // K_left, D_left, K_right, D_right, R, T, R1, R2, P1, P2, Q
};

struct CalibrationLoad
{
  std::optional<StereoCalibration> calibration;  // set only when the file is valid
  std::string reason;                            // why it is not; empty when valid
  bool ok() const { return calibration.has_value(); }
};

// Empty string when `cal` is valid, else the first violation naming its key. Does not check the processing size.
std::string validate_calibration(const StereoCalibration & cal, double max_rms_px);

// Returns `cal` scaled to `size` (see the header comment). The aspect ratios must already agree.
StereoCalibration scale_calibration(const StereoCalibration & cal, cv::Size size);

// Reads, validates and (when recorded at another resolution) scales the file to `processing`.
CalibrationLoad load_calibration(const std::string & path, cv::Size processing, double max_rms_px);

// Writes `cal` in the file format atomically (temp file + rename). Returns an error string, empty on success.
std::string save_calibration(const std::string & path, const StereoCalibration & cal);

}  // namespace depthd

#endif  // DEPTHD_CALIBRATION_HPP_
