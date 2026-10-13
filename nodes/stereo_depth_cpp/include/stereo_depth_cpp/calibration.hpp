// Stereo calibration files (reader side): the C++ mirror of nodes/stereo_depth/stereo_depth/calibration.py.
// One directory holds stereo_left.yaml and stereo_right.yaml (ROS camera_info YAML) and stereo.yaml (R, T, Q, rms).
// The same validation as the pydantic models: positive sizes, matrix shapes, finite values, positive focal lengths,
// the coefficient count of the distortion model, one image size for all three files, a rectified pair sharing
// fx, fy, cx and cy, left P[3] = 0, right P[3] < 0 and a baseline (-P_right[3] / P_right[0]) within 2 % of |T|.
#ifndef STEREO_DEPTH_CPP__CALIBRATION_HPP_
#define STEREO_DEPTH_CPP__CALIBRATION_HPP_

#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace stereo_depth_cpp
{

constexpr const char * LEFT_FILE = "stereo_left.yaml";
constexpr const char * RIGHT_FILE = "stereo_right.yaml";
constexpr const char * STEREO_FILE = "stereo.yaml";
constexpr double RECTIFIED_REL_TOL = 1e-6;
constexpr double LEFT_TX_ABS_TOL = 1e-6;
constexpr double BASELINE_REL_TOL = 0.02;

// One camera of the pair (a ROS camera_info YAML); matrices row-major.
struct CameraCalibration
{
  std::string camera_name;
  int width = 0;
  int height = 0;
  std::string distortion_model;
  std::vector<double> d;
  std::array<double, 9> k{};
  std::array<double, 9> r{};
  std::array<double, 12> p{};
};

// stereo.yaml.
struct StereoExtrinsics
{
  int width = 0;
  int height = 0;
  std::array<double, 9> r{};
  std::array<double, 3> t{};
  std::array<double, 16> q{};
  double rms = 0.0;
};

// The validated pair.
struct StereoCalibration
{
  CameraCalibration left;
  CameraCalibration right;
  StereoExtrinsics extrinsics;

  double fx() const { return left.p[0]; }
  double baseline_m() const { return -right.p[3] / right.p[0]; }
  int width() const { return left.width; }
  int height() const { return left.height; }
};

// Result of load_stereo_calibration: the pair, or nullopt and why.
struct CalibrationLoad
{
  std::optional<StereoCalibration> calibration;
  std::string reason;
};

// Fixed-point (CV_16SC2 + CV_16UC1) remap tables of both cameras.
struct RectificationMaps
{
  cv::Mat left_map1;
  cv::Mat left_map2;
  cv::Mat right_map1;
  cv::Mat right_map2;
};

// Thrown by the single-file loaders; the message names the file and the reason.
struct CalibrationError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

// Load and validate one ROS camera_info YAML. Throws CalibrationError.
CameraCalibration load_camera_info_yaml(const std::string & path);

// Load and validate stereo.yaml. Throws CalibrationError.
StereoExtrinsics load_stereo_extrinsics_yaml(const std::string & path);

// Why the three files do not form a valid rectified pair; empty when they do.
std::string check_pair(const CameraCalibration & left, const CameraCalibration & right, const StereoExtrinsics & ext);

// Load and validate the three files of the pair; never throws.
CalibrationLoad load_stereo_calibration(const std::string & directory);

// Both cameras' undistort-rectify maps for the calibrated size (cv::initUndistortRectifyMap, CV_16SC2).
RectificationMaps rectification_maps(const StereoCalibration & calibration);

// Rectify a raw pair with the prebuilt maps (cv::remap, INTER_LINEAR).
void rectify(
  const RectificationMaps & maps, const cv::Mat & left, const cv::Mat & right, cv::Mat & rect_left,
  cv::Mat & rect_right);

// (inode, mtime ns, size) of one file, or nullopt when it is missing.
using FileStamp = std::optional<std::array<std::int64_t, 3>>;
// One FileStamp per calibration file (left, right, stereo), to detect a new or changed calibration cheaply.
using CalibrationSignature = std::array<FileStamp, 3>;

CalibrationSignature calibration_signature(const std::string & directory);

}  // namespace stereo_depth_cpp

#endif  // STEREO_DEPTH_CPP__CALIBRATION_HPP_
