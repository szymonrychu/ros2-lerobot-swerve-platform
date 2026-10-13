#include "stereo_depth_cpp/calibration.hpp"

#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace stereo_depth_cpp
{

namespace
{

const std::map<std::string, std::size_t> DISTORTION_LENGTHS{{"plumb_bob", 5}, {"rational_polynomial", 8}};
constexpr long NS_PER_S_INT = 1000000000L;  // NOLINT(runtime/int)

bool isclose(double a, double b, double rel_tol)
{
  return std::abs(a - b) <= rel_tol * std::max(std::abs(a), std::abs(b));
}

std::string fmt(double value, int precision)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}

YAML::Node read_yaml_mapping(const std::string & path)
{
  struct stat info
  {
  };
  if (stat(path.c_str(), &info) != 0 || !S_ISREG(info.st_mode)) {
    throw CalibrationError(path + " is missing");
  }
  YAML::Node data;
  try {
    data = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw CalibrationError(path + ": unreadable: " + e.what());
  }
  if (!data.IsMap()) {
    throw CalibrationError(path + ": expected a mapping at the top level");
  }
  return data;
}

// A {rows, cols, data} entry with the required shape (rows or cols < 0 = any); every value a finite float.
std::vector<double> matrix_data(const YAML::Node & data, const std::string & key, int rows, int cols)
{
  const YAML::Node entry = data[key];
  if (!entry.IsDefined() || !entry.IsMap() || !entry["data"].IsSequence()) {
    throw CalibrationError(key + " must be a mapping with rows, cols and data");
  }
  int entry_rows = 0;
  int entry_cols = 0;
  try {
    entry_rows = entry["rows"].as<int>();
    entry_cols = entry["cols"].as<int>();
  } catch (const YAML::Exception &) {
    throw CalibrationError(key + ": data must hold rows x cols values");
  }
  const YAML::Node values = entry["data"];
  if (entry_rows < 0 || entry_cols < 0 || values.size() != static_cast<std::size_t>(entry_rows) * entry_cols) {
    throw CalibrationError(key + ": data must hold rows x cols values");
  }
  if ((rows >= 0 && entry_rows != rows) || (cols >= 0 && entry_cols != cols)) {
    throw CalibrationError(
      key + " must be " + std::to_string(rows >= 0 ? rows : entry_rows) + "x" +
      std::to_string(cols >= 0 ? cols : entry_cols));
  }
  std::vector<double> out;
  for (const auto & value : values) {
    double v = 0.0;
    try {
      v = value.as<double>();
    } catch (const YAML::Exception &) {
      throw CalibrationError(key + ": values must be floats");
    }
    if (!std::isfinite(v)) {
      throw CalibrationError(key + ": values must be finite");
    }
    out.push_back(v);
  }
  return out;
}

template <std::size_t N>
std::array<double, N> fixed(const std::vector<double> & values)
{
  std::array<double, N> out{};
  std::copy_n(values.begin(), N, out.begin());
  return out;
}

int positive_int(const YAML::Node & data, const std::string & key)
{
  const YAML::Node value = data[key];
  if (!value.IsDefined() || value.IsNull()) {
    throw CalibrationError(key + ": field required");
  }
  int out = 0;
  try {
    out = value.as<int>();
  } catch (const YAML::Exception &) {
    throw CalibrationError(key + ": expected int");
  }
  if (out <= 0) {
    throw CalibrationError(key + ": must be > 0");
  }
  return out;
}

FileStamp file_stamp(const std::string & path)
{
  struct stat info
  {
  };
  if (stat(path.c_str(), &info) != 0) {
    return std::nullopt;
  }
  const std::int64_t mtime_ns = static_cast<std::int64_t>(info.st_mtim.tv_sec) * NS_PER_S_INT + info.st_mtim.tv_nsec;
  return std::array<std::int64_t, 3>{
    static_cast<std::int64_t>(info.st_ino), mtime_ns, static_cast<std::int64_t>(info.st_size)};
}

cv::Mat to_mat(const double * values, int rows, int cols, int stride)
{
  cv::Mat out(rows, cols, CV_64F);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      out.at<double>(r, c) = values[r * stride + c];
    }
  }
  return out;
}

void camera_maps(const CameraCalibration & camera, const cv::Size & size, cv::Mat & map1, cv::Mat & map2)
{
  const cv::Mat k = to_mat(camera.k.data(), 3, 3, 3);
  const cv::Mat r = to_mat(camera.r.data(), 3, 3, 3);
  const cv::Mat p = to_mat(camera.p.data(), 3, 3, 4);
  const cv::Mat d(camera.d, true);
  cv::initUndistortRectifyMap(k, d, r, p, size, CV_16SC2, map1, map2);
}

}  // namespace

CameraCalibration load_camera_info_yaml(const std::string & path)
{
  const YAML::Node data = read_yaml_mapping(path);
  try {
    CameraCalibration camera;
    const YAML::Node name = data["camera_name"];
    camera.camera_name = (name.IsDefined() && name.IsScalar()) ? name.as<std::string>() : "";
    const std::vector<double> d = matrix_data(data, "distortion_coefficients", 1, -1);
    const std::vector<double> k = matrix_data(data, "camera_matrix", 3, 3);
    const std::vector<double> r = matrix_data(data, "rectification_matrix", 3, 3);
    const std::vector<double> p = matrix_data(data, "projection_matrix", 3, 4);
    camera.width = positive_int(data, "image_width");
    camera.height = positive_int(data, "image_height");
    const YAML::Node model = data["distortion_model"];
    camera.distortion_model = (model.IsDefined() && model.IsScalar()) ? model.as<std::string>() : "";
    const auto expected = DISTORTION_LENGTHS.find(camera.distortion_model);
    if (expected == DISTORTION_LENGTHS.end()) {
      throw CalibrationError("distortion_model: must be 'plumb_bob' or 'rational_polynomial'");
    }
    camera.d = d;
    camera.k = fixed<9>(k);
    camera.r = fixed<9>(r);
    camera.p = fixed<12>(p);
    if (camera.k[0] <= 0.0 || camera.k[4] <= 0.0) {
      throw CalibrationError("focal lengths in K must be positive");
    }
    if (camera.p[0] <= 0.0 || camera.p[5] <= 0.0) {
      throw CalibrationError("focal lengths in P must be positive");
    }
    if (camera.d.size() != expected->second) {
      throw CalibrationError(
        "distortion_model " + camera.distortion_model + " needs " + std::to_string(expected->second) + " coefficients");
    }
    return camera;
  } catch (const CalibrationError & e) {
    throw CalibrationError(path + ": " + e.what());
  }
}

StereoExtrinsics load_stereo_extrinsics_yaml(const std::string & path)
{
  const YAML::Node data = read_yaml_mapping(path);
  try {
    StereoExtrinsics ext;
    ext.r = fixed<9>(matrix_data(data, "R", 3, 3));
    ext.t = fixed<3>(matrix_data(data, "T", 3, 1));
    ext.q = fixed<16>(matrix_data(data, "Q", 4, 4));
    ext.width = positive_int(data, "image_width");
    ext.height = positive_int(data, "image_height");
    const YAML::Node rms = data["rms"];
    if (!rms.IsDefined() || rms.IsNull()) {
      throw CalibrationError("rms: field required");
    }
    try {
      ext.rms = rms.as<double>();
    } catch (const YAML::Exception &) {
      throw CalibrationError("rms: expected float");
    }
    if (!std::isfinite(ext.rms) || ext.rms < 0.0) {
      throw CalibrationError("rms: must be a finite value >= 0");
    }
    return ext;
  } catch (const CalibrationError & e) {
    throw CalibrationError(path + ": " + e.what());
  }
}

std::string check_pair(const CameraCalibration & left, const CameraCalibration & right, const StereoExtrinsics & ext)
{
  if (
    left.width != right.width || left.width != ext.width || left.height != right.height || left.height != ext.height) {
    return "image size mismatch: left " + std::to_string(left.width) + "x" + std::to_string(left.height) + ", right " +
           std::to_string(right.width) + "x" + std::to_string(right.height) + ", stereo " + std::to_string(ext.width) +
           "x" + std::to_string(ext.height);
  }
  for (const int index : {0, 2, 5, 6}) {
    if (!isclose(left.p[index], right.p[index], RECTIFIED_REL_TOL)) {
      return "rectified left and right P must share fx, fy, cx and cy (P[" + std::to_string(index) + "] differs)";
    }
  }
  if (std::abs(left.p[3]) > LEFT_TX_ABS_TOL) {
    return "left P[3] must be 0: the left camera is the rectified reference";
  }
  if (right.p[3] >= 0.0) {
    return "right P[3] must be -fx * B < 0 (a positive baseline, right camera on the right)";
  }
  const double baseline_m = -right.p[3] / right.p[0];
  const double t_norm = std::sqrt(ext.t[0] * ext.t[0] + ext.t[1] * ext.t[1] + ext.t[2] * ext.t[2]);
  if (!isclose(baseline_m, t_norm, BASELINE_REL_TOL)) {
    return "baseline from right P (" + fmt(baseline_m, 4) + " m) does not match |T| (" + fmt(t_norm, 4) + " m)";
  }
  return "";
}

CalibrationLoad load_stereo_calibration(const std::string & directory)
{
  StereoCalibration calibration;
  try {
    calibration.left = load_camera_info_yaml(directory + "/" + LEFT_FILE);
    calibration.right = load_camera_info_yaml(directory + "/" + RIGHT_FILE);
    calibration.extrinsics = load_stereo_extrinsics_yaml(directory + "/" + STEREO_FILE);
  } catch (const CalibrationError & e) {
    return {std::nullopt, e.what()};
  }
  const std::string problem = check_pair(calibration.left, calibration.right, calibration.extrinsics);
  if (!problem.empty()) {
    return {std::nullopt, directory + ": inconsistent stereo calibration: " + problem};
  }
  return {calibration, ""};
}

RectificationMaps rectification_maps(const StereoCalibration & calibration)
{
  const cv::Size size(calibration.width(), calibration.height());
  RectificationMaps maps;
  camera_maps(calibration.left, size, maps.left_map1, maps.left_map2);
  camera_maps(calibration.right, size, maps.right_map1, maps.right_map2);
  return maps;
}

void rectify(
  const RectificationMaps & maps, const cv::Mat & left, const cv::Mat & right, cv::Mat & rect_left,
  cv::Mat & rect_right)
{
  cv::remap(left, rect_left, maps.left_map1, maps.left_map2, cv::INTER_LINEAR);
  cv::remap(right, rect_right, maps.right_map1, maps.right_map2, cv::INTER_LINEAR);
}

CalibrationSignature calibration_signature(const std::string & directory)
{
  return {
    file_stamp(directory + "/" + LEFT_FILE), file_stamp(directory + "/" + RIGHT_FILE),
    file_stamp(directory + "/" + STEREO_FILE)};
}

}  // namespace stereo_depth_cpp
