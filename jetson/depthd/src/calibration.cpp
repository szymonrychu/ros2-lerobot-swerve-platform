#include "depthd/calibration.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>

namespace depthd
{
namespace
{
struct MatSpec
{
  const char * key;
  cv::Mat StereoCalibration::*member;
  int rows;
  int cols;      // 0: any of alt_cols
  int alt_cols;  // second accepted column count (0: none)
};

const MatSpec kMatrices[] = {
  {"K_left", &StereoCalibration::K1, 3, 3, 0},   {"D_left", &StereoCalibration::D1, 1, 5, 8},
  {"K_right", &StereoCalibration::K2, 3, 3, 0},  {"D_right", &StereoCalibration::D2, 1, 5, 8},
  {"R", &StereoCalibration::R, 3, 3, 0},         {"T", &StereoCalibration::T, 3, 1, 0},
  {"R1", &StereoCalibration::R1, 3, 3, 0},       {"R2", &StereoCalibration::R2, 3, 3, 0},
  {"P1", &StereoCalibration::P1, 3, 4, 0},       {"P2", &StereoCalibration::P2, 3, 4, 0},
  {"Q", &StereoCalibration::Q, 4, 4, 0},
};

std::string shape_text(const MatSpec & s)
{
  std::ostringstream o;
  o << s.rows << "x" << s.cols;
  if (s.alt_cols) o << " or " << s.rows << "x" << s.alt_cols;
  return o.str();
}

bool shape_ok(const MatSpec & s, const cv::Mat & m)
{
  return m.rows == s.rows && (m.cols == s.cols || (s.alt_cols && m.cols == s.alt_cols));
}

bool finite(const cv::Mat & m)
{
  for (int r = 0; r < m.rows; ++r)
    for (int c = 0; c < m.cols; ++c)
      if (!std::isfinite(m.at<double>(r, c))) return false;
  return true;
}

std::string fmt(double v)
{
  std::ostringstream o;
  o << v;
  return o.str();
}

std::string read_error(const std::string & key) { return "missing key " + key; }
}  // namespace

std::string validate_calibration(const StereoCalibration & cal, double max_rms_px)
{
  if (cal.size.width <= 0 || cal.size.height <= 0) return "image_width and image_height must be positive";
  for (const auto & s : kMatrices) {
    const cv::Mat & m = cal.*(s.member);
    if (m.empty() || m.type() != CV_64F) return std::string(s.key) + " must be a double matrix";
    if (!shape_ok(s, m)) return std::string(s.key) + " must be " + shape_text(s) + ", got " + std::to_string(m.rows) + "x" + std::to_string(m.cols);
    if (!finite(m)) return std::string(s.key) + " values must be finite";
  }
  if (!std::isfinite(cal.rms) || !std::isfinite(cal.baseline_m)) return "rms and baseline_m must be finite";
  const struct
  {
    const char * key;
    const cv::Mat & m;
  } focal[] = {{"K_left", cal.K1}, {"K_right", cal.K2}, {"P1", cal.P1}, {"P2", cal.P2}};
  for (const auto & f : focal)
    if (f.m.at<double>(0, 0) <= 0.0 || f.m.at<double>(1, 1) <= 0.0)
      return std::string(f.key) + " focal length (entries (0,0) and (1,1)) must be positive";
  if (cal.rms < 0.0 || cal.rms > max_rms_px)
    return "rms_px " + fmt(cal.rms) + " is above calibration.max_rms_px " + fmt(max_rms_px);
  if (cal.baseline_m < kMinBaselineM || cal.baseline_m > kMaxBaselineM)
    return "baseline_m " + fmt(cal.baseline_m) + " is outside " + fmt(kMinBaselineM) + "-" + fmt(kMaxBaselineM) + " m";
  const double t_norm = cv::norm(cal.T);
  if (std::abs(cal.baseline_m - t_norm) > kBaselineTranslationTolerance * t_norm)
    return "baseline_m " + fmt(cal.baseline_m) + " does not match |T| " + fmt(t_norm) + " within 1 %";
  return {};
}

StereoCalibration scale_calibration(const StereoCalibration & cal, cv::Size size)
{
  const double sx = static_cast<double>(size.width) / cal.size.width;
  const double sy = static_cast<double>(size.height) / cal.size.height;
  StereoCalibration out = cal;
  out.size = size;
  for (cv::Mat * m : {&out.K1, &out.K2, &out.P1, &out.P2}) {
    *m = m->clone();
    for (int c = 0; c < m->cols; ++c) {
      m->at<double>(0, c) *= sx;
      m->at<double>(1, c) *= sy;
    }
  }
  out.Q = cal.Q.clone();
  out.Q.at<double>(0, 3) *= sx;  // -cx
  out.Q.at<double>(1, 3) *= sy;  // -cy
  out.Q.at<double>(2, 3) *= sx;  // f
  out.Q.at<double>(3, 3) *= sx;  // (cx - cx') / Tx
  return out;
}

CalibrationLoad load_calibration(const std::string & path, cv::Size processing, double max_rms_px)
{
  CalibrationLoad result;
  std::FILE * probe = std::fopen(path.c_str(), "r");
  if (!probe) {
    result.reason = "calibration file " + path + " is missing or unreadable";
    return result;
  }
  std::fclose(probe);

  StereoCalibration cal;
  try {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      result.reason = path + " is not a readable OpenCV YAML file";
      return result;
    }
    const cv::FileNode w = fs["image_width"], h = fs["image_height"], rms = fs["rms"], base = fs["baseline_m"];
    if (w.empty() || !w.isInt()) {
      result.reason = w.empty() ? read_error("image_width") : "image_width must be an integer";
      return result;
    }
    if (h.empty() || !h.isInt()) {
      result.reason = h.empty() ? read_error("image_height") : "image_height must be an integer";
      return result;
    }
    if (rms.empty() || !(rms.isReal() || rms.isInt())) {
      result.reason = rms.empty() ? read_error("rms") : "rms must be a number";
      return result;
    }
    if (base.empty() || !(base.isReal() || base.isInt())) {
      result.reason = base.empty() ? read_error("baseline_m") : "baseline_m must be a number";
      return result;
    }
    cal.size = cv::Size(static_cast<int>(w), static_cast<int>(h));
    cal.rms = static_cast<double>(rms);
    cal.baseline_m = static_cast<double>(base);
    for (const auto & s : kMatrices) {
      const cv::FileNode node = fs[s.key];
      if (node.empty()) {
        result.reason = read_error(s.key);
        return result;
      }
      cv::Mat m;
      node >> m;
      if (m.empty()) {
        result.reason = std::string(s.key) + " must be an opencv-matrix with rows, cols, dt and data";
        return result;
      }
      m.convertTo(m, CV_64F);
      cal.*(s.member) = m;
    }
  } catch (const cv::Exception & e) {
    result.reason = path + " is not valid OpenCV YAML: " + e.err;
    return result;
  }

  result.reason = validate_calibration(cal, max_rms_px);
  if (!result.reason.empty()) return result;

  if (processing.width <= 0 || processing.height <= 0) {
    result.reason = "processing size must be positive";
    return result;
  }
  const double file_aspect = static_cast<double>(cal.size.width) / cal.size.height;
  const double proc_aspect = static_cast<double>(processing.width) / processing.height;
  if (std::abs(file_aspect / proc_aspect - 1.0) > kAspectTolerance) {
    result.reason = "aspect ratio of the calibration (" + std::to_string(cal.size.width) + "x" +
                    std::to_string(cal.size.height) + ") differs from processing " + std::to_string(processing.width) +
                    "x" + std::to_string(processing.height) + " by more than 1 %";
    return result;
  }
  result.calibration = cal.size == processing ? cal : scale_calibration(cal, processing);
  return result;
}

std::string save_calibration(const std::string & path, const StereoCalibration & cal)
{
  const std::string tmp = path + ".tmp";
  try {
    cv::FileStorage fs(tmp, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return "cannot write " + tmp;
    fs << "image_width" << cal.size.width << "image_height" << cal.size.height << "rms" << cal.rms << "baseline_m"
       << cal.baseline_m;
    for (const auto & s : kMatrices) fs << s.key << cal.*(s.member);
    fs.release();
  } catch (const cv::Exception & e) {
    return "cannot write " + tmp + ": " + e.err;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) return "cannot rename " + tmp + " to " + path;
  return {};
}

}  // namespace depthd
