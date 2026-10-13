#include "stereo_depth_cpp/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "stereo_depth_cpp/depth.hpp"

namespace stereo_depth_cpp
{

namespace
{

// Encoding -> (channels, cv::cvtColor code to grey; -1 when the image is grey already).
const std::map<std::string, std::pair<int, int>> GREY_CONVERSIONS{
  {"mono8", {1, -1}},
  {"bgr8", {3, cv::COLOR_BGR2GRAY}},
  {"rgb8", {3, cv::COLOR_RGB2GRAY}},
  {"bgra8", {4, cv::COLOR_BGRA2GRAY}},
  {"rgba8", {4, cv::COLOR_RGBA2GRAY}},
};

std::string size_text(const cv::Mat & image) { return std::to_string(image.cols) + "x" + std::to_string(image.rows); }

}  // namespace

const std::vector<double> & matcher_buckets()
{
  static const std::vector<double> buckets{0.01, 0.025, 0.05, 0.075, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0, 2.0};
  return buckets;
}

const std::vector<double> & skew_buckets()
{
  static const std::vector<double> buckets{0.0001, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1};
  return buckets;
}

std::optional<cv::Mat> image_to_grey(const ImageView & image)
{
  const auto conversion = GREY_CONVERSIONS.find(image.encoding);
  if (conversion == GREY_CONVERSIONS.end() || image.width == 0 || image.height == 0 || image.data == nullptr) {
    return std::nullopt;
  }
  const auto [channels, code] = conversion->second;
  const std::size_t row_bytes = static_cast<std::size_t>(image.width) * channels;
  if (image.step < row_bytes || image.size < static_cast<std::size_t>(image.height) * image.step) {
    return std::nullopt;
  }
  // A header over the message bytes (no copy), cropped to width by the column count; the output is a new image.
  const cv::Mat pixels(
    static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC(channels),
    const_cast<std::uint8_t *>(image.data), image.step);
  cv::Mat grey;
  if (code < 0) {
    grey = pixels.clone();
  } else {
    cv::cvtColor(pixels, grey, code);
  }
  return grey;
}

RateCap::RateCap(double max_rate_hz)
: period_s_(1.0 / max_rate_hz), next_due_s_(-std::numeric_limits<double>::infinity())
{
}

bool RateCap::admit(double now_s)
{
  if (now_s < next_due_s_ - RATE_JITTER_FRACTION * period_s_) {
    return false;
  }
  next_due_s_ = std::max(next_due_s_, now_s) + period_s_;
  return true;
}

StereoDepthCore::StereoDepthCore(const DepthConfig & config, ros2_common_cpp::Metrics & metrics)
: config_(config),
  metrics_(metrics),
  published_(metrics.counter(METRIC_PUBLISHED, "Depth frames published (depth and camera_info)")),
  matcher_seconds_(
    metrics.histogram(METRIC_MATCHER_SECONDS, "Rectify plus StereoSGBM wall time per pair", matcher_buckets())),
  pair_skew_seconds_(metrics.histogram(
    METRIC_PAIR_SKEW_SECONDS, "Left/right header stamp difference of each synced pair", skew_buckets())),
  calibration_loaded_(metrics.gauge(METRIC_CALIBRATION_LOADED, "1 while a valid stereo calibration is loaded")),
  rate_cap_(config.max_rate_hz)
{
  cv::setNumThreads(config_.matcher_threads);
  matcher_ = create_matcher(config_);
  calibration_loaded_.Set(0);
}

CalibrationCheck StereoDepthCore::check_calibration()
{
  const CalibrationSignature signature = calibration_signature(config_.calibration_dir);
  if (signature_ && *signature_ == signature) {
    return {false, calibration_.has_value(), ""};
  }
  signature_ = signature;
  CalibrationLoad loaded = load_stereo_calibration(config_.calibration_dir);
  if (!loaded.calibration) {
    calibration_.reset();
    maps_ = RectificationMaps{};
    reason_ = loaded.reason;
    calibration_loaded_.Set(0);
    return {true, false, "No valid stereo calibration, publishing no depth: " + reason_};
  }
  calibration_ = std::move(loaded.calibration);
  maps_ = rectification_maps(*calibration_);
  reason_.clear();
  calibration_loaded_.Set(1);
  std::ostringstream message;
  message.setf(std::ios::fixed);
  message << "Stereo calibration loaded: " << calibration_->width() << "x" << calibration_->height();
  message.precision(1);
  message << ", fx=" << calibration_->fx() << " px";
  message.precision(4);
  message << ", baseline=" << calibration_->baseline_m() << " m";
  message.precision(3);
  message << ", rms=" << calibration_->extrinsics.rms << " px";
  return {true, true, message.str()};
}

PairResult StereoDepthCore::drop(const std::string & reason, const std::string & message)
{
  metrics_.counter(METRIC_DROPPED, "Synced pairs that published nothing, by reason", {{"reason", reason}}).Increment();
  return {reason, message, {}, {}};
}

PairResult StereoDepthCore::process(const ImageView & left, const ImageView & right, double now_s)
{
  const double skew_s = static_cast<double>(std::llabs(left.stamp_ns - right.stamp_ns)) / NS_PER_S;
  pair_skew_seconds_.Observe(skew_s);
  if (skew_s > config_.sync_slop_s) {
    std::ostringstream message;
    message.setf(std::ios::fixed);
    message.precision(2);
    message << "Pair skew " << skew_s * 1000.0 << " ms is above sync_slop_s; no depth for it.";
    return drop(REASON_SKEW, message.str());
  }
  if (!calibration_) {
    return drop(REASON_NO_CALIBRATION, "No valid stereo calibration, publishing no depth: " + reason_);
  }
  if (!rate_cap_.admit(now_s)) {
    return drop(REASON_RATE_LIMIT, "");
  }
  const auto left_grey = image_to_grey(left);
  const auto right_grey = image_to_grey(right);
  if (!left_grey || !right_grey) {
    return drop(
      REASON_BAD_IMAGE, "Undecodable stereo image (encodings '" + left.encoding + "', '" + right.encoding + "').");
  }
  const cv::Size expected(calibration_->width(), calibration_->height());
  if (left_grey->size() != right_grey->size() || left_grey->size() != expected) {
    return drop(
      REASON_SIZE_MISMATCH, "Stereo image sizes left " + size_text(*left_grey) + ", right " + size_text(*right_grey) +
                              " do not match the calibration " + std::to_string(expected.width) + "x" +
                              std::to_string(expected.height) + ".");
  }
  const auto start = std::chrono::steady_clock::now();
  cv::Mat rect_left;
  cv::Mat rect_right;
  rectify(maps_, *left_grey, *right_grey, rect_left, rect_right);
  cv::Mat disparity = compute_disparity(matcher_, rect_left, rect_right);
  matcher_seconds_.Observe(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
  cv::Mat depth_mm = disparity_to_depth_mm(
    disparity, calibration_->fx(), calibration_->baseline_m(), static_cast<double>(MIN_DISPARITY), config_.min_depth_m,
    config_.max_depth_m);
  if (cv::countNonZero(depth_mm) == 0) {
    return drop(REASON_NO_VALID_PIXELS, "No pixel of the pair gave a valid depth; nothing published.");
  }
  published_.Increment();
  return {"", "", std::move(depth_mm), std::move(disparity)};
}

}  // namespace stereo_depth_cpp
