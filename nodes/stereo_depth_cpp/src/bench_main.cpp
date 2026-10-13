// stereo_depth_bench: CPU cost of the C++ depth path per pair, run by scripts/bench_compare.py beside the Python path.
//
//   stereo_depth_bench <job.yaml>
//
// job.yaml (written by bench_compare.py): calibration_dir, left_png, right_png (grey PNGs of the calibrated size),
// encoding (camera Image encoding the pair is sent as, e.g. bgr8), pairs, warmup and depth (a stereo_depth config
// mapping, validated like the node's). Each pair runs what the node does per Image pair: decode to grey, rectify,
// StereoSGBM, depth conversion and the 16UC1 output bytes. Prints one line:
//   cpp cpu_s_per_pair=<process CPU s> wall_s_per_pair=<s> pairs=<n> valid_pixels=<n>
// and exits 0, or prints the problem and exits 78 (bad job or config) or 1 (no depth).
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

#include "ros2_common_cpp/config.hpp"
#include "ros2_common_cpp/metrics.hpp"
#include "stereo_depth_cpp/config.hpp"
#include "stereo_depth_cpp/pipeline.hpp"

namespace
{

constexpr int EXIT_NO_DEPTH = 1;
constexpr double SLOT_STEP_S = 1000.0;  // far apart, so the rate cap admits every pair

std::vector<std::uint8_t> encode(const cv::Mat & grey, const std::string & encoding)
{
  int channels = 3;
  if (encoding == "mono8") {
    channels = 1;
  } else if (encoding == "bgra8" || encoding == "rgba8") {
    channels = 4;
  }
  std::vector<cv::Mat> planes(static_cast<std::size_t>(channels), grey);
  if (channels == 4) {
    planes[3] = cv::Mat(grey.size(), CV_8UC1, cv::Scalar(255));
  }
  cv::Mat merged;
  cv::merge(planes, merged);
  return std::vector<std::uint8_t>(merged.datastart, merged.dataend);
}

double cpu_seconds() { return static_cast<double>(std::clock()) / CLOCKS_PER_SEC; }

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: stereo_depth_bench <job.yaml>\n";
    return ros2_common_cpp::EXIT_CONFIG;
  }
  YAML::Node job;
  try {
    job = YAML::LoadFile(argv[1]);
  } catch (const YAML::Exception & e) {
    std::cerr << "bench job " << argv[1] << ": " << e.what() << "\n";
    return ros2_common_cpp::EXIT_CONFIG;
  }
  ros2_common_cpp::Errors errors;
  ros2_common_cpp::forbid_unknown(
    job, {"calibration_dir", "left_png", "right_png", "encoding", "pairs", "warmup", "depth"}, errors);
  const auto calibration_dir = ros2_common_cpp::require<std::string>(job, "calibration_dir", errors);
  const auto left_png = ros2_common_cpp::require<std::string>(job, "left_png", errors);
  const auto right_png = ros2_common_cpp::require<std::string>(job, "right_png", errors);
  const auto encoding = ros2_common_cpp::get_or<std::string>(job, "encoding", "bgr8", errors);
  const auto pairs = ros2_common_cpp::require<int>(job, "pairs", errors);
  const auto warmup = ros2_common_cpp::get_or<int>(job, "warmup", 1, errors);
  if (pairs) {
    ros2_common_cpp::ge("pairs", *pairs, 1, errors);
  }
  ros2_common_cpp::ge("warmup", warmup, 0, errors);
  const YAML::Node depth_node = job["depth"].IsDefined() ? job["depth"] : YAML::Node(YAML::NodeType::Map);
  for (const auto & error : stereo_depth_cpp::validate(depth_node)) {
    errors.push_back("depth." + error);
  }
  if (!errors.empty()) {
    for (const auto & error : errors) {
      std::cerr << "bench job " << argv[1] << ": " << error << "\n";
    }
    return ros2_common_cpp::EXIT_CONFIG;
  }
  stereo_depth_cpp::DepthConfig config = stereo_depth_cpp::from_yaml(depth_node);
  config.calibration_dir = *calibration_dir;

  const cv::Mat left = cv::imread(*left_png, cv::IMREAD_GRAYSCALE);
  const cv::Mat right = cv::imread(*right_png, cv::IMREAD_GRAYSCALE);
  if (left.empty() || right.empty()) {
    std::cerr << "cannot read " << *left_png << " or " << *right_png << "\n";
    return ros2_common_cpp::EXIT_CONFIG;
  }
  const std::vector<std::uint8_t> left_bytes = encode(left, encoding);
  const std::vector<std::uint8_t> right_bytes = encode(right, encoding);
  const auto channels = static_cast<std::uint32_t>(left_bytes.size() / left.total());
  const stereo_depth_cpp::ImageView left_view{
    encoding,
    static_cast<std::uint32_t>(left.cols),
    static_cast<std::uint32_t>(left.rows),
    static_cast<std::uint32_t>(left.cols) * channels,
    left_bytes.data(),
    left_bytes.size(),
    0};
  stereo_depth_cpp::ImageView right_view = left_view;
  right_view.data = right_bytes.data();
  right_view.size = right_bytes.size();

  ros2_common_cpp::Metrics metrics(stereo_depth_cpp::NODE_NAME);
  stereo_depth_cpp::StereoDepthCore core(config, metrics);
  const auto check = core.check_calibration();
  if (!check.loaded) {
    std::cerr << check.message << "\n";
    return ros2_common_cpp::EXIT_CONFIG;
  }
  std::vector<std::uint8_t> output;
  int valid_pixels = 0;
  double now_s = 0.0;
  const auto run = [&]() {
    const auto result = core.process(left_view, right_view, now_s);
    now_s += SLOT_STEP_S;
    if (!result.reason.empty()) {
      return false;
    }
    output.assign(result.depth_mm.datastart, result.depth_mm.dataend);  // the 16UC1 message bytes
    valid_pixels = cv::countNonZero(result.depth_mm);
    return true;
  };
  for (int i = 0; i < warmup; ++i) {
    run();
  }
  const double cpu_start = cpu_seconds();
  const auto wall_start = std::chrono::steady_clock::now();
  for (int i = 0; i < *pairs; ++i) {
    if (!run()) {
      std::cerr << "pair " << i << " published no depth\n";
      return EXIT_NO_DEPTH;
    }
  }
  const double cpu_s = cpu_seconds() - cpu_start;
  const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  std::printf(
    "cpp cpu_s_per_pair=%.6f wall_s_per_pair=%.6f pairs=%d valid_pixels=%d\n", cpu_s / *pairs, wall_s / *pairs, *pairs,
    valid_pixels);
  return 0;
}
