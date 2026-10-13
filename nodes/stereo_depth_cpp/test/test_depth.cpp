// Disparity to depth (mm) conversion and the StereoSGBM matcher: the cases of nodes/stereo_depth/tests/test_depth.py
// plus parity with the Python outputs in test/fixtures (conversion.yaml and each case's Python disparity and depth).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "fixtures.hpp"
#include "stereo_depth_cpp/depth.hpp"

namespace
{

using stereo_depth_cpp::DepthConfig;

constexpr double FX = 400.0;         // conftest FX
constexpr double BASELINE_M = 0.08;  // conftest BASELINE_M
constexpr int SHIFT_PX = 20;

cv::Mat row(const std::vector<double> & values)
{
  cv::Mat m(1, static_cast<int>(values.size()), CV_32F);
  for (std::size_t i = 0; i < values.size(); ++i) {
    m.at<float>(0, static_cast<int>(i)) = static_cast<float>(values[i]);
  }
  return m;
}

std::vector<int> as_ints(const cv::Mat & depth)
{
  std::vector<int> out;
  for (int x = 0; x < depth.cols; ++x) {
    out.push_back(depth.at<std::uint16_t>(0, x));
  }
  return out;
}

// The Python fixed-point disparity of a case as CV_32F pixels.
cv::Mat python_disparity(const std::string & case_name)
{
  const int offset = fixtures::yaml("cases.yaml")["fixed_point_offset"].as<int>();
  cv::Mat stored = fixtures::png(case_name + "/disparity_fp.png");
  cv::Mat fixed;
  stored.convertTo(fixed, CV_32S);
  fixed -= offset;
  cv::Mat disparity;
  fixed.convertTo(disparity, CV_32F, 1.0 / stereo_depth_cpp::DISPARITY_SCALE);
  return disparity;
}

cv::Mat sgbm_depth(const cv::Mat & left, const cv::Mat & right, const DepthConfig & config = DepthConfig{})
{
  const auto matcher = stereo_depth_cpp::create_matcher(config);
  const cv::Mat disparity = stereo_depth_cpp::compute_disparity(matcher, left, right);
  return stereo_depth_cpp::disparity_to_depth_mm(
    disparity, FX, BASELINE_M, 0.0, config.min_depth_m, config.max_depth_m);
}

}  // namespace

TEST(Depth, KnownDisparityGivesExactMillimetres)
{
  const cv::Mat out = stereo_depth_cpp::disparity_to_depth_mm(row({40.0, 20.0, 10.0}), 400.0, 0.1, 0.0, 0.2, 4.0);
  EXPECT_EQ(out.type(), CV_16UC1);
  EXPECT_EQ(as_ints(out), (std::vector<int>{1000, 2000, 4000}));
}

TEST(Depth, NanInfNonPositiveAndRangesAreNoReading)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    as_ints(stereo_depth_cpp::disparity_to_depth_mm(row({nan, inf, -inf, 40.0}), 400.0, 0.1, 0.0, 0.2, 4.0)),
    (std::vector<int>{0, 0, 0, 1000}));
  EXPECT_EQ(
    as_ints(stereo_depth_cpp::disparity_to_depth_mm(row({0.0, -1.0, 5.0, 40.0}), 400.0, 0.1, 8.0, 0.2, 4.0)),
    (std::vector<int>{0, 0, 0, 1000}));
  EXPECT_EQ(
    as_ints(stereo_depth_cpp::disparity_to_depth_mm(row({400.0, 200.0, 9.0, 10.0}), 400.0, 0.1, 0.0, 0.2, 4.0)),
    (std::vector<int>{0, 200, 0, 4000}));
}

TEST(Depth, ClampedToUint16AndShapePreserved)
{
  EXPECT_EQ(
    as_ints(stereo_depth_cpp::disparity_to_depth_mm(row({0.5}), 400.0, 0.1, 0.0, 0.2, 1000.0)),
    (std::vector<int>{65535}));
  const cv::Mat out = stereo_depth_cpp::disparity_to_depth_mm(cv::Mat(3, 2, CV_32F, 40.0f), 400.0, 0.1, 0.0, 0.2, 4.0);
  EXPECT_EQ(out.rows, 3);
  EXPECT_EQ(out.cols, 2);
}

TEST(DepthParity, ConversionRowsMatchPython)
{
  const YAML::Node rows = fixtures::yaml("conversion.yaml");
  ASSERT_GT(rows.size(), 5u);
  for (const auto & r : rows) {
    const cv::Mat out = stereo_depth_cpp::disparity_to_depth_mm(
      row(r["disparity"].as<std::vector<double>>()), r["f"].as<double>(), r["t"].as<double>(),
      r["min_disparity"].as<double>(), r["min_depth_m"].as<double>(), r["max_depth_m"].as<double>());
    EXPECT_EQ(as_ints(out), r["depth_mm"].as<std::vector<int>>());
  }
}

TEST(DepthParity, PythonDisparityGivesThePythonDepth)
{
  const YAML::Node cases = fixtures::yaml("cases.yaml")["cases"];
  for (const char * name : {"ideal", "distorted"}) {
    SCOPED_TRACE(name);
    const YAML::Node c = cases[name];
    const cv::Mat depth = stereo_depth_cpp::disparity_to_depth_mm(
      python_disparity(name), c["fx"].as<double>(), c["baseline_m"].as<double>(), 0.0, c["min_depth_m"].as<double>(),
      c["max_depth_m"].as<double>());
    const cv::Mat expected = fixtures::png(std::string(name) + "/depth_mm.png");
    ASSERT_EQ(expected.type(), CV_16UC1);
    EXPECT_EQ(cv::countNonZero(depth != expected), 0);
    EXPECT_EQ(cv::countNonZero(depth), c["valid_pixels"].as<int>());
  }
}

TEST(Matcher, UsesTheConfig)
{
  DepthConfig config;
  config.num_disparities = 64;
  config.block_size = 7;
  config.p1 = 100;
  config.p2 = 400;
  config.uniqueness_ratio = 5;
  const auto matcher = stereo_depth_cpp::create_matcher(config);
  EXPECT_EQ(matcher->getNumDisparities(), 64);
  EXPECT_EQ(matcher->getBlockSize(), 7);
  EXPECT_EQ(matcher->getP1(), 100);
  EXPECT_EQ(matcher->getP2(), 400);
  EXPECT_EQ(matcher->getUniquenessRatio(), 5);
  EXPECT_EQ(matcher->getMinDisparity(), 0);
  EXPECT_EQ(matcher->getMode(), cv::StereoSGBM::MODE_SGBM_3WAY);
  EXPECT_EQ(matcher->getSpeckleWindowSize(), 100);
  EXPECT_EQ(matcher->getSpeckleRange(), 2);
  EXPECT_EQ(matcher->getDisp12MaxDiff(), 1);
  EXPECT_EQ(matcher->getPreFilterCap(), 31);
}

TEST(Matcher, KnownShiftGivesDepthWithinTwoPercent)
{
  const cv::Mat depth = sgbm_depth(fixtures::png("left.png"), fixtures::png("right.png"));
  const double expected_mm = FX * BASELINE_M / SHIFT_PX * 1000.0;
  const cv::Mat interior = depth(cv::Range(10, depth.rows - 10), cv::Range(48 + 10, depth.cols - 10));
  int valid = 0;
  int close = 0;
  for (int y = 0; y < interior.rows; ++y) {
    for (int x = 0; x < interior.cols; ++x) {
      const int v = interior.at<std::uint16_t>(y, x);
      if (v > 0) {
        ++valid;
        close += std::abs(v - expected_mm) / expected_mm < 0.02 ? 1 : 0;
      }
    }
  }
  EXPECT_GT(valid, 0.9 * static_cast<double>(interior.total()));
  EXPECT_GT(close, 0.95 * valid);
}

TEST(Matcher, ComputeDisparityIsInPixels)
{
  const cv::Mat disparity = stereo_depth_cpp::compute_disparity(
    stereo_depth_cpp::create_matcher(DepthConfig{}), fixtures::png("left.png"), fixtures::png("right.png"));
  EXPECT_EQ(disparity.type(), CV_32FC1);
  EXPECT_EQ(disparity.size(), fixtures::png("left.png").size());
  const cv::Mat interior = disparity(cv::Range(10, disparity.rows - 10), cv::Range(60, disparity.cols - 10));
  std::vector<float> values(interior.begin<float>(), interior.end<float>());
  std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
  EXPECT_NEAR(values[values.size() / 2], SHIFT_PX, 0.25);
}

TEST(Matcher, LeftBorderWithoutAMatchIsZero)
{
  const cv::Mat depth = sgbm_depth(fixtures::png("left.png"), fixtures::png("right.png"));
  EXPECT_EQ(cv::countNonZero(depth(cv::Range::all(), cv::Range(0, DepthConfig{}.num_disparities - 1))), 0);
}

TEST(Matcher, TexturelessPairIsAllZero)
{
  const cv::Mat flat(360, 480, CV_8UC1, cv::Scalar(128));
  EXPECT_EQ(cv::countNonZero(sgbm_depth(flat, flat.clone())), 0);
}

TEST(Matcher, OutOfRangeDepthIsZero)
{
  const cv::Mat left = fixtures::png("left.png");
  const cv::Mat right = fixtures::png("right.png");
  DepthConfig near;
  near.max_depth_m = 1.0;
  EXPECT_EQ(cv::countNonZero(sgbm_depth(left, right, near)), 0);
  DepthConfig far;
  far.min_depth_m = 2.0;
  far.max_depth_m = 4.0;
  EXPECT_EQ(cv::countNonZero(sgbm_depth(left, right, far)), 0);
}
