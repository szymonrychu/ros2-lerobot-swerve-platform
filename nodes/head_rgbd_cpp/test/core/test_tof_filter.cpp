// Tests of the ToF flying-pixel filter on synthetic depth scenes.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <random>

#include "head_rgbd_cpp/config.hpp"
#include "head_rgbd_cpp/tof_filter.hpp"

using head_rgbd_cpp::filter_jump_edges;
using head_rgbd_cpp::FilterConfig;
using head_rgbd_cpp::FilterStats;

namespace
{
constexpr int WIDTH = 240;
constexpr int HEIGHT = 180;
constexpr double FX = 191.0;
constexpr double CX = WIDTH / 2.0;
constexpr double PI = 3.14159265358979323846;

// Plane through (0, 0, z0_m) slanted slant_deg about the vertical axis, rendered through the pinhole. Pixels where the
// plane is seen at a grazing angle (denominator < 0.2) or beyond 6 m stay invalid.
cv::Mat slanted_plane(double z0_m, double slant_deg)
{
  const double s = std::sin(slant_deg * PI / 180.0);
  const double c = std::cos(slant_deg * PI / 180.0);
  cv::Mat d = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
  for (int v = 0; v < HEIGHT; ++v) {
    for (int u = 0; u < WIDTH; ++u) {
      const double x = (u - CX) / FX;
      const double den = c + s * x;
      if (den < 0.2) {
        continue;
      }
      const double z = z0_m * c / den;
      if (z > 0.0 && z <= 6.0) {
        d.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(z * 1000.0));
      }
    }
  }
  return d;
}

double kept_fraction(const cv::Mat & before, const cv::Mat & after)
{
  const int valid = cv::countNonZero(before);
  return static_cast<double>(cv::countNonZero(after)) / valid;
}
}  // namespace

TEST(TofFilter, StepEdgeRemovesMixedColumnOnly)
{
  cv::Mat d(HEIGHT, WIDTH, CV_16UC1);
  const int step = 120;
  for (int v = 0; v < HEIGHT; ++v) {
    for (int u = 0; u < WIDTH; ++u) {
      d.at<uint16_t>(v, u) = u < step ? 1000 : (u == step ? 2000 : 3000);
    }
  }
  FilterStats stats;
  const cv::Mat out = filter_jump_edges(d, FilterConfig{}, &stats);
  for (int v = 1; v < HEIGHT - 1; ++v) {
    EXPECT_EQ(out.at<uint16_t>(v, step), 0) << "row " << v;
    for (int u = 2; u < WIDTH - 2; ++u) {
      if (std::abs(u - step) > 1) {
        EXPECT_EQ(out.at<uint16_t>(v, u), d.at<uint16_t>(v, u)) << "row " << v << " col " << u;
      }
    }
  }
  EXPECT_GT(stats.removed_jump, 0);
}

TEST(TofFilter, FlatPlanesKeepAlmostAllPixels)
{
  for (const double z : {0.5, 2.0, 4.0}) {
    const cv::Mat d = slanted_plane(z, 0.0);
    EXPECT_GE(kept_fraction(d, filter_jump_edges(d, FilterConfig{})), 0.99) << "z=" << z;
  }
}

TEST(TofFilter, SlantedPlanesKeepAlmostAllPixels)
{
  for (const double z : {0.5, 2.0, 4.0}) {
    for (const double slant : {30.0, 60.0, 70.0}) {
      const cv::Mat d = slanted_plane(z, slant);
      ASSERT_GT(cv::countNonZero(d), 1000);
      EXPECT_GE(kept_fraction(d, filter_jump_edges(d, FilterConfig{})), 0.99) << "z=" << z << " slant=" << slant;
    }
  }
}

TEST(TofFilter, IsolatedPixelIsRemoved)
{
  cv::Mat d = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
  d.at<uint16_t>(90, 120) = 1500;
  FilterStats stats;
  const cv::Mat out = filter_jump_edges(d, FilterConfig{}, &stats);
  EXPECT_EQ(cv::countNonZero(out), 0);
  EXPECT_EQ(stats.removed_isolated, 1);
  EXPECT_EQ(stats.removed_jump, 0);
}

TEST(TofFilter, AllInvalidStaysInvalid)
{
  const cv::Mat d = cv::Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
  FilterStats stats;
  const cv::Mat out = filter_jump_edges(d, FilterConfig{}, &stats);
  EXPECT_EQ(out.size(), d.size());
  EXPECT_EQ(out.type(), CV_16UC1);
  EXPECT_EQ(cv::countNonZero(out), 0);
  EXPECT_EQ(stats.removed_jump, 0);
  EXPECT_EQ(stats.removed_isolated, 0);
}

TEST(TofFilter, OnePercentNoiseRemovesUnderOnePercent)
{
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.01);
  for (const double z : {0.5, 2.0}) {
    cv::Mat d(HEIGHT, WIDTH, CV_16UC1);
    for (int v = 0; v < HEIGHT; ++v) {
      for (int u = 0; u < WIDTH; ++u) {
        d.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(z * 1000.0 * (1.0 + noise(rng))));
      }
    }
    EXPECT_GE(kept_fraction(d, filter_jump_edges(d, FilterConfig{})), 0.99) << "z=" << z;
  }
}

TEST(TofFilter, InputIsNotModified)
{
  cv::Mat d = slanted_plane(1.0, 30.0);
  d.at<uint16_t>(10, 10) = 4000;  // a flying pixel the filter will remove from the output
  const cv::Mat copy = d.clone();
  const cv::Mat out = filter_jump_edges(d, FilterConfig{});
  EXPECT_EQ(cv::countNonZero(d != copy), 0);
  EXPECT_EQ(out.at<uint16_t>(10, 10), 0);
}
