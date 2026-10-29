// ros2_common_cpp depth_mask: the depth-discontinuity ("veil") mask on small hand-made 16UC1 depth images.
#include <gtest/gtest.h>

#include <cstdint>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <vector>

#include "ros2_common_cpp/depth_mask.hpp"

namespace rc = ros2_common_cpp;

namespace
{

constexpr int ROWS = 7;
constexpr int COLS = 12;
constexpr uint16_t NEAR_MM = 1000;
constexpr uint16_t FAR_MM = 1500;

// A depth image with NEAR_MM in the columns before split and FAR_MM from split on.
cv::Mat step_image(int split)
{
  cv::Mat depth(ROWS, COLS, CV_16UC1, cv::Scalar(FAR_MM));
  depth.colRange(0, split).setTo(cv::Scalar(NEAR_MM));
  return depth;
}

rc::DepthEdgeMaskParams params(int window_px = 3, double abs_m = 0.02, double rel = 0.03, int grow_px = 0)
{
  rc::DepthEdgeMaskParams p;
  p.window_px = window_px;
  p.abs_m = abs_m;
  p.rel = rel;
  p.grow_px = grow_px;
  return p;
}

// The columns of row r that the mask removes.
std::vector<int> masked_columns(const cv::Mat & mask, int r)
{
  std::vector<int> cols;
  for (int c = 0; c < mask.cols; ++c) {
    if (mask.at<uint8_t>(r, c) != 0) {
      cols.push_back(c);
    }
  }
  return cols;
}

}  // namespace

TEST(DepthEdgeMask, KeepsASmoothSurface)
{
  cv::Mat depth(ROWS, COLS, CV_16UC1);
  for (int r = 0; r < ROWS; ++r) {
    for (int c = 0; c < COLS; ++c) {
      depth.at<uint16_t>(r, c) = static_cast<uint16_t>(1000 + 20 * r + 10 * c);  // 2 % per row at most
    }
  }
  const cv::Mat mask = rc::depth_edge_mask(depth, params());
  ASSERT_EQ(mask.type(), CV_8UC1);
  ASSERT_EQ(mask.size(), depth.size());
  EXPECT_EQ(cv::countNonZero(mask), 0);
}

TEST(DepthEdgeMask, RemovesTheFarSideOfAJumpAndKeepsTheNearSide)
{
  const cv::Mat mask = rc::depth_edge_mask(step_image(5), params());
  for (int r = 0; r < ROWS; ++r) {
    EXPECT_EQ(masked_columns(mask, r), std::vector<int>({5})) << r;
  }
}

TEST(DepthEdgeMask, RemovesEveryPixelOfALinearBlendAcrossTheEdge)
{
  // 1000 | 1125 1250 1375 | 1500: a 3 px blend, steps of 125 mm (above 3 % of 1.5 m).
  cv::Mat depth = step_image(4);
  for (int i = 0; i < 3; ++i) {
    depth.col(4 + i).setTo(cv::Scalar(1125 + 125 * i));
  }
  const cv::Mat mask = rc::depth_edge_mask(depth, params());
  for (int r = 0; r < ROWS; ++r) {
    EXPECT_EQ(masked_columns(mask, r), std::vector<int>({4, 5, 6, 7})) << r;
  }
}

TEST(DepthEdgeMask, ThresholdIsTheLargerOfTheAbsoluteAndRelativeOne)
{
  // A 25 mm step at 1 m.
  cv::Mat depth(ROWS, COLS, CV_16UC1, cv::Scalar(1025));
  depth.colRange(0, 5).setTo(cv::Scalar(1000));
  EXPECT_EQ(cv::countNonZero(rc::depth_edge_mask(depth, params(3, 0.02, 0.03))), 0);     // 30.75 mm relative
  EXPECT_EQ(cv::countNonZero(rc::depth_edge_mask(depth, params(3, 0.03, 0.01))), 0);     // 30 mm absolute
  EXPECT_EQ(cv::countNonZero(rc::depth_edge_mask(depth, params(3, 0.02, 0.01))), ROWS);  // 20.5 mm
}

TEST(DepthEdgeMask, MissingDepthIsNeverANearerNeighbourAndStaysUnmasked)
{
  cv::Mat depth(ROWS, COLS, CV_16UC1, cv::Scalar(FAR_MM));
  depth.colRange(3, 6).setTo(cv::Scalar(0));
  depth.at<uint16_t>(3, 9) = 0;
  const cv::Mat mask = rc::depth_edge_mask(depth, params());
  EXPECT_EQ(cv::countNonZero(mask), 0);
}

TEST(DepthEdgeMask, AWiderWindowReachesFartherNeighbours)
{
  // 1000 | 0 | 1500: the far pixel two columns from the near one.
  cv::Mat depth = step_image(4);
  depth.col(4).setTo(cv::Scalar(0));
  EXPECT_EQ(cv::countNonZero(rc::depth_edge_mask(depth, params(3))), 0);
  const cv::Mat wide = rc::depth_edge_mask(depth, params(5));
  for (int r = 0; r < ROWS; ++r) {
    EXPECT_EQ(masked_columns(wide, r), std::vector<int>({5})) << r;
  }
}

TEST(DepthEdgeMask, GrowExtendsTheMaskIntoValidPixelsOnly)
{
  cv::Mat depth = step_image(5);
  depth.col(7).setTo(cv::Scalar(0));
  const cv::Mat mask = rc::depth_edge_mask(depth, params(3, 0.02, 0.03, 1));
  for (int r = 0; r < ROWS; ++r) {
    EXPECT_EQ(masked_columns(mask, r), std::vector<int>({4, 5, 6})) << r;
  }
}

TEST(DepthEdgeMask, ApplyZeroesTheMaskedPixelsAndCountsThem)
{
  cv::Mat depth = step_image(5);
  const int removed = rc::apply_depth_edge_mask(depth, params());
  EXPECT_EQ(removed, ROWS);
  for (int r = 0; r < ROWS; ++r) {
    EXPECT_EQ(depth.at<uint16_t>(r, 4), NEAR_MM);
    EXPECT_EQ(depth.at<uint16_t>(r, 5), 0);
    EXPECT_EQ(depth.at<uint16_t>(r, 6), FAR_MM);
  }
}

TEST(DepthEdgeMask, RejectsANon16uc1ImageAndInvalidParameters)
{
  const cv::Mat floats(ROWS, COLS, CV_32FC1, cv::Scalar(1.0));
  EXPECT_THROW(rc::depth_edge_mask(floats, params()), std::invalid_argument);
  const cv::Mat depth = step_image(5);
  EXPECT_THROW(rc::depth_edge_mask(depth, params(4)), std::invalid_argument);  // even window
  EXPECT_THROW(rc::depth_edge_mask(depth, params(1)), std::invalid_argument);  // no neighbour
  EXPECT_THROW(rc::depth_edge_mask(depth, params(3, -0.01)), std::invalid_argument);
  EXPECT_THROW(rc::depth_edge_mask(depth, params(3, 0.02, -0.1)), std::invalid_argument);
  EXPECT_THROW(rc::depth_edge_mask(depth, params(3, 0.0, 0.0)), std::invalid_argument);  // removes all noise
  EXPECT_THROW(rc::depth_edge_mask(depth, params(3, 0.02, 0.03, -1)), std::invalid_argument);
}
