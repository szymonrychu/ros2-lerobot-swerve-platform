#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

#include "ground_estimator_cpp/edge_points.hpp"

namespace ge = ground_estimator_cpp;

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

constexpr double BOX_FRONT_X_M = 1.00;  // true front edge of the box (the side nearest the robot)
constexpr double BOX_BACK_X_M = 1.05;
constexpr double BOX_HALF_Y_M = 0.10;
constexpr double BOX_H_M = 0.05;
constexpr double STEP_M = 0.005;
constexpr double EDGE_TOL_M = 0.01;

ge::EdgeGuidanceConfig test_config()
{
  ge::EdgeGuidanceConfig c;
  c.voxel_m = 0.01;
  return c;
}

// Dense guidance points on a box top, from x0 to x1.
void add_top(Cloud & pts, std::vector<float> & h, double x0, double x1)
{
  for (double x = x0; x <= x1 + 1e-9; x += STEP_M) {
    for (double y = -BOX_HALF_Y_M; y <= BOX_HALF_Y_M + 1e-9; y += STEP_M) {
      pts.push_back(pcl::PointXYZ(x, y, BOX_H_M));
      h.push_back(BOX_H_M);
    }
  }
}

// Coarse lethal points on a 5 cm grid that start 4 cm behind the true front edge.
Cloud coarse_box()
{
  Cloud c;
  for (double x = BOX_FRONT_X_M + 0.04; x <= BOX_BACK_X_M + 1e-9; x += 0.05) {
    for (double y = -BOX_HALF_Y_M; y <= BOX_HALF_Y_M + 1e-9; y += 0.05) {
      c.push_back(pcl::PointXYZ(x, y, BOX_H_M));
    }
  }
  return c;
}

}  // namespace

TEST(EdgePoints, CoarseBoxIsFilledToItsTrueEdge)
{
  Cloud pts;
  std::vector<float> h;
  add_top(pts, h, BOX_FRONT_X_M, BOX_BACK_X_M);
  const auto e = ge::edge_points(pts, h, coarse_box(), Cloud(), test_config());
  ASSERT_FALSE(e.obstacle.empty());
  EXPECT_TRUE(e.drop.empty());
  double min_x = 1e9;
  for (const auto & p : e.obstacle) {
    min_x = std::min<double>(min_x, p.x);
  }
  EXPECT_NEAR(min_x, BOX_FRONT_X_M, EDGE_TOL_M);
}

TEST(EdgePoints, NothingWithoutAStableLethalPoint)
{
  Cloud pts;
  std::vector<float> h;
  add_top(pts, h, BOX_FRONT_X_M, BOX_BACK_X_M);
  const auto e = ge::edge_points(pts, h, Cloud(), Cloud(), test_config());
  EXPECT_TRUE(e.obstacle.empty());
  EXPECT_TRUE(e.drop.empty());
}

TEST(EdgePoints, TexturedFlatFloorAddsNothing)
{
  Cloud pts;
  std::vector<float> h;
  add_top(pts, h, BOX_FRONT_X_M, BOX_BACK_X_M);
  const std::size_t box_n = pts.size();
  // A textured floor with +-1 cm noise in front of the box, 2 cm to 20 cm from it.
  unsigned s = 12345;
  for (double x = BOX_FRONT_X_M - 0.20; x < BOX_FRONT_X_M - 0.02; x += STEP_M) {
    for (double y = -BOX_HALF_Y_M; y <= BOX_HALF_Y_M; y += STEP_M) {
      s = s * 1664525U + 1013904223U;
      const float noise = (static_cast<float>(s >> 8) / static_cast<float>(1 << 24) - 0.5F) * 0.02F;
      pts.push_back(pcl::PointXYZ(x, y, noise));
      h.push_back(noise);
    }
  }
  const auto e = ge::edge_points(pts, h, coarse_box(), Cloud(), test_config());
  ASSERT_FALSE(e.obstacle.empty());
  for (const auto & p : e.obstacle) {
    EXPECT_GE(p.z, BOX_H_M - 1e-6) << "floor point at x=" << p.x;
  }
  (void)box_n;
}

TEST(EdgePoints, ShiftLimitIsRespected)
{
  const auto cfg = test_config();
  Cloud stable;
  stable.push_back(pcl::PointXYZ(1.0F, 0.0F, 0.05F));
  Cloud pts;
  std::vector<float> h;
  pts.push_back(pcl::PointXYZ(static_cast<float>(1.0 - cfg.max_shift_m - 0.01), 0.0F, 0.05F));
  h.push_back(0.05F);
  auto e = ge::edge_points(pts, h, stable, Cloud(), cfg);
  EXPECT_TRUE(e.obstacle.empty());
  pts[0].x = static_cast<float>(1.0 - cfg.max_shift_m + 0.01);
  e = ge::edge_points(pts, h, stable, Cloud(), cfg);
  EXPECT_EQ(e.obstacle.size(), 1U);
}

TEST(EdgePoints, DropLipIsRefined)
{
  const auto cfg = test_config();
  // The drop region starts at x = 1.0; the stable drop points start 4 cm inside it.
  Cloud stable;
  for (double x = 1.04; x <= 1.30; x += 0.05) {
    stable.push_back(pcl::PointXYZ(x, 0.0F, -0.10F));
  }
  Cloud pts;
  std::vector<float> h;
  for (double x = 0.90; x <= 1.30; x += STEP_M) {
    const bool inside = x >= 1.0;
    pts.push_back(pcl::PointXYZ(x, 0.0F, inside ? -0.10F : 0.0F));
    h.push_back(inside ? -0.10F : 0.0F);
  }
  const auto e = ge::edge_points(pts, h, Cloud(), stable, cfg);
  EXPECT_TRUE(e.obstacle.empty());
  ASSERT_FALSE(e.drop.empty());
  double min_x = 1e9;
  for (const auto & p : e.drop) {
    EXPECT_LE(p.z, -cfg.min_height_m);
    min_x = std::min<double>(min_x, p.x);
  }
  EXPECT_NEAR(min_x, 1.0, EDGE_TOL_M);
}

TEST(EdgePoints, VoxelDecimationBoundsTheCount)
{
  const auto cfg = test_config();
  Cloud pts;
  std::vector<float> h;
  add_top(pts, h, BOX_FRONT_X_M, BOX_BACK_X_M);
  const auto e = ge::edge_points(pts, h, coarse_box(), Cloud(), cfg);
  // The top is 5 cm x 20 cm: at most (5/1+2) x (20/1+2) voxels, far fewer than the input points.
  EXPECT_LE(e.obstacle.size(), 7U * 22U);
  EXPECT_LT(e.obstacle.size(), pts.size() / 3);
  // No two edge points share a voxel.
  for (std::size_t i = 0; i < e.obstacle.size(); ++i) {
    for (std::size_t j = i + 1; j < e.obstacle.size(); ++j) {
      const bool same = std::floor(e.obstacle[i].x / cfg.voxel_m) == std::floor(e.obstacle[j].x / cfg.voxel_m) &&
                        std::floor(e.obstacle[i].y / cfg.voxel_m) == std::floor(e.obstacle[j].y / cfg.voxel_m);
      EXPECT_FALSE(same);
    }
  }
}

TEST(GuidancePoints, AgeAndFixedFrameRoundTrip)
{
  ge::GuidancePoints g;
  EXPECT_TRUE(g.empty());
  EXPECT_TRUE(std::isinf(g.age_s(1'000'000'000)));

  ge::GuidanceSamples s;
  s.points.push_back(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
  s.points.push_back(pcl::PointXYZ(0.0F, 2.0F, 0.1F));
  s.heights = {0.0F, 0.1F};
  Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
  fixed_T_base.rotate(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  fixed_T_base.pretranslate(Eigen::Vector3d(3.0, -1.0, 0.0));
  g.set(s, fixed_T_base, 2'000'000'000);
  EXPECT_EQ(g.size(), 2U);
  EXPECT_NEAR(g.age_s(2'500'000'000), 0.5, 1e-9);

  // Back into the same base: identical to the input.
  const auto same = g.points_in_base(fixed_T_base.inverse());
  ASSERT_EQ(same.first.size(), 2U);
  EXPECT_NEAR(same.first[0].x, 1.0, 1e-5);
  EXPECT_NEAR(same.first[1].y, 2.0, 1e-5);
  EXPECT_FLOAT_EQ(same.second[1], 0.1F);

  // A moved base sees the points shifted by the inverse motion.
  Eigen::Isometry3d fixed_T_base2 = Eigen::Isometry3d::Identity();
  fixed_T_base2.pretranslate(Eigen::Vector3d(1.0, 0.0, 0.0));
  const auto moved = g.points_in_base(fixed_T_base2.inverse());
  const Eigen::Vector3d expect = fixed_T_base2.inverse() * (fixed_T_base * Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_NEAR(moved.first[0].x, expect.x(), 1e-5);
  EXPECT_NEAR(moved.first[0].y, expect.y(), 1e-5);

  g.clear();
  EXPECT_TRUE(g.empty());
}
