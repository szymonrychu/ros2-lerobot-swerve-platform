#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "ground_estimator_cpp/plane_fit.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::fit_plane;
using ground_estimator_cpp::FitConfig;
using ground_estimator_cpp::FitReject;
using ground_estimator_cpp::FitResult;
using ground_estimator_cpp::height_above;
using ground_estimator_cpp::height_at_origin;
using ground_test::add_box;
using ground_test::add_table_top;
using ground_test::add_wall;
using ground_test::DEFAULT_SIGMA_M;
using ground_test::DEG;
using ground_test::floor_scene;
using ground_test::render;
using ground_test::Scene;
using ground_test::tilted_normal;

constexpr unsigned SEED = 7;
constexpr double ANGLE_TOL_DEG = 0.5;
constexpr double HEIGHT_TOL_M = 0.005;

double angle_deg(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return std::acos(std::min(1.0, a.normalized().dot(b.normalized()))) / DEG;
}

FitResult fit(const Scene & scene, const Eigen::Vector3d & prior, double bias = 0.0)
{
  return fit_plane(render(scene, DEFAULT_SIGMA_M, bias, 3), prior, FitConfig{}, SEED);
}

bool rejected_with(const FitResult & r, std::initializer_list<FitReject> any_of)
{
  if (r.plane || !r.reject) {
    return false;
  }
  for (const auto reject : any_of) {
    if (*r.reject == reject) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(PlaneFit, LevelFloor)
{
  const auto r = fit(floor_scene(), Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(r.plane.has_value());
  EXPECT_LT(angle_deg(r.plane->normal, Eigen::Vector3d::UnitZ()), ANGLE_TOL_DEG);
  EXPECT_NEAR(height_at_origin(*r.plane), 0.0, HEIGHT_TOL_M);
  EXPECT_NEAR(height_above(*r.plane, Eigen::Vector3d(1.0, 0.5, 0.2)), 0.2, HEIGHT_TOL_M);
  EXPECT_GT(r.plane->normal.z(), 0.0);
  EXPECT_GT(r.inlier_ratio, 0.9);
}

TEST(PlaneFit, PitchedFloorMatchingPrior)
{
  const Eigen::Vector3d n = tilted_normal(0.0, 5.0);
  const auto r = fit(floor_scene(0.0, 5.0), n);
  ASSERT_TRUE(r.plane.has_value());
  EXPECT_LT(angle_deg(r.plane->normal, n), ANGLE_TOL_DEG);
}

TEST(PlaneFit, PitchedFloorLevelPrior)
{
  const Eigen::Vector3d n = tilted_normal(0.0, 5.0);
  const auto r = fit(floor_scene(0.0, 5.0), Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(r.plane.has_value());
  EXPECT_LT(angle_deg(r.plane->normal, n), 1.0);
}

TEST(PlaneFit, WallOnlyIsNotAFloor)
{
  Scene scene;
  add_wall(scene, 1.0);
  const auto r = fit(scene, Eigen::Vector3d::UnitZ());
  // A thin tilted sliver of the wall may become a model; the gates (here the inlier count) must still refuse it.
  EXPECT_TRUE(rejected_with(
    r, {FitReject::normal_off_prior, FitReject::no_plane, FitReject::too_few_points, FitReject::too_few_inliers}));
}

TEST(PlaneFit, TableTopOutsideBandIsNotAFloor)
{
  Scene scene;
  add_table_top(scene, 0.3, 0.0, 4.0, -3.0, 3.0);
  const auto r = fit(scene, Eigen::Vector3d::UnitZ());
  EXPECT_TRUE(rejected_with(r, {FitReject::offset_off_prior, FitReject::too_few_points}));
}

TEST(PlaneFit, FloorWithBoxCoveringHalfTheView)
{
  Scene scene = floor_scene();
  add_box(scene, Eigen::Vector3d(0.9, 0.0, 0.15), Eigen::Vector3d(0.2, 1.6, 0.3));
  const auto r = fit(scene, Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(r.plane.has_value());
  EXPECT_LT(angle_deg(r.plane->normal, Eigen::Vector3d::UnitZ()), ANGLE_TOL_DEG);
  EXPECT_NEAR(height_at_origin(*r.plane), 0.0, HEIGHT_TOL_M);
}

TEST(PlaneFit, BoxFaceFootDoesNotTiltThePlaneForAnyRansacSeed)
{
  // The foot of a box face (z 0..2 cm at its base) lies inside the RANSAC inlier distance; a least squares refit
  // over all inliers took it as floor and tilted the plane by up to 1.7 deg and 1.6 cm at the origin. The trimmed
  // refit holds every seed and noise draw within 0.35 deg and 3 mm (measured worst: 0.24 deg, 2.1 mm).
  Scene scene = floor_scene();
  add_box(scene, Eigen::Vector3d(0.9, 0.0, 0.15), Eigen::Vector3d(0.2, 1.6, 0.3));
  double worst_angle_deg = 0.0;
  double worst_height_m = 0.0;
  for (const unsigned noise_seed : {3U, 4U, 5U}) {
    const auto cloud = render(scene, DEFAULT_SIGMA_M, 0.0, noise_seed);
    for (unsigned seed = 1; seed <= 12; ++seed) {
      const auto r = fit_plane(cloud, Eigen::Vector3d::UnitZ(), FitConfig{}, seed);
      ASSERT_TRUE(r.plane.has_value()) << "noise " << noise_seed << " seed " << seed;
      worst_angle_deg = std::max(worst_angle_deg, angle_deg(r.plane->normal, Eigen::Vector3d::UnitZ()));
      worst_height_m = std::max(worst_height_m, std::fabs(height_at_origin(*r.plane)));
    }
  }
  EXPECT_LT(worst_angle_deg, 0.35);
  EXPECT_LT(worst_height_m, 0.003);
}

TEST(PlaneFit, DepthBiasShowsInReportedHeight)
{
  const auto r = fit(floor_scene(), Eigen::Vector3d::UnitZ(), 0.03);
  ASSERT_TRUE(r.plane.has_value());
  const double h = height_at_origin(*r.plane);
  EXPECT_LT(h, -0.005);  // the biased floor sits below the true one
  EXPECT_GT(h, -0.03);   // by at most the bias
  EXPECT_LT(std::fabs(h), FitConfig{}.max_offset_m);
}

TEST(PlaneFit, TooFewPoints)
{
  pcl::PointCloud<pcl::PointXYZ> empty;
  auto r = fit_plane(empty, Eigen::Vector3d::UnitZ(), FitConfig{}, SEED);
  EXPECT_TRUE(rejected_with(r, {FitReject::too_few_points}));
  EXPECT_EQ(r.candidates, 0u);

  pcl::PointCloud<pcl::PointXYZ> ten;
  for (int i = 0; i < 10; ++i) {
    ten.push_back(pcl::PointXYZ(0.1f * i, 0.0f, 0.0f));
  }
  r = fit_plane(ten, Eigen::Vector3d::UnitZ(), FitConfig{}, SEED);
  EXPECT_TRUE(rejected_with(r, {FitReject::too_few_points}));
  EXPECT_EQ(r.candidates, 10u);
}

TEST(PlaneFit, DeterministicForASeed)
{
  Scene scene = floor_scene(1.0, 2.0);
  add_box(scene, Eigen::Vector3d(0.9, 0.0, 0.15), Eigen::Vector3d(0.2, 1.0, 0.3));
  const auto cloud = render(scene, DEFAULT_SIGMA_M, 0.0, 5);
  const auto a = fit_plane(cloud, Eigen::Vector3d::UnitZ(), FitConfig{}, SEED);
  const auto b = fit_plane(cloud, Eigen::Vector3d::UnitZ(), FitConfig{}, SEED);
  ASSERT_TRUE(a.plane.has_value());
  ASSERT_TRUE(b.plane.has_value());
  EXPECT_EQ(a.plane->normal, b.plane->normal);
  EXPECT_EQ(a.plane->d, b.plane->d);
  EXPECT_EQ(a.inliers, b.inliers);
}

TEST(PlaneFit, RejectNames)
{
  EXPECT_STREQ(ground_estimator_cpp::to_string(FitReject::low_inlier_ratio), "low_inlier_ratio");
  EXPECT_STREQ(ground_estimator_cpp::to_string(FitReject::no_plane), "no_plane");
}

// Task T3 (drop safety): a RANSAC plane within fit.max_normal_angle_deg (10 deg) of the prior can tilt about 2 deg to
// pass within fit.distance_m of both levels of a 5 cm drop or step that splits the view (inlier ratio 0.83), and the
// drop then reads 27 mm, under the 40 mm step limit. The fit must keep the level the robot stands on: the floor near
// the robot, level and at the base_link origin, for every edge distance and RANSAC seed.
TEST(PlaneFit, FiveCentimetreDropOrStepAcrossTheViewKeepsTheLevelTheRobotStandsOn)
{
  for (const double edge_x : {0.8, 1.0, 1.2, 1.5}) {
    for (const double level_m : {-0.05, 0.05}) {
      Scene scene = ground_test::floor_until(edge_x);
      if (level_m < 0.0) {
        ground_test::add_drop_off(scene, edge_x, -level_m);
      } else {
        ground_test::add_step_up(scene, edge_x, level_m);
      }
      const auto cloud = render(scene, DEFAULT_SIGMA_M, 0.0, 3);
      for (unsigned seed = 1; seed <= 6; ++seed) {
        const auto r = fit_plane(cloud, Eigen::Vector3d::UnitZ(), FitConfig{}, seed);
        ASSERT_TRUE(r.plane.has_value()) << "edge " << edge_x << " level " << level_m << " seed " << seed;
        EXPECT_LT(angle_deg(r.plane->normal, Eigen::Vector3d::UnitZ()), ANGLE_TOL_DEG)
          << "edge " << edge_x << " level " << level_m << " seed " << seed;
        EXPECT_NEAR(height_at_origin(*r.plane), 0.0, HEIGHT_TOL_M)
          << "edge " << edge_x << " level " << level_m << " seed " << seed;
      }
    }
  }
}

// The same rule must not turn a genuinely sloped floor into a level one, nor take a 14 deg ramp beyond the floor
// (a surface steeper than the fit band) for the floor.
TEST(PlaneFit, PitchedFloorAndARampBeyondTheFloorAreUnchangedByTheLevelRule)
{
  for (const double pitch_deg : {-4.0, 3.0}) {
    const auto r = fit(floor_scene(0.0, pitch_deg), Eigen::Vector3d::UnitZ());
    ASSERT_TRUE(r.plane.has_value()) << pitch_deg;
    EXPECT_LT(angle_deg(r.plane->normal, tilted_normal(0.0, pitch_deg)), 1.0) << pitch_deg;
  }
  Scene ramp = ground_test::floor_until(1.0);
  ramp.surfaces.push_back(ground_test::plane_surface(
    Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-std::sin(14.0 * DEG), 0.0, std::cos(14.0 * DEG)),
    [](const Eigen::Vector3d & p) { return p.x() >= 1.0; }));
  const auto r = fit(ramp, Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(r.plane.has_value());
  EXPECT_LT(angle_deg(r.plane->normal, Eigen::Vector3d::UnitZ()), ANGLE_TOL_DEG);
  EXPECT_NEAR(height_at_origin(*r.plane), 0.0, HEIGHT_TOL_M);
}
