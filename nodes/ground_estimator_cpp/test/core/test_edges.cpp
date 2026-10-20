#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#include "ground_estimator_cpp/edges.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"
#include "ground_estimator_cpp/segmentation.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::BandsConfig;
using ground_estimator_cpp::Cluster;
using ground_estimator_cpp::EdgesConfig;
using ground_estimator_cpp::extract_clusters;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::Region;
using ground_estimator_cpp::regions_from_clusters;
using ground_estimator_cpp::segment;
using ground_estimator_cpp::Segmented;
using ground_test::add_box;
using ground_test::add_drop_off;
using ground_test::add_table_top;
using ground_test::DEG;
using ground_test::floor_scene;
using ground_test::floor_until;
using ground_test::render;
using ground_test::Scene;

constexpr double FLOOR_DISTANCE_M = 0.02;
constexpr double RAMP_FLOOR_DISTANCE_M = 0.005;
constexpr double SIGMA_M = 0.001;
constexpr double EDGE_BAND_M = 0.05;
constexpr unsigned SEED = 3;
constexpr int DECIMATION = 2;
constexpr double HEIGHT_TOL_M = 0.01;
constexpr double LINE_TOL_M = 0.05;
constexpr double DIRECTION_TOL_DEG = 3.0;
constexpr double RAMP_SLOPE_TOL_DEG = 1.5;

// A narrow strip seen from straight above the lip: the head sees the floor just behind the edge of a vertical drop
// (a flat-looking head sees a shadow of depth / tan(view angle) behind the edge line) and the strip runs along it.
ground_test::Sensor over_the_lip()
{
  ground_test::Sensor sensor;
  sensor.origin = Eigen::Vector3d(0.9, 0.0, 0.8);
  sensor.centre_down_deg = 85.0;
  sensor.h_fov_deg = 120.0;
  sensor.v_fov_deg = 20.0;
  return sensor;
}

// Looking steeply down from 0.8 m.
ground_test::Sensor overhead()
{
  ground_test::Sensor sensor;
  sensor.origin = Eigen::Vector3d(0.8, 0.0, 0.8);
  sensor.centre_down_deg = 70.0;
  sensor.v_fov_deg = 40.0;
  return sensor;
}

struct Found
{
  std::vector<Cluster> above, below;
  std::vector<int> unclustered_above, unclustered_below;
  Segmented seg;
};

Found analyse(
  const Scene & scene, const ground_test::Sensor & sensor = ground_test::Sensor(),
  const EdgesConfig & cfg = EdgesConfig{}, double floor_distance = FLOOR_DISTANCE_M)
{
  Found f;
  f.seg = segment(render(scene, SIGMA_M, 0.0, SEED, DECIMATION, sensor), Plane{}, BandsConfig{}, floor_distance);
  f.above = extract_clusters(f.seg.above, f.seg.above_height, "step", Plane{}, cfg, EDGE_BAND_M, &f.unclustered_above);
  f.below = extract_clusters(f.seg.below, f.seg.below_height, "drop", Plane{}, cfg, EDGE_BAND_M, &f.unclustered_below);
  return f;
}

// Drop-off of the given depth whose edge line is the set n . p = offset_m (n a unit xy normal pointing away from the
// robot): floor on the near side, lower floor beyond.
Scene vertical_drop(const Eigen::Vector2d & n, double offset_m, double depth_m)
{
  const auto along = [n](const Eigen::Vector3d & p) { return n.x() * p.x() + n.y() * p.y(); };
  Scene scene;
  scene.surfaces.push_back(ground_test::plane_surface(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), [=](const Eigen::Vector3d & p) { return along(p) < offset_m; }));
  scene.surfaces.push_back(ground_test::plane_surface(
    Eigen::Vector3d(0.0, 0.0, -depth_m), Eigen::Vector3d::UnitZ(),
    [=](const Eigen::Vector3d & p) { return along(p) >= offset_m; }));
  return scene;
}

Scene straight_drop(double depth_m = 0.15) { return vertical_drop(Eigen::Vector2d::UnitX(), 1.0, depth_m); }

// Half plane test of mcp surfaces.py HalfPlaneEdge: left = cross(direction, p - point) > 0.
bool inside(const Region & r, const Eigen::Vector2d & p)
{
  const Eigen::Vector2d d = p - r.point;
  const double cross = r.direction.x() * d.y() - r.direction.y() * d.x();
  return r.side == "left" ? cross > 0.0 : cross < 0.0;
}

double angle_between_lines_deg(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const double c = std::fabs(a.normalized().dot(b.normalized()));
  return std::acos(std::min(1.0, c)) / DEG;
}

bool point_in_hull(const std::vector<Eigen::Vector2d> & hull, const Eigen::Vector2d & p)
{
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Eigen::Vector2d a = hull[i];
    const Eigen::Vector2d b = hull[(i + 1) % hull.size()];
    const Eigen::Vector2d e = b - a;
    const Eigen::Vector2d d = p - a;
    if (e.x() * d.y() - e.y() * d.x() < -1e-6) {
      return false;
    }
  }
  return true;
}

}  // namespace

TEST(Edges, StraightDropOff)
{
  const auto f = analyse(straight_drop(), over_the_lip());
  ASSERT_EQ(f.below.size(), 1u);
  EXPECT_TRUE(f.above.empty());
  const Cluster & c = f.below[0];
  EXPECT_EQ(c.kind, "drop");
  EXPECT_NEAR(c.median_height_m, -0.15, HEIGHT_TOL_M);
  EXPECT_NEAR(c.edge_step_m, 0.15, HEIGHT_TOL_M);
  EXPECT_NEAR(c.edge_point.x(), 1.0, LINE_TOL_M);
  EXPECT_NEAR(c.distance_m, 1.0, LINE_TOL_M);
  EXPECT_LT(angle_between_lines_deg(c.edge_direction, Eigen::Vector2d::UnitY()), DIRECTION_TOL_DEG);
  EXPECT_EQ(c.heights.size(), static_cast<std::size_t>(c.points.size()));

  const auto regions = regions_from_clusters(f.below, EdgesConfig{});
  ASSERT_EQ(regions.size(), 1u);
  EXPECT_EQ(regions[0].name, "drop_0");
  EXPECT_EQ(regions[0].kind, "drop");
  EXPECT_NEAR(regions[0].height_m, -0.15, HEIGHT_TOL_M);
  EXPECT_EQ(regions[0].points, static_cast<std::size_t>(c.points.size()));
  EXPECT_NEAR(regions[0].distance_m, c.distance_m, 1e-9);
}

TEST(Edges, SideMatchesMcpHalfPlaneEdge)
{
  const auto f = analyse(straight_drop(), over_the_lip());
  const auto regions = regions_from_clusters(f.below, EdgesConfig{});
  ASSERT_EQ(regions.size(), 1u);
  EXPECT_TRUE(inside(regions[0], Eigen::Vector2d(1.5, 0.0)));
  EXPECT_FALSE(inside(regions[0], Eigen::Vector2d(0.0, 0.0)));
  EXPECT_TRUE(regions[0].side == "left" || regions[0].side == "right");
}

TEST(Edges, RotatedDropOff)
{
  // The straight drop turned by 30 degrees about the origin.
  constexpr double ANGLE_DEG = 30.0;
  const auto f = analyse(straight_drop(), over_the_lip());
  ASSERT_EQ(f.seg.below.size(), f.seg.below_height.size());
  const Eigen::Rotation2Dd rot(ANGLE_DEG * DEG);
  pcl::PointCloud<pcl::PointXYZ> turned;
  for (const auto & p : f.seg.below) {
    const Eigen::Vector2d q = rot * Eigen::Vector2d(p.x, p.y);
    turned.push_back(pcl::PointXYZ(static_cast<float>(q.x()), static_cast<float>(q.y()), p.z));
  }
  const auto clusters = extract_clusters(turned, f.seg.below_height, "drop", Plane{}, EdgesConfig{}, EDGE_BAND_M);
  ASSERT_EQ(clusters.size(), 1u);
  const Eigen::Vector2d expected_dir = rot * Eigen::Vector2d::UnitY();
  EXPECT_LT(angle_between_lines_deg(clusters[0].edge_direction, expected_dir), DIRECTION_TOL_DEG);
  EXPECT_NEAR(clusters[0].distance_m, 1.0, LINE_TOL_M);
}

TEST(Edges, FlatPlateIsStepRegion)
{
  auto scene = floor_scene();
  add_table_top(scene, 0.08, 0.85, 1.15, -0.3, 0.3);
  const auto f = analyse(scene);
  ASSERT_EQ(f.above.size(), 1u);
  const Cluster & c = f.above[0];
  EXPECT_EQ(c.kind, "step");
  EXPECT_NEAR(c.median_height_m, 0.08, HEIGHT_TOL_M);
  EXPECT_NEAR(c.edge_step_m, 0.08, HEIGHT_TOL_M);
  EXPECT_LT(c.slope_deg, 3.0);
  const auto regions = regions_from_clusters(f.above, EdgesConfig{});
  ASSERT_EQ(regions.size(), 1u);
  EXPECT_EQ(regions[0].name, "step_0");
  EXPECT_NEAR(regions[0].height_m, 0.08, HEIGHT_TOL_M);
  EXPECT_TRUE(inside(regions[0], Eigen::Vector2d(1.0, 0.0)));
  EXPECT_FALSE(inside(regions[0], Eigen::Vector2d(0.0, 0.0)));
}

TEST(Edges, RampFromFloor)
{
  constexpr double RAMP_DEG = 10.0;
  auto scene = floor_until(1.0);
  const Eigen::Vector3d ramp_normal(-std::sin(RAMP_DEG * DEG), 0.0, std::cos(RAMP_DEG * DEG));
  scene.surfaces.push_back(ground_test::plane_surface(
    Eigen::Vector3d(1.0, 0.0, 0.0), ramp_normal, [](const Eigen::Vector3d & p) { return p.x() >= 1.0; }));
  const auto f = analyse(scene, overhead(), EdgesConfig{}, RAMP_FLOOR_DISTANCE_M);
  ASSERT_EQ(f.above.size(), 1u);
  const Cluster & c = f.above[0];
  EXPECT_LT(c.edge_step_m, 0.02);
  EXPECT_NEAR(c.slope_deg, RAMP_DEG, RAMP_SLOPE_TOL_DEG);
  EXPECT_TRUE(regions_from_clusters(f.above, EdgesConfig{}).empty());  // the height spread is no flat step
}

TEST(Edges, BoxHasNoRegion)
{
  auto scene = floor_scene();
  add_box(scene, Eigen::Vector3d(1.0, 0.0, 0.15), Eigen::Vector3d(0.4, 0.6, 0.30));
  const auto f = analyse(scene);
  ASSERT_EQ(f.above.size(), 1u);
  EXPECT_NEAR(f.above[0].edge_step_m, 0.30, 0.03);
  EXPECT_TRUE(regions_from_clusters(f.above, EdgesConfig{}).empty());
}

TEST(Edges, HullContainsClusterPoints)
{
  const auto f = analyse(straight_drop(), over_the_lip());
  ASSERT_EQ(f.below.size(), 1u);
  const Cluster & c = f.below[0];
  ASSERT_GE(c.hull.size(), 3u);
  for (const auto & p : c.points) {
    EXPECT_TRUE(point_in_hull(c.hull, Eigen::Vector2d(p.x, p.y)));
  }
}

TEST(Edges, NoiseBelowMinPointsIsUnclustered)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<float> heights;
  for (int i = 0; i < 5; ++i) {
    cloud.push_back(pcl::PointXYZ(1.0F + 0.01F * static_cast<float>(i), 0.0F, 0.1F));
    heights.push_back(0.1F);
  }
  cloud.push_back(pcl::PointXYZ(-1.0F, 1.0F, 0.1F));
  heights.push_back(0.1F);
  std::vector<int> unclustered;
  const auto clusters = extract_clusters(cloud, heights, "step", Plane{}, EdgesConfig{}, EDGE_BAND_M, &unclustered);
  EXPECT_TRUE(clusters.empty());
  EXPECT_EQ(unclustered.size(), cloud.size());
}

TEST(Edges, UnclusteredCoversTheRest)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<float> heights;
  for (int i = 0; i < 20; ++i) {
    cloud.push_back(pcl::PointXYZ(1.0F + 0.02F * static_cast<float>(i), 0.0F, 0.1F));
    heights.push_back(0.1F);
  }
  cloud.push_back(pcl::PointXYZ(-1.0F, 1.0F, 0.1F));
  heights.push_back(0.1F);
  std::vector<int> unclustered;
  const auto clusters = extract_clusters(cloud, heights, "step", Plane{}, EdgesConfig{}, EDGE_BAND_M, &unclustered);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].points.size(), 20u);
  ASSERT_EQ(unclustered.size(), 1u);
  EXPECT_EQ(unclustered[0], 20);
}

TEST(Edges, ThreeDropsNearestFirstAndCapped)
{
  // Three isolated 10 cm holes at x = 0.8, 1.2, 1.6, given in scrambled order.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<float> heights;
  const double xs[] = {1.6, 0.8, 1.2};  // not in distance order
  for (const double x0 : xs) {
    for (int i = 0; i < 10; ++i) {
      for (int j = 0; j < 10; ++j) {
        cloud.push_back(pcl::PointXYZ(static_cast<float>(x0 + 0.02 * i), static_cast<float>(-0.1 + 0.02 * j), -0.1F));
        heights.push_back(-0.1F);
      }
    }
  }
  const auto clusters = extract_clusters(cloud, heights, "drop", Plane{}, EdgesConfig{}, EDGE_BAND_M);
  ASSERT_EQ(clusters.size(), 3u);
  EXPECT_LT(clusters[0].distance_m, clusters[1].distance_m);
  EXPECT_LT(clusters[1].distance_m, clusters[2].distance_m);

  const auto all = regions_from_clusters(clusters, EdgesConfig{});
  ASSERT_EQ(all.size(), 3u);
  EXPECT_EQ(all[0].name, "drop_0");
  EXPECT_EQ(all[2].name, "drop_2");
  EXPECT_LT(all[0].distance_m, all[1].distance_m);

  EdgesConfig capped;
  capped.max_regions = 2;
  const auto two = regions_from_clusters(clusters, capped);
  ASSERT_EQ(two.size(), 2u);
  EXPECT_NEAR(two[0].distance_m, clusters[0].distance_m, 1e-9);
}
