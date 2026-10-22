#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <utility>
#include <vector>

#include "ground_estimator_cpp/edges.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"
#include "ground_estimator_cpp/segmentation.hpp"
#include "ground_estimator_cpp/traversability.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::apply_rule;
using ground_estimator_cpp::BandsConfig;
using ground_estimator_cpp::classify_cluster;
using ground_estimator_cpp::Cls;
using ground_estimator_cpp::Cluster;
using ground_estimator_cpp::colour_warning_areas;
using ground_estimator_cpp::EdgesConfig;
using ground_estimator_cpp::extract_clusters;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::regions_from_clusters;
using ground_estimator_cpp::segment;
using ground_estimator_cpp::Traversability;
using ground_estimator_cpp::TraversabilityConfig;
using ground_test::DEG;
using ground_test::render;
using ground_test::Scene;

constexpr double FLOOR_DISTANCE_M = 0.02;
constexpr double RAMP_FLOOR_DISTANCE_M = 0.005;
constexpr double SIGMA_M = 0.001;
constexpr double EDGE_BAND_M = 0.05;
constexpr unsigned SEED = 3;
constexpr int DECIMATION = 2;
constexpr double LOW_SILL_M = 0.035;
constexpr double HIGH_SILL_M = 0.045;

ground_test::Sensor over_the_lip()
{
  ground_test::Sensor sensor;
  sensor.origin = Eigen::Vector3d(0.9, 0.0, 0.8);
  sensor.centre_down_deg = 85.0;
  sensor.h_fov_deg = 120.0;
  sensor.v_fov_deg = 20.0;
  return sensor;
}

ground_test::Sensor overhead()
{
  ground_test::Sensor sensor;
  sensor.origin = Eigen::Vector3d(0.8, 0.0, 0.8);
  sensor.centre_down_deg = 70.0;
  sensor.v_fov_deg = 40.0;
  return sensor;
}

Traversability run(
  const Scene & scene, const ground_test::Sensor & sensor = overhead(), double floor_distance = FLOOR_DISTANCE_M,
  const TraversabilityConfig & cfg = TraversabilityConfig{})
{
  const auto seg =
    segment(render(scene, SIGMA_M, 0.0, SEED, DECIMATION, sensor), Plane{}, BandsConfig{}, floor_distance);
  const EdgesConfig edges;
  std::vector<int> ua, ub;
  const auto above = extract_clusters(seg.above, seg.above_height, "step", Plane{}, edges, EDGE_BAND_M, &ua);
  const auto below = extract_clusters(seg.below, seg.below_height, "drop", Plane{}, edges, EDGE_BAND_M, &ub);
  pcl::PointCloud<pcl::PointXYZ> pa, pb;
  std::vector<float> ha, hb;
  for (int i : ua) {
    pa.push_back(seg.above[i]);
    ha.push_back(seg.above_height[i]);
  }
  for (int i : ub) {
    pb.push_back(seg.below[i]);
    hb.push_back(seg.below_height[i]);
  }
  auto regions = regions_from_clusters(above, edges);
  const auto drop_regions = regions_from_clusters(below, edges);
  regions.insert(regions.end(), drop_regions.begin(), drop_regions.end());
  return apply_rule(above, below, pa, ha, pb, hb, regions, edges, cfg);
}

Scene sill(double height_m)
{
  auto scene = ground_test::floor_until(1.0);
  ground_test::add_step_up(scene, 1.0, height_m);
  return scene;
}

Scene drop(double depth_m)
{
  Scene scene;
  scene.surfaces.push_back(ground_test::plane_surface(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), [](const Eigen::Vector3d & p) { return p.x() < 1.0; }));
  scene.surfaces.push_back(ground_test::plane_surface(
    Eigen::Vector3d(0.0, 0.0, -depth_m), Eigen::Vector3d::UnitZ(),
    [](const Eigen::Vector3d & p) { return p.x() >= 1.0; }));
  return scene;
}

Scene ramp(double deg)
{
  auto scene = ground_test::floor_until(1.0);
  scene.surfaces.push_back(ground_test::plane_surface(
    Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-std::sin(deg * DEG), 0.0, std::cos(deg * DEG)),
    [](const Eigen::Vector3d & p) { return p.x() >= 1.0; }));
  return scene;
}

bool has_kind(const Traversability & t, const std::string & kind)
{
  for (const auto & w : t.warnings) {
    if (w.kind == kind) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(Traversability, ClassifyThresholdsAreWarnings)
{
  const TraversabilityConfig cfg;
  EXPECT_EQ(classify_cluster(0.04, 15.0, true, cfg).cls, Cls::warning);
  EXPECT_EQ(classify_cluster(0.04, 15.0, true, cfg).reason, "traversable");
  const auto step = classify_cluster(0.0401, 0.0, true, cfg);
  EXPECT_EQ(step.cls, Cls::obstacle);
  EXPECT_EQ(step.reason, "step_height");
  const auto slope = classify_cluster(0.0, 15.01, true, cfg);
  EXPECT_EQ(slope.cls, Cls::obstacle);
  EXPECT_EQ(slope.reason, "slope");
}

TEST(Traversability, SillBothSides)
{
  const auto low = run(sill(LOW_SILL_M));
  EXPECT_TRUE(has_kind(low, "step"));
  EXPECT_TRUE(low.obstacle.empty());
  const auto high = run(sill(HIGH_SILL_M));
  EXPECT_FALSE(high.obstacle.empty());
  EXPECT_FALSE(has_kind(high, "step"));
}

TEST(Traversability, DropBothSides)
{
  const auto low = run(drop(LOW_SILL_M), over_the_lip());
  EXPECT_TRUE(has_kind(low, "drop"));
  EXPECT_TRUE(low.drop.empty());
  const auto high = run(drop(HIGH_SILL_M), over_the_lip());
  EXPECT_FALSE(high.drop.empty());
  EXPECT_FALSE(has_kind(high, "drop"));
}

TEST(Traversability, RampBothSides)
{
  const auto gentle = run(ramp(14.0), overhead(), RAMP_FLOOR_DISTANCE_M);
  EXPECT_TRUE(has_kind(gentle, "ramp"));
  EXPECT_TRUE(gentle.obstacle.empty());
  const auto steep = run(ramp(17.0), overhead(), RAMP_FLOOR_DISTANCE_M);
  EXPECT_FALSE(steep.obstacle.empty());
  EXPECT_FALSE(has_kind(steep, "ramp"));
  ASSERT_EQ(steep.warnings.size(), 0u);
}

TEST(Traversability, BoxIsObstacle)
{
  auto scene = ground_test::floor_scene();
  ground_test::add_box(scene, Eigen::Vector3d(1.0, 0.0, 0.05), Eigen::Vector3d(0.3, 0.4, 0.10));
  const auto t = run(scene);
  EXPECT_FALSE(t.obstacle.empty());
  EXPECT_TRUE(t.warnings.empty());
}

TEST(Traversability, LargerWheelRidesFiveCentimetres)
{
  TraversabilityConfig cfg;
  cfg.wheel_radius_m = 0.09;
  const auto t = run(sill(0.05), overhead(), FLOOR_DISTANCE_M, cfg);
  EXPECT_TRUE(t.obstacle.empty());
  EXPECT_TRUE(has_kind(t, "step"));
}

TEST(Traversability, RegionsCarryCls)
{
  const auto low = run(sill(LOW_SILL_M));
  ASSERT_FALSE(low.regions.empty());
  EXPECT_EQ(low.regions[0].cls, "warning");
  const auto high = run(sill(HIGH_SILL_M));
  ASSERT_FALSE(high.regions.empty());
  EXPECT_EQ(high.regions[0].cls, "obstacle");
}

namespace
{
pcl::PointCloud<pcl::PointXYZ> returns(const std::vector<std::pair<float, float>> & xy, float h)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto & p : xy) {
    cloud.push_back(pcl::PointXYZ(p.first, p.second, h));
  }
  return cloud;
}
}  // namespace

TEST(Traversability, IsolatedReturns)
{
  const pcl::PointCloud<pcl::PointXYZ> empty;
  const EdgesConfig edges;
  const auto tall = returns({{1.0F, 0.0F}}, 0.10F);
  const auto none = returns({{1.0F, 0.0F}}, 0.03F);
  const auto lone = apply_rule({}, {}, tall, {0.10F}, empty, {}, {}, edges, TraversabilityConfig{});
  EXPECT_TRUE(lone.obstacle.empty());
  TraversabilityConfig single;
  single.lethal_min_points = 1;
  const auto a = apply_rule({}, {}, tall, {0.10F}, empty, {}, {}, edges, single);
  EXPECT_EQ(a.obstacle.size(), 1u);
  const auto b = apply_rule({}, {}, none, {0.03F}, empty, {}, {}, edges, single);
  EXPECT_TRUE(b.obstacle.empty());
  EXPECT_TRUE(b.drop.empty());
  const auto c = apply_rule({}, {}, empty, {}, tall, {-0.10F}, {}, edges, single);
  EXPECT_EQ(c.drop.size(), 1u);
}

TEST(Traversability, LethalNeedsNeighbour)
{
  const pcl::PointCloud<pcl::PointXYZ> empty;
  const EdgesConfig edges;
  const TraversabilityConfig cfg;
  const auto close = returns({{1.0F, 0.0F}, {1.05F, 0.0F}}, 0.10F);
  const auto far = returns({{1.0F, 0.0F}, {1.5F, 0.0F}}, 0.10F);
  EXPECT_EQ(apply_rule({}, {}, close, {0.10F, 0.10F}, empty, {}, {}, edges, cfg).obstacle.size(), 2u);
  EXPECT_TRUE(apply_rule({}, {}, far, {0.10F, 0.10F}, empty, {}, {}, edges, cfg).obstacle.empty());
  const auto close_below = returns({{1.0F, 0.0F}, {1.05F, 0.0F}}, -0.10F);
  const auto far_below = returns({{1.0F, 0.0F}, {1.5F, 0.0F}}, -0.10F);
  EXPECT_EQ(apply_rule({}, {}, empty, {}, close_below, {-0.10F, -0.10F}, {}, edges, cfg).drop.size(), 2u);
  EXPECT_TRUE(apply_rule({}, {}, empty, {}, far_below, {-0.10F, -0.10F}, {}, edges, cfg).drop.empty());
}

namespace
{
// A square patch of mismatch points at the given height.
void patch(pcl::PointCloud<pcl::PointXYZ> & cloud, std::vector<float> & heights, double h)
{
  for (int i = 0; i < 15; ++i) {
    for (int j = 0; j < 15; ++j) {
      cloud.push_back(pcl::PointXYZ(
        1.0F + 0.02F * static_cast<float>(i), -0.15F + 0.02F * static_cast<float>(j), static_cast<float>(h)));
      heights.push_back(static_cast<float>(h));
    }
  }
}
}  // namespace

TEST(Traversability, ColourSheetIsWarning)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<float> heights;
  patch(cloud, heights, 0.0);
  const auto areas = colour_warning_areas(cloud, heights, Plane{}, EdgesConfig{}, TraversabilityConfig{});
  ASSERT_EQ(areas.size(), 1u);
  EXPECT_EQ(areas[0].kind, "colour");
  EXPECT_EQ(areas[0].source, "colour");
  EXPECT_FALSE(areas[0].polygon.empty());
}

TEST(Traversability, ColourPlateIsWarningOnly)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<float> heights;
  patch(cloud, heights, 0.06);
  const auto areas = colour_warning_areas(cloud, heights, Plane{}, EdgesConfig{}, TraversabilityConfig{});
  ASSERT_EQ(areas.size(), 1u);
  EXPECT_EQ(areas[0].kind, "colour");
  EXPECT_EQ(areas[0].source, "colour");
}

TEST(Traversability, UnknownSlopeUsesEdgeStepOnly)
{
  const TraversabilityConfig cfg;
  EXPECT_EQ(classify_cluster(0.03, 40.0, false, cfg).cls, Cls::warning);
  EXPECT_EQ(classify_cluster(0.05, 0.0, false, cfg).cls, Cls::obstacle);
  EXPECT_EQ(classify_cluster(0.03, 40.0, true, cfg).cls, Cls::obstacle);

  Cluster c;
  c.kind = "step";
  c.edge_step_m = 0.03;
  c.median_height_m = 0.03;
  c.slope_deg = 40.0;
  c.slope_known = false;
  c.points = returns({{1.0F, 0.0F}, {1.0F, 0.02F}, {1.02F, 0.0F}}, 0.03F);
  c.hull = {Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(1.0, 0.02), Eigen::Vector2d(1.02, 0.0)};
  const pcl::PointCloud<pcl::PointXYZ> empty;
  auto t = apply_rule({c}, {}, empty, {}, empty, {}, {}, EdgesConfig{}, cfg);
  ASSERT_EQ(t.warnings.size(), 1u);
  EXPECT_EQ(t.warnings[0].kind, "step");
  EXPECT_TRUE(t.obstacle.empty());
  c.slope_known = true;
  c.slope_deg = 6.0;
  t = apply_rule({c}, {}, empty, {}, empty, {}, {}, EdgesConfig{}, cfg);
  ASSERT_EQ(t.warnings.size(), 1u);
  EXPECT_EQ(t.warnings[0].kind, "ramp");
}

TEST(Traversability, WarningPointsAreKept)
{
  const auto up = run(sill(0.03));
  ASSERT_TRUE(has_kind(up, "step"));
  EXPECT_FALSE(up.warning_above.empty());
  EXPECT_TRUE(up.warning_below.empty());
  const auto down = run(drop(0.03), over_the_lip());
  ASSERT_TRUE(has_kind(down, "drop"));
  EXPECT_FALSE(down.warning_below.empty());
  EXPECT_TRUE(down.warning_above.empty());
}
