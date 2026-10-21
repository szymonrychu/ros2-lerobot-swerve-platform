#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "ground_estimator_cpp/cell_median.hpp"
#include "ground_estimator_cpp/edges.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"
#include "ground_estimator_cpp/segmentation.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::BandsConfig;
using ground_estimator_cpp::CellMedianFilter;
using ground_estimator_cpp::Cluster;
using ground_estimator_cpp::EdgesConfig;
using ground_estimator_cpp::extract_clusters;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::Region;
using ground_estimator_cpp::regions_from_clusters;
using ground_estimator_cpp::segment;
using ground_estimator_cpp::Segmented;
using ground_estimator_cpp::TemporalConfig;
using ground_test::add_box;
using ground_test::add_drop_off;
using ground_test::add_ramp;
using ground_test::add_table_top;
using ground_test::add_z_noise;
using ground_test::DEG;
using ground_test::floor_scene;
using ground_test::floor_until;
using ground_test::grid_cloud;
using ground_test::keep_fraction;
using ground_test::render;
using ground_test::sample_grid;
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

// The user's ramp (black rubber mat): 25 cm long, 63 mm high (14.1 deg), 0.25 m wide at x 1.0 .. 1.25.
constexpr double MAT_X0_M = 1.0;
constexpr double MAT_LENGTH_M = 0.25;
constexpr double MAT_HEIGHT_M = 0.063;
constexpr double MAT_Y0_M = -0.45;
constexpr double MAT_Y1_M = -0.20;
constexpr double MAT_GRID_M = 0.02;
constexpr double MAT_KEEP_FRACTION = 0.3;
constexpr double MAT_SIGMA_M = 0.015;
constexpr int MAT_FRAMES = 10;
constexpr double MAT_FRAME_PERIOD_S = 0.1;
constexpr unsigned MAT_SEEDS = 30;
constexpr double MAT_P50_TOL_DEG = 1.5;
constexpr double MAT_P90_ERROR_DEG = 3.0;
constexpr double MISALIGNED_OFFSET_M = 0.03;
constexpr double MISALIGNED_YAW_DEG = 20.0;

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

// Nearest-rank percentile of a copy of the values (q in 0..1).
double percentile_of(std::vector<double> values, double q)
{
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(std::lround(q * static_cast<double>(values.size() - 1)))];
}

// The largest cluster, or nullptr.
const Cluster * largest(const std::vector<Cluster> & clusters)
{
  const Cluster * best = nullptr;
  for (const auto & c : clusters) {
    if (best == nullptr || c.points.size() > best->points.size()) {
      best = &c;
    }
  }
  return best;
}

// Slopes of the sparse mat ramp over MAT_SEEDS seeds: per seed MAT_FRAMES frames of a seeded MAT_KEEP_FRACTION of the
// 2 cm grid with MAT_SIGMA_M height noise through a fresh CellMedianFilter (default temporal config), then segment and
// extract_clusters on the last output. Seeds whose largest cluster has no known slope are counted in unknown.
// @param fixed_T_base pose of base_link in the fixed frame of the temporal cells.
// @param unknown receives the number of seeds without a known slope.
// @return the known slopes in degrees.
std::vector<double> sparse_mat_slopes(const Eigen::Isometry3d & fixed_T_base, unsigned & unknown)
{
  Scene scene;
  add_ramp(scene, MAT_X0_M, MAT_LENGTH_M, MAT_HEIGHT_M, MAT_Y0_M, MAT_Y1_M);
  const auto dense = sample_grid(scene, 0.8, 1.45, -0.65, 0.0, MAT_GRID_M);
  std::vector<double> slopes;
  unknown = 0;
  for (unsigned seed = 1; seed <= MAT_SEEDS; ++seed) {
    CellMedianFilter filter{TemporalConfig{}};
    pcl::PointCloud<pcl::PointXYZ> filtered;
    for (int frame = 0; frame < MAT_FRAMES; ++frame) {
      const unsigned frame_seed = seed * 1000U + static_cast<unsigned>(frame);
      const auto sparse = keep_fraction(dense, MAT_KEEP_FRACTION, frame_seed);
      const auto noisy = add_z_noise(sparse, MAT_SIGMA_M, 0.0, 0.0, frame_seed + 500U);
      filtered = filter.apply(noisy, Plane{}, fixed_T_base, frame * MAT_FRAME_PERIOD_S);
    }
    // The floor band of the ramp tests keeps the low end of the ramp in the cluster.
    const auto seg = segment(filtered, Plane{}, BandsConfig{}, RAMP_FLOOR_DISTANCE_M);
    const auto clusters = extract_clusters(seg.above, seg.above_height, "step", Plane{}, EdgesConfig{}, EDGE_BAND_M);
    const Cluster * ramp = largest(clusters);
    if (ramp == nullptr || !ramp->slope_known) {
      ++unknown;
      continue;
    }
    slopes.push_back(ramp->slope_deg);
  }
  return slopes;
}

void expect_mat_slope(const Eigen::Isometry3d & fixed_T_base)
{
  const double truth_deg = std::atan(MAT_HEIGHT_M / MAT_LENGTH_M) / DEG;
  unsigned unknown = 0;
  const auto slopes = sparse_mat_slopes(fixed_T_base, unknown);
  EXPECT_EQ(unknown, 0u);
  ASSERT_FALSE(slopes.empty());
  std::vector<double> errors;
  for (const double s : slopes) {
    errors.push_back(std::fabs(s - truth_deg));
  }
  EXPECT_NEAR(percentile_of(slopes, 0.5), truth_deg, MAT_P50_TOL_DEG);
  EXPECT_LE(percentile_of(errors, 0.9), MAT_P90_ERROR_DEG);
}

// Cluster of a regular grid whose heights (and z, with the default floor plane) are given per point.
std::vector<Cluster> grid_clusters(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  std::vector<float> heights;
  for (const auto & p : cloud) {
    heights.push_back(p.z);
  }
  return extract_clusters(cloud, heights, "step", Plane{}, EdgesConfig{}, EDGE_BAND_M);
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
  // edge_step_m is the p50 of the edge band (was the p90): seen from the head the band holds the riser, its heights
  // spread evenly from the floor band to 0.30 m, and a strip of the top, so the median is about half the box height,
  // far above the 4 cm rule and never above the box.
  EXPECT_GT(f.above[0].edge_step_m, 0.12);
  EXPECT_LT(f.above[0].edge_step_m, 0.30 + HEIGHT_TOL_M);
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

TEST(Edges, SparseMatRampSlopeThroughCellMedian)
{
  // The temporal cells and the slope bins share the grid.
  const Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
  expect_mat_slope(fixed_T_base);
}

TEST(Edges, SparseMatRampSlopeThroughMisalignedCells)
{
  // The temporal cells live in the fixed frame: offset and turned so the slope bins do not line up with them.
  Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
  fixed_T_base.translate(Eigen::Vector3d(MISALIGNED_OFFSET_M, MISALIGNED_OFFSET_M, 0.0));
  fixed_T_base.rotate(Eigen::AngleAxisd(MISALIGNED_YAW_DEG * DEG, Eigen::Vector3d::UnitZ()));
  expect_mat_slope(fixed_T_base);
}

TEST(Edges, DenseSteepRampSlope)
{
  constexpr double RAMP_DEG = 25.0;
  constexpr double LENGTH_M = 0.3;
  Scene scene;
  add_ramp(scene, 1.0, LENGTH_M, LENGTH_M * std::tan(RAMP_DEG * DEG), -0.2, 0.2);
  const auto cloud = sample_grid(scene, 0.9, 1.4, -0.3, 0.3, 0.01);
  const auto seg = segment(cloud, Plane{}, BandsConfig{}, RAMP_FLOOR_DISTANCE_M);
  const auto clusters = extract_clusters(seg.above, seg.above_height, "step", Plane{}, EdgesConfig{}, EDGE_BAND_M);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_TRUE(clusters[0].slope_known);
  EXPECT_GE(clusters[0].slope_deg, 23.0);
}

TEST(Edges, FlatBoxTopSlope)
{
  // The user's 5 cm box, seen by the head with its riser.
  auto scene = floor_scene();
  add_box(scene, Eigen::Vector3d(0.775, 0.225, 0.025), Eigen::Vector3d(0.31, 0.25, 0.05));
  const auto f = analyse(scene);
  ASSERT_EQ(f.above.size(), 1u);
  EXPECT_TRUE(f.above[0].slope_known);
  EXPECT_LT(f.above[0].slope_deg, 3.0);
  // The riser's points in the edge band do not pull the median under the 5 cm top: the box stays above the 4 cm rule.
  EXPECT_NEAR(f.above[0].edge_step_m, 0.05, 0.005);
}

TEST(Edges, TooFewSlopeBinsIsUnknown)
{
  // A steep 30 deg patch over two 0.10 m bins (slope_min_cells 3): no slope estimate.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (double x = 1.01; x < 1.19; x += 0.01) {
    for (double y = 0.01; y < 0.09; y += 0.01) {
      cloud.push_back(pcl::PointXYZ(
        static_cast<float>(x), static_cast<float>(y), static_cast<float>((x - 1.0) * std::tan(30.0 * DEG))));
    }
  }
  const auto clusters = grid_clusters(cloud);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_FALSE(clusters[0].slope_known);
  EXPECT_EQ(clusters[0].slope_deg, 0.0);
}

TEST(Edges, SlopeBinsInOneLineAreUnknown)
{
  // Six bins in one column across the slope direction: the tilt along x is not measurable from them.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (double x = 1.01; x < 1.09; x += 0.01) {
    for (double y = -0.29; y < 0.29; y += 0.01) {
      cloud.push_back(pcl::PointXYZ(
        static_cast<float>(x), static_cast<float>(y), static_cast<float>((x - 1.0) * std::tan(30.0 * DEG))));
    }
  }
  const auto clusters = grid_clusters(cloud);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_FALSE(clusters[0].slope_known);
  EXPECT_EQ(clusters[0].slope_deg, 0.0);
}

TEST(Edges, EdgeStepIsTheMedianOfTheBand)
{
  // The 5 cm box top with heights spread 0.035 .. 0.065 m: the p50 reads 5 cm, the old p90 would read ~6.2 cm.
  auto cloud = grid_cloud(0.62, 0.93, 0.10, 0.35, 0.01);
  std::mt19937 rng(SEED);
  std::uniform_real_distribution<double> spread(0.035, 0.065);
  for (auto & p : cloud) {
    p.z = static_cast<float>(spread(rng));
  }
  const auto clusters = grid_clusters(cloud);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_NEAR(clusters[0].edge_step_m, 0.05, 0.005);
}

TEST(Edges, NoisySillEdgeStepStaysBelowTheRule)
{
  // A 3 cm sill with 1 cm height noise: the p50 stays under the 4 cm rule (the p90 would be ~4.3 cm).
  const auto cloud = add_z_noise(grid_cloud(1.0, 1.15, -0.4, 0.4, 0.01, 0.03), 0.01, 0.0, 0.0, SEED);
  const auto clusters = grid_clusters(cloud);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_LT(clusters[0].edge_step_m, 0.04);
}
