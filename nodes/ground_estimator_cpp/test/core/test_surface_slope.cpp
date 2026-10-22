#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "ground_estimator_cpp/surface_slope.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::HeightSample;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::RawHeightHistory;
using ground_estimator_cpp::surface_slope_deg;
using ground_estimator_cpp::TemporalConfig;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double BAND_M = 0.02;      // fit.distance_m default
constexpr double FRAME_DT_S = 0.08;  // the slope history of 16 clouds within temporal.max_age_s
// The user's black rubber mat: 25 cm long, 63 mm high (14.1 deg), 25 cm wide.
constexpr double MAT_X0 = 1.0, MAT_LENGTH_M = 0.25, MAT_Y0 = -0.45, MAT_Y1 = -0.20;
constexpr double MAT_HEIGHT_M = 0.063;
constexpr double MAT_GRID_M = 0.02;
constexpr double MAT_KEEP_FRACTION = 0.3;
constexpr double MAT_SIGMA_M = 0.015;
constexpr double FLOOR_GRID_M = 0.03;
constexpr double FLOOR_SIGMA_M = 0.004;
constexpr unsigned SEED_SETS = 20;
// The estimate of the sparse mat over the slope history scatters by ~0.5 deg; one seed set in twenty may cross the
// 15 deg rule, which the class hysteresis absorbs.
constexpr double SPARSE_LOW_TOLERANCE_DEG = 2.5;
constexpr double SPARSE_HIGH_TOLERANCE_DEG = 1.5;
constexpr double MAX_SLOPE_DEG = 15.0;  // traversability.max_slope_deg default

// The node's slope history settings: SLOPE_HISTORY_WINDOWS windows of temporal.window_frames clouds.
TemporalConfig history_config() { return ground_estimator_cpp::slope_history_config(TemporalConfig()); }

double mat_slope_deg(double height_m) { return std::atan(height_m / MAT_LENGTH_M) * RAD_TO_DEG; }

bool under_mat(const pcl::PointXYZ & p)
{
  return p.x >= MAT_X0 && p.x <= MAT_X0 + MAT_LENGTH_M && p.y >= MAT_Y0 && p.y <= MAT_Y1;
}

// One cloud of a mat of the given height (sign -1: a pit of that depth) on a floor around it, as the node test
// scene: the mat on a 2 cm grid with keep_fraction of the returns and sigma_m height noise, a 3 cm floor grid.
Cloud mat_cloud(double height_m, double keep_fraction, double sigma_m, unsigned seed, double sign = 1.0)
{
  ground_test::Scene mat;
  ground_test::add_ramp(mat, MAT_X0, MAT_LENGTH_M, height_m, MAT_Y0, MAT_Y1);
  Cloud cloud = ground_test::add_z_noise(
    ground_test::keep_fraction(ground_test::sample_grid(mat, 0.95, 1.3, -0.5, -0.15, MAT_GRID_M), keep_fraction, seed),
    sigma_m, 0.0, 0.0, seed);
  for (auto & p : cloud) {
    p.z = static_cast<float>(sign * p.z);
  }
  std::mt19937 rng(seed + 500U);
  std::normal_distribution<double> noise(0.0, FLOOR_SIGMA_M);
  for (auto p : ground_test::grid_cloud(0.7, 1.45, -0.7, 0.05, FLOOR_GRID_M)) {
    if (!under_mat(p)) {
      p.z = static_cast<float>(noise(rng));
      cloud.push_back(p);
    }
  }
  return cloud;
}

// The points of a cloud beyond the floor band: the cluster the node would hand over.
Cloud beyond_band(const Cloud & cloud, double sign)
{
  Cloud out;
  for (const auto & p : cloud) {
    if (sign * p.z > BAND_M) {
      out.push_back(p);
    }
  }
  return out;
}

// The slope of the mat over the slope history of one seed set, base_link at a yawed fixed-frame pose.
std::optional<double> mat_estimate(
  double height_m, double keep_fraction, double sigma_m, unsigned seed_set, double sign)
{
  const TemporalConfig cfg = history_config();
  RawHeightHistory history(cfg);
  Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
  fixed_T_base.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);
  fixed_T_base.linear() = Eigen::AngleAxisd(30.0 / RAD_TO_DEG, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Cloud last;
  for (int i = 0; i < cfg.window_frames; ++i) {
    last = mat_cloud(height_m, keep_fraction, sigma_m, seed_set * 1000U + static_cast<unsigned>(i), sign);
    history.add(last, Plane(), fixed_T_base, i * FRAME_DT_S);
  }
  return surface_slope_deg(history.around(beyond_band(last, sign), fixed_T_base), sign, BAND_M);
}

// A cloud of one point per cell centre (cells 0.1 m), heights given.
Cloud one_point(double x, double y, double z)
{
  Cloud c;
  c.push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
  return c;
}

}  // namespace

TEST(SurfaceSlope, DenseNoiseFreeRampWithFloorAroundReadsTheTrueSlope)
{
  const auto slope = mat_estimate(MAT_HEIGHT_M, 1.0, 0.0, 1U, 1.0);
  ASSERT_TRUE(slope.has_value());
  EXPECT_NEAR(*slope, mat_slope_deg(MAT_HEIGHT_M), 0.5);
}

TEST(SurfaceSlope, SparseNoisyMatOverTheSlopeHistoryStaysUnderTheRule)
{
  unsigned under_rule = 0;
  for (unsigned s = 0; s < SEED_SETS; ++s) {
    const auto slope = mat_estimate(MAT_HEIGHT_M, MAT_KEEP_FRACTION, MAT_SIGMA_M, s, 1.0);
    ASSERT_TRUE(slope.has_value()) << "seed set " << s;
    under_rule += *slope <= MAX_SLOPE_DEG ? 1U : 0U;
    EXPECT_GT(*slope, mat_slope_deg(MAT_HEIGHT_M) - SPARSE_LOW_TOLERANCE_DEG) << "seed set " << s;
    EXPECT_LT(*slope, mat_slope_deg(MAT_HEIGHT_M) + SPARSE_HIGH_TOLERANCE_DEG) << "seed set " << s;
  }
  EXPECT_GE(under_rule, SEED_SETS - 1U);
}

TEST(SurfaceSlope, SparseNoisyTwentyDegreeRampReadsAboveTheRuleForEverySeedSet)
{
  const double height_m = MAT_LENGTH_M * std::tan(20.0 / RAD_TO_DEG);
  for (unsigned s = 0; s < SEED_SETS; ++s) {
    const auto slope = mat_estimate(height_m, MAT_KEEP_FRACTION, MAT_SIGMA_M, s, 1.0);
    ASSERT_TRUE(slope.has_value()) << "seed set " << s;
    EXPECT_GT(*slope, MAX_SLOPE_DEG + 2.0) << "seed set " << s;
  }
}

TEST(SurfaceSlope, InclinedPitBelowTheFloorUsesTheNegativeSign)
{
  const auto slope = mat_estimate(MAT_HEIGHT_M, 1.0, 0.0, 2U, -1.0);
  ASSERT_TRUE(slope.has_value());
  EXPECT_NEAR(*slope, mat_slope_deg(MAT_HEIGHT_M), 0.5);
}

TEST(SurfaceSlope, FlatPlateOnTheFloorReadsLevel)
{
  std::vector<HeightSample> samples;
  std::mt19937 rng(7U);
  std::normal_distribution<double> noise(0.0, FLOOR_SIGMA_M);
  for (const auto & p : ground_test::grid_cloud(0.0, 0.6, 0.0, 0.6, 0.02)) {
    const bool on_plate = p.x > 0.2 && p.x < 0.4 && p.y > 0.2 && p.y < 0.4;
    samples.push_back(HeightSample{Eigen::Vector2d(p.x, p.y), (on_plate ? 0.05 : 0.0) + noise(rng)});
  }
  const auto slope = surface_slope_deg(samples, 1.0, BAND_M);
  ASSERT_TRUE(slope.has_value());
  EXPECT_LT(*slope, 1.0);
}

// A 3 cm sill seen by the ToF: the rays on its front face read every height between floor and top and tilt the raw
// fit by a few degrees. It stays under SLOPE_REFIT_MIN_DEG, and the node refits only clusters whose cell-median slope
// reads at least that, so a flat top keeps its cell-median slope (and stays a step, not a ramp).
TEST(SurfaceSlope, SillTopWithItsFrontFaceReturnsStaysUnderTheRefitGate)
{
  constexpr double SILL_HEIGHT_M = 0.03;
  constexpr int DECIMATION = 2;
  auto scene = ground_test::floor_scene();
  ground_test::add_box(
    scene, Eigen::Vector3d(1.075, 0.0, SILL_HEIGHT_M / 2.0), Eigen::Vector3d(0.15, 0.8, SILL_HEIGHT_M));
  const TemporalConfig cfg = history_config();
  RawHeightHistory history(cfg);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Cloud last;
  for (int i = 0; i < cfg.window_frames; ++i) {
    last = ground_test::render(scene, ground_test::DEFAULT_SIGMA_M, 0.0, 900U + static_cast<unsigned>(i), DECIMATION);
    history.add(last, Plane(), identity, i * FRAME_DT_S);
  }
  const auto slope = surface_slope_deg(history.around(beyond_band(last, 1.0), identity), 1.0, BAND_M);
  ASSERT_TRUE(slope.has_value());
  EXPECT_LT(*slope, ground_estimator_cpp::SLOPE_REFIT_MIN_DEG);
}

TEST(SurfaceSlope, FewerThanThreeSurfaceSamplesHaveNoSlope)
{
  std::vector<HeightSample> samples = {
    {Eigen::Vector2d(0.0, 0.0), 0.0},
    {Eigen::Vector2d(0.1, 0.0), 0.0},
    {Eigen::Vector2d(0.2, 0.0), 0.0},
    {Eigen::Vector2d(0.3, 0.0), 0.05},
    {Eigen::Vector2d(0.3, 0.1), 0.05}};
  EXPECT_FALSE(surface_slope_deg(samples, 1.0, BAND_M).has_value());
  EXPECT_FALSE(surface_slope_deg({}, 1.0, BAND_M).has_value());
}

TEST(SurfaceSlope, HistoryHoldsTwoTemporalWindows)
{
  TemporalConfig temporal;
  temporal.window_frames = 5;
  const TemporalConfig history = ground_estimator_cpp::slope_history_config(temporal);
  EXPECT_EQ(history.window_frames, 10);
  EXPECT_EQ(history.cell_size_m, temporal.cell_size_m);
  EXPECT_EQ(history.max_age_s, temporal.max_age_s);
}

TEST(RawHeightHistory, AroundGivesTheCellsOfThePointsAndTheirNeighbours)
{
  const TemporalConfig cfg;
  RawHeightHistory history(cfg);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Cloud cloud;
  for (const double x : {0.05, 0.15, 0.25, 0.35}) {
    cloud.push_back(pcl::PointXYZ(static_cast<float>(x), 0.05F, static_cast<float>(x)));
  }
  history.add(cloud, Plane(), identity, 0.0);
  const auto samples = history.around(one_point(0.05, 0.05, 0.0), identity);
  ASSERT_EQ(samples.size(), 2u);  // the cell at x 0.0 .. 0.1 and its neighbour 0.1 .. 0.2, not 0.2 .. 0.4
  for (const auto & s : samples) {
    EXPECT_LT(s.xy.x(), 0.2);
    EXPECT_NEAR(s.height, s.xy.x(), 1e-6);
  }
}

TEST(RawHeightHistory, AroundCountsTheCloudsWithSamplesThere)
{
  const TemporalConfig cfg;
  RawHeightHistory history(cfg);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  history.add(one_point(0.05, 0.05, 0.0), Plane(), identity, 0.0);
  history.add(one_point(0.95, 0.05, 0.0), Plane(), identity, 0.1);  // elsewhere
  history.add(one_point(0.15, 0.05, 0.0), Plane(), identity, 0.2);  // a neighbour cell
  std::size_t clouds = 0;
  EXPECT_EQ(history.around(one_point(0.05, 0.05, 0.0), identity, &clouds).size(), 2u);
  EXPECT_EQ(clouds, 2u);
  EXPECT_TRUE(history.around(one_point(5.0, 5.0, 0.0), identity, &clouds).empty());
  EXPECT_EQ(clouds, 0u);
}

TEST(RawHeightHistory, HeightsAreRelativeToTheCloudsPlaneAndXyIsInTheFixedFrame)
{
  const TemporalConfig cfg;
  RawHeightHistory history(cfg);
  Eigen::Isometry3d fixed_T_base = Eigen::Isometry3d::Identity();
  fixed_T_base.translation() = Eigen::Vector3d(5.0, -3.0, 0.0);
  Plane raised;
  raised.d = -0.1;  // z = 0.1
  history.add(one_point(0.05, 0.05, 0.15), raised, fixed_T_base, 0.0);
  const auto samples = history.around(one_point(0.05, 0.05, 0.0), fixed_T_base);
  ASSERT_EQ(samples.size(), 1u);
  EXPECT_NEAR(samples[0].height, 0.05, 1e-6);
  EXPECT_NEAR(samples[0].xy.x(), 5.05, 1e-6);
  EXPECT_NEAR(samples[0].xy.y(), -2.95, 1e-6);
}

TEST(RawHeightHistory, KeepsWindowFramesAndDropsOldOrRewoundClouds)
{
  TemporalConfig cfg;
  cfg.window_frames = 3;
  cfg.max_age_s = 1.0;
  RawHeightHistory history(cfg);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 5; ++i) {
    history.add(one_point(0.05, 0.05, 0.01 * i), Plane(), identity, i * FRAME_DT_S);
  }
  EXPECT_EQ(history.frames(), 3u);
  EXPECT_EQ(history.around(one_point(0.05, 0.05, 0.0), identity).size(), 3u);
  history.add(one_point(0.05, 0.05, 0.0), Plane(), identity, 2.0);  // the others are older than max_age_s
  EXPECT_EQ(history.frames(), 1u);
  history.add(one_point(0.05, 0.05, 0.0), Plane(), identity, 1.0);  // time jumped back
  EXPECT_EQ(history.frames(), 1u);
  history.reset();
  EXPECT_EQ(history.frames(), 0u);
  EXPECT_TRUE(history.around(one_point(0.05, 0.05, 0.0), identity).empty());
}
