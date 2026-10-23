#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "ground_estimator_cpp/plane_fit.hpp"
#include "ground_estimator_cpp/segmentation.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::Band;
using ground_estimator_cpp::BandsConfig;
using ground_estimator_cpp::classify;
using ground_estimator_cpp::fit_plane;
using ground_estimator_cpp::FitConfig;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::segment;
using ground_estimator_cpp::Segmented;
using ground_estimator_cpp::to_cloud_msg;
using ground_test::add_box;
using ground_test::add_drop_off;
using ground_test::add_step_up;
using ground_test::add_table_top;
using ground_test::floor_scene;
using ground_test::floor_until;
using ground_test::render;

constexpr double FLOOR_DISTANCE_M = 0.02;
constexpr double SIGMA_M = 0.001;
constexpr double HEIGHT_TOL_M = 0.012;
constexpr double LEVEL_BIAS_M = 0.03;
constexpr double LOST_MARGIN_M = -0.02;
constexpr unsigned SEED = 3;
constexpr int DECIMATION = 3;

double max_of(const std::vector<float> & v) { return *std::max_element(v.begin(), v.end()); }

double min_of(const std::vector<float> & v) { return *std::min_element(v.begin(), v.end()); }

Segmented seg(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  return segment(cloud, Plane{}, BandsConfig{}, FLOOR_DISTANCE_M);
}

}  // namespace

TEST(Segmentation, ClassifyBands)
{
  const BandsConfig b;
  EXPECT_EQ(classify(0.0, 1.0, b, FLOOR_DISTANCE_M), Band::floor);
  EXPECT_EQ(classify(0.02, 1.0, b, FLOOR_DISTANCE_M), Band::floor);
  EXPECT_EQ(classify(-0.02, 1.0, b, FLOOR_DISTANCE_M), Band::floor);
  EXPECT_EQ(classify(0.03, 1.0, b, FLOOR_DISTANCE_M), Band::above);
  EXPECT_EQ(classify(b.obstacle_max_m, 1.0, b, FLOOR_DISTANCE_M), Band::above);
  EXPECT_EQ(classify(b.obstacle_max_m + 0.01, 1.0, b, FLOOR_DISTANCE_M), Band::none);
  EXPECT_EQ(classify(-0.03, 1.0, b, FLOOR_DISTANCE_M), Band::below);
  EXPECT_EQ(classify(-b.drop_max_m, 1.0, b, FLOOR_DISTANCE_M), Band::below);
  EXPECT_EQ(classify(-b.drop_max_m - 0.01, 1.0, b, FLOOR_DISTANCE_M), Band::none);
  EXPECT_EQ(classify(0.1, b.max_range_m + 0.01, b, FLOOR_DISTANCE_M), Band::none);
  EXPECT_EQ(classify(0.0, b.max_range_m + 0.01, b, FLOOR_DISTANCE_M), Band::none);
}

TEST(Segmentation, ShippedFloorBandCoversThirtyFiveMillimetresBothWays)
{
  const BandsConfig b;
  EXPECT_DOUBLE_EQ(b.floor_band_m, 0.04);
  EXPECT_EQ(classify(0.035, 1.0, b, b.floor_band_m), Band::floor);
  EXPECT_EQ(classify(-0.035, 1.0, b, b.floor_band_m), Band::floor);
  EXPECT_EQ(classify(0.045, 1.0, b, b.floor_band_m), Band::above);
  EXPECT_EQ(classify(-0.045, 1.0, b, b.floor_band_m), Band::below);
}

TEST(Segmentation, ClassifyIgnoresTheRobotsOwnNearZone)
{
  // The parked gripper and the mixed pixels trailing from it read as a 6 cm obstacle up to 0.42 m from base_link.
  const BandsConfig b;
  EXPECT_DOUBLE_EQ(b.min_range_m, 0.42);
  EXPECT_EQ(classify(0.06, b.min_range_m - 0.01, b, FLOOR_DISTANCE_M), Band::none);
  EXPECT_EQ(classify(0.0, b.min_range_m - 0.01, b, FLOOR_DISTANCE_M), Band::none);
  EXPECT_EQ(classify(-0.2, b.min_range_m - 0.01, b, FLOOR_DISTANCE_M), Band::none);
  EXPECT_EQ(classify(0.06, b.min_range_m, b, FLOOR_DISTANCE_M), Band::above);
  BandsConfig off;
  off.min_range_m = 0.0;
  EXPECT_EQ(classify(0.06, 0.1, off, FLOOR_DISTANCE_M), Band::above);
}

TEST(Segmentation, BoxOnLevelFloor)
{
  auto scene = floor_scene();
  add_box(scene, Eigen::Vector3d(0.7, 0.0, 0.05), Eigen::Vector3d(0.3, 0.3, 0.10));
  const auto s = seg(render(scene, SIGMA_M, 0.0, SEED, DECIMATION));
  ASSERT_GT(s.above.size(), 10u);
  ASSERT_EQ(s.above.size(), s.above_height.size());
  EXPECT_NEAR(max_of(s.above_height), 0.10, HEIGHT_TOL_M);
  EXPECT_GT(s.floor.size(), 100u);
  EXPECT_TRUE(s.below.empty());
  EXPECT_TRUE(s.below_height.empty());
  for (std::size_t i = 0; i < s.above.size(); ++i) {
    EXPECT_NEAR(s.above[i].z, s.above_height[i], 1e-6);  // level plane at z = 0: coordinates are unchanged
  }
}

TEST(Segmentation, DropOffBeyondOneMetre)
{
  auto scene = floor_until(1.0);
  add_drop_off(scene, 1.0, 0.10);
  const auto s = seg(render(scene, SIGMA_M, 0.0, SEED, DECIMATION));
  ASSERT_GT(s.below.size(), 10u);
  EXPECT_NEAR(min_of(s.below_height), -0.10, HEIGHT_TOL_M);
  EXPECT_NEAR(max_of(s.below_height), -0.10, HEIGHT_TOL_M);
  EXPECT_GT(s.floor.size(), 50u);
}

TEST(Segmentation, ThreeCentimetreSill)
{
  auto scene = floor_until(0.8);
  add_step_up(scene, 0.8, 0.03);
  const auto s = seg(render(scene, SIGMA_M, 0.0, SEED, DECIMATION));
  ASSERT_GT(s.above.size(), 20u);
  EXPECT_NEAR(max_of(s.above_height), 0.03, HEIGHT_TOL_M);
  EXPECT_GT(min_of(s.above_height), FLOOR_DISTANCE_M);
  EXPECT_TRUE(s.below.empty());
}

TEST(Segmentation, PlaneAbsorbsDepthBias)
{
  const auto cloud = render(floor_scene(), SIGMA_M, LEVEL_BIAS_M, SEED, DECIMATION);
  const auto fit = fit_plane(cloud, Eigen::Vector3d::UnitZ(), FitConfig{}, SEED);
  ASSERT_TRUE(fit.plane.has_value());
  const auto s = segment(cloud, *fit.plane, BandsConfig{}, FLOOR_DISTANCE_M);
  EXPECT_TRUE(s.below.empty());
  EXPECT_GT(s.floor.size(), cloud.size() / 2);

  // The fixed bands at odom z = 0 lose the margin: the same floor reaches below -0.02 m in raw base_link z.
  float min_z = std::numeric_limits<float>::max();
  for (const auto & p : cloud) {
    min_z = std::min(min_z, p.z);
  }
  EXPECT_LT(min_z, LOST_MARGIN_M);
}

TEST(Segmentation, BeyondMaxRangeIsNone)
{
  auto scene = floor_until(1.0);
  add_drop_off(scene, 1.0, 0.10);
  BandsConfig bands;
  bands.max_range_m = 0.5;
  const auto s = segment(render(scene, SIGMA_M, 0.0, SEED, DECIMATION), Plane{}, bands, FLOOR_DISTANCE_M);
  EXPECT_TRUE(s.below.empty());
  EXPECT_TRUE(s.above.empty());
  for (const auto & p : s.floor) {
    EXPECT_LE(std::hypot(p.x, p.y), bands.max_range_m);
  }
}

TEST(Segmentation, OverhangIsNone)
{
  auto scene = floor_scene();
  add_table_top(scene, 0.7, 0.4, 1.5, -1.0, 1.0);
  const auto s = seg(render(scene, SIGMA_M, 0.0, SEED, DECIMATION));
  EXPECT_TRUE(s.above.empty());
  EXPECT_TRUE(s.below.empty());
}

TEST(Segmentation, NonFinitePointsSkipped)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(pcl::PointXYZ(nan, 0.5f, 0.1f));
  cloud.push_back(pcl::PointXYZ(0.5f, std::numeric_limits<float>::infinity(), 0.1f));
  cloud.push_back(pcl::PointXYZ(0.5f, 0.0f, nan));
  cloud.push_back(pcl::PointXYZ(0.5f, 0.0f, 0.1f));
  const auto s = seg(cloud);
  ASSERT_EQ(s.above.size(), 1u);
  EXPECT_TRUE(s.floor.empty());
  EXPECT_TRUE(s.below.empty());
}

TEST(Segmentation, TiltedPlaneHeightsAreSigned)
{
  Plane plane;
  plane.d = -0.05;  // plane z = 0.05
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.5f, 0.0f, 0.15f));
  const auto s = segment(cloud, plane, BandsConfig{}, FLOOR_DISTANCE_M);
  ASSERT_EQ(s.above.size(), 1u);
  EXPECT_NEAR(s.above_height[0], 0.10, 1e-6);
  EXPECT_FLOAT_EQ(s.above[0].z, 0.15f);
}

TEST(Segmentation, CloudMessageRoundTrip)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));
  cloud.push_back(pcl::PointXYZ(-1.0f, 0.5f, 0.25f));
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 345;
  const auto msg = to_cloud_msg(cloud, "base_link", stamp);
  EXPECT_EQ(msg.header.frame_id, "base_link");
  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.header.stamp.nanosec, 345u);
  EXPECT_EQ(msg.width, 2u);
  EXPECT_EQ(msg.height, 1u);
  EXPECT_TRUE(msg.is_dense);
  ASSERT_GE(msg.fields.size(), 3u);
  const char * names[] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(msg.fields[i].name, names[i]);
    EXPECT_EQ(msg.fields[i].datatype, sensor_msgs::msg::PointField::FLOAT32);
  }
  pcl::PointCloud<pcl::PointXYZ> back;
  pcl::fromROSMsg(msg, back);
  ASSERT_EQ(back.size(), 2u);
  EXPECT_FLOAT_EQ(back[1].x, -1.0f);
  EXPECT_FLOAT_EQ(back[1].z, 0.25f);
}

TEST(Segmentation, EmptyCloudMessage)
{
  const auto msg = to_cloud_msg(pcl::PointCloud<pcl::PointXYZ>(), "base_link", builtin_interfaces::msg::Time());
  EXPECT_EQ(msg.width, 0u);
  EXPECT_EQ(msg.header.frame_id, "base_link");
  EXPECT_TRUE(msg.data.empty());
}
