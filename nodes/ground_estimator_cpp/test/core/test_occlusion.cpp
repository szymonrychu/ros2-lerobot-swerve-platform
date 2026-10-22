#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "ground_estimator_cpp/occlusion.hpp"
#include "synthetic_floor.hpp"

namespace
{

using ground_estimator_cpp::occluded_mask;
using ground_estimator_cpp::OcclusionConfig;

const Eigen::Vector3d SENSOR{0.15, 0.0, 0.45};
constexpr double BOX_H = 0.05;
constexpr double BOX_X0 = 0.60;
constexpr double BOX_X1 = 0.78;
constexpr double BOX_Y0 = 0.05;
constexpr double BOX_Y1 = 0.33;
constexpr double TOP_GRID = 0.004;  // dense enough that every azimuth bin holds far-edge points
constexpr double FACE_GRID = 0.02;
constexpr double TAIL_SHARE = 0.95;
constexpr int TAIL_POINTS = 400;
constexpr std::size_t MIN_TAIL_POINTS = 300;
constexpr double FLOOR_NOISE_M = 0.006;
constexpr double MAX_FALSE_SHARE = 0.01;
constexpr unsigned SEED = 7;

struct Scene
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<float> h;
  void add(double x, double y, double z, double height)
  {
    cloud.push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
    h.push_back(static_cast<float>(height));
  }
};

std::vector<bool> run(const Scene & s, const OcclusionConfig & cfg = OcclusionConfig{})
{
  return occluded_mask(s.cloud, s.h, Eigen::Vector2d(SENSOR.x(), SENSOR.y()), SENSOR.z(), cfg);
}

double share(const std::vector<bool> & mask, std::size_t first, std::size_t last)
{
  std::size_t n = 0;
  for (std::size_t i = first; i < last; ++i) {
    n += mask[i] ? 1 : 0;
  }
  return last > first ? static_cast<double>(n) / static_cast<double>(last - first) : 0.0;
}

// True when the segment from the sensor to p passes through the box (or p lies in it).
bool ray_hits_box(const Eigen::Vector3d & p)
{
  constexpr int STEPS = 400;
  for (int i = 1; i < STEPS; ++i) {
    const Eigen::Vector3d q = SENSOR + (p - SENSOR) * (static_cast<double>(i) / STEPS);
    if (q.z() <= BOX_H && q.x() >= BOX_X0 && q.x() <= BOX_X1 && q.y() >= BOX_Y0 && q.y() <= BOX_Y1) {
      return true;
    }
  }
  return p.x() >= BOX_X0 && p.x() <= BOX_X1 && p.y() >= BOX_Y0 && p.y() <= BOX_Y1;
}

// The floor in front of, beside and behind the box, plus the box top and front face, as the sensor sees them.
Scene box_scene(std::size_t & box_first, std::size_t & box_last, std::size_t & tail_first)
{
  Scene s;
  for (double x = 0.3; x < 2.0; x += 0.02) {
    for (double y = -0.5; y < 0.7; y += 0.02) {
      const bool hidden = ray_hits_box(Eigen::Vector3d(x, y, 0.0));
      if (!hidden) {
        s.add(x, y, 0.0, 0.0);
      }
    }
  }
  box_first = s.cloud.size();
  for (double x = BOX_X0; x <= BOX_X1 + 1e-9; x += TOP_GRID) {
    for (double y = BOX_Y0; y <= BOX_Y1 + 1e-9; y += TOP_GRID) {
      s.add(x, y, BOX_H, BOX_H);
    }
  }
  for (double z = FACE_GRID; z < BOX_H; z += FACE_GRID) {  // front face
    for (double y = BOX_Y0; y <= BOX_Y1 + 1e-9; y += TOP_GRID) {
      s.add(BOX_X0, y, z, z);
    }
  }
  box_last = s.cloud.size();
  tail_first = ground_test::add_mixed_pixel_tail(
    s.cloud, SENSOR, Eigen::Vector3d(BOX_X1, BOX_Y0, BOX_H), Eigen::Vector3d(BOX_X1, BOX_Y1, BOX_H), 0.0, TAIL_POINTS,
    SEED);
  s.h.resize(s.cloud.size());
  for (std::size_t i = tail_first; i < s.cloud.size(); ++i) {
    s.h[i] = s.cloud[i].z;
  }
  return s;
}

}  // namespace

TEST(OcclusionGate, TailBehindBoxIsOccluded)
{
  std::size_t box_first = 0;
  std::size_t box_last = 0;
  std::size_t tail_first = 0;
  const Scene s = box_scene(box_first, box_last, tail_first);
  const auto mask = run(s);
  ASSERT_EQ(mask.size(), s.cloud.size());
  const std::size_t tail_points = s.cloud.size() - tail_first;
  ASSERT_GT(tail_points, MIN_TAIL_POINTS);
  const double tail_share = share(mask, tail_first, s.cloud.size());
  EXPECT_GE(tail_share, TAIL_SHARE);
  for (std::size_t i = box_first; i < box_last; ++i) {
    EXPECT_FALSE(mask[i]) << "box point " << s.cloud[i].x << " " << s.cloud[i].y << " " << s.cloud[i].z;
  }
}

TEST(OcclusionGate, FlatNoisyFloorIsNotOccluded)
{
  Scene s;
  std::mt19937 rng(SEED);
  std::normal_distribution<double> noise(0.0, FLOOR_NOISE_M);
  for (double x = 0.3; x <= 2.0; x += 0.01) {
    for (double y = -0.6; y <= 0.6; y += 0.02) {
      const double h = noise(rng);
      s.add(x, y, h, h);
    }
  }
  EXPECT_LT(share(run(s), 0, s.cloud.size()), MAX_FALSE_SHARE);
}

TEST(OcclusionGate, DropOffBelowTheFloorIsNotOccluded)
{
  Scene s;
  for (double x = 0.3; x <= 2.0; x += 0.01) {
    for (double y = -0.4; y <= 0.4; y += 0.02) {
      const double h = x < 1.0 ? 0.0 : -0.10;
      s.add(x, y, h, h);
    }
  }
  const auto mask = run(s);
  for (std::size_t i = 0; i < mask.size(); ++i) {
    EXPECT_FALSE(mask[i]) << "point " << i << " x " << s.cloud[i].x << " h " << s.h[i];
  }
}

TEST(OcclusionGate, RampRisingFromTheFloorIsNotOccluded)
{
  Scene s;
  const double slope = std::tan(14.0 * ground_test::DEG);
  for (double x = 0.3; x <= 1.4; x += 0.01) {
    for (double y = -0.4; y <= 0.4; y += 0.02) {
      const double h = x < 1.0 ? 0.0 : (x - 1.0) * slope;
      s.add(x, y, h, h);
    }
  }
  EXPECT_LT(share(run(s), 0, s.cloud.size()), MAX_FALSE_SHARE);
}

TEST(OcclusionGate, WallIsNotOccluded)
{
  Scene s;
  for (double x = 0.3; x <= 1.2; x += 0.02) {
    for (double y = -0.4; y <= 0.4; y += 0.02) {
      s.add(x, y, 0.0, 0.0);
    }
  }
  for (double z = 0.01; z <= 0.8; z += 0.02) {
    for (double y = -0.4; y <= 0.4; y += 0.01) {
      s.add(1.2, y, z, z);
    }
  }
  const auto mask = run(s);
  for (std::size_t i = 0; i < mask.size(); ++i) {
    EXPECT_FALSE(mask[i]) << "point " << i;
  }
}

TEST(OcclusionGate, DisabledGivesAllFalse)
{
  std::size_t a = 0, b = 0, c = 0;
  const Scene s = box_scene(a, b, c);
  OcclusionConfig cfg;
  cfg.enabled = false;
  const auto mask = run(s, cfg);
  ASSERT_EQ(mask.size(), s.cloud.size());
  EXPECT_EQ(share(mask, 0, mask.size()), 0.0);
}

TEST(OcclusionGate, EmptyInputGivesEmptyMask) { EXPECT_TRUE(run(Scene{}).empty()); }

TEST(OcclusionGate, NonFinitePointsAreFalseAndNeverOccluders)
{
  std::size_t a = 0, b = 0, c = 0;
  Scene s = box_scene(a, b, c);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  s.cloud.push_back(pcl::PointXYZ(nan, nan, nan));
  s.h.push_back(nan);
  s.cloud.push_back(pcl::PointXYZ(1.0F, 0.2F, 0.5F));
  s.h.push_back(nan);
  const auto mask = run(s);
  EXPECT_FALSE(mask[mask.size() - 1]);
  EXPECT_FALSE(mask[mask.size() - 2]);
}
