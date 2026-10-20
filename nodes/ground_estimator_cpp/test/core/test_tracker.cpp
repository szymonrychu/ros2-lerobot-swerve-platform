#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <optional>
#include <vector>

#include "ground_estimator_cpp/tracker.hpp"

namespace
{

using ground_estimator_cpp::base_tilt_deg;
using ground_estimator_cpp::choose_prior;
using ground_estimator_cpp::FitConfig;
using ground_estimator_cpp::gravity_up_in_base;
using ground_estimator_cpp::ImuConfig;
using ground_estimator_cpp::ImuSample;
using ground_estimator_cpp::Level;
using ground_estimator_cpp::Plane;
using ground_estimator_cpp::plane_vs_base_deg;
using ground_estimator_cpp::PlaneSource;
using ground_estimator_cpp::PlaneTracker;
using ground_estimator_cpp::slope_vs_gravity;
using ground_estimator_cpp::SlopeConfig;

constexpr double DEG = 0.017453292519943295;  // pi / 180
constexpr double TOL = 1e-9;

Eigen::Quaterniond rpy(double roll_deg, double pitch_deg)
{
  const Eigen::AngleAxisd pitch(pitch_deg * DEG, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd roll(roll_deg * DEG, Eigen::Vector3d::UnitX());
  return Eigen::Quaterniond(pitch * roll);
}

Plane floor_at(double z, double tilt_pitch_deg = 0.0)
{
  Plane p;
  p.normal = Eigen::Vector3d(-std::sin(tilt_pitch_deg * DEG), 0.0, std::cos(tilt_pitch_deg * DEG));
  p.d = -z * p.normal.z();
  return p;
}

FitConfig fit_cfg(double hold_s, double smoothing)
{
  FitConfig c;
  c.hold_s = hold_s;
  c.smoothing = smoothing;
  return c;
}

Eigen::Isometry3d pose(double x, double y, double yaw_deg)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translate(Eigen::Vector3d(x, y, 0.0));
  t.rotate(Eigen::AngleAxisd(yaw_deg * DEG, Eigen::Vector3d::UnitZ()));
  return t;
}

Eigen::Isometry3d identity_pose() { return Eigen::Isometry3d::Identity(); }

}  // namespace

TEST(GravityUp, IdentityIsUp)
{
  const Eigen::Vector3d up = gravity_up_in_base(Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(), 0, 0);
  EXPECT_NEAR((up - Eigen::Vector3d::UnitZ()).norm(), 0.0, TOL);
}

TEST(GravityUp, LevelReferenceRemoved)
{
  const Eigen::Vector3d up = gravity_up_in_base(rpy(0.7, 1.3), Eigen::Quaterniond::Identity(), 0.7, 1.3);
  EXPECT_NEAR((up - Eigen::Vector3d::UnitZ()).norm(), 0.0, TOL);
}

TEST(GravityUp, PitchedTenDegreesNoseDown)
{
  const Eigen::Vector3d up = gravity_up_in_base(rpy(0, 10), Eigen::Quaterniond::Identity(), 0, 0);
  EXPECT_NEAR(up.x(), -std::sin(10 * DEG), TOL);
  EXPECT_NEAR(up.y(), 0.0, TOL);
  EXPECT_NEAR(up.z(), std::cos(10 * DEG), TOL);
}

TEST(GravityUp, RollLeftUpIsPositiveY)
{
  const Eigen::Vector3d up = gravity_up_in_base(rpy(6, 0), Eigen::Quaterniond::Identity(), 0, 0);
  EXPECT_NEAR(up.y(), std::sin(6 * DEG), TOL);
}

TEST(ChoosePrior, FreshImuIsUsed)
{
  const Eigen::Vector3d tilted = Eigen::Vector3d(-0.1, 0.0, std::sqrt(0.99));
  const auto prior = choose_prior(ImuSample{tilted, 10.0}, 10.2, ImuConfig{});
  EXPECT_TRUE(prior.from_imu);
  EXPECT_NEAR((prior.up - tilted).norm(), 0.0, TOL);
}

TEST(ChoosePrior, StaleImuFallsBackToBaseZ)
{
  const auto prior = choose_prior(ImuSample{Eigen::Vector3d(-0.1, 0.0, 0.99), 10.0}, 10.6, ImuConfig{});
  EXPECT_FALSE(prior.from_imu);
  EXPECT_NEAR((prior.up - Eigen::Vector3d::UnitZ()).norm(), 0.0, TOL);
}

TEST(ChoosePrior, NoSampleFallsBackToBaseZ)
{
  const auto prior = choose_prior(std::nullopt, 10.0, ImuConfig{});
  EXPECT_FALSE(prior.from_imu);
  EXPECT_NEAR((prior.up - Eigen::Vector3d::UnitZ()).norm(), 0.0, TOL);
}

TEST(Tracker, HoldsUpToHoldThenNullopt)
{
  PlaneTracker t(fit_cfg(1.0, 1.0));
  t.update(floor_at(-0.3), identity_pose(), 10.0);
  const auto at_hold = t.current(identity_pose(), 11.0);
  ASSERT_TRUE(at_hold.has_value());
  EXPECT_NEAR(at_hold->age_s, 1.0, TOL);
  EXPECT_FALSE(t.current(identity_pose(), 11.001).has_value());
}

TEST(Tracker, QueryBetweenFitsGetsEarlier)
{
  PlaneTracker t(fit_cfg(2.0, 1.0));
  t.update(floor_at(-0.3), identity_pose(), 10.0);
  t.update(floor_at(-0.31), identity_pose(), 10.5);
  const auto got = t.current(identity_pose(), 10.3);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->source, PlaneSource::fit);
  EXPECT_NEAR(got->age_s, 0.3, TOL);
  EXPECT_NEAR(ground_estimator_cpp::height_at_origin(got->plane), -0.3, TOL);
}

TEST(Tracker, QueryOlderThanEveryFitIsNullopt)
{
  PlaneTracker t(fit_cfg(2.0, 1.0));
  t.update(floor_at(-0.3), identity_pose(), 10.0);
  EXPECT_FALSE(t.current(identity_pose(), 9.9).has_value());
}

TEST(Tracker, FailedCloudMakesHeld)
{
  PlaneTracker t(fit_cfg(2.0, 1.0));
  t.update(floor_at(-0.3), identity_pose(), 10.0);
  t.mark_failed(10.2);
  EXPECT_EQ(t.current(identity_pose(), 10.1)->source, PlaneSource::fit);
  const auto got = t.current(identity_pose(), 10.4);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->source, PlaneSource::held);
  t.update(floor_at(-0.3), identity_pose(), 10.5);
  EXPECT_EQ(t.current(identity_pose(), 10.6)->source, PlaneSource::fit);
}

TEST(Tracker, HistoryPrunedToHoldPlusOneSecond)
{
  PlaneTracker t(fit_cfg(1.0, 1.0));
  t.update(floor_at(-0.3), identity_pose(), 10.0);
  t.update(floor_at(-0.3), identity_pose(), 11.0);
  t.update(floor_at(-0.3), identity_pose(), 12.0);
  EXPECT_EQ(t.history_size(), 3u);
  t.update(floor_at(-0.3), identity_pose(), 12.5);
  EXPECT_EQ(t.history_size(), 3u);  // 10.0 is older than 12.5 - 2.0
  EXPECT_FALSE(t.current(identity_pose(), 10.5).has_value());
}

TEST(Tracker, PlaneFollowsRobotMotionInOdom)
{
  PlaneTracker t(fit_cfg(2.0, 1.0));
  Plane p = floor_at(-0.3, 4.0);
  t.update(p, identity_pose(), 10.0);
  const auto moved = pose(0.5, 0.0, 30.0);
  const auto got = t.current(moved, 10.2);
  ASSERT_TRUE(got.has_value());
  // Floor height at the new base origin: the old plane evaluated at the new origin expressed in the old base.
  const Eigen::Vector3d new_origin_in_old = Eigen::Vector3d(0.5, 0.0, 0.0);
  const double expected_z = -(p.d + p.normal.x() * new_origin_in_old.x()) / p.normal.z();
  EXPECT_NEAR(ground_estimator_cpp::height_at_origin(got->plane), expected_z, 1e-9);
  EXPECT_NEAR(got->plane.normal.norm(), 1.0, TOL);
}

TEST(Tracker, PlaneStoredFromMovedPoseMapsPointsConsistently)
{
  PlaneTracker t(fit_cfg(2.0, 1.0));
  const Plane p = floor_at(-0.3, 4.0);
  const Eigen::Isometry3d pose_a = pose(1.0, -0.4, 40.0);
  const Eigen::Isometry3d pose_b = pose(1.8, 0.3, -20.0);
  t.update(p, pose_a, 10.0);
  const auto got = t.current(pose_b, 10.2);
  ASSERT_TRUE(got.has_value());
  // Points on the plane in base A, taken through odom into base B, must lie on the returned plane.
  const std::vector<Eigen::Vector2d> samples{{0.0, 0.0}, {1.0, 0.5}, {-0.7, 1.2}};
  for (const Eigen::Vector2d & xy : samples) {
    const double z = -(p.d + p.normal.x() * xy.x() + p.normal.y() * xy.y()) / p.normal.z();
    const Eigen::Vector3d in_a(xy.x(), xy.y(), z);
    const Eigen::Vector3d in_b = pose_b.inverse() * (pose_a * in_a);
    EXPECT_NEAR(got->plane.normal.dot(in_b) + got->plane.d, 0.0, 1e-9);
  }
}

TEST(Tracker, FlatFloorKeepsHeightAfterMotion)
{
  PlaneTracker t(fit_cfg(2.0, 1.0));
  t.update(floor_at(-0.3), identity_pose(), 10.0);
  const auto got = t.current(pose(0.5, 0.0, 30.0), 10.2);
  ASSERT_TRUE(got.has_value());
  EXPECT_NEAR(ground_estimator_cpp::height_at_origin(got->plane), -0.3, TOL);
}

TEST(Tracker, EmaBlendsAsConfigured)
{
  PlaneTracker t(fit_cfg(2.0, 0.25));
  t.update(floor_at(-0.30), identity_pose(), 10.0);
  t.update(floor_at(-0.34), identity_pose(), 10.1);
  const auto got = t.current(identity_pose(), 10.1);
  ASSERT_TRUE(got.has_value());
  EXPECT_NEAR(ground_estimator_cpp::height_at_origin(got->plane), 0.25 * -0.34 + 0.75 * -0.30, TOL);
}

TEST(Tracker, SmoothingOneIsLatestOnly)
{
  PlaneTracker t(fit_cfg(2.0, 1.0));
  t.update(floor_at(-0.30, 3.0), identity_pose(), 10.0);
  t.update(floor_at(-0.34), identity_pose(), 10.1);
  const auto got = t.current(identity_pose(), 10.1);
  EXPECT_NEAR(ground_estimator_cpp::height_at_origin(got->plane), -0.34, TOL);
  EXPECT_NEAR((got->plane.normal - Eigen::Vector3d::UnitZ()).norm(), 0.0, TOL);
}

TEST(Slope, FivePitchedPlaneOnLevelRobot)
{
  // Floor rising ahead: the base would pitch nose up, a negative pitch.
  const auto s = slope_vs_gravity(floor_at(-0.3, 5.0), Eigen::Vector3d::UnitZ(), SlopeConfig{});
  EXPECT_NEAR(s.pitch_deg, -5.0, 1e-6);
  EXPECT_NEAR(s.roll_deg, 0.0, 1e-6);
  EXPECT_NEAR(s.total_deg, 5.0, 1e-6);
  EXPECT_EQ(s.level, Level::ok);
  const auto down = slope_vs_gravity(floor_at(-0.3, -5.0), Eigen::Vector3d::UnitZ(), SlopeConfig{});
  EXPECT_NEAR(down.pitch_deg, 5.0, 1e-6);
}

TEST(Slope, PlaneParallelToTiltedBaseIsFlat)
{
  const Eigen::Vector3d g = gravity_up_in_base(rpy(0, 10), Eigen::Quaterniond::Identity(), 0, 0);
  Plane p;
  p.normal = g;
  p.d = 0.3;
  const auto s = slope_vs_gravity(p, g, SlopeConfig{});
  EXPECT_NEAR(s.total_deg, 0.0, 1e-6);
  EXPECT_NEAR(s.pitch_deg, 0.0, 1e-6);
}

TEST(Slope, RollSignLeftDownNegative)
{
  Plane p;
  p.normal = Eigen::Vector3d(0.0, std::sin(6 * DEG), std::cos(6 * DEG));
  const auto s = slope_vs_gravity(p, Eigen::Vector3d::UnitZ(), SlopeConfig{});
  // Normal leaning to +y: floor falls off to the left, so the base would sit left side down.
  EXPECT_NEAR(s.roll_deg, -6.0, 1e-6);
}

TEST(Slope, LevelsAtLimits)
{
  const SlopeConfig c;
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  EXPECT_EQ(slope_vs_gravity(floor_at(0, 11.9), z, c).level, Level::ok);
  EXPECT_EQ(slope_vs_gravity(floor_at(0, 12.1), z, c).level, Level::warning);
  EXPECT_EQ(slope_vs_gravity(floor_at(0, 19.9), z, c).level, Level::warning);
  EXPECT_EQ(slope_vs_gravity(floor_at(0, 20.1), z, c).level, Level::error);
  Plane r;
  r.normal = Eigen::Vector3d(0.0, std::sin(7.9 * DEG), std::cos(7.9 * DEG));
  EXPECT_EQ(slope_vs_gravity(r, z, c).level, Level::ok);
  r.normal = Eigen::Vector3d(0.0, std::sin(8.1 * DEG), std::cos(8.1 * DEG));
  EXPECT_EQ(slope_vs_gravity(r, z, c).level, Level::warning);
  r.normal = Eigen::Vector3d(0.0, std::sin(15.1 * DEG), std::cos(15.1 * DEG));
  EXPECT_EQ(slope_vs_gravity(r, z, c).level, Level::error);
}

TEST(Angles, PlaneVsBaseAndBaseTilt)
{
  EXPECT_NEAR(plane_vs_base_deg(floor_at(0, 3.0)), 3.0, 1e-9);
  EXPECT_NEAR(base_tilt_deg(gravity_up_in_base(rpy(0, 10), Eigen::Quaterniond::Identity(), 0, 0)), 10.0, 1e-9);
}
