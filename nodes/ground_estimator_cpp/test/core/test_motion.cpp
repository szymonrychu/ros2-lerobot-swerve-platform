#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "ground_estimator_cpp/motion.hpp"

namespace
{

using ground_estimator_cpp::is_moving;
using ground_estimator_cpp::MotionConfig;
using ground_estimator_cpp::rotation_rate_deg_s;

constexpr double DEG = M_PI / 180.0;

Eigen::Matrix3d about(const Eigen::Vector3d & axis, double deg)
{
  return Eigen::AngleAxisd(deg * DEG, axis.normalized()).toRotationMatrix();
}

}  // namespace

TEST(Motion, NoRotationIsZeroRate)
{
  const Eigen::Matrix3d r = about(Eigen::Vector3d(1, 2, 3), 40.0);
  EXPECT_NEAR(rotation_rate_deg_s(r, r, 0.1), 0.0, 1e-9);
}

TEST(Motion, RateIsTheRelativeAngleOverTheTime)
{
  // A 5.73 deg pan in 0.1 s is 1 rad/s = 57.3 deg/s, whatever the starting orientation.
  const Eigen::Matrix3d start = about(Eigen::Vector3d(0.3, -0.2, 1.0), 25.0);
  const Eigen::Matrix3d turn = about(Eigen::Vector3d::UnitZ(), 5.729577951308232);
  EXPECT_NEAR(rotation_rate_deg_s(start, turn * start, 0.1), 57.29577951308232, 1e-6);
  EXPECT_NEAR(rotation_rate_deg_s(start, turn * start, 0.2), 28.64788975654116, 1e-6);
}

TEST(Motion, RateDoesNotDependOnTheDirectionOfTheTurn)
{
  const Eigen::Matrix3d start = about(Eigen::Vector3d::UnitY(), 30.0);
  const Eigen::Matrix3d left = about(Eigen::Vector3d::UnitZ(), 2.0) * start;
  const Eigen::Matrix3d right = about(Eigen::Vector3d::UnitZ(), -2.0) * start;
  EXPECT_NEAR(rotation_rate_deg_s(start, left, 0.1), rotation_rate_deg_s(start, right, 0.1), 1e-9);
  EXPECT_NEAR(rotation_rate_deg_s(start, left, 0.1), 20.0, 1e-6);
}

TEST(Motion, GateIsStrictlyAboveTheLimit)
{
  MotionConfig cfg;
  cfg.max_head_rate_deg_s = 3.0;
  EXPECT_FALSE(is_moving(0.0, cfg));
  EXPECT_FALSE(is_moving(3.0, cfg));
  EXPECT_TRUE(is_moving(3.01, cfg));
  EXPECT_TRUE(is_moving(57.3, cfg));
}

TEST(Motion, DisabledGateNeverMoves)
{
  MotionConfig cfg;
  cfg.enabled = false;
  EXPECT_FALSE(is_moving(1000.0, cfg));
}
