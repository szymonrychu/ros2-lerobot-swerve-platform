#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include "head_aim_cpp/aim.hpp"
#include "head_aim_cpp/config.hpp"

namespace
{

using head_aim_cpp::Aim;
using head_aim_cpp::Config;
using head_aim_cpp::FootprintBox;
using head_aim_cpp::Point2;
using head_aim_cpp::Point3;

constexpr double DEG = M_PI / 180.0;
constexpr double TOL = 1e-9;
constexpr double STEP_M = 0.05;
const Point3 HEAD{0.15, 0.10, 0.35};  // head_link origin in base_link (head.urdf)
const Point2 STILL{0.0, 0.0};
constexpr double D435_VFOV = 58.0 * DEG;

// A straight path from the origin along bearing (rad) of the given length, points every STEP_M.
std::vector<Point2> straight(double bearing, double length)
{
  std::vector<Point2> path;
  for (double s = 0.0; s <= length + 1e-9; s += STEP_M) {
    path.push_back({s * std::cos(bearing), s * std::sin(bearing)});
  }
  return path;
}

// The full chain of the node for one path: direction, leading edge, look point, aim.
std::optional<Aim> aim_path(const std::vector<Point2> & path, const Point2 & cmd_vel, const Config & cfg)
{
  const auto dir = head_aim_cpp::travel_direction(path, cmd_vel, cfg.lookahead_m, cfg.min_travel_speed_mps);
  if (!dir) {
    return std::nullopt;
  }
  const Point2 look = head_aim_cpp::look_point(path, *dir, cfg.footprint, cfg.lookahead_m);
  return head_aim_cpp::aim(look, HEAD, head_aim_cpp::leading_edge_point(*dir, cfg.footprint), D435_VFOV, cfg);
}

}  // namespace

TEST(TravelDirection, ForwardPathPointsAlongX)
{
  const auto dir = head_aim_cpp::travel_direction(straight(0.0, 2.0), STILL, 0.4, 0.02);
  ASSERT_TRUE(dir);
  EXPECT_NEAR(dir->x, 1.0, TOL);
  EXPECT_NEAR(dir->y, 0.0, TOL);
}

TEST(TravelDirection, ShortPathFallsBackToCmdVel)
{
  const auto dir = head_aim_cpp::travel_direction({{0.0, 0.0}}, {0.0, 0.1}, 0.4, 0.02);
  ASSERT_TRUE(dir);
  EXPECT_NEAR(dir->x, 0.0, TOL);
  EXPECT_NEAR(dir->y, 1.0, TOL);
  // Two coincident points are as degenerate as one.
  const auto same = head_aim_cpp::travel_direction({{0.2, 0.0}, {0.2, 0.0}}, {0.1, 0.0}, 0.4, 0.02);
  ASSERT_TRUE(same);
  EXPECT_NEAR(same->x, 1.0, TOL);
}

TEST(TravelDirection, NoPathAndNoSpeedGivesNothing)
{
  EXPECT_FALSE(head_aim_cpp::travel_direction({}, STILL, 0.4, 0.02));
  EXPECT_FALSE(head_aim_cpp::travel_direction({}, {0.01, 0.0}, 0.4, 0.02));  // below min_travel_speed_mps
}

TEST(LeadingEdge, ForwardPicksTheFrontEdge)
{
  const FootprintBox box;
  EXPECT_NEAR(head_aim_cpp::leading_edge_distance({1.0, 0.0}, box), 0.3225, TOL);
  const Point2 p = head_aim_cpp::leading_edge_point({1.0, 0.0}, box);
  EXPECT_NEAR(p.x, 0.3225, TOL);
  EXPECT_NEAR(p.y, 0.0, TOL);
}

TEST(LeadingEdge, LeftSlidePicksTheLeftEdgeAndRearThePack)
{
  const FootprintBox box;
  EXPECT_NEAR(head_aim_cpp::leading_edge_distance({0.0, 1.0}, box), 0.196, TOL);
  EXPECT_NEAR(head_aim_cpp::leading_edge_distance({0.0, -1.0}, box), 0.196, TOL);
  EXPECT_NEAR(head_aim_cpp::leading_edge_distance({-1.0, 0.0}, box), 0.215, TOL);
}

TEST(LeadingEdge, DiagonalPicksTheCornerSupport)
{
  const FootprintBox box;
  const double c = std::sqrt(0.5);
  EXPECT_NEAR(head_aim_cpp::leading_edge_distance({c, c}, box), (0.3225 + 0.196) * c, TOL);
  EXPECT_NEAR(head_aim_cpp::leading_edge_distance({c, -c}, box), (0.3225 + 0.196) * c, TOL);
}

TEST(Aim, ForwardPathAimsStraightAheadUpToTheHeadOffset)
{
  const Config cfg;
  const auto path = straight(0.0, 3.0);
  const Point2 look = head_aim_cpp::look_point(path, {1.0, 0.0}, cfg.footprint, cfg.lookahead_m);
  // The first point beyond 0.3225 + 0.4 = 0.7225 m.
  EXPECT_NEAR(look.x, 0.75, 1e-6);
  const auto a = aim_path(path, STILL, cfg);
  ASSERT_TRUE(a);
  EXPECT_NEAR(a->pan, std::atan2(0.0 - 0.10, 0.75 - 0.15), 1e-6);
  EXPECT_LT(a->pan, 0.0);
  EXPECT_FALSE(a->clamped);
}

TEST(Aim, LeftSlideAimsNearPlus90)
{
  const Config cfg;
  const auto path = straight(M_PI / 2.0, 3.0);
  const auto a = aim_path(path, STILL, cfg);
  ASSERT_TRUE(a);
  // Look point: first point beyond 0.196 + 0.4 = 0.596 m to the left, (0, 0.6); its bearing from the head origin,
  // atan2(0.5, -0.15) = 106.7 deg, lies past the pan limit by the head x offset, so the pan is clamped to +90 deg.
  const Point2 look = head_aim_cpp::look_point(path, {0.0, 1.0}, cfg.footprint, cfg.lookahead_m);
  EXPECT_NEAR(look.x, 0.0, 1e-9);
  EXPECT_NEAR(look.y, 0.6, 1e-6);
  EXPECT_NEAR(a->pan, cfg.pan_max_rad, 1e-6);
  EXPECT_TRUE(a->clamped);
}

TEST(Aim, PathBehindClampsPanAndFlagsIt)
{
  const Config cfg;
  const auto a = aim_path(straight(M_PI, 3.0), STILL, cfg);
  ASSERT_TRUE(a);
  EXPECT_NEAR(std::abs(a->pan), 90.0 * DEG, 1e-4);
  EXPECT_TRUE(a->clamped);
}

TEST(Aim, PathTurningAfterThirtyCentimetresLooksBeyondTheTurn)
{
  const Config cfg;
  std::vector<Point2> path = straight(0.0, 0.3);
  for (double y = STEP_M; y <= 1.0 + 1e-9; y += STEP_M) {
    path.push_back({0.3, y});
  }
  const auto dir = head_aim_cpp::travel_direction(path, STILL, cfg.lookahead_m, cfg.min_travel_speed_mps);
  ASSERT_TRUE(dir);
  const Point2 look = head_aim_cpp::look_point(path, *dir, cfg.footprint, cfg.lookahead_m);
  // No point projects beyond the leading edge + lookahead, so the look point is the tip, not the straight extension.
  EXPECT_NEAR(look.x, 0.3, 1e-9);
  EXPECT_NEAR(look.y, 1.0, 1e-9);
  const auto a = aim_path(path, STILL, cfg);
  ASSERT_TRUE(a);
  EXPECT_NEAR(a->pan, std::atan2(1.0 - 0.10, 0.3 - 0.15), 1e-6);
}

TEST(Aim, InPlaceRotationAimsAtThePostTurnPath)
{
  // The path does not rotate with the robot: while the robot is yawed 120 deg off the path, the path lies at bearing
  // 120 deg in base_link and the head waits at the pan limit; at 60 deg off it aims at the path itself.
  const Config cfg;
  const auto before = aim_path(straight(120.0 * DEG, 3.0), STILL, cfg);
  ASSERT_TRUE(before);
  EXPECT_NEAR(before->pan, cfg.pan_max_rad, 1e-9);
  EXPECT_TRUE(before->clamped);

  const auto path = straight(60.0 * DEG, 3.0);
  const Point2 dir{std::cos(60.0 * DEG), std::sin(60.0 * DEG)};
  const Point2 look = head_aim_cpp::look_point(path, dir, cfg.footprint, cfg.lookahead_m);
  const auto later = aim_path(path, STILL, cfg);
  ASSERT_TRUE(later);
  EXPECT_NEAR(later->pan, std::atan2(look.y - 0.10, look.x - 0.15), 1e-6);
  EXPECT_NEAR(later->pan, 60.0 * DEG, 10.0 * DEG);  // 67.7 deg: the look point seen from the head origin offset
  EXPECT_FALSE(later->clamped);
}

TEST(Aim, TiltPutsTheFrustumBottomJustPastTheLeadingEdge)
{
  const Config cfg;
  const auto a = aim_path(straight(0.0, 3.0), STILL, cfg);
  ASSERT_TRUE(a);
  // Head at 0.35 m; leading edge point (0.3225, 0); horizontal distance hypot(0.1725, 0.10) + 0.05 margin.
  const double expected = std::atan2(0.35, std::hypot(0.1725, 0.10) + 0.05) - 29.0 * DEG;
  EXPECT_NEAR(a->tilt, expected, 1e-9);
  EXPECT_NEAR(a->tilt, 25.53 * DEG, 0.05 * DEG);  // hand computed: 54.53 - 29 deg
}

TEST(Aim, TiltIsClampedWhenTheEdgeIsTooClose)
{
  Config cfg;
  cfg.floor_margin_m = 0.0;
  // Leading edge right under the head: atan2(0.35, 0) - 29 deg = 61 deg > 45 deg.
  const Aim a = head_aim_cpp::aim({2.0, 0.10}, HEAD, {HEAD.x, HEAD.y}, D435_VFOV, cfg);
  EXPECT_NEAR(a.tilt, 45.0 * DEG, 1e-4);
  EXPECT_NEAR(a.tilt, cfg.tilt_max_rad, 1e-12);
  EXPECT_TRUE(a.clamped);
}

TEST(Aim, ShortPathFallsBackToCmdVelDirection)
{
  const Config cfg;
  const auto a = aim_path({}, {0.0, 0.2}, cfg);  // sliding left, no path
  ASSERT_TRUE(a);
  EXPECT_NEAR(a->pan, cfg.pan_max_rad, 1e-9);
  EXPECT_FALSE(aim_path({}, STILL, cfg));
}

TEST(LookPoint, EmptyPathLooksAlongTheDirection)
{
  const Config cfg;
  const Point2 look = head_aim_cpp::look_point({}, {1.0, 0.0}, cfg.footprint, cfg.lookahead_m);
  EXPECT_NEAR(look.x, 0.3225 + 0.4, 1e-12);
  EXPECT_NEAR(look.y, 0.0, 1e-12);
}

TEST(VerticalFov, PinholeFormula)
{
  EXPECT_NEAR(head_aim_cpp::vertical_fov(480.0, 240.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(head_aim_cpp::vertical_fov(480.0, 432.9), 2.0 * std::atan(240.0 / 432.9), 1e-12);
}
