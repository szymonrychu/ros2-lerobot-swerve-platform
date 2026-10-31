// Aim geometry of the head: where the robot travels, which footprint edge leads, which path point to look at and the
// pan/tilt that puts the D435 on it. Pure functions in base_link (x forward, y left, z up from the floor), no ROS types.
#ifndef HEAD_AIM_CPP__AIM_HPP_
#define HEAD_AIM_CPP__AIM_HPP_

#include <optional>
#include <vector>

#include "head_aim_cpp/config.hpp"

namespace head_aim_cpp
{

// A point or vector in the base_link xy plane, metres.
struct Point2
{
  double x = 0.0;
  double y = 0.0;
};

// A point in base_link, metres.
struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// A head pose, rad: pan + = left, tilt + = down. clamped is true when either angle was limited to the configured range.
struct Aim
{
  double pan = 0.0;
  double tilt = 0.0;
  bool clamped = false;
};

// Unit vector of the travel direction: from the first path point to the point lookahead_m along the path (the tip when
// the path is shorter). Falls back to the (vx, vy) of cmd_vel when the path has fewer than 2 points or no length and
// |cmd_vel| >= min_travel_speed_mps.
// @param path the path in base_link (Point2 list, nearest first).
// @param cmd_vel the commanded (vx, vy), m/s.
// @param lookahead_m arc length along the path that sets the direction, metres.
// @param min_travel_speed_mps slowest cmd_vel that gives a direction, m/s.
// @return the unit direction (Point2), or nullopt when neither the path nor cmd_vel gives one.
std::optional<Point2> travel_direction(
  const std::vector<Point2> & path, const Point2 & cmd_vel, double lookahead_m, double min_travel_speed_mps);

// Support of the footprint box along direction: max over its 4 corners of corner . direction. The corners that reach it
// are the leading wheels for any swerve direction (forward: front pair, left slide: left pair, diagonal: one corner).
// @param direction unit travel direction (Point2).
// @param box footprint bounding box in base_link (FootprintBox).
// @return metres from base_link to the leading edge along direction (double).
double leading_edge_distance(const Point2 & direction, const FootprintBox & box);

// The point where the travel ray from base_link crosses the leading edge: direction * leading_edge_distance.
// @param direction unit travel direction (Point2).
// @param box footprint bounding box (FootprintBox).
// @return the leading edge point in base_link (Point2).
Point2 leading_edge_point(const Point2 & direction, const FootprintBox & box);

// The first path point whose projection on direction exceeds leading_edge_distance + lookahead_m; the path tip when no
// point does; for an empty path (direction from cmd_vel) the point that far along direction.
// @param path the path in base_link (Point2 list, nearest first, may be empty).
// @param direction unit travel direction (Point2).
// @param box footprint bounding box (FootprintBox).
// @param lookahead_m distance beyond the leading edge, metres.
// @return the look point (Point2).
Point2 look_point(
  const std::vector<Point2> & path, const Point2 & direction, const FootprintBox & box, double lookahead_m);

// Pan and tilt for the head. pan = bearing from the head origin to the look point; tilt (+ = down) = atan2(head height,
// horizontal distance from the head origin to the leading edge point + floor_margin_m) - vfov / 2, so the bottom of the
// frustum lands just past the leading wheels. Both are clamped to the configured ranges.
// @param look the look point in base_link (Point2).
// @param head_origin the head origin (on the tilt axis) in base_link, z above the floor (Point3).
// @param leading_edge the leading edge point in base_link (Point2).
// @param vfov_rad vertical field of view of the camera, rad.
// @param cfg ranges and floor_margin_m (Config).
// @return the clamped pan and tilt (Aim).
Aim aim(
  const Point2 & look, const Point3 & head_origin, const Point2 & leading_edge, double vfov_rad, const Config & cfg);

// Vertical field of view of a pinhole camera: 2 * atan(height / (2 fy)).
// @param height image height, pixels.
// @param fy focal length in y, pixels (> 0).
// @return rad (double).
double vertical_fov(double height, double fy);

}  // namespace head_aim_cpp

#endif  // HEAD_AIM_CPP__AIM_HPP_
