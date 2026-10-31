#include "head_aim_cpp/aim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace head_aim_cpp
{

namespace
{

constexpr double MIN_LENGTH_M = 1e-6;  // shorter path spans give no direction

// The value limited to [low, high]; sets clamped when it had to be limited.
double clamp_flag(double value, double low, double high, bool & clamped)
{
  const double limited = std::clamp(value, low, high);
  if (limited != value) {
    clamped = true;
  }
  return limited;
}

}  // namespace

std::optional<Point2> travel_direction(
  const std::vector<Point2> & path, const Point2 & cmd_vel, double lookahead_m, double min_travel_speed_mps)
{
  if (path.size() >= 2) {
    const Point2 & first = path.front();
    Point2 end = path.back();
    double along = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      along += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      if (along >= lookahead_m) {
        end = path[i];
        break;
      }
    }
    const double dx = end.x - first.x;
    const double dy = end.y - first.y;
    const double length = std::hypot(dx, dy);
    if (length > MIN_LENGTH_M) {
      return Point2{dx / length, dy / length};
    }
  }
  const double speed = std::hypot(cmd_vel.x, cmd_vel.y);
  if (speed >= min_travel_speed_mps && speed > 0.0) {
    return Point2{cmd_vel.x / speed, cmd_vel.y / speed};
  }
  return std::nullopt;
}

double leading_edge_distance(const Point2 & direction, const FootprintBox & box)
{
  const std::array<Point2, 4> corners = {
    Point2{box.max_x, box.max_y}, Point2{box.max_x, box.min_y}, Point2{box.min_x, box.max_y},
    Point2{box.min_x, box.min_y}};
  double support = std::numeric_limits<double>::lowest();
  for (const Point2 & c : corners) {
    support = std::max(support, c.x * direction.x + c.y * direction.y);
  }
  return support;
}

Point2 leading_edge_point(const Point2 & direction, const FootprintBox & box)
{
  const double d = leading_edge_distance(direction, box);
  return Point2{direction.x * d, direction.y * d};
}

Point2 look_point(
  const std::vector<Point2> & path, const Point2 & direction, const FootprintBox & box, double lookahead_m)
{
  const double threshold = leading_edge_distance(direction, box) + lookahead_m;
  for (const Point2 & p : path) {
    if (p.x * direction.x + p.y * direction.y > threshold) {
      return p;
    }
  }
  if (path.empty()) {
    return Point2{direction.x * threshold, direction.y * threshold};
  }
  return path.back();
}

Aim aim(
  const Point2 & look, const Point3 & head_origin, const Point2 & leading_edge, double vfov_rad, const Config & cfg)
{
  Aim result;
  const double pan = std::atan2(look.y - head_origin.y, look.x - head_origin.x);
  const double edge_distance =
    std::hypot(leading_edge.x - head_origin.x, leading_edge.y - head_origin.y) + cfg.floor_margin_m;
  const double tilt = std::atan2(head_origin.z, edge_distance) - vfov_rad / 2.0;
  result.pan = clamp_flag(pan, cfg.pan_min_rad, cfg.pan_max_rad, result.clamped);
  result.tilt = clamp_flag(tilt, cfg.tilt_min_rad, cfg.tilt_max_rad, result.clamped);
  return result;
}

double vertical_fov(double height, double fy) { return 2.0 * std::atan(height / (2.0 * fy)); }

}  // namespace head_aim_cpp
