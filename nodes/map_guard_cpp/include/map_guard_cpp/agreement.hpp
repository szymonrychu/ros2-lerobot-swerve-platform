// Scan-to-map agreement: the share of laser endpoints that land next to an occupied cell of the current map.
// Port of nodes/map_guard/map_guard/agreement.py (numpy replaced by plain loops over row-major vectors).
//
// The scan is projected through the map <- laser transform, so a pose jump shows up as endpoints landing in free
// space. Points outside the map or in unknown space (no known cell within the hit radius) are not evaluated.
#ifndef MAP_GUARD_CPP__AGREEMENT_HPP_
#define MAP_GUARD_CPP__AGREEMENT_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "map_guard_cpp/timing.hpp"

namespace map_guard_cpp
{

// Occupancy value of an unknown cell.
constexpr int UNKNOWN = -1;

struct Point2
{
  double x;
  double y;
};
using Points = std::vector<Point2>;
// (x, y, z, w)
using Quaternion = std::array<double, 4>;
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<std::array<double, 3>, 3>;
// Row-major boolean mask (0 or 1 per cell).
using Mask = std::vector<std::uint8_t>;

// An occupancy grid (nav_msgs/OccupancyGrid without ROS). cells is row-major: row = y index, column = x index;
// -1 unknown, 0..100 occupancy probability.
struct GridMap
{
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  int width = 0;
  int height = 0;
  std::vector<std::int8_t> cells;
};

// A grid with its dilated hit and known masks, computed once per map message.
struct PreparedMap
{
  GridMap grid;
  Mask hit_mask;         // an occupied cell is within the hit radius
  Mask known_mask;       // a known cell is within the hit radius
  long known_cells = 0;  // NOLINT(runtime/int) - number of known cells
};

// One agreement sample: ratio = agreeing / evaluated.
struct AgreementResult
{
  double ratio;
  long evaluated;  // NOLINT(runtime/int)
  long agreeing;   // NOLINT(runtime/int)
};

// Square binary dilation (Chebyshev distance <= radius) of a width x height mask; radius <= 0 copies it.
Mask dilate(const Mask & mask, int width, int height, int radius);

// Precompute the dilated masks used to score scans against grid.
PreparedMap prepare_map(GridMap grid, int occupied_threshold, int hit_radius_cells);

// Laser-frame endpoints of the finite ranges within [range_min, min(range_max, max_range_m)]; beam i is at
// angle_min + angle_increment * i.
Points scan_endpoints(
  const std::vector<float> & ranges, double angle_min, double angle_increment, double range_min, double range_max,
  double max_range_m);
Points scan_endpoints(
  const std::vector<double> & ranges, double angle_min, double angle_increment, double range_min, double range_max,
  double max_range_m);

// Quaternion of a rotation about z.
Quaternion yaw_quaternion(double yaw);

// Yaw of an orientation (the map origin of an OccupancyGrid), rad.
double quaternion_yaw(const Quaternion & q);

// 3x3 rotation matrix of a quaternion.
Matrix3 quaternion_matrix(const Quaternion & q);

// Transform laser-plane points (z = 0) into the map frame and drop z.
Points transform_points(const Points & points, const Vector3 & translation, const Quaternion & rotation);

// Score map-frame endpoints against the prepared map: nullopt without a map (nullptr), with an empty map, without
// points or with fewer than min_points evaluated points.
std::optional<AgreementResult> agreement(const Points & points_map, const PreparedMap * prepared, int min_points);

// Median of a non-empty sample (mean of the two middle values for an even count), as Python's statistics.median.
double median(std::vector<double> values);

// Mismatch: the rolling median ratio stays below mismatch_ratio for mismatch_s on an established map. A map only
// counts once it has min_map_cells_known known cells and the median was at least healthy_ratio on it (armed).
class MismatchMonitor
{
public:
  MismatchMonitor(
    double mismatch_ratio, double healthy_ratio, double mismatch_s,
    long min_map_cells_known,  // NOLINT(runtime/int)
    int window);

  // Rolling median of the ratio, nullopt before the first sample.
  std::optional<double> median() const;

  // Add one sample at now; true while the mismatch condition has held for mismatch_s.
  bool update(double now, double ratio, long known_cells);  // NOLINT(runtime/int)

  // Restart the mismatch hold (the map was found healthy again).
  void reset_hold();

  // Forget everything (the map was reset): disarmed until the new map is healthy.
  void reset();

  bool armed() const { return armed_; }

private:
  double mismatch_ratio_;
  double healthy_ratio_;
  long min_map_cells_known_;  // NOLINT(runtime/int)
  std::size_t window_;
  std::deque<double> samples_;
  HoldTimer hold_;
  bool armed_ = false;
};

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__AGREEMENT_HPP_
