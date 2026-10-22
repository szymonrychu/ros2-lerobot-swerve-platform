#include "ground_estimator_cpp/occlusion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>

namespace ground_estimator_cpp
{

namespace
{

constexpr double DEG_TO_RAD = M_PI / 180.0;
// A vertical face (a wall, a box front) spreads over about range * bin width of horizontal range inside one azimuth
// bin, so points that close in range are compared only along the sensor ray (see occluded_mask); below this floor the
// gap is never smaller.
constexpr double MIN_NEAR_RANGE_M = 0.003;

// A near occluder shadows only a point on (nearly) its own ray: this share of the elevation margin.
constexpr double NEAR_RAY_FRACTION = 0.25;

// A point less than this far behind a near occluder in range is on the occluder's own surface, not behind it: the face
// below an edge seen at a graze lies a fraction of a degree from the edge's ray, and a vertical face spreads over a few
// centimetres of range inside one azimuth bin.
constexpr double SAME_SURFACE_M = 0.04;

// An occluder is supported by its two nearer neighbours in the bin: its height is the median of the three, so one noisy
// return is never a shadow caster (a box top is, a single +4 cm outlier on a noisy floor is not).
constexpr std::size_t SUPPORT_POINTS = 3;

struct Polar
{
  double range = 0.0;      // horizontal distance from the sensor
  double elevation = 0.0;  // atan2(h - sensor_height, range)
  double height = 0.0;
  std::size_t index = 0;
  double occluder_height = -std::numeric_limits<double>::infinity();  // supported height, -inf without full support
  double occluder_elevation = 0.0;                                    // atan2(occluder_height - sensor_height, range)
};

}  // namespace

std::vector<bool> occluded_mask(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const std::vector<float> & heights,
  const Eigen::Vector2d & sensor_xy, double sensor_height_m, const OcclusionConfig & cfg)
{
  const std::size_t n = points_base.size();
  std::vector<bool> mask(n, false);
  if (!cfg.enabled || n == 0 || heights.size() != n) {
    return mask;
  }
  const double bin_rad = cfg.azimuth_bin_deg * DEG_TO_RAD;
  const double margin_rad = cfg.elevation_margin_deg * DEG_TO_RAD;
  std::map<long, std::vector<Polar>> bins;  // NOLINT(runtime/int)
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p = points_base[i];
    const double h = heights[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(h)) {
      continue;
    }
    const double dx = p.x - sensor_xy.x();
    const double dy = p.y - sensor_xy.y();
    const double range = std::hypot(dx, dy);
    if (range <= 0.0) {
      continue;
    }
    const long bin = std::lround(std::floor(std::atan2(dy, dx) / bin_rad));  // NOLINT(runtime/int)
    bins[bin].push_back({range, std::atan2(h - sensor_height_m, range), h, i});
  }
  for (auto & entry : bins) {
    auto & pts = entry.second;
    std::sort(pts.begin(), pts.end(), [](const Polar & a, const Polar & b) { return a.range < b.range; });
    for (std::size_t k = SUPPORT_POINTS - 1; k < pts.size(); ++k) {
      std::array<double, SUPPORT_POINTS> window;
      for (std::size_t w = 0; w < SUPPORT_POINTS; ++w) {
        window[w] = heights[pts[k - w].index];
      }
      std::sort(window.begin(), window.end());
      pts[k].occluder_height = window[SUPPORT_POINTS / 2];
      pts[k].occluder_elevation = std::atan2(pts[k].occluder_height - sensor_height_m, pts[k].range);
    }
    // Far occluders (at least one bin width of range nearer) cast a shadow over everything lower in elevation; their
    // running maximum of elevation, with the height of that point, is the shadow horizon. Near occluders (closer than
    // that) shadow only along their own ray: a mixed pixel on the ray through an edge, never the face below an edge.
    double horizon = -std::numeric_limits<double>::infinity();
    double horizon_height = 0.0;
    std::size_t next = 0;  // first point not yet a far occluder of the point tested
    for (std::size_t k = 0; k < pts.size(); ++k) {
      const Polar & p = pts[k];
      const double near_range = std::max(MIN_NEAR_RANGE_M, p.range * bin_rad);
      while (next < k && pts[next].range <= p.range - near_range) {
        const double h = pts[next].occluder_height;
        if (h >= cfg.min_occluder_height_m && pts[next].occluder_elevation > horizon) {
          horizon = pts[next].occluder_elevation;
          horizon_height = h;
        }
        ++next;
      }
      const double h = heights[p.index];
      bool occluded = horizon > -std::numeric_limits<double>::infinity() &&
                      h <= horizon_height - cfg.min_height_drop_m && p.elevation <= horizon + margin_rad;
      for (std::size_t j = next; !occluded && j < k; ++j) {
        // The near occluder is a supported point (its own height and elevation count: the median of its neighbours is a
        // height no point has), a surface of its own at least SAME_SURFACE_M nearer.
        const Polar & q = pts[j];
        occluded = p.range - q.range >= SAME_SURFACE_M && q.occluder_height >= cfg.min_occluder_height_m &&
                   h <= q.height - cfg.min_height_drop_m &&
                   std::fabs(p.elevation - q.elevation) <= NEAR_RAY_FRACTION * margin_rad;
      }
      mask[p.index] = occluded;
    }
  }
  return mask;
}

}  // namespace ground_estimator_cpp
