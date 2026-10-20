#include "ground_estimator_cpp/traversability.hpp"

#include <cmath>
#include <utility>

namespace ground_estimator_cpp
{

namespace
{
constexpr double STEP_TOLERANCE_M = 1e-5;  // absorbs the rounding of max_step_fraction 0.3333
constexpr double SAME_EDGE_M = 1e-9;
}  // namespace

double max_step_m(const TraversabilityConfig & cfg) { return 2.0 * cfg.wheel_radius_m * cfg.max_step_fraction; }

Verdict classify_cluster(double edge_step_m, double slope_deg, const TraversabilityConfig & cfg)
{
  if (edge_step_m > max_step_m(cfg) + STEP_TOLERANCE_M) {
    return {Cls::obstacle, "step_height"};
  }
  if (slope_deg > cfg.max_slope_deg) {
    return {Cls::obstacle, "slope"};
  }
  return {Cls::warning, "traversable"};
}

namespace
{

WarningArea area_of(const Cluster & c, const char * kind, const char * source)
{
  WarningArea a;
  a.kind = kind;
  a.polygon = c.hull;
  a.height_m = c.median_height_m;
  a.slope_deg = c.slope_deg;
  a.source = source;
  return a;
}

void append(pcl::PointCloud<pcl::PointXYZ> & to, const pcl::PointCloud<pcl::PointXYZ> & from)
{
  to.insert(to.end(), from.begin(), from.end());
}

// Class of a cluster; stores it for the region matching.
struct Judged
{
  const Cluster * cluster;
  Cls cls;
};

void lethal_unclustered(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<float> & heights, double limit,
  pcl::PointCloud<pcl::PointXYZ> & out)
{
  for (std::size_t i = 0; i < points.size() && i < heights.size(); ++i) {
    if (std::fabs(static_cast<double>(heights[i])) > limit) {
      out.push_back(points[i]);
    }
  }
}

}  // namespace

Traversability apply_rule(
  const std::vector<Cluster> & above, const std::vector<Cluster> & below,
  const pcl::PointCloud<pcl::PointXYZ> & unclustered_above, const std::vector<float> & unclustered_above_h,
  const pcl::PointCloud<pcl::PointXYZ> & unclustered_below, const std::vector<float> & unclustered_below_h,
  std::vector<Region> regions, const TraversabilityConfig & cfg)
{
  Traversability out;
  std::vector<Judged> judged;
  for (const auto * group : {&above, &below}) {
    for (const Cluster & c : *group) {
      const Verdict v = classify_cluster(c.edge_step_m, c.slope_deg, cfg);
      judged.push_back({&c, v.cls});
      if (v.cls == Cls::obstacle) {
        append(c.kind == "drop" ? out.drop : out.obstacle, c.points);
      } else {
        const char * kind = c.slope_deg >= RAMP_MIN_SLOPE_DEG ? "ramp" : (c.kind == "drop" ? "drop" : "step");
        out.warnings.push_back(area_of(c, kind, "tof"));
      }
    }
  }
  const double limit = max_step_m(cfg);
  lethal_unclustered(unclustered_above, unclustered_above_h, limit, out.obstacle);
  lethal_unclustered(unclustered_below, unclustered_below_h, limit, out.drop);

  for (Region & r : regions) {
    r.cls = "warning";
    for (const Judged & j : judged) {
      const Cluster & c = *j.cluster;
      const bool same = c.kind == r.kind && std::fabs(c.distance_m - r.distance_m) < SAME_EDGE_M &&
                        c.points.size() == r.points && (c.edge_point - r.point).norm() < SAME_EDGE_M;
      if (same) {
        r.cls = j.cls == Cls::obstacle ? "obstacle" : "warning";
        break;
      }
    }
  }
  out.regions = std::move(regions);
  return out;
}

std::vector<WarningArea> colour_warning_areas(
  const pcl::PointCloud<pcl::PointXYZ> & mismatch_points_base, const std::vector<float> & heights, const Plane & plane,
  const EdgesConfig & edges, const TraversabilityConfig & cfg, pcl::PointCloud<pcl::PointXYZ> * obstacle_points)
{
  std::vector<WarningArea> areas;
  for (const Cluster & c : extract_clusters(mismatch_points_base, heights, "step", plane, edges, cfg.edge_band_m)) {
    WarningArea a = area_of(c, "colour", "colour");
    if (classify_cluster(c.edge_step_m, c.slope_deg, cfg).cls == Cls::obstacle) {
      a.cls = "obstacle";
      if (obstacle_points != nullptr) {
        append(*obstacle_points, c.points);
      }
    }
    areas.push_back(std::move(a));
  }
  return areas;
}

}  // namespace ground_estimator_cpp
