#include "ground_estimator_cpp/traversability.hpp"

#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace ground_estimator_cpp
{

namespace
{
constexpr double STEP_TOLERANCE_M = 1e-5;  // absorbs the rounding of max_step_fraction 0.3333
constexpr double SAME_EDGE_M = 1e-9;
}  // namespace

Verdict classify_cluster(double edge_step_m, double slope_deg, bool slope_known, const TraversabilityConfig & cfg)
{
  if (edge_step_m > max_step_m(cfg) + STEP_TOLERANCE_M) {
    return {Cls::obstacle, "step_height"};
  }
  if (slope_known && slope_deg > cfg.max_slope_deg) {
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

// Unclustered points with |h| > limit that have at least min_points such points (itself included) within radius.
void lethal_unclustered(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<float> & heights, double limit, double radius,
  int min_points, pcl::PointCloud<pcl::PointXYZ> & out)
{
  const std::size_t n = std::min(points.size(), heights.size());
  auto tall = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (std::size_t i = 0; i < n; ++i) {
    if (std::fabs(static_cast<double>(heights[i])) > limit) {
      tall->push_back(points[i]);
    }
  }
  if (tall->empty()) {
    return;
  }
  if (min_points <= 1) {
    append(out, *tall);
    return;
  }
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(tall);
  std::vector<int> found;
  std::vector<float> squared;
  for (const auto & p : *tall) {
    if (tree.radiusSearch(p, radius, found, squared) >= min_points) {
      out.push_back(p);
    }
  }
}

}  // namespace

pcl::PointCloud<pcl::PointXYZ> lethal_marks(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<float> & filtered_heights,
  const std::vector<float> & raw_heights, double limit, double radius, int min_points)
{
  // The height a point clears the floor by on both counts: the smaller magnitude when both lie on one side, else 0.
  const std::size_t n = std::min({points.size(), filtered_heights.size(), raw_heights.size()});
  std::vector<float> both(n, 0.0f);
  for (std::size_t i = 0; i < n; ++i) {
    const float f = filtered_heights[i];
    const float r = raw_heights[i];
    if ((f > 0.0f) == (r > 0.0f)) {
      both[i] = std::fabs(f) < std::fabs(r) ? f : r;
    }
  }
  pcl::PointCloud<pcl::PointXYZ> out;
  lethal_unclustered(points, both, limit, radius, min_points, out);
  return out;
}

Traversability apply_rule(
  const std::vector<Cluster> & above, const std::vector<Cluster> & below,
  const pcl::PointCloud<pcl::PointXYZ> & unclustered_above, const std::vector<float> & unclustered_above_h,
  const pcl::PointCloud<pcl::PointXYZ> & unclustered_below, const std::vector<float> & unclustered_below_h,
  std::vector<Region> regions, const EdgesConfig & edges, const TraversabilityConfig & cfg)
{
  Traversability out;
  std::vector<Judged> judged;
  for (const auto * group : {&above, &below}) {
    for (const Cluster & c : *group) {
      const Verdict v = classify_cluster(c.edge_step_m, c.slope_deg, c.slope_known, cfg);
      judged.push_back({&c, v.cls});
      if (v.cls == Cls::obstacle) {
        append(c.kind == "drop" ? out.drop : out.obstacle, c.points);
      } else {
        const bool ramp = c.slope_known && c.slope_deg >= RAMP_MIN_SLOPE_DEG;
        const char * kind = ramp ? "ramp" : (c.kind == "drop" ? "drop" : "step");
        append(c.kind == "drop" ? out.warning_below : out.warning_above, c.points);
        out.warnings.push_back(area_of(c, kind, "tof"));
      }
    }
  }
  const double limit = max_step_m(cfg);
  lethal_unclustered(
    unclustered_above, unclustered_above_h, limit, edges.cluster_tolerance_m, cfg.lethal_min_points, out.obstacle);
  lethal_unclustered(
    unclustered_below, unclustered_below_h, limit, edges.cluster_tolerance_m, cfg.lethal_min_points, out.drop);

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
  const EdgesConfig & edges, const TraversabilityConfig & cfg)
{
  std::vector<WarningArea> areas;
  for (const Cluster & c : extract_clusters(mismatch_points_base, heights, "step", plane, edges, cfg.edge_band_m)) {
    areas.push_back(area_of(c, "colour", "colour"));
  }
  return areas;
}

bool within_deadband(const std::optional<double> & a, const std::optional<double> & b, double deadband)
{
  if (!a || !b) {
    return !a && !b;
  }
  return std::fabs(*a - *b) <= deadband;
}

}  // namespace ground_estimator_cpp
