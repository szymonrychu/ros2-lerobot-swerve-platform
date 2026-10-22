// The obstacle versus warning rule: a cluster is lethal when its edge step exceeds a third of the wheel diameter or its
// measurable surface is steeper than traversability.max_slope_deg, otherwise the base rides over it at reduced speed.
#ifndef GROUND_ESTIMATOR_CPP__TRAVERSABILITY_HPP_
#define GROUND_ESTIMATOR_CPP__TRAVERSABILITY_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/edges.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

enum class Cls
{
  obstacle,
  warning
};

// Class of a cluster and why.
struct Verdict
{
  Cls cls = Cls::warning;
  std::string reason;  // "step_height", "slope" (obstacle) or "traversable" (warning)
};

// A slow-down area in base_link (every published area is a warning).
struct WarningArea
{
  std::string kind;                      // "step", "drop", "ramp" (slope >= RAMP_MIN_SLOPE_DEG) or "colour"
  std::vector<Eigen::Vector2d> polygon;  // cluster hull in base_link
  std::optional<double> height_m;        // signed median height above the floor; nullopt: not measured
  std::optional<double> slope_deg;       // nullopt: not measured
  std::string source;                    // "tof" or "colour"
};

// Output of the rule.
struct Traversability
{
  pcl::PointCloud<pcl::PointXYZ> obstacle, drop;  // lethal points
  // Points of the warning ToF clusters above and below the floor (a cell holding one is not lethal; consumers use them
  // to keep a cell lethal-free).
  pcl::PointCloud<pcl::PointXYZ> warning_above, warning_below;
  std::vector<WarningArea> warnings;
  std::vector<Region> regions;  // the input regions with cls set
};

constexpr double RAMP_MIN_SLOPE_DEG = 2.0;

// Classify one cluster.
// @param edge_step_m height of the cluster at its edge line in metres.
// @param slope_deg angle of the cluster surface to the floor in degrees.
// @param slope_known whether slope_deg is measurable; an unknown slope is ignored and the edge step alone decides.
// @param cfg the rule settings.
// @return obstacle when edge_step_m > max_step_m(cfg) (with a 10 um tolerance for the rounded fraction 0.3333) or
//         (slope_known and slope_deg > cfg.max_slope_deg); warning otherwise (exactly at a threshold is a warning).
Verdict classify_cluster(double edge_step_m, double slope_deg, bool slope_known, const TraversabilityConfig & cfg);

// Apply the rule to the clusters of one frame.
// @param above step clusters.
// @param below drop clusters.
// @param unclustered_above points above the floor in no cluster, with signed heights parallel.
// @param unclustered_below points below the floor in no cluster, with signed heights parallel.
// @param regions regions of the clusters (regions_from_clusters); returned with cls set.
// @param edges clustering settings; edges.cluster_tolerance_m is the neighbour radius of the unclustered lethal points.
// @param cfg the rule settings; cfg.lethal_min_points is the fewest same-group points (counting the point itself)
//        within that radius for an unclustered point to be lethal.
// @return the lethal clouds (obstacle clusters plus unclustered points with |h| > max_step_m that have enough
//         neighbours), the warning areas of the warning clusters (none of their points are lethal), their points
//         (warning_above/warning_below) and the classed regions.
Traversability apply_rule(
  const std::vector<Cluster> & above, const std::vector<Cluster> & below,
  const pcl::PointCloud<pcl::PointXYZ> & unclustered_above, const std::vector<float> & unclustered_above_h,
  const pcl::PointCloud<pcl::PointXYZ> & unclustered_below, const std::vector<float> & unclustered_below_h,
  std::vector<Region> regions, const EdgesConfig & edges, const TraversabilityConfig & cfg);

// Lethal marks of one frame (the points published as obstacle or drop): the points whose own height in this frame and
// whose temporal (cell-median) height both clear the step on the same side of the floor, |h| > limit, and that have
// at least min_points such points (counting the point itself) within radius. The temporal filter moves every point
// of a cell to the cell's median, so a cell straddling an obstacle's edge lifts its floor points to the obstacle's
// height, and a mixed-pixel tail cell reads a few centimetres; neither is a lethal mark, and a lone tall return is not
// either (design D7 applied to the marks).
// @param points candidate points (in stable lethal cells) in base_link.
// @param filtered_heights signed temporal heights above the floor, parallel to points.
// @param raw_heights signed heights of the points in this frame, parallel to points.
// @param limit the step limit in metres (max_step_m of the rule).
// @param radius neighbour radius in metres (edges.cluster_tolerance_m).
// @param min_points fewest marks within radius (traversability.lethal_min_points), <= 1 keeps every passing point.
// @return the marks, in input order.
pcl::PointCloud<pcl::PointXYZ> lethal_marks(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<float> & filtered_heights,
  const std::vector<float> & raw_heights, double limit, double radius, int min_points);

// Warning areas of the floor-colour mismatch points: clustered like the ToF candidates; every cluster is a warning.
// @param mismatch_points_base mismatch points in base_link.
// @param heights signed heights above the plane, parallel to the points.
// @param plane the floor plane.
// @param edges clustering settings.
// @param cfg the rule settings (edge band).
// @return one area per cluster, kind "colour", source "colour".
std::vector<WarningArea> colour_warning_areas(
  const pcl::PointCloud<pcl::PointXYZ> & mismatch_points_base, const std::vector<float> & heights, const Plane & plane,
  const EdgesConfig & edges, const TraversabilityConfig & cfg);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__TRAVERSABILITY_HPP_
