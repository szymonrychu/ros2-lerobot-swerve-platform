// The obstacle versus warning rule: a cluster is lethal when its edge step exceeds a third of the wheel diameter or its
// surface is steeper than traversability.max_slope_deg, otherwise the base rides over it at reduced speed.
#ifndef GROUND_ESTIMATOR_CPP__TRAVERSABILITY_HPP_
#define GROUND_ESTIMATOR_CPP__TRAVERSABILITY_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
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

// A slow-down (or, for a failed colour cluster, lethal) area in base_link.
struct WarningArea
{
  std::string kind;                      // "step", "drop", "ramp" (slope >= RAMP_MIN_SLOPE_DEG) or "colour"
  std::vector<Eigen::Vector2d> polygon;  // cluster hull in base_link
  double height_m = 0.0;                 // signed median height above the floor
  double slope_deg = 0.0;
  std::string source;           // "tof" or "colour"
  std::string cls = "warning";  // "warning", or "obstacle" for a colour cluster that fails the rule
};

// Output of the rule.
struct Traversability
{
  pcl::PointCloud<pcl::PointXYZ> obstacle, drop;  // lethal points
  std::vector<WarningArea> warnings;
  std::vector<Region> regions;  // the input regions with cls set
};

constexpr double RAMP_MIN_SLOPE_DEG = 2.0;

// Largest rideable step: max_step_fraction of the wheel diameter.
// @param cfg the rule settings.
// @return metres (0.04 for the 0.06 m wheel radius).
double max_step_m(const TraversabilityConfig & cfg);

// Classify one cluster.
// @param edge_step_m height of the cluster at its edge line in metres.
// @param slope_deg angle of the cluster surface to the floor in degrees.
// @param cfg the rule settings.
// @return obstacle when edge_step_m > max_step_m(cfg) (with a 10 um tolerance for the rounded fraction 0.3333) or
//         slope_deg > cfg.max_slope_deg; warning otherwise (exactly at a threshold is a warning).
Verdict classify_cluster(double edge_step_m, double slope_deg, const TraversabilityConfig & cfg);

// Apply the rule to the clusters of one frame.
// @param above step clusters.
// @param below drop clusters.
// @param unclustered_above points above the floor in no cluster, with signed heights parallel.
// @param unclustered_below points below the floor in no cluster, with signed heights parallel.
// @param regions regions of the clusters (regions_from_clusters); returned with cls set.
// @param cfg the rule settings.
// @return the lethal clouds (obstacle clusters plus unclustered points with |h| > max_step_m), the warning areas of
//         the warning clusters (none of their points are lethal) and the classed regions.
Traversability apply_rule(
  const std::vector<Cluster> & above, const std::vector<Cluster> & below,
  const pcl::PointCloud<pcl::PointXYZ> & unclustered_above, const std::vector<float> & unclustered_above_h,
  const pcl::PointCloud<pcl::PointXYZ> & unclustered_below, const std::vector<float> & unclustered_below_h,
  std::vector<Region> regions, const TraversabilityConfig & cfg);

// Warning areas of the floor-colour mismatch points: clustered like the ToF candidates and classified by the same rule.
// @param mismatch_points_base mismatch points in base_link.
// @param heights signed heights above the plane, parallel to the points.
// @param plane the floor plane.
// @param edges clustering settings.
// @param cfg the rule settings.
// @param obstacle_points if not null, receives the points of the clusters that fail the rule.
// @return one area per cluster, kind "colour", source "colour"; cls "obstacle" for a cluster that fails the rule.
std::vector<WarningArea> colour_warning_areas(
  const pcl::PointCloud<pcl::PointXYZ> & mismatch_points_base, const std::vector<float> & heights, const Plane & plane,
  const EdgesConfig & edges, const TraversabilityConfig & cfg,
  pcl::PointCloud<pcl::PointXYZ> * obstacle_points = nullptr);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__TRAVERSABILITY_HPP_
