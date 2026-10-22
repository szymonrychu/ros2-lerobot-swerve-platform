// Steps, thresholds, ramps and drop-offs from the above and below candidates: Euclidean clusters with the features the
// obstacle versus warning rule needs, and the flat ones as half-plane regions in the mcp SurfaceRegion shape.
#ifndef GROUND_ESTIMATOR_CPP__EDGES_HPP_
#define GROUND_ESTIMATOR_CPP__EDGES_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

// One connected group of above-floor ("step") or below-floor ("drop") points, in base_link.
struct Cluster
{
  std::string kind;                       // "step" (above the floor) or "drop" (below it)
  pcl::PointCloud<pcl::PointXYZ> points;  // the member points
  std::vector<float> heights;             // signed height above the floor plane, parallel to points
  double median_height_m = 0.0;           // median of heights
  // The step at the edge: the points within edge_band_m of the edge line are ordered along edge_direction and
  // edge_step_m is the largest median (p50) of |height| over the windows of edges.step_segment_m along the edge that
  // hold at least edges.step_segment_min_points of them (windows slide by a quarter window). A band shorter than one
  // window, or with no window holding enough points, keeps the p50 of the whole band. For a known slope of 5 deg or
  // more it is then at most |height| of the robust surface fit at edge_point (a ramp's cell-median staircase reads
  // high at its foot).
  double edge_step_m = 0.0;
  // Slope of the surface to the floor plane in degrees: atan(hypot(a, b)) of a robust (Huber IRLS) fit of
  // h = a x + b y + c over one sample per level set of equal heights (a CellMedianFilter cell), not a per-point fit.
  // 0.0 when slope_known is false: not measurable.
  double slope_deg = 0.0;
  // True only when the points cover at least edges.slope_min_cells bins of edges.slope_bin_m in base_link xy and the
  // bins span at least two distinct bins along the principal axis and across it.
  bool slope_known = false;
  std::vector<Eigen::Vector2d> hull;                          // convex hull in base_link xy, counter-clockwise
  Eigen::Vector2d edge_point = Eigen::Vector2d::Zero();       // a point of the robot-facing edge line
  Eigen::Vector2d edge_direction = Eigen::Vector2d::UnitY();  // unit direction, canonical sign (y > 0, or x > 0)
  std::string side;         // "left" or "right": the cluster's side of the line (mcp HalfPlaneEdge)
  double distance_m = 0.0;  // base_link origin to the edge line
};

// A region in the mcp SurfaceRegion shape (frame base_link, half-plane edge).
struct Region
{
  std::string name;  // "drop_<i>" or "step_<i>", i by distance within the kind
  std::string kind;  // "drop" or "step"
  std::string cls;   // "obstacle" or "warning"; empty here, set by the traversability rule
  double height_m = 0.0;
  Eigen::Vector2d point = Eigen::Vector2d::Zero();
  Eigen::Vector2d direction = Eigen::Vector2d::UnitY();
  std::string side;
  double distance_m = 0.0;
  std::size_t points = 0;
};

// Cluster candidate points and compute the features of each cluster.
// @param points candidate points in base_link (Segmented::above or ::below).
// @param heights signed heights above the plane, parallel to points.
// @param kind "step" or "drop".
// @param plane the floor plane; heights are already relative to it, so the features use heights and base_link xy.
// @param cfg cluster_tolerance_m, min_cluster_points, slope_bin_m and slope_min_cells.
// @param edge_band_m width of the band at the edge line used for edge_step_m (traversability.edge_band_m).
// @param unclustered if not null, receives the indices into points of the points in no cluster (too small groups).
// @return the clusters, nearest edge line first.
std::vector<Cluster> extract_clusters(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<float> & heights, const std::string & kind,
  const Plane & plane, const EdgesConfig & cfg, double edge_band_m, std::vector<int> * unclustered = nullptr);

// Regions of the clusters: every drop, and every step with |median| <= step_max_height_m and a height spread
// (p90 - p10) <= step_flatness_m.
// @param clusters clusters of one or both kinds.
// @param cfg step_max_height_m, step_flatness_m and max_regions.
// @return at most max_regions regions, nearest first, named per kind by distance.
std::vector<Region> regions_from_clusters(const std::vector<Cluster> & clusters, const EdgesConfig & cfg);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__EDGES_HPP_
