// Occlusion shadow gate (D16): points hidden behind a nearer, higher point as seen from the sensor are mixed pixels
// or shadow, not a drop or a ramp. Pure function over base_link points and plane-relative heights.
#ifndef GROUND_ESTIMATOR_CPP__OCCLUSION_HPP_
#define GROUND_ESTIMATOR_CPP__OCCLUSION_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <vector>

#include "ground_estimator_cpp/config.hpp"

namespace ground_estimator_cpp
{

// Which points lie in the shadow of a nearer occluder.
// A point P is occluded when a nearer point Q (smaller horizontal distance from the sensor) in the same azimuth bin
// of cfg.azimuth_bin_deg has h_Q >= cfg.min_occluder_height_m, h_P <= h_Q - cfg.min_height_drop_m and
// elevation(P) <= elevation(Q) + cfg.elevation_margin_deg, elevation = atan2(h - sensor_height_m, distance).
// One refinement keeps vertical faces (a wall, a box front) from shadowing themselves: a Q closer than one bin width
// of range (distance * bin width in radians) counts only when |elevation(P) - elevation(Q)| is within half the
// margin, i.e. P lies on Q's ray, as a mixed pixel does, not on the face below it.
// O(n log n): one sort per azimuth bin and a running maximum of the occluder elevation.
// @param points_base points in base_link.
// @param heights signed height above the floor plane per point, parallel to points_base.
// @param sensor_xy sensor origin in base_link xy.
// @param sensor_height_m sensor origin height above the floor plane.
// @param cfg the shadow gate settings; enabled false gives all false.
// @return one flag per point: true = occluded. Non-finite points are never occluders and are reported false.
std::vector<bool> occluded_mask(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const std::vector<float> & heights,
  const Eigen::Vector2d & sensor_xy, double sensor_height_m, const OcclusionConfig & cfg);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__OCCLUSION_HPP_
