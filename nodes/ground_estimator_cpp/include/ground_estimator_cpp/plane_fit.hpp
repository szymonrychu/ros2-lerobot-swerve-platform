// RANSAC fit of the floor plane in base_link from the ToF cloud, constrained by the IMU prior of the floor normal.
#ifndef GROUND_ESTIMATOR_CPP__PLANE_FIT_HPP_
#define GROUND_ESTIMATOR_CPP__PLANE_FIT_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <cstddef>
#include <optional>

#include "ground_estimator_cpp/config.hpp"

namespace ground_estimator_cpp
{

// A plane in base_link: normal.dot(p) + d = 0, unit normal pointing up.
struct Plane
{
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double d = 0.0;
};

// Why a fit was rejected, in the order the gates are applied.
enum class FitReject
{
  too_few_points,
  no_plane,
  normal_off_prior,
  offset_off_prior,
  too_few_inliers,
  low_inlier_ratio
};

// Result of fit_plane. plane is set exactly when reject is not.
struct FitResult
{
  std::optional<Plane> plane;
  std::optional<FitReject> reject;
  std::size_t candidates = 0;  // points inside the candidate band and range
  std::size_t inliers = 0;     // candidates within fit.distance_m of the refit plane (0 when no model)
  double inlier_ratio = 0.0;   // inliers / candidates
};

// Signed height of a point above the plane.
// @param plane the plane.
// @param point a point in base_link.
// @return metres, > 0 above the plane.
double height_above(const Plane & plane, const Eigen::Vector3d & point);

// Height of the plane at base_link x = y = 0.
// @param plane the plane.
// @return the plane z in metres.
double height_at_origin(const Plane & plane);

// Name of a reject reason.
// @param reject the reason.
// @return a static snake_case string.
const char * to_string(FitReject reject);

// Fit the floor: candidates are points with |z| <= fit.candidate_band_m and horizontal range <= fit.max_range_m,
// the model is a RANSAC plane perpendicular to prior_up within fit.max_normal_angle_deg, refit by least squares on the
// candidates within three robust sigmas of it (at most fit.distance_m, repeated three times) so the foot of a box face
// is not taken as floor. Deterministic for a seed.
// @param points_base cloud in base_link.
// @param prior_up IMU prior of the floor normal in base_link (any length, non-zero).
// @param cfg fit settings.
// @param seed RANSAC random seed.
// @return the plane, or the first failed gate.
FitResult fit_plane(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const Eigen::Vector3d & prior_up, const FitConfig & cfg,
  unsigned seed);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__PLANE_FIT_HPP_
