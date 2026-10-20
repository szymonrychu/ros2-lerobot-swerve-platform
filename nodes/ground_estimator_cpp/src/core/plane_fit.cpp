#include "ground_estimator_cpp/plane_fit.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace ground_estimator_cpp
{

namespace
{
constexpr double RAD_TO_DEG = 180.0 / M_PI;
}  // namespace

double height_above(const Plane & plane, const Eigen::Vector3d & point) { return plane.normal.dot(point) + plane.d; }

double height_at_origin(const Plane & plane) { return -plane.d / plane.normal.z(); }

const char * to_string(FitReject reject)
{
  switch (reject) {
    case FitReject::too_few_points:
      return "too_few_points";
    case FitReject::no_plane:
      return "no_plane";
    case FitReject::normal_off_prior:
      return "normal_off_prior";
    case FitReject::offset_off_prior:
      return "offset_off_prior";
    case FitReject::too_few_inliers:
      return "too_few_inliers";
    case FitReject::low_inlier_ratio:
      return "low_inlier_ratio";
  }
  return "unknown";
}

FitResult fit_plane(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const Eigen::Vector3d & prior_up, const FitConfig & cfg,
  unsigned seed)
{
  FitResult result;
  const auto min_inliers = static_cast<std::size_t>(cfg.min_inliers);
  const Eigen::Vector3d up = prior_up.normalized();

  pcl::PointCloud<pcl::PointXYZ>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZ>);
  candidates->reserve(points_base.size());
  const double max_range_sq = cfg.max_range_m * cfg.max_range_m;
  for (const auto & p : points_base.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    if (
      std::fabs(p.z) <= cfg.candidate_band_m &&
      static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y <= max_range_sq) {
      candidates->push_back(p);
    }
  }
  result.candidates = candidates->size();
  if (result.candidates < min_inliers) {
    result.reject = FitReject::too_few_points;
    return result;
  }

  // PCL 1.14 SACSegmentation has no seed setter (its sampler uses a fixed internal seed), so the seed decides the
  // candidate order, which decides the samples drawn.
  std::mt19937 rng(seed);
  std::shuffle(candidates->points.begin(), candidates->points.end(), rng);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(up.cast<float>());
  seg.setEpsAngle(cfg.max_normal_angle_deg / RAD_TO_DEG);
  seg.setDistanceThreshold(cfg.distance_m);
  seg.setMaxIterations(cfg.max_iterations);
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(candidates);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  seg.segment(inliers, coefficients);
  if (inliers.indices.empty() || coefficients.values.size() < 4) {
    result.reject = FitReject::no_plane;
    return result;
  }
  result.inliers = inliers.indices.size();
  result.inlier_ratio = static_cast<double>(result.inliers) / static_cast<double>(result.candidates);

  Plane plane;
  plane.normal = Eigen::Vector3d(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  plane.d = coefficients.values[3];
  const double norm = plane.normal.norm();
  if (norm < 1e-9) {
    result.reject = FitReject::no_plane;
    return result;
  }
  plane.normal /= norm;
  plane.d /= norm;
  if (plane.normal.dot(up) < 0.0) {
    plane.normal = -plane.normal;
    plane.d = -plane.d;
  }

  const double angle_deg = std::acos(std::clamp(plane.normal.dot(up), -1.0, 1.0)) * RAD_TO_DEG;
  if (angle_deg > cfg.max_normal_angle_deg) {
    result.reject = FitReject::normal_off_prior;
  } else if (std::fabs(height_at_origin(plane)) > cfg.max_offset_m) {
    result.reject = FitReject::offset_off_prior;
  } else if (result.inliers < min_inliers) {
    result.reject = FitReject::too_few_inliers;
  } else if (result.inlier_ratio < cfg.min_inlier_ratio) {
    result.reject = FitReject::low_inlier_ratio;
  } else {
    result.plane = plane;
  }
  return result;
}

}  // namespace ground_estimator_cpp
