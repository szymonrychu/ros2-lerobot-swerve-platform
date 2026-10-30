#include "ground_estimator_cpp/plane_fit.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <random>
#include <utility>
#include <vector>

namespace ground_estimator_cpp
{

namespace
{
constexpr double RAD_TO_DEG = 180.0 / M_PI;
// Trimmed refit after RANSAC. A least squares plane over every RANSAC inlier takes the foot of a box face (z 0..2 cm at
// its base, inside fit.distance_m) as floor and tilts the plane; the refit keeps only the points within TRIM_SIGMAS
// robust sigmas of the plane (never beyond fit.distance_m) and repeats. It starts from the plain least squares plane
// (the anchor, what PCL's model optimiser gave before) and never moves further than MAX_REFIT_TILT_DEG or
// fit.distance_m from it, nor onto fewer than MIN_KEPT_SHARE of the inliers, so noisy or sparse floors keep the anchor.
constexpr int REFIT_ITERATIONS = 3;
constexpr double TRIM_SIGMAS = 3.0;
constexpr double MAD_TO_SIGMA = 1.4826;  // turns a median of absolute residuals into a Gaussian sigma
constexpr double MIN_TRIM_M = 0.004;     // smallest trim band: noise-free data must keep its points
constexpr std::size_t MIN_REFIT_POINTS = 3;
constexpr double MIN_KEPT_SHARE =
  0.5;  // the trim band must keep at least this share of the points within fit.distance_m
constexpr double MAX_REFIT_TILT_DEG = 2.0;  // a refit never turns the plane further than this from the RANSAC model

// Least squares plane through the points of `cloud` selected by `use`, normal oriented along `up`.
// @return false when fewer than MIN_REFIT_POINTS points are selected or they are collinear.
bool least_squares_plane(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<bool> & use, const Eigen::Vector3d & up, Plane & out)
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  std::size_t n = 0;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (use[i]) {
      centroid += Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z);
      ++n;
    }
  }
  if (n < MIN_REFIT_POINTS) {
    return false;
  }
  centroid /= static_cast<double>(n);
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (use[i]) {
      const Eigen::Vector3d d = Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z) - centroid;
      scatter += d * d.transpose();
    }
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  Eigen::Vector3d normal = solver.eigenvectors().col(0);  // smallest eigenvalue
  if (normal.dot(up) < 0.0) {
    normal = -normal;
  }
  out.normal = normal;
  out.d = -normal.dot(centroid);
  return true;
}

// Refit `plane` on the candidates within its robust trim band; keeps the plane when a refit is not possible.
// @return the refit plane and the number of candidates within distance_m of the anchor plane (the inlier count PCL's
// own model optimiser used to report).
std::pair<Plane, std::size_t> trimmed_refit(
  const pcl::PointCloud<pcl::PointXYZ> & candidates, const Plane & plane, const Eigen::Vector3d & up, double distance_m)
{
  // The least squares plane over the RANSAC inliers (what the model optimiser of PCL gave before): the anchor and the
  // fallback of the trimmed iterations.
  std::vector<bool> use(candidates.size());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto & p = candidates[i];
    use[i] = std::fabs(height_above(plane, Eigen::Vector3d(p.x, p.y, p.z))) <= distance_m;
  }
  Plane anchor = plane;
  if (!least_squares_plane(candidates, use, up, anchor)) {
    return {plane, static_cast<std::size_t>(std::count(use.begin(), use.end(), true))};
  }
  std::size_t anchor_inliers = 0;
  for (const auto & p : candidates) {
    anchor_inliers += std::fabs(height_above(anchor, Eigen::Vector3d(p.x, p.y, p.z))) <= distance_m ? 1U : 0U;
  }
  Plane current = anchor;
  std::vector<double> inlier_abs;
  for (int iteration = 0; iteration < REFIT_ITERATIONS; ++iteration) {
    inlier_abs.clear();
    for (const auto & p : candidates) {
      const double r = std::fabs(height_above(current, Eigen::Vector3d(p.x, p.y, p.z)));
      if (r <= distance_m) {
        inlier_abs.push_back(r);
      }
    }
    if (inlier_abs.size() < MIN_REFIT_POINTS) {
      break;
    }
    const auto mid = inlier_abs.begin() + static_cast<std::ptrdiff_t>(inlier_abs.size() / 2);
    std::nth_element(inlier_abs.begin(), mid, inlier_abs.end());
    const double trim = std::clamp(TRIM_SIGMAS * MAD_TO_SIGMA * (*mid), MIN_TRIM_M, distance_m);
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const auto & p = candidates[i];
      use[i] = std::fabs(height_above(current, Eigen::Vector3d(p.x, p.y, p.z))) <= trim;
    }
    const auto kept = static_cast<std::size_t>(std::count(use.begin(), use.end(), true));
    Plane next;
    if (
      static_cast<double>(kept) < MIN_KEPT_SHARE * static_cast<double>(inlier_abs.size()) ||
      !least_squares_plane(candidates, use, up, next)) {
      break;
    }
    const double tilt_deg = std::acos(std::clamp(next.normal.dot(anchor.normal), -1.0, 1.0)) * RAD_TO_DEG;
    if (tilt_deg > MAX_REFIT_TILT_DEG || std::fabs(height_at_origin(next) - height_at_origin(anchor)) > distance_m) {
      break;
    }
    current = next;
  }
  return {current, anchor_inliers};
}

// Two levels (task T3, drop safety). A plane tilted 2 to 5 deg passes within fit.distance_m of both levels of a 5 cm
// drop or step that splits the view and wins the RANSAC count; the drop then reads half its depth, under the step
// limit. The floor near the robot decides instead. The near level is the level of the candidates within
// LEVEL_NEAR_DEPTH_M of the nearest one; when the global plane's inliers off it (by more than fit.distance_m) hold a
// second level parallel to it (within LEVEL_PARALLEL_DEG) and more than fit.distance_m away, the inliers are two
// levels, not one tilted floor, and the fit continues from the level nearer to the nominal floor at the base_link
// origin (the robot's wheels). A genuinely sloped floor is one plane near and far (no inliers off the near level); a
// ramp or kink beyond the floor gives a second set that is not parallel; both keep the global plane.
constexpr double LEVEL_NEAR_DEPTH_M = 0.4;            // the near level is fit beyond the nearest candidate up to this
constexpr double LEVEL_SEED_DISTANCE_FRACTION = 0.5;  // of fit.distance_m: a level's RANSAC cannot span two levels
constexpr double LEVEL_PARALLEL_DEG = 1.0;            // the two levels must be this parallel
constexpr double LEVEL_MIN_SHARE = 0.05;              // the second level holds at least this share of the inliers
constexpr int LEVEL_GROW_ITERATIONS = 2;              // least squares refits of a level over the candidates near it

// Which points of cloud lie within distance_m of plane.
std::vector<bool> within(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Plane & plane, double distance_m)
{
  std::vector<bool> use(cloud.size());
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud[i];
    use[i] = std::fabs(height_above(plane, Eigen::Vector3d(p.x, p.y, p.z))) <= distance_m;
  }
  return use;
}

double angle_between_deg(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return std::acos(std::clamp(a.dot(b), -1.0, 1.0)) * RAD_TO_DEG;
}

// The RANSAC plane of the (shuffled) points within cfg, oriented along up; nullopt without a model.
std::optional<Plane> ransac_plane(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points, const Eigen::Vector3d & up, const FitConfig & cfg)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(up.cast<float>());
  seg.setEpsAngle(cfg.max_normal_angle_deg / RAD_TO_DEG);
  seg.setDistanceThreshold(cfg.distance_m);
  seg.setMaxIterations(cfg.max_iterations);
  seg.setOptimizeCoefficients(false);
  seg.setInputCloud(points);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  seg.segment(inliers, coefficients);
  if (inliers.indices.empty() || coefficients.values.size() < 4) {
    return std::nullopt;
  }
  Plane plane;
  plane.normal = Eigen::Vector3d(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  plane.d = coefficients.values[3];
  const double norm = plane.normal.norm();
  if (norm < 1e-9) {
    return std::nullopt;
  }
  plane.normal /= norm;
  plane.d /= norm;
  if (plane.normal.dot(up) < 0.0) {
    plane.normal = -plane.normal;
    plane.d = -plane.d;
  }
  return plane;
}

// The level of a seed set: its RANSAC plane at LEVEL_SEED_DISTANCE_FRACTION of the inlier distance (too thin to span
// two levels 5 cm apart), the least squares plane of that model's inliers (RANSAC keeps the first model with the most
// inliers, and many tilts hold every point of a small level), then grown: the least squares plane of all candidates
// within fit.distance_m of it, LEVEL_GROW_ITERATIONS times.
std::optional<Plane> level_of(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & seed, const pcl::PointCloud<pcl::PointXYZ> & candidates,
  const Eigen::Vector3d & up, const FitConfig & cfg)
{
  if (seed->size() < MIN_REFIT_POINTS) {
    return std::nullopt;
  }
  FitConfig thin = cfg;
  thin.distance_m = LEVEL_SEED_DISTANCE_FRACTION * cfg.distance_m;
  std::optional<Plane> level = ransac_plane(seed, up, thin);
  if (!level || !least_squares_plane(*seed, within(*seed, *level, thin.distance_m), up, *level)) {
    return std::nullopt;
  }
  for (int i = 0; i < LEVEL_GROW_ITERATIONS; ++i) {
    if (!least_squares_plane(candidates, within(candidates, *level, cfg.distance_m), up, *level)) {
      return std::nullopt;
    }
  }
  return level;
}

// The level the robot stands on when `plane`'s inliers are two levels, nullopt when they are one surface.
std::optional<Plane> robot_level(
  const pcl::PointCloud<pcl::PointXYZ> & candidates, const Plane & plane, const Eigen::Vector3d & up,
  const FitConfig & cfg)
{
  // 1. The near level.
  double nearest_sq = std::numeric_limits<double>::infinity();
  for (const auto & p : candidates) {
    nearest_sq = std::min(nearest_sq, static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y);
  }
  const double near_limit = std::sqrt(nearest_sq) + LEVEL_NEAR_DEPTH_M;
  pcl::PointCloud<pcl::PointXYZ>::Ptr near(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & p : candidates) {
    if (static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y <= near_limit * near_limit) {
      near->push_back(p);
    }
  }
  const std::optional<Plane> near_level = level_of(near, candidates, up, cfg);
  if (!near_level) {
    return std::nullopt;
  }
  // 2. The global inliers off the near level.
  const std::vector<bool> global = within(candidates, plane, cfg.distance_m);
  const std::vector<bool> on_near = within(candidates, *near_level, cfg.distance_m);
  pcl::PointCloud<pcl::PointXYZ>::Ptr off(new pcl::PointCloud<pcl::PointXYZ>);
  std::size_t global_n = 0;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    global_n += global[i] ? 1U : 0U;
    if (global[i] && !on_near[i]) {
      off->push_back(candidates[i]);
    }
  }
  if (static_cast<double>(off->size()) < LEVEL_MIN_SHARE * static_cast<double>(global_n)) {
    return std::nullopt;  // the global plane is the near level, or holds little else
  }
  // 3. A second level among them, parallel to the near one and apart by more than the inlier distance.
  const std::optional<Plane> other = level_of(off, candidates, up, cfg);
  if (
    !other || angle_between_deg(other->normal, near_level->normal) > LEVEL_PARALLEL_DEG ||
    std::fabs(height_at_origin(*other) - height_at_origin(*near_level)) <= cfg.distance_m) {
    return std::nullopt;
  }
  // 4. The robot stands on the level nearer to the nominal floor at its origin.
  return std::fabs(height_at_origin(*near_level)) <= std::fabs(height_at_origin(*other)) ? near_level : other;
}
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

  const std::optional<Plane> model = ransac_plane(candidates, up, cfg);
  if (!model) {
    result.reject = FitReject::no_plane;
    return result;
  }
  Plane plane = *model;
  if (const auto level = robot_level(*candidates, plane, up, cfg)) {
    plane = *level;
  }
  const auto [refit, anchor_inliers] = trimmed_refit(*candidates, plane, up, cfg.distance_m);
  plane = refit;
  result.inliers = anchor_inliers;
  result.inlier_ratio = static_cast<double>(result.inliers) / static_cast<double>(result.candidates);

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
