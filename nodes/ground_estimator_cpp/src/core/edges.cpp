#include "ground_estimator_cpp/edges.hpp"

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <memory>

namespace ground_estimator_cpp
{

namespace
{

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double NEAR_EDGE_PERCENTILE = 0.1;
constexpr double STEP_PERCENTILE = 0.9;
constexpr double LOW_PERCENTILE = 0.1;
constexpr double HIGH_PERCENTILE = 0.9;
constexpr double MEDIAN = 0.5;
constexpr double MIN_AXIS_NORM = 1e-9;
constexpr const char * KIND_DROP = "drop";
constexpr const char * KIND_STEP = "step";

// Nearest-rank percentile of a copy of the values (q in 0..1); 0 for no values.
double percentile(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(std::lround(q * static_cast<double>(values.size() - 1)))];
}

// Convex hull (Andrew's monotone chain), counter-clockwise, no repeated first vertex.
std::vector<Eigen::Vector2d> convex_hull(std::vector<Eigen::Vector2d> pts)
{
  std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  if (pts.size() < 3) {
    return pts;
  }
  const auto cross = [](const Eigen::Vector2d & o, const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
  };
  std::vector<Eigen::Vector2d> hull(2 * pts.size());
  std::size_t k = 0;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0.0) {
      --k;
    }
    hull[k++] = pts[i];
  }
  for (std::size_t i = pts.size() - 1, lower = k + 1; i > 0; --i) {
    while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i - 1]) <= 0.0) {
      --k;
    }
    hull[k++] = pts[i - 1];
  }
  hull.resize(k - 1);
  return hull;
}

// Fill every feature of a cluster whose kind, points and heights are set.
void compute_features(Cluster & c, const Plane & plane, double edge_band_m)
{
  const std::size_t n = c.points.size();
  std::vector<Eigen::Vector2d> xy;
  xy.reserve(n);
  Eigen::Vector3d centroid3 = Eigen::Vector3d::Zero();
  for (const auto & p : c.points) {
    xy.emplace_back(p.x, p.y);
    centroid3 += Eigen::Vector3d(p.x, p.y, p.z);
  }
  centroid3 /= static_cast<double>(n);
  const Eigen::Vector2d centroid = centroid3.head<2>();

  std::vector<double> h(c.heights.begin(), c.heights.end());
  c.median_height_m = percentile(h, MEDIAN);

  // Principal axis in xy.
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto & p : xy) {
    cov += (p - centroid) * (p - centroid).transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> axis_solver(cov / static_cast<double>(n));
  Eigen::Vector2d axis = axis_solver.eigenvectors().col(1);  // largest eigenvalue last
  if (axis.norm() < MIN_AXIS_NORM) {
    axis = Eigen::Vector2d::UnitY();
  }
  axis.normalize();
  if (axis.y() < 0.0 || (axis.y() == 0.0 && axis.x() < 0.0)) {
    axis = -axis;
  }
  c.edge_direction = axis;

  // Normal oriented from the origin towards the cluster; the line sits at the near-edge percentile of the offsets.
  Eigen::Vector2d normal(-axis.y(), axis.x());
  std::vector<double> offsets;
  offsets.reserve(n);
  for (const auto & p : xy) {
    offsets.push_back(normal.dot(p));
  }
  if (percentile(offsets, MEDIAN) < 0.0) {
    normal = -normal;
    for (auto & o : offsets) {
      o = -o;
    }
  }
  const double line_offset = percentile(offsets, NEAR_EDGE_PERCENTILE);
  c.edge_point = normal * line_offset + axis * axis.dot(centroid);
  c.distance_m = std::fabs(line_offset);
  // Cluster on the side of +normal: left when cross(axis, normal) > 0.
  c.side = (axis.x() * normal.y() - axis.y() * normal.x()) > 0.0 ? "left" : "right";

  std::vector<double> band_heights;
  for (std::size_t i = 0; i < n; ++i) {
    if (std::fabs(offsets[i] - line_offset) <= edge_band_m) {
      band_heights.push_back(std::fabs(h[i]));
    }
  }
  c.edge_step_m = percentile(band_heights, STEP_PERCENTILE);

  // Best-fit plane normal: smallest eigenvector of the 3D covariance.
  Eigen::Matrix3d cov3 = Eigen::Matrix3d::Zero();
  for (const auto & p : c.points) {
    const Eigen::Vector3d d = Eigen::Vector3d(p.x, p.y, p.z) - centroid3;
    cov3 += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> plane_solver(cov3 / static_cast<double>(n));
  const Eigen::Vector3d fit_normal = plane_solver.eigenvectors().col(0);
  c.slope_deg = std::acos(std::min(1.0, std::fabs(fit_normal.dot(plane.normal.normalized())))) * RAD_TO_DEG;

  c.hull = convex_hull(xy);
}

}  // namespace

std::vector<Cluster> extract_clusters(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<float> & heights, const std::string & kind,
  const Plane & plane, const EdgesConfig & cfg, double edge_band_m, std::vector<int> * unclustered)
{
  std::vector<Cluster> clusters;
  std::vector<bool> used(points.size(), false);
  if (points.size() >= static_cast<std::size_t>(cfg.min_cluster_points) && heights.size() == points.size()) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(points);
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(cfg.cluster_tolerance_m);
    extraction.setMinClusterSize(cfg.min_cluster_points);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(cloud);
    std::vector<pcl::PointIndices> found;
    extraction.extract(found);
    for (const auto & group : found) {
      Cluster c;
      c.kind = kind;
      for (const int idx : group.indices) {
        c.points.push_back(points[idx]);
        c.heights.push_back(heights[idx]);
        used[idx] = true;
      }
      compute_features(c, plane, edge_band_m);
      clusters.push_back(std::move(c));
    }
  }
  std::sort(
    clusters.begin(), clusters.end(), [](const Cluster & a, const Cluster & b) { return a.distance_m < b.distance_m; });
  if (unclustered != nullptr) {
    unclustered->clear();
    for (std::size_t i = 0; i < used.size(); ++i) {
      if (!used[i]) {
        unclustered->push_back(static_cast<int>(i));
      }
    }
  }
  return clusters;
}

std::vector<Region> regions_from_clusters(const std::vector<Cluster> & clusters, const EdgesConfig & cfg)
{
  std::vector<const Cluster *> chosen;
  for (const auto & c : clusters) {
    if (c.kind == KIND_STEP) {
      std::vector<double> h(c.heights.begin(), c.heights.end());
      const double spread = percentile(h, HIGH_PERCENTILE) - percentile(h, LOW_PERCENTILE);
      if (std::fabs(c.median_height_m) > cfg.step_max_height_m || spread > cfg.step_flatness_m) {
        continue;
      }
    }
    chosen.push_back(&c);
  }
  std::stable_sort(
    chosen.begin(), chosen.end(), [](const Cluster * a, const Cluster * b) { return a->distance_m < b->distance_m; });
  if (chosen.size() > static_cast<std::size_t>(cfg.max_regions)) {
    chosen.resize(static_cast<std::size_t>(cfg.max_regions));
  }
  std::size_t drops = 0;
  std::size_t steps = 0;
  std::vector<Region> regions;
  for (const Cluster * c : chosen) {
    Region r;
    r.kind = c->kind;
    r.name = c->kind + "_" + std::to_string(c->kind == KIND_DROP ? drops++ : steps++);
    r.height_m = c->median_height_m;
    r.point = c->edge_point;
    r.direction = c->edge_direction;
    r.side = c->side;
    r.distance_m = c->distance_m;
    r.points = c->points.size();
    regions.push_back(std::move(r));
  }
  return regions;
}

}  // namespace ground_estimator_cpp
