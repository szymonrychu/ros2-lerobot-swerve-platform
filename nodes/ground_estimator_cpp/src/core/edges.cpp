#include "ground_estimator_cpp/edges.hpp"

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <utility>

namespace ground_estimator_cpp
{

namespace
{

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double NEAR_EDGE_PERCENTILE = 0.1;
constexpr double STEP_PERCENTILE = 0.5;
constexpr double LOW_PERCENTILE = 0.1;
constexpr double HIGH_PERCENTILE = 0.9;
constexpr double MEDIAN = 0.5;
constexpr double MIN_AXIS_NORM = 1e-9;
constexpr double LEVEL_STEP_M = 1e-6;          // heights equal to this are one level set (one temporal cell)
constexpr double MIN_BIN_SPAN_FRACTION = 0.5;  // bin centres this many bin edges apart are distinct bins along an axis
constexpr double HUBER_K = 1.345;              // Huber tuning constant, in robust standard deviations
constexpr double MAD_TO_SIGMA = 1.4826;        // MAD to standard deviation for Gaussian residuals
constexpr double MIN_RESIDUAL_SCALE_M = 1e-3;  // robust residual scale floor, so a noise-free fit keeps unit weights
constexpr int IRLS_ITERATIONS = 20;
constexpr double IRLS_TOLERANCE = 1e-9;  // change of the fit coefficients that ends the iterations
constexpr double RANK_THRESHOLD = 1e-8;  // relative pivot below which a direction of the fit is undetermined
// Fewest samples of a plane fit.
constexpr std::size_t MIN_SLOPE_SAMPLES = 3;
// Slope from which the edge step is capped by the fitted surface's height at the edge line (a 10 cm cell rises
// under 9 mm below it, so flat tops never use the fit).
constexpr double RAMP_EDGE_MIN_SLOPE_DEG = 5.0;
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

// One sample of the slope fit: centroid in base_link xy and plane-relative height.
struct SlopeSample
{
  Eigen::Vector2d xy;
  double height;
};

// Slope samples: one per level set of equal heights. CellMedianFilter gives every point of a temporal cell the cell's
// median height, so the level sets are its cells and each sample sits at the centroid of the cell's points; a
// per-point fit would regress the staircase and flatten the slope by the spread of the points within a cell. For a
// plane the centroid of points of equal height has that height, so unfiltered points (mostly one per level set) are
// sampled without bias too.
// @param c the cluster (points and heights set).
// @return the samples, by increasing height.
std::vector<SlopeSample> slope_samples(const Cluster & c)
{
  std::map<std::int64_t, std::pair<Eigen::Vector3d, std::size_t>> levels;  // level -> (sum x, y, h), count
  for (std::size_t i = 0; i < c.points.size(); ++i) {
    const double h = c.heights[i];
    // Eigen vectors are not zero-initialised: every new level starts from an explicit zero.
    auto & level = levels.try_emplace(std::llround(h / LEVEL_STEP_M), Eigen::Vector3d::Zero(), 0).first->second;
    level.first += Eigen::Vector3d(c.points[i].x, c.points[i].y, h);
    ++level.second;
  }
  std::vector<SlopeSample> samples;
  samples.reserve(levels.size());
  for (const auto & [key, level] : levels) {
    const Eigen::Vector3d mean = level.first / static_cast<double>(level.second);
    samples.push_back(SlopeSample{mean.head<2>(), mean.z()});
  }
  return samples;
}

// Whether the cluster supports a slope estimate: its points cover at least slope_min_cells bins of slope_bin_m in
// base_link xy, and the bin centres span at least two distinct bins along the principal axis and along the axis
// across it (bins in one line leave the tilt across that line unmeasured).
// @param c the cluster (points set).
// @param axis unit principal axis of the cluster in xy.
// @param cfg slope_bin_m and slope_min_cells.
// @return true when the slope is measurable.
bool slope_measurable(const Cluster & c, const Eigen::Vector2d & axis, const EdgesConfig & cfg)
{
  std::set<std::pair<std::int64_t, std::int64_t>> bins;
  for (const auto & p : c.points) {
    bins.emplace(
      static_cast<std::int64_t>(std::floor(p.x / cfg.slope_bin_m)),
      static_cast<std::int64_t>(std::floor(p.y / cfg.slope_bin_m)));
  }
  if (bins.size() < static_cast<std::size_t>(cfg.slope_min_cells)) {
    return false;
  }
  const Eigen::Vector2d across(-axis.y(), axis.x());
  Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d hi = -lo;
  for (const auto & bin : bins) {
    const Eigen::Vector2d centre =
      Eigen::Vector2d(static_cast<double>(bin.first) + 0.5, static_cast<double>(bin.second) + 0.5) * cfg.slope_bin_m;
    const Eigen::Vector2d along(axis.dot(centre), across.dot(centre));
    lo = lo.cwiseMin(along);
    hi = hi.cwiseMax(along);
  }
  return ((hi - lo).array() >= MIN_BIN_SPAN_FRACTION * cfg.slope_bin_m).all();
}

// A robust plane fit of the plane-relative height: h(xy) = a (x - cx) + b (y - cy) + c.
struct SurfaceFit
{
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();  // (cx, cy)
  Eigen::Vector3d coef = Eigen::Vector3d::Zero();      // (a, b, c)

  double slope_deg() const { return std::atan(std::hypot(coef.x(), coef.y())) * RAD_TO_DEG; }
  double height_at(const Eigen::Vector2d & xy) const
  {
    const Eigen::Vector2d d = xy - centroid;
    return coef.x() * d.x() + coef.y() * d.y() + coef.z();
  }
};

// Robust plane fit h = a x + b y + c over the samples: iteratively reweighted least squares with Huber weights on a
// MAD residual scale, started from ordinary least squares. Each step takes the minimum-norm solution: samples on one
// line (level sets that are whole contour lines of a noise-free plane, e.g. rows of a grid across a ramp) leave the
// slope along the line undetermined, and it is zero there.
// @param samples at least three samples.
// @return the fit; nullopt when the solution is not finite.
std::optional<SurfaceFit> robust_surface_fit(const std::vector<SlopeSample> & samples)
{
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto & s : samples) {
    centroid += s.xy;
  }
  centroid /= static_cast<double>(samples.size());
  std::vector<double> weights(samples.size(), 1.0);
  std::vector<double> residuals(samples.size(), 0.0);
  Eigen::Vector3d coef = Eigen::Vector3d::Zero();
  for (int iteration = 0; iteration < IRLS_ITERATIONS; ++iteration) {
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const Eigen::Vector2d d = samples[i].xy - centroid;
      const Eigen::Vector3d row(d.x(), d.y(), 1.0);
      normal += weights[i] * row * row.transpose();
      rhs += weights[i] * samples[i].height * row;
    }
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> solver(normal.rows(), normal.cols());
    solver.setThreshold(RANK_THRESHOLD);
    solver.compute(normal);
    const Eigen::Vector3d next = solver.solve(rhs);
    if (!next.allFinite()) {
      return std::nullopt;
    }
    const bool converged = iteration > 0 && (next - coef).norm() < IRLS_TOLERANCE;
    coef = next;
    if (converged) {
      break;
    }
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const Eigen::Vector2d d = samples[i].xy - centroid;
      residuals[i] = samples[i].height - (coef.x() * d.x() + coef.y() * d.y() + coef.z());
    }
    const double centre = percentile(residuals, MEDIAN);
    std::vector<double> deviations;
    deviations.reserve(residuals.size());
    for (const double r : residuals) {
      deviations.push_back(std::fabs(r - centre));
    }
    const double k = HUBER_K * std::max(MIN_RESIDUAL_SCALE_M, MAD_TO_SIGMA * percentile(deviations, MEDIAN));
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const double r = std::fabs(residuals[i]);
      weights[i] = r <= k ? 1.0 : k / r;
    }
  }
  return SurfaceFit{centroid, coef};
}

// Fill every feature of a cluster whose kind, points and heights are set.
void compute_features(Cluster & c, const EdgesConfig & cfg, double edge_band_m)
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

  // Robust slope of the plane-relative height over per-level samples; 0 and unknown without enough support.
  c.slope_known = false;
  c.slope_deg = 0.0;
  if (slope_measurable(c, axis, cfg)) {
    const auto samples = slope_samples(c);
    const auto fit = samples.size() >= MIN_SLOPE_SAMPLES ? robust_surface_fit(samples) : std::nullopt;
    c.slope_known = fit.has_value();
    c.slope_deg = fit ? fit->slope_deg() : 0.0;
    // An inclined surface rising from the floor has no step at its foot, but its cells read a staircase whose first
    // cell above the floor band sits up to a cell's rise above the surface: the step at the edge is at most the
    // fitted surface's height there. Flat tops (boxes, sills) keep the band median.
    if (fit && c.slope_deg >= RAMP_EDGE_MIN_SLOPE_DEG) {
      c.edge_step_m = std::min(c.edge_step_m, std::fabs(fit->height_at(c.edge_point)));
    }
  }

  c.hull = convex_hull(xy);
}

}  // namespace

std::vector<Cluster> extract_clusters(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<float> & heights, const std::string & kind,
  const Plane & /*plane*/, const EdgesConfig & cfg, double edge_band_m, std::vector<int> * unclustered)
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
      compute_features(c, cfg, edge_band_m);
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
