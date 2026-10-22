#include "ground_estimator_cpp/surface_slope.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

namespace ground_estimator_cpp
{

namespace
{

constexpr int KEY_SHIFT = 32;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr std::size_t MIN_SURFACE_SAMPLES = 3;
constexpr int EM_ITERATIONS = 50;
constexpr double EM_TOLERANCE = 1e-7;          // change of the plane coefficients that ends the iterations
constexpr double MIN_SURFACE_SIGMA_M = 0.002;  // scale floors: a noise-free component keeps a finite likelihood
constexpr double MIN_FLOOR_SIGMA_M = 0.001;
constexpr double START_FLOOR_SIGMA_FRACTION = 0.25;  // of surface_min_m: the floor starts well inside its band
constexpr double MIN_CELL_SHARE = 0.01;              // a cell keeps a little of both the surface and the floor
constexpr double MAX_CELL_SHARE = 0.99;
constexpr double START_OUTLIER_MIX = 0.05;
constexpr double MIN_OUTLIER_MIX = 0.001;
constexpr double MAX_OUTLIER_MIX = 0.5;
constexpr double MIN_OUTLIER_RANGE_M = 0.01;  // the outlier density is uniform over at least this height range
constexpr double MIN_WEIGHT_SUM = 1e-9;
constexpr double RANK_THRESHOLD = 1e-10;

std::int64_t cell_key(std::int32_t ix, std::int32_t iy)
{
  return (static_cast<std::int64_t>(ix) << KEY_SHIFT) | static_cast<std::uint32_t>(iy);
}

double gaussian(double residual, double sigma)
{
  const double u = residual / sigma;
  return std::exp(-0.5 * u * u) / sigma;
}

// Weighted least-squares plane h = a (x - cx) + b (y - cy) + c; nullopt when not finite.
std::optional<Eigen::Vector3d> weighted_plane(
  const std::vector<HeightSample> & samples, const std::vector<double> & weights, const Eigen::Vector2d & centroid)
{
  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (weights[i] <= 0.0) {
      continue;
    }
    const Eigen::Vector2d d = samples[i].xy - centroid;
    const Eigen::Vector3d row(d.x(), d.y(), 1.0);
    normal += weights[i] * row * row.transpose();
    rhs += weights[i] * samples[i].height * row;
  }
  Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> solver(normal.rows(), normal.cols());
  solver.setThreshold(RANK_THRESHOLD);
  solver.compute(normal);
  const Eigen::Vector3d coef = solver.solve(rhs);
  if (!coef.allFinite()) {
    return std::nullopt;
  }
  return coef;
}

}  // namespace

TemporalConfig slope_history_config(const TemporalConfig & temporal)
{
  TemporalConfig cfg = temporal;
  cfg.window_frames = SLOPE_HISTORY_WINDOWS * temporal.window_frames;
  return cfg;
}

RawHeightHistory::RawHeightHistory(const TemporalConfig & cfg) : cfg_(cfg) {}

std::int64_t RawHeightHistory::key_of(const Eigen::Vector2d & xy_fixed) const
{
  return cell_key(
    static_cast<std::int32_t>(std::floor(xy_fixed.x() / cfg_.cell_size_m)),
    static_cast<std::int32_t>(std::floor(xy_fixed.y() / cfg_.cell_size_m)));
}

void RawHeightHistory::add(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const Eigen::Isometry3d & fixed_T_base,
  double stamp_s)
{
  if (!frames_.empty() && stamp_s < frames_.back().stamp_s) {
    frames_.clear();
  }
  while (!frames_.empty() && stamp_s - frames_.front().stamp_s > cfg_.max_age_s) {
    frames_.pop_front();
  }
  Frame frame;
  frame.stamp_s = stamp_s;
  for (const auto & p : points_base) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const Eigen::Vector3d pb(p.x, p.y, p.z);
    const Eigen::Vector2d xy = (fixed_T_base * pb).head<2>();
    const std::int64_t key = key_of(xy);
    frame.cells[key].push_back(HeightSample{xy, height_above(plane, pb), key});
  }
  frames_.push_back(std::move(frame));
  while (frames_.size() > static_cast<std::size_t>(std::max(cfg_.window_frames, 1))) {
    frames_.pop_front();
  }
}

std::vector<HeightSample> RawHeightHistory::around(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const Eigen::Isometry3d & fixed_T_base,
  std::size_t * clouds) const
{
  std::set<std::pair<std::int32_t, std::int32_t>> cells;
  for (const auto & p : points_base) {
    const Eigen::Vector2d xy = (fixed_T_base * Eigen::Vector3d(p.x, p.y, p.z)).head<2>();
    const auto ix = static_cast<std::int32_t>(std::floor(xy.x() / cfg_.cell_size_m));
    const auto iy = static_cast<std::int32_t>(std::floor(xy.y() / cfg_.cell_size_m));
    for (std::int32_t dx = -1; dx <= 1; ++dx) {
      for (std::int32_t dy = -1; dy <= 1; ++dy) {
        cells.emplace(ix + dx, iy + dy);
      }
    }
  }
  std::vector<HeightSample> out;
  std::size_t with_samples = 0;
  for (const Frame & frame : frames_) {
    const std::size_t before = out.size();
    for (const auto & [ix, iy] : cells) {
      const auto found = frame.cells.find(cell_key(ix, iy));
      if (found != frame.cells.end()) {
        out.insert(out.end(), found->second.begin(), found->second.end());
      }
    }
    with_samples += out.size() > before ? 1 : 0;
  }
  if (clouds != nullptr) {
    *clouds = with_samples;
  }
  return out;
}

void RawHeightHistory::reset() { frames_.clear(); }

std::optional<double> surface_slope_deg(const std::vector<HeightSample> & samples, double sign, double surface_min_m)
{
  const std::size_t n = samples.size();
  // Start: the surface is the plane through the samples beyond the floor band, the floor is the rest.
  std::vector<double> surface_w(n, 0.0);
  std::vector<double> floor_w(n, 0.0);
  std::unordered_map<std::int64_t, std::size_t> cell_index;
  std::vector<std::size_t> cell_of(n, 0);
  std::size_t surface = 0;
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    centroid += samples[i].xy;
    cell_of[i] = cell_index.try_emplace(samples[i].cell, cell_index.size()).first->second;
    if (sign * samples[i].height > surface_min_m) {
      surface_w[i] = 1.0;
      ++surface;
    }
  }
  if (surface < MIN_SURFACE_SAMPLES) {
    return std::nullopt;
  }
  centroid /= static_cast<double>(n);
  auto coef = weighted_plane(samples, surface_w, centroid);
  if (!coef) {
    return std::nullopt;
  }
  const auto predicted = [&](std::size_t i) {
    const Eigen::Vector2d d = samples[i].xy - centroid;
    return coef->x() * d.x() + coef->y() * d.y() + coef->z();
  };

  // Per cell, the share of the surface among its surface and floor samples, started from the samples beyond the band:
  // a cell on the floor gives the plane no room to take floor returns, a cell on the surface keeps its low returns.
  std::vector<double> cell_surface(cell_index.size(), 0.0);
  std::vector<double> cell_total(cell_index.size(), 0.0);
  double surface_ss = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    cell_surface[cell_of[i]] += surface_w[i];
    cell_total[cell_of[i]] += 1.0;
    if (surface_w[i] > 0.0) {
      surface_ss += (samples[i].height - predicted(i)) * (samples[i].height - predicted(i));
    }
  }
  std::vector<double> cell_share(cell_index.size(), 0.0);
  for (std::size_t c = 0; c < cell_share.size(); ++c) {
    cell_share[c] = std::clamp(cell_surface[c] / cell_total[c], MIN_CELL_SHARE, MAX_CELL_SHARE);
  }
  double surface_sigma = std::max(MIN_SURFACE_SIGMA_M, std::sqrt(surface_ss / static_cast<double>(surface)));
  double floor_sigma = std::max(MIN_FLOOR_SIGMA_M, START_FLOOR_SIGMA_FRACTION * surface_min_m);
  double floor_mean = 0.0;
  // Outliers, returns of neither (a step's front face, mixed pixels), are uniform over the samples' height range.
  double outlier_mix = START_OUTLIER_MIX;
  const auto [low, high] = std::minmax_element(
    samples.begin(), samples.end(), [](const HeightSample & a, const HeightSample & b) { return a.height < b.height; });
  const double outlier_density = 1.0 / std::max(MIN_OUTLIER_RANGE_M, high->height - low->height);

  for (int iteration = 0; iteration < EM_ITERATIONS; ++iteration) {
    // E step: the probability that each sample is on the surface, on the floor or neither. Where the plane lies on the
    // floor's far side the surface does not exist (ahead of a ramp's foot).
    for (std::size_t i = 0; i < n; ++i) {
      const double p = predicted(i);
      const double share = cell_share[cell_of[i]];
      const double on_surface =
        sign * p > 0.0 ? (1.0 - outlier_mix) * share * gaussian(samples[i].height - p, surface_sigma) : 0.0;
      const double on_floor =
        (1.0 - outlier_mix) * (1.0 - share) * gaussian(samples[i].height - floor_mean, floor_sigma);
      const double total = on_surface + on_floor + outlier_mix * outlier_density;
      surface_w[i] = on_surface / total;
      floor_w[i] = on_floor / total;
    }
    // M step: the plane, both scales, the floor offset, the cell shares and the outlier share.
    const auto next = weighted_plane(samples, surface_w, centroid);
    if (!next) {
      return std::nullopt;
    }
    const bool converged = (*next - *coef).norm() < EM_TOLERANCE;
    coef = next;
    std::fill(cell_surface.begin(), cell_surface.end(), 0.0);
    std::fill(cell_total.begin(), cell_total.end(), 0.0);
    double s_sum = 0.0, ss_surface = 0.0, f_sum = 0.0, f_h = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double r = samples[i].height - predicted(i);
      s_sum += surface_w[i];
      ss_surface += surface_w[i] * r * r;
      f_sum += floor_w[i];
      f_h += floor_w[i] * samples[i].height;
      cell_surface[cell_of[i]] += surface_w[i];
      cell_total[cell_of[i]] += surface_w[i] + floor_w[i];
    }
    if (s_sum < MIN_WEIGHT_SUM) {
      return std::nullopt;
    }
    floor_mean = f_sum > MIN_WEIGHT_SUM ? f_h / f_sum : 0.0;
    double ss_floor = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double r = samples[i].height - floor_mean;
      ss_floor += floor_w[i] * r * r;
    }
    surface_sigma = std::max(MIN_SURFACE_SIGMA_M, std::sqrt(ss_surface / s_sum));
    floor_sigma = f_sum > MIN_WEIGHT_SUM ? std::max(MIN_FLOOR_SIGMA_M, std::sqrt(ss_floor / f_sum)) : floor_sigma;
    for (std::size_t c = 0; c < cell_share.size(); ++c) {
      if (cell_total[c] > MIN_WEIGHT_SUM) {
        cell_share[c] = std::clamp(cell_surface[c] / cell_total[c], MIN_CELL_SHARE, MAX_CELL_SHARE);
      }
    }
    outlier_mix =
      std::clamp((static_cast<double>(n) - s_sum - f_sum) / static_cast<double>(n), MIN_OUTLIER_MIX, MAX_OUTLIER_MIX);
    if (converged) {
      break;
    }
  }
  return std::atan(std::hypot(coef->x(), coef->y())) * RAD_TO_DEG;
}

}  // namespace ground_estimator_cpp
