#include "ground_estimator_cpp/cell_median.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ground_estimator_cpp
{

namespace
{

constexpr int KEY_SHIFT = 32;

std::int64_t cell_key(std::int32_t ix, std::int32_t iy)
{
  return (static_cast<std::int64_t>(ix) << KEY_SHIFT) | static_cast<std::uint32_t>(iy);
}

// Median of a non-empty vector (mean of the two middle values for an even count); reorders the vector.
double median(std::vector<double> & v)
{
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  const double upper = v[mid];
  if (v.size() % 2 == 1) {
    return upper;
  }
  const double lower = *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid));
  return 0.5 * (lower + upper);
}

// Consistency factor turning the median absolute deviation into a Gaussian sigma.
constexpr double MAD_TO_SIGMA = 1.4826;

// Smallest history for which the most deviant value is dropped before taking the standard deviation.
constexpr std::size_t MIN_TRIMMED_FRAMES = 5;

// Dropping the largest deviation shrinks the estimate to about 0.77 of the true Gaussian sigma (8 values), so the
// consistent factor is 1/0.77 = 1.3. With 8 values the estimate scatters ~25%; 1.3 keeps measured 30..42 mm cells
// at <= 10% visible while 4..20 mm floor cells stay >= 95% visible (1.35 failed the floor bound; see the README).
constexpr double TRIMMED_SPREAD_SCALE = 1.3;

// Robust spread of the per-frame medians in a cell history. With few values it is 1.4826 * MAD. From
// MIN_TRIMMED_FRAMES values on it is the standard deviation without the most deviant value: robust to one bad frame
// like the MAD, but it varies far less between windows, so a cell with real temporal noise does not slip under the
// limit by chance.
double robust_spread(const std::deque<CellSample> & history, double centre)
{
  std::vector<double> deviations;
  deviations.reserve(history.size());
  for (const auto & entry : history) {
    deviations.push_back(std::fabs(entry.height_m - centre));
  }
  if (history.size() < MIN_TRIMMED_FRAMES) {
    return MAD_TO_SIGMA * median(deviations);
  }
  std::sort(deviations.begin(), deviations.end());
  deviations.pop_back();
  // Variance about the median of the kept values (the dropped one is the largest deviation from the median).
  double sum_sq = 0.0;
  for (const double d : deviations) {
    sum_sq += d * d;
  }
  return TRIMMED_SPREAD_SCALE * std::sqrt(sum_sq / static_cast<double>(deviations.size() - 1));
}

// Points a cell needs in a frame before its gradient is fitted; with fewer the cell is taken flat.
constexpr std::size_t MIN_GRADIENT_POINTS = 12;

// Parameters of the fitted plane (height, two gradients).
constexpr double FIT_PARAMETERS = 3.0;

// Smallest spread (variance of dx plus dy, m^2) of the points of a cell for a gradient fit; below it the points sit
// in one spot and the fit is not determined.
constexpr double MIN_POSITION_VARIANCE_M2 = 1e-5;

// Residuals beyond this many robust sigmas are left out of the second fit.
constexpr double GRADIENT_TRIM_SIGMAS = 2.5;

// Least squares gradient (gx, gy; optional variance outputs per unit sigma^2 may be null) of h = c + gx dx + gy dy over the selected points; false if undetermined.
bool fit_gradient(
  const std::vector<double> & dx, const std::vector<double> & dy, const std::vector<double> & h,
  const std::vector<bool> & use, double & gx, double & gy, double * var_x_per_sigma2 = nullptr,
  double * var_y_per_sigma2 = nullptr)
{
  double n = 0.0, mx = 0.0, my = 0.0, mh = 0.0;
  for (std::size_t k = 0; k < h.size(); ++k) {
    if (use[k]) {
      n += 1.0;
      mx += dx[k];
      my += dy[k];
      mh += h[k];
    }
  }
  if (n < static_cast<double>(MIN_GRADIENT_POINTS)) {
    return false;
  }
  mx /= n;
  my /= n;
  mh /= n;
  double sxx = 0.0, sxy = 0.0, syy = 0.0, sxh = 0.0, syh = 0.0;
  for (std::size_t k = 0; k < h.size(); ++k) {
    if (use[k]) {
      const double ax = dx[k] - mx;
      const double ay = dy[k] - my;
      sxx += ax * ax;
      sxy += ax * ay;
      syy += ay * ay;
      sxh += ax * (h[k] - mh);
      syh += ay * (h[k] - mh);
    }
  }
  const double det = sxx * syy - sxy * sxy;
  if ((sxx + syy) / n < MIN_POSITION_VARIANCE_M2 || det <= 1e-6 * (sxx + syy) * (sxx + syy)) {
    return false;
  }
  gx = (syy * sxh - sxy * syh) / det;
  gy = (sxx * syh - sxy * sxh) / det;
  if (var_x_per_sigma2 != nullptr) {
    *var_x_per_sigma2 = syy / det;
  }
  if (var_y_per_sigma2 != nullptr) {
    *var_y_per_sigma2 = sxx / det;
  }
  return true;
}

// Robust plane of one cell in one frame: height at the cell centre (median of the gradient-corrected heights) and
// the gradient from a least squares fit that is repeated without the points beyond GRADIENT_TRIM_SIGMAS.
// @param dx, dy fixed-frame offsets of the points from the cell centre.
// @param h plane-relative heights of the points.
// @return the sample (stamp left 0).
CellSample fit_cell(const std::vector<double> & dx, const std::vector<double> & dy, const std::vector<double> & h)
{
  CellSample out;
  std::vector<bool> use(h.size(), true);
  double gx = 0.0, gy = 0.0;
  if (fit_gradient(dx, dy, h, use, gx, gy)) {
    std::vector<double> res(h.size());
    for (std::size_t k = 0; k < h.size(); ++k) {
      res[k] = h[k] - gx * dx[k] - gy * dy[k];
    }
    std::vector<double> scratch = res;
    const double centre = median(scratch);
    for (double & r : scratch) {
      r = std::fabs(r - centre);
    }
    const double limit = GRADIENT_TRIM_SIGMAS * MAD_TO_SIGMA * median(scratch);
    for (std::size_t k = 0; k < h.size(); ++k) {
      use[k] = std::fabs(res[k] - centre) <= limit;
    }
    double tx = 0.0, ty = 0.0, vx = 0.0, vy = 0.0;
    if (limit > 0.0 && fit_gradient(dx, dy, h, use, tx, ty, &vx, &vy)) {
      gx = tx;
      gy = ty;
    } else {
      fit_gradient(dx, dy, h, std::vector<bool>(h.size(), true), tx, ty, &vx, &vy);
    }
    // Standard error of the fitted gradient from the scatter of the kept residuals about the fitted plane.
    std::vector<double> kept;
    for (std::size_t k = 0; k < h.size(); ++k) {
      if (use[k]) {
        kept.push_back(h[k] - gx * dx[k] - gy * dy[k]);
      }
    }
    const double kept_centre = median(kept);
    for (double & r : kept) {
      r = std::fabs(r - kept_centre);
    }
    // Fitting three parameters to n points shrinks the residuals by sqrt((n - 3) / n).
    const auto n_kept = static_cast<double>(kept.size());
    const double sigma_res = MAD_TO_SIGMA * median(kept) * std::sqrt(n_kept / std::max(1.0, n_kept - FIT_PARAMETERS));
    out.gradient_se_x = sigma_res * std::sqrt(vx);
    out.gradient_se_y = sigma_res * std::sqrt(vy);
  }
  out.gradient_x = gx;
  out.gradient_y = gy;
  std::vector<double> corrected(h.size());
  for (std::size_t k = 0; k < h.size(); ++k) {
    corrected[k] = h[k] - gx * dx[k] - gy * dy[k];
  }
  out.height_m = median(corrected);
  return out;
}

// Standard error of the median of n Gaussian values is MEDIAN_SE_FACTOR * sigma / sqrt(n); a gradient is applied only
// when it is GRADIENT_SIGNIFICANCE standard errors away from zero.
constexpr double MEDIAN_SE_FACTOR = 1.2533;
constexpr double GRADIENT_SIGNIFICANCE = 3.0;

// Largest in-cell gradient magnitude kept (tan 22 deg). A steeper fit is a step edge inside the cell, which a plane
// would smear into a ramp, so such a cell stays flat at its median height.
constexpr double MAX_GRADIENT = 0.4;

// Temporal median of one gradient component of a cell, or 0 if it is not significant: a flat noisy cell fits a random
// gradient every frame, and applying it would tilt the cell by up to half a cell edge times the gradient.
// @param history the cell's per-frame samples (not empty).
// @param field the gradient member of CellSample.
// @return the median gradient, or 0.0 when its absolute value is below the significance limit.
double significant_gradient(
  const std::deque<CellSample> & history, double CellSample::*field, double CellSample::*error_field)
{
  std::vector<double> values;
  values.reserve(history.size());
  for (const auto & entry : history) {
    values.push_back(entry.*field);
  }
  const double centre = median(values);
  for (double & v : values) {
    v = std::fabs(v - centre);
  }
  // The scatter between frames, but never less than the typical error of one frame's fit: 4 to 8 values can agree by
  // chance.
  std::vector<double> errors;
  for (const auto & entry : history) {
    errors.push_back(entry.*error_field);
  }
  const double sigma = std::max(MAD_TO_SIGMA * median(values), median(errors));
  const double limit =
    GRADIENT_SIGNIFICANCE * MEDIAN_SE_FACTOR * sigma / std::sqrt(static_cast<double>(history.size()));
  return std::fabs(centre) > limit ? centre : 0.0;
}

}  // namespace

CellMedianFilter::CellMedianFilter(const TemporalConfig & cfg) : cfg_(cfg) {}

void CellMedianFilter::reset()
{
  cells_.clear();
  has_stamp_ = false;
  newest_stamp_s_ = 0.0;
  gated_cell_count_ = 0;
}

pcl::PointCloud<pcl::PointXYZ> CellMedianFilter::apply(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const Eigen::Isometry3d & fixed_T_base,
  double stamp_s)
{
  gated_cell_count_ = 0;
  if (!cfg_.enabled) {
    return points_base;
  }
  if (has_stamp_ && stamp_s < newest_stamp_s_) {
    reset();
  }
  has_stamp_ = true;
  newest_stamp_s_ = stamp_s;

  // Drop expired values and empty cells.
  for (auto it = cells_.begin(); it != cells_.end();) {
    auto & history = it->second.history;
    while (!history.empty() && stamp_s - history.front().stamp_s > cfg_.max_age_s) {
      history.pop_front();
    }
    it = history.empty() ? cells_.erase(it) : std::next(it);
  }

  // Bin this frame's points by fixed-frame cell.
  struct CellPlane
  {
    double height_m = 0.0;
    double gradient_x = 0.0;
    double gradient_y = 0.0;
  };
  std::unordered_map<std::int64_t, std::vector<std::size_t>> frame_cells;
  std::vector<double> heights(points_base.size(), 0.0);
  std::vector<double> fixed_x(points_base.size(), 0.0);
  std::vector<double> fixed_y(points_base.size(), 0.0);
  std::vector<std::int64_t> keys(points_base.size(), 0);
  std::vector<bool> valid(points_base.size(), false);
  for (std::size_t i = 0; i < points_base.size(); ++i) {
    const auto & p = points_base[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const Eigen::Vector3d pb(p.x, p.y, p.z);
    const Eigen::Vector3d pf = fixed_T_base * pb;
    fixed_x[i] = pf.x();
    fixed_y[i] = pf.y();
    const auto ix = static_cast<std::int32_t>(std::floor(pf.x() / cfg_.cell_size_m));
    const auto iy = static_cast<std::int32_t>(std::floor(pf.y() / cfg_.cell_size_m));
    keys[i] = cell_key(ix, iy);
    heights[i] = height_above(plane, pb);
    valid[i] = true;
    frame_cells[keys[i]].push_back(i);
  }

  // Per-frame plane of every cell seen, then the temporal median over its history.
  std::unordered_map<std::int64_t, CellPlane> cell_plane;
  std::vector<double> dx, dy, h, scratch;
  for (const auto & [key, indices] : frame_cells) {
    const double centre_x = (static_cast<std::int32_t>(key >> KEY_SHIFT) + 0.5) * cfg_.cell_size_m;
    const double centre_y = (static_cast<std::int32_t>(static_cast<std::uint32_t>(key)) + 0.5) * cfg_.cell_size_m;
    dx.clear();
    dy.clear();
    h.clear();
    for (const std::size_t i : indices) {
      dx.push_back(fixed_x[i] - centre_x);
      dy.push_back(fixed_y[i] - centre_y);
      h.push_back(heights[i]);
    }
    CellSample sample = fit_cell(dx, dy, h);
    sample.stamp_s = stamp_s;
    auto & history = cells_[key].history;
    history.push_back(sample);
    while (history.size() > static_cast<std::size_t>(cfg_.window_frames)) {
      history.pop_front();
    }
    if (history.size() >= static_cast<std::size_t>(cfg_.min_frames)) {
      const auto temporal_median = [&](double CellSample::*field) {
        scratch.clear();
        for (const auto & entry : history) {
          scratch.push_back(entry.*field);
        }
        return median(scratch);
      };
      const double centre = temporal_median(&CellSample::height_m);
      const double spread = robust_spread(history, centre);
      if (spread > cfg_.max_spread_m) {
        ++gated_cell_count_;
      } else {
        CellPlane plane_of_cell{
          centre, significant_gradient(history, &CellSample::gradient_x, &CellSample::gradient_se_x),
          significant_gradient(history, &CellSample::gradient_y, &CellSample::gradient_se_y)};
        if (std::hypot(plane_of_cell.gradient_x, plane_of_cell.gradient_y) > MAX_GRADIENT) {
          plane_of_cell.gradient_x = 0.0;
          plane_of_cell.gradient_y = 0.0;
        }
        cell_plane[key] = plane_of_cell;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> out;
  out.header = points_base.header;
  for (std::size_t i = 0; i < points_base.size(); ++i) {
    if (!valid[i]) {
      continue;
    }
    const auto found = cell_plane.find(keys[i]);
    if (found == cell_plane.end()) {
      continue;
    }
    const double centre_x = (static_cast<std::int32_t>(keys[i] >> KEY_SHIFT) + 0.5) * cfg_.cell_size_m;
    const double centre_y = (static_cast<std::int32_t>(static_cast<std::uint32_t>(keys[i])) + 0.5) * cfg_.cell_size_m;
    const CellPlane & cp = found->second;
    const double target =
      cp.height_m + cp.gradient_x * (fixed_x[i] - centre_x) + cp.gradient_y * (fixed_y[i] - centre_y);
    const Eigen::Vector3d p(points_base[i].x, points_base[i].y, points_base[i].z);
    const Eigen::Vector3d moved = p + (target - heights[i]) * plane.normal;
    out.push_back(
      pcl::PointXYZ(static_cast<float>(moved.x()), static_cast<float>(moved.y()), static_cast<float>(moved.z())));
  }
  return out;
}

}  // namespace ground_estimator_cpp
