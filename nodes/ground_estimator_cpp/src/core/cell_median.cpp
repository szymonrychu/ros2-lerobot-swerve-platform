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

// Robust spread of one per-frame height of a cell history (field: the median or the plane's centre height). With few
// values it is 1.4826 * MAD. From MIN_TRIMMED_FRAMES values on it is the standard deviation without the most deviant
// value: robust to one bad frame like the MAD, but it varies far less between windows, so a cell with real temporal
// noise does not slip under the limit by chance.
double robust_spread(const std::deque<CellSample> & history, double CellSample::*field, double centre)
{
  std::vector<double> deviations;
  deviations.reserve(history.size());
  for (const auto & entry : history) {
    deviations.push_back(std::fabs(entry.*field - centre));
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
  std::vector<double> plain = h;
  out.median_m = median(plain);
  std::vector<bool> use(h.size(), true);
  double gx = 0.0, gy = 0.0;
  const bool fitted = fit_gradient(dx, dy, h, use, gx, gy);
  if (fitted) {
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
  if (fitted) {
    double sum = 0.0, sum_sq = 0.0;
    for (std::size_t k = 0; k < h.size(); ++k) {
      const double r = h[k] - gx * dx[k] - gy * dy[k];
      sum += r;
      sum_sq += r * r;
    }
    const auto n = static_cast<double>(h.size());
    out.residual_m = std::sqrt(std::max(0.0, sum_sq / n - (sum / n) * (sum / n)));
  }
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

// Largest ratio of a cell's plane residual to the smallest one of its neighbours for an inclined cell. On an inclined
// surface the plane leaves the sensor noise, as on the flat cells around it; across a step edge (a sill's or a box's
// foot, a corner) it leaves about a quarter of the step height (measured 9 mm against 3 mm noise for a rendered 3 cm
// sill), so such a cell keeps its median height instead of smearing the step into a ramp.
constexpr double MAX_RESIDUAL_RATIO = 1.5;

// Smallest cosine between the gradients of an inclined cell and of a neighbour that confirms it (45 deg).
constexpr double MIN_NEIGHBOUR_GRADIENT_COS = 0.7;

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

std::int64_t CellMedianFilter::key_of(const Eigen::Vector2d & xy_fixed) const
{
  return cell_key(
    static_cast<std::int32_t>(std::floor(xy_fixed.x() / cfg_.cell_size_m)),
    static_cast<std::int32_t>(std::floor(xy_fixed.y() / cfg_.cell_size_m)));
}

bool CellMedianFilter::steady_at(const Eigen::Vector2d & xy_fixed, double max_spread_m, int quiet_frames) const
{
  if (!cfg_.enabled) {
    return true;
  }
  const auto it = cells_.find(key_of(xy_fixed));
  if (it == cells_.end()) {
    return false;
  }
  const Cell & cell = it->second;
  if (!(cell.output && cell.spread && *cell.spread <= max_spread_m && cell.frames_since_gated >= quiet_frames)) {
    return false;
  }
  // No neighbour gated in its last quiet_frames frames either.
  const auto ix = static_cast<std::int32_t>(std::floor(xy_fixed.x() / cfg_.cell_size_m));
  const auto iy = static_cast<std::int32_t>(std::floor(xy_fixed.y() / cfg_.cell_size_m));
  for (std::int32_t dx = -1; dx <= 1; ++dx) {
    for (std::int32_t dy = -1; dy <= 1; ++dy) {
      const auto neighbour = cells_.find(cell_key(ix + dx, iy + dy));
      if ((dx != 0 || dy != 0) && neighbour != cells_.end() && neighbour->second.frames_since_gated < quiet_frames) {
        return false;
      }
    }
  }
  return true;
}

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
    keys[i] = key_of(pf.head<2>());
    heights[i] = height_above(plane, pb);
    valid[i] = true;
    frame_cells[keys[i]].push_back(i);
  }

  for (auto & [key, cell] : cells_) {
    cell.output = false;
  }

  // Per-frame plane of every cell seen, then the temporal median gradient and plane residual of every cell with
  // enough history.
  struct ReadyCell
  {
    Eigen::Vector2d gradient;  // significant and at most MAX_GRADIENT, else zero
    double residual_m;         // median plane residual of the frames with a plane fit; NaN without one
  };
  std::unordered_map<std::int64_t, ReadyCell> ready;
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
    if (history.size() < static_cast<std::size_t>(cfg_.min_frames)) {
      continue;
    }
    Eigen::Vector2d gradient(
      significant_gradient(history, &CellSample::gradient_x, &CellSample::gradient_se_x),
      significant_gradient(history, &CellSample::gradient_y, &CellSample::gradient_se_y));
    if (gradient.norm() > MAX_GRADIENT) {
      gradient.setZero();
    }
    scratch.clear();
    for (const auto & entry : history) {
      if (std::isfinite(entry.residual_m)) {
        scratch.push_back(entry.residual_m);
      }
    }
    ready.emplace(
      key, ReadyCell{gradient, scratch.empty() ? std::numeric_limits<double>::quiet_NaN() : median(scratch)});
  }

  // A candidate for an inclined cell has a gradient and a plane residual at most MAX_RESIDUAL_RATIO times the smallest
  // one of its neighbours.
  const auto neighbours_of = [](std::int64_t key, const auto & visit) {
    const auto ix = static_cast<std::int32_t>(key >> KEY_SHIFT);
    const auto iy = static_cast<std::int32_t>(static_cast<std::uint32_t>(key));
    for (std::int32_t ox = -1; ox <= 1; ++ox) {
      for (std::int32_t oy = -1; oy <= 1; ++oy) {
        if (ox != 0 || oy != 0) {
          visit(cell_key(ix + ox, iy + oy));
        }
      }
    }
  };
  std::unordered_map<std::int64_t, Eigen::Vector2d> candidates;
  for (const auto & [key, info] : ready) {
    if (info.gradient.isZero()) {
      continue;
    }
    // The smallest neighbour residual: beside a step edge most neighbours can straddle the step too (a sill 1.5 cells
    // deep, at an angle to the grid), the floor or the top beside it does not.
    double reference = std::numeric_limits<double>::infinity();
    neighbours_of(key, [&](std::int64_t n) {
      const auto found = ready.find(n);
      if (found != ready.end() && std::isfinite(found->second.residual_m)) {
        reference = std::min(reference, found->second.residual_m);
      }
    });
    if (std::isfinite(reference) && info.residual_m <= MAX_RESIDUAL_RATIO * reference) {
      candidates.emplace(key, info.gradient);
    }
  }

  // An inclined cell (a candidate with a candidate neighbour of a similar gradient: an inclined surface spans cells,
  // a lone cell at a step edge or corner does not) is output on its median plane, its spread that of its centre
  // heights; every other cell (flat, a step edge) at the median of its per-frame medians, its spread theirs.
  std::unordered_map<std::int64_t, CellPlane> cell_plane;
  for (const auto & [key, info] : ready) {
    Cell & cell = cells_[key];
    const auto & history = cell.history;
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
    const auto found = candidates.find(key);
    if (found != candidates.end()) {
      const Eigen::Vector2d direction = found->second.normalized();
      neighbours_of(key, [&](std::int64_t n) {
        const auto other = candidates.find(n);
        if (other != candidates.end() && direction.dot(other->second.normalized()) >= MIN_NEIGHBOUR_GRADIENT_COS) {
          gradient = found->second;
        }
      });
    }
    double CellSample::*value = gradient.isZero() ? &CellSample::median_m : &CellSample::height_m;
    scratch.clear();
    for (const auto & entry : history) {
      scratch.push_back(entry.*value);
    }
    const double centre = median(scratch);
    const double spread = robust_spread(history, value, centre);
    cell.spread = spread;
    if (spread > cfg_.max_spread_m) {
      ++gated_cell_count_;
      cell.frames_since_gated = 0;
    } else {
      cell.output = true;
      if (cell.frames_since_gated < std::numeric_limits<int>::max()) {
        ++cell.frames_since_gated;
      }
      cell_plane[key] = CellPlane{centre, gradient.x(), gradient.y()};
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
