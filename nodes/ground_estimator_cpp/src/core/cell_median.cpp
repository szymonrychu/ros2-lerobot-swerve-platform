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

// A gated cell becomes known again only when its spread is under this fraction of the limit, so a cell whose noise
// sits near the limit does not flicker in and out.
constexpr double RELEASE_FRACTION = 0.7;

// Smallest history for which the most deviant value is dropped before taking the standard deviation.
constexpr std::size_t MIN_TRIMMED_FRAMES = 5;

// Dropping the largest deviation shrinks the estimate to about 0.77 of the true Gaussian sigma (8 values). This factor
// restores most of it (1/0.77 = 1.3 would be fully consistent): enough that a cell with real temporal noise rarely
// slips under the limit, little enough that 20 mm floor noise stays under it.
constexpr double TRIMMED_SPREAD_SCALE = 1.15;

// Robust spread of the per-frame medians in a cell history. With few values it is 1.4826 * MAD. From
// MIN_TRIMMED_FRAMES values on it is the standard deviation without the most deviant value: robust to one bad frame
// like the MAD, but it varies far less between windows, so a cell with real temporal noise does not slip under the
// limit by chance.
double robust_spread(const std::deque<std::pair<double, double>> & history, double centre)
{
  std::vector<double> deviations;
  deviations.reserve(history.size());
  for (const auto & entry : history) {
    deviations.push_back(std::fabs(entry.second - centre));
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
    while (!history.empty() && stamp_s - history.front().first > cfg_.max_age_s) {
      history.pop_front();
    }
    it = history.empty() ? cells_.erase(it) : std::next(it);
  }

  // Bin this frame's points by fixed-frame cell.
  std::unordered_map<std::int64_t, std::vector<std::size_t>> frame_cells;
  std::vector<double> heights(points_base.size(), 0.0);
  std::vector<std::int64_t> keys(points_base.size(), 0);
  std::vector<bool> valid(points_base.size(), false);
  for (std::size_t i = 0; i < points_base.size(); ++i) {
    const auto & p = points_base[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const Eigen::Vector3d pb(p.x, p.y, p.z);
    const Eigen::Vector3d pf = fixed_T_base * pb;
    keys[i] = cell_key(
      static_cast<std::int32_t>(std::floor(pf.x() / cfg_.cell_size_m)),
      static_cast<std::int32_t>(std::floor(pf.y() / cfg_.cell_size_m)));
    heights[i] = height_above(plane, pb);
    valid[i] = true;
    frame_cells[keys[i]].push_back(i);
  }

  // Per-frame median of every cell seen, then the temporal median over its history.
  std::unordered_map<std::int64_t, double> cell_height;
  std::vector<double> scratch;
  for (const auto & [key, indices] : frame_cells) {
    scratch.clear();
    for (const std::size_t i : indices) {
      scratch.push_back(heights[i]);
    }
    auto & cell = cells_[key];
    auto & history = cell.history;
    history.emplace_back(stamp_s, median(scratch));
    while (history.size() > static_cast<std::size_t>(cfg_.window_frames)) {
      history.pop_front();
    }
    if (history.size() >= static_cast<std::size_t>(cfg_.min_frames)) {
      scratch.clear();
      for (const auto & entry : history) {
        scratch.push_back(entry.second);
      }
      const double centre = median(scratch);
      const double spread = robust_spread(history, centre);
      cell.gated = spread > cfg_.max_spread_m || (cell.gated && spread > RELEASE_FRACTION * cfg_.max_spread_m);
      if (cell.gated) {
        ++gated_cell_count_;
      } else {
        cell_height[key] = centre;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> out;
  out.header = points_base.header;
  for (std::size_t i = 0; i < points_base.size(); ++i) {
    if (!valid[i]) {
      continue;
    }
    const auto found = cell_height.find(keys[i]);
    if (found == cell_height.end()) {
      continue;
    }
    const Eigen::Vector3d p(points_base[i].x, points_base[i].y, points_base[i].z);
    const Eigen::Vector3d moved = p + (found->second - heights[i]) * plane.normal;
    out.push_back(
      pcl::PointXYZ(static_cast<float>(moved.x()), static_cast<float>(moved.y()), static_cast<float>(moved.z())));
  }
  return out;
}

}  // namespace ground_estimator_cpp
