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

// Robust spread (1.4826 * MAD) of the per-frame medians in a cell history.
double robust_spread(const std::deque<std::pair<double, double>> & history, double centre)
{
  std::vector<double> deviations;
  deviations.reserve(history.size());
  for (const auto & entry : history) {
    deviations.push_back(std::fabs(entry.second - centre));
  }
  return MAD_TO_SIGMA * median(deviations);
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
    auto & history = it->second;
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
    auto & history = cells_[key];
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
      if (robust_spread(history, centre) > cfg_.max_spread_m) {
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
