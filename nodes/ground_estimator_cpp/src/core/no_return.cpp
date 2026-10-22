#include "ground_estimator_cpp/no_return.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <stdexcept>

namespace ground_estimator_cpp
{

namespace
{

// This frame's counts of one cell.
struct FrameCount
{
  int samples = 0;     // no-return samples plus floor-band depth samples
  int mismatches = 0;  // no-return samples whose colour is not the floor's
};

bool finite(const pcl::PointXYZ & p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }

CellKey key_in_fixed(const pcl::PointXYZ & p, const Eigen::Isometry3d & fixed_T_base, double cell_size_m)
{
  const Eigen::Vector3d pf = fixed_T_base * Eigen::Vector3d(p.x, p.y, p.z);
  return cell_key_of(pf.head<2>(), cell_size_m);
}

}  // namespace

NoReturnTracker::NoReturnTracker(const NoReturnConfig & cfg, double cell_size_m, double floor_band_m)
: cfg_(cfg), cell_size_m_(cell_size_m), floor_band_m_(floor_band_m)
{
  if (!(cell_size_m_ > 0.0)) {
    throw std::invalid_argument("NoReturnTracker: cell_size_m must be > 0");
  }
  if (!(floor_band_m_ >= 0.0)) {
    throw std::invalid_argument("NoReturnTracker: floor_band_m must be >= 0");
  }
  cfg_.window_frames = std::max(cfg_.window_frames, 1);
  cfg_.enter_frames = std::clamp(cfg_.enter_frames, 1, cfg_.window_frames);
  cfg_.min_cells = std::max(cfg_.min_cells, 1);
  cfg_.clearance_cells = std::max(cfg_.clearance_cells, 0);
}

void NoReturnTracker::observe(
  const GuidanceSamples & samples_base, const FloorColourModel & colour, const ColourConfig & colour_cfg,
  const Eigen::Isometry3d & fixed_T_base, const std::vector<CellKey> & blocked_cells)
{
  // No colour, no claim: without a ready model no cell is a no-return cell, but every tracked cell still gets a vote.
  std::set<CellKey> frame_cells;
  if (colour.ready()) {
    std::map<CellKey, FrameCount> counts;
    for (std::size_t i = 0; i < samples_base.no_return.size(); ++i) {
      const pcl::PointXYZ & p = samples_base.no_return[i];
      if (!finite(p)) {
        continue;
      }
      FrameCount & c = counts[key_in_fixed(p, fixed_T_base, cell_size_m_)];
      ++c.samples;
      if (
        i < samples_base.no_return_colours.size() &&
        colour.mahalanobis(samples_base.no_return_colours[i]) > colour_cfg.max_mahalanobis) {
        ++c.mismatches;
      }
    }
    for (std::size_t i = 0; i < samples_base.points.size() && i < samples_base.heights.size(); ++i) {
      const pcl::PointXYZ & p = samples_base.points[i];
      if (!finite(p) || !(std::fabs(samples_base.heights[i]) <= floor_band_m_)) {
        continue;
      }
      // Only cells with no-return samples can become no-return cells; floor returns elsewhere need no count.
      const auto it = counts.find(key_in_fixed(p, fixed_T_base, cell_size_m_));
      if (it != counts.end()) {
        ++it->second.samples;
      }
    }
    const std::set<CellKey> blocked(blocked_cells.begin(), blocked_cells.end());
    const int r = cfg_.clearance_cells;
    const auto near_blocked = [&](const CellKey & key) {
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          if (blocked.count({key.first + dx, key.second + dy}) > 0) {
            return true;
          }
        }
      }
      return false;
    };
    for (const auto & [key, c] : counts) {
      if (
        c.samples >= cfg_.cell_min_samples &&
        static_cast<double>(c.mismatches) >= cfg_.cell_min_share * static_cast<double>(c.samples) &&
        !near_blocked(key)) {
        frame_cells.insert(key);
      }
    }
  }

  for (const CellKey & key : frame_cells) {
    cells_[key];
  }
  const auto window = static_cast<std::size_t>(cfg_.window_frames);
  for (auto it = cells_.begin(); it != cells_.end();) {
    Cell & cell = it->second;
    cell.votes.push_back(frame_cells.count(it->first) > 0);
    while (cell.votes.size() > window) {
      cell.votes.pop_front();
    }
    const auto hits = std::count(cell.votes.begin(), cell.votes.end(), true);
    if (hits == 0) {
      it = cells_.erase(it);
      continue;
    }
    cell.stable = hits >= cfg_.enter_frames;
    ++it;
  }
}

std::vector<CellKey> NoReturnTracker::stable_cells() const
{
  std::vector<CellKey> out;
  for (const auto & [key, cell] : cells_) {
    if (cell.stable) {
      out.push_back(key);
    }
  }
  return out;
}

std::vector<WarningArea> NoReturnTracker::areas(const Eigen::Isometry3d & base_T_fixed) const
{
  const std::vector<CellKey> stable = stable_cells();
  std::vector<WarningArea> out;
  for (const std::set<CellKey> & component : cell_components({stable.begin(), stable.end()})) {
    if (static_cast<int>(component.size()) < cfg_.min_cells) {
      continue;
    }
    WarningArea area;
    area.kind = KIND_NO_RETURN;
    area.source = SOURCE_RGB;
    for (const Eigen::Vector2d & v : cell_outline(component, cell_size_m_)) {
      area.polygon.push_back((base_T_fixed * Eigen::Vector3d(v.x(), v.y(), 0.0)).head<2>());
    }
    out.push_back(std::move(area));
  }
  return out;
}

void NoReturnTracker::reset() { cells_.clear(); }

}  // namespace ground_estimator_cpp
