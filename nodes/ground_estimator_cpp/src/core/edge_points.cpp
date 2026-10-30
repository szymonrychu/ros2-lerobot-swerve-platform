#include "ground_estimator_cpp/edge_points.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_set>

namespace ground_estimator_cpp
{

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using VoxelKey = std::array<int64_t, 3>;

struct Accum
{
  double x = 0.0, y = 0.0, z = 0.0;
  int n = 0;
};

// Guidance points that pass the height test and lie near a stable point, one centroid per voxel.
Cloud select(
  const Cloud & points, const std::vector<float> & heights, const Cloud & stable, bool obstacle,
  const EdgeGuidanceConfig & config)
{
  Cloud out;
  if (stable.empty()) {
    return out;
  }
  Cloud flat;
  flat.reserve(stable.size());
  for (const auto & p : stable) {
    flat.push_back(pcl::PointXYZ(p.x, p.y, 0.0F));
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(flat.makeShared());

  const double min_h = config.min_height_m;
  std::map<VoxelKey, Accum> voxels;
  std::vector<int> idx(1);
  std::vector<float> d2(1);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double h = heights[i];
    if (!std::isfinite(h) || (obstacle ? h < min_h : h > -min_h)) {
      continue;
    }
    const auto & p = points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    if (tree.nearestKSearch(pcl::PointXYZ(p.x, p.y, 0.0F), 1, idx, d2) < 1) {
      continue;
    }
    if (std::sqrt(static_cast<double>(d2[0])) > config.max_shift_m) {
      continue;
    }
    const VoxelKey key{
      static_cast<int64_t>(std::floor(p.x / config.voxel_m)), static_cast<int64_t>(std::floor(p.y / config.voxel_m)),
      static_cast<int64_t>(std::floor(p.z / config.voxel_m))};
    auto & a = voxels[key];
    a.x += p.x;
    a.y += p.y;
    a.z += p.z;
    ++a.n;
  }
  out.reserve(voxels.size());
  for (const auto & kv : voxels) {
    const auto & a = kv.second;
    const double n = a.n;
    out.push_back(pcl::PointXYZ(static_cast<float>(a.x / n), static_cast<float>(a.y / n), static_cast<float>(a.z / n)));
  }
  return out;
}

}  // namespace

int64_t GuidancePoints::cell_of(const pcl::PointXYZ & p) const
{
  constexpr int64_t ROW_STRIDE = int64_t{1} << 32;
  const auto cx = static_cast<int64_t>(std::floor(p.x / cell_size_m_));
  const auto cy = static_cast<int64_t>(std::floor(p.y / cell_size_m_));
  return cx * ROW_STRIDE + cy;
}

void GuidancePoints::set(
  const GuidanceSamples & samples, const Eigen::Isometry3d & fixed_T_base, int64_t stamp_ns, double keep_s)
{
  Cloud incoming;
  incoming.reserve(samples.points.size());
  std::unordered_set<int64_t> covered;
  for (const auto & p : samples.points) {
    const Eigen::Vector3d q = fixed_T_base * Eigen::Vector3d(p.x, p.y, p.z);
    incoming.push_back(pcl::PointXYZ(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())));
    covered.insert(cell_of(incoming.back()));
  }
  // Keep the samples of the cells this frame does not cover, unless they are older than keep_s.
  Cloud kept;
  std::vector<float> kept_h;
  std::vector<int64_t> kept_stamps;
  for (std::size_t i = 0; i < points_.size(); ++i) {
    const double age_s = static_cast<double>(stamp_ns - stamps_ns_[i]) * 1e-9;
    if (covered.count(cell_of(points_[i])) == 0 && age_s <= keep_s) {
      kept.push_back(points_[i]);
      kept_h.push_back(heights_[i]);
      kept_stamps.push_back(stamps_ns_[i]);
    }
  }
  std::vector<float> heights = samples.heights;
  heights.resize(incoming.size(), std::numeric_limits<float>::quiet_NaN());
  points_ = std::move(kept);
  heights_ = std::move(kept_h);
  stamps_ns_ = std::move(kept_stamps);
  points_ += incoming;
  heights_.insert(heights_.end(), heights.begin(), heights.end());
  stamps_ns_.insert(stamps_ns_.end(), incoming.size(), stamp_ns);
  newest_ns_ = std::max(newest_ns_, stamp_ns);
}

void GuidancePoints::clear()
{
  points_.clear();
  heights_.clear();
  stamps_ns_.clear();
  newest_ns_ = 0;
}

double GuidancePoints::age_s(int64_t stamp_ns) const
{
  if (points_.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(stamp_ns - newest_ns_) * 1e-9;
}

std::pair<Cloud, std::vector<float>> GuidancePoints::points_in_base(
  const Eigen::Isometry3d & base_T_fixed, std::optional<int64_t> stamp_ns, double max_age_s) const
{
  Cloud out;
  std::vector<float> heights;
  out.reserve(points_.size());
  for (std::size_t i = 0; i < points_.size(); ++i) {
    if (stamp_ns && std::fabs(static_cast<double>(*stamp_ns - stamps_ns_[i]) * 1e-9) > max_age_s) {
      continue;
    }
    const auto & p = points_[i];
    const Eigen::Vector3d q = base_T_fixed * Eigen::Vector3d(p.x, p.y, p.z);
    out.push_back(pcl::PointXYZ(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())));
    heights.push_back(heights_[i]);
  }
  return {out, heights};
}

EdgePoints edge_points(
  const Cloud & guidance_base, const std::vector<float> & heights, const Cloud & stable_obstacle,
  const Cloud & stable_drop, const EdgeGuidanceConfig & config)
{
  EdgePoints out;
  if (!config.enabled || guidance_base.size() != heights.size() || config.voxel_m <= 0.0) {
    return out;
  }
  out.obstacle = select(guidance_base, heights, stable_obstacle, true, config);
  out.drop = select(guidance_base, heights, stable_drop, false, config);
  return out;
}

}  // namespace ground_estimator_cpp
