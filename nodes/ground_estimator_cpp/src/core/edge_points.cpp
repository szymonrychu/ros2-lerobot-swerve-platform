#include "ground_estimator_cpp/edge_points.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <array>
#include <cmath>
#include <limits>
#include <map>

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
    out.push_back(pcl::PointXYZ(
      static_cast<float>(a.x / a.n), static_cast<float>(a.y / a.n), static_cast<float>(a.z / a.n)));
  }
  return out;
}

}  // namespace

void GuidancePoints::set(const GuidanceSamples & samples, const Eigen::Isometry3d & fixed_T_base, int64_t stamp_ns)
{
  points_.clear();
  heights_.clear();
  points_.reserve(samples.points.size());
  for (const auto & p : samples.points) {
    const Eigen::Vector3d q = fixed_T_base * Eigen::Vector3d(p.x, p.y, p.z);
    points_.push_back(pcl::PointXYZ(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())));
  }
  heights_ = samples.heights;
  heights_.resize(points_.size(), std::numeric_limits<float>::quiet_NaN());
  stamp_ns_ = stamp_ns;
}

void GuidancePoints::clear()
{
  points_.clear();
  heights_.clear();
  stamp_ns_ = 0;
}

double GuidancePoints::age_s(int64_t stamp_ns) const
{
  if (points_.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(stamp_ns - stamp_ns_) * 1e-9;
}

std::pair<Cloud, std::vector<float>> GuidancePoints::points_in_base(const Eigen::Isometry3d & base_T_fixed) const
{
  Cloud out;
  out.reserve(points_.size());
  for (const auto & p : points_) {
    const Eigen::Vector3d q = base_T_fixed * Eigen::Vector3d(p.x, p.y, p.z);
    out.push_back(pcl::PointXYZ(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())));
  }
  return {out, heights_};
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
