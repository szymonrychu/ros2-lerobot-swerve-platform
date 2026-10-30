// Guidance edge points (design D1, D3): lethal boundaries refined from the guided depth. Pure core code.
#ifndef GROUND_ESTIMATOR_CPP__EDGE_POINTS_HPP_
#define GROUND_ESTIMATOR_CPP__EDGE_POINTS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/guidance.hpp"

namespace ground_estimator_cpp
{

// The guidance samples, kept in the fixed frame so they can be moved into any later base_link. Each sample keeps the
// stamp of its frame, and a newer frame replaces only the fixed-frame cells (cell_size_m) its samples fall in: the last
// good guidance of a cell stays until a newer frame covers that cell, so a view that moved on, or one lost frame, does
// not switch the edges of the cells it no longer sees off.
class GuidancePoints
{
public:
  // @param cell_size_m edge of a fixed-frame cell, metres (> 0); the grid of the temporal median.
  explicit GuidancePoints(double cell_size_m = 0.10) : cell_size_m_(cell_size_m) {}

  // Add a frame: its cells replace the kept samples of the same cells.
  // @param samples the frame's samples in base_link (points and heights; the colours are not kept).
  // @param fixed_T_base base_link in the fixed frame at the frame's stamp.
  // @param stamp_ns the frame stamp, nanoseconds.
  // @param keep_s samples older than this relative to stamp_ns are forgotten (infinity keeps every cell).
  void set(
    const GuidanceSamples & samples, const Eigen::Isometry3d & fixed_T_base, int64_t stamp_ns,
    double keep_s = std::numeric_limits<double>::infinity());

  // Forget the samples.
  void clear();

  // @return true when no samples are kept.
  bool empty() const { return points_.empty(); }

  // @return the number of kept samples.
  std::size_t size() const { return points_.size(); }

  // @param stamp_ns the reference stamp, nanoseconds.
  // @return seconds from the newest kept frame's stamp to stamp_ns (negative when the frame is newer); +infinity when
  // empty.
  double age_s(int64_t stamp_ns) const;

  // @param base_T_fixed the fixed frame in base_link.
  // @param stamp_ns when set, only samples whose frame is within max_age_s of this stamp (either side) are returned.
  // @param max_age_s the age limit, seconds.
  // @return the kept points in base_link and their heights (same order).
  std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<float>> points_in_base(
    const Eigen::Isometry3d & base_T_fixed, std::optional<int64_t> stamp_ns = std::nullopt,
    double max_age_s = std::numeric_limits<double>::infinity()) const;

private:
  int64_t cell_of(const pcl::PointXYZ & p) const;

  double cell_size_m_;
  pcl::PointCloud<pcl::PointXYZ> points_;  // fixed frame
  std::vector<float> heights_;
  std::vector<int64_t> stamps_ns_;  // of each sample's frame
  int64_t newest_ns_ = 0;
};

// The edge points of one cycle, in base_link. The caller appends them to the stable lethal clouds.
struct EdgePoints
{
  pcl::PointCloud<pcl::PointXYZ> obstacle;
  pcl::PointCloud<pcl::PointXYZ> drop;
};

// Guidance points that extend the stable lethal clouds to the edge the image shows.
// An obstacle edge point has height >= min_height_m and lies within max_shift_m (xy) of a stable obstacle point; a
// drop edge point has height <= -min_height_m and lies near a stable drop point. Each set is decimated to one centroid
// per voxel_m voxel. With an empty stable cloud the matching set is empty: guidance never makes a lethal point alone.
// @param guidance_base guidance points in base_link.
// @param heights their heights above the floor (same size).
// @param stable_obstacle the stable obstacle points in base_link.
// @param stable_drop the stable drop points in base_link.
// @param config the edge settings.
// @return the edge points (empty when config.enabled is false or the sizes differ).
EdgePoints edge_points(
  const pcl::PointCloud<pcl::PointXYZ> & guidance_base, const std::vector<float> & heights,
  const pcl::PointCloud<pcl::PointXYZ> & stable_obstacle, const pcl::PointCloud<pcl::PointXYZ> & stable_drop,
  const EdgeGuidanceConfig & config);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__EDGE_POINTS_HPP_
