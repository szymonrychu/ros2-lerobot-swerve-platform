// Per-cell temporal median of plane-relative heights. Points are binned by their xy in the fixed frame, so the cells
// stay put while the robot or the head moves; a cell's height is the median of its last few per-frame medians.
#ifndef GROUND_ESTIMATOR_CPP__CELL_MEDIAN_HPP_
#define GROUND_ESTIMATOR_CPP__CELL_MEDIAN_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <utility>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

class CellMedianFilter
{
public:
  // @param cfg the temporal settings.
  explicit CellMedianFilter(const TemporalConfig & cfg);

  // Add one frame and return the points of the cells that have enough history and a steady value (a spread over
  // temporal.max_spread_m makes the cell unknown and its history is kept; it is known again once the spread is well
  // under the limit), moved along the plane normal to the cell's temporal median height. Points of younger cells are not output. O(points + cells).
  // @param points_base the cloud in base_link.
  // @param plane the floor plane in base_link.
  // @param fixed_T_base pose of base_link in the fixed frame.
  // @param stamp_s frame time in seconds; a stamp older than the newest seen resets the filter first.
  // @return the filtered points in base_link; the input unchanged when disabled.
  pcl::PointCloud<pcl::PointXYZ> apply(
    const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const Eigen::Isometry3d & fixed_T_base,
    double stamp_s);

  // Forget every cell.
  void reset();

  // @return cells that currently hold at least one per-frame value.
  std::size_t cell_count() const { return cells_.size(); }

  // @return cells that were unknown in the last apply because the robust spread (1.4826 * MAD) of their per-frame
  //   medians exceeded temporal.max_spread_m.
  std::size_t gated_cell_count() const { return gated_cell_count_; }

private:
  struct Cell
  {
    std::deque<std::pair<double, double>> history;  // (stamp, per-frame median)
    // unknown until the spread falls well under the limit again
    bool gated = false;
  };

  TemporalConfig cfg_;
  std::unordered_map<std::int64_t, Cell> cells_;
  double newest_stamp_s_ = 0.0;
  bool has_stamp_ = false;
  std::size_t gated_cell_count_ = 0;
};

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__CELL_MEDIAN_HPP_
