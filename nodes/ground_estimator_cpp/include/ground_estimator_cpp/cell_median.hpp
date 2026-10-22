// Per-cell temporal median of plane-relative heights. Points are binned by their xy in the fixed frame, so the cells
// stay put while the robot or the head moves; a cell's height at its centre is the median of its last few per-frame
// values, and so is its in-cell height gradient (a robust plane fit per frame), so a ramp keeps its slope inside a cell.
#ifndef GROUND_ESTIMATOR_CPP__CELL_MEDIAN_HPP_
#define GROUND_ESTIMATOR_CPP__CELL_MEDIAN_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

// One frame of one cell: the plane h = height + gradient_x * dx + gradient_y * dy about the cell centre (fixed-frame
// dx, dy), height and gradient from a robust fit of the cell's points, and the plain median of the points' heights.
struct CellSample
{
  double stamp_s = 0.0;
  double median_m = 0.0;  // median plane-relative height of the cell's points
  double height_m = 0.0;  // height of the fitted plane at the cell centre
  double gradient_x = 0.0;
  double gradient_y = 0.0;
  double gradient_se_x = 0.0;  // standard error of gradient_x from the fit residuals
  double gradient_se_y = 0.0;
  double residual_m = std::numeric_limits<double>::quiet_NaN();  // RMS of the heights about the plane; NaN unfitted
};

class CellMedianFilter
{
public:
  // @param cfg the temporal settings.
  explicit CellMedianFilter(const TemporalConfig & cfg);

  // Add one frame and return the points of the cells that have enough history and a steady value (a spread over
  // temporal.max_spread_m makes the cell unknown and its history is kept; it is known again once the spread is well
  // under the limit), moved along the plane normal to the cell's temporal median: for an inclined cell (a significant
  // in-cell gradient, a plane residual like its flat neighbours' and a neighbour of a similar gradient) the median
  // height at the cell centre plus the median gradient times the offset from the centre, for any other cell (flat, a
  // step edge) the median of its per-frame median heights. The spread is that of the value used. Points of younger
  // cells are not output. O(points + cells).
  // @param points_base the cloud in base_link.
  // @param plane the floor plane in base_link.
  // @param fixed_T_base pose of base_link in the fixed frame.
  // @param stamp_s frame time in seconds; a stamp older than the newest seen resets the filter first.
  // @param raw_heights if not null, receives the plane-relative height of each output point before it was moved (its
  //   own height in this frame), parallel to the returned cloud.
  // @return the filtered points in base_link; the input unchanged when disabled.
  pcl::PointCloud<pcl::PointXYZ> apply(
    const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const Eigen::Isometry3d & fixed_T_base,
    double stamp_s, std::vector<float> * raw_heights = nullptr);

  // Forget every cell.
  void reset();

  // @return cells that currently hold at least one per-frame value.
  std::size_t cell_count() const { return cells_.size(); }

  // @return cells that were unknown in the last apply because the robust spread of their per-frame medians
  //   exceeded temporal.max_spread_m (1.4826 * MAD below 5 values, scaled trimmed standard deviation from 5 on; see
  //   the README).
  std::size_t gated_cell_count() const { return gated_cell_count_; }

  // Whether a cell is steady enough to be lethal: output in the last apply with a robust spread of at most
  // max_spread_m, and neither it nor any of its 8 neighbours spread-gated in its last quiet_frames frames with enough
  // history (at the edge of a noisy band a cell mixing floor and band returns has a steady median between the two).
  // A cell never gated is steady from its first output on, so a new steady obstacle has no extra delay. A gated run
  // that is a turnover (the cell was output before it, the run is at most window_frames long, every value that arrived
  // during it lies within max_spread_m of the level output after it, and that level differs from the one before by
  // more than max_spread_m: an object placed or removed) does not count as a gate. Always true when disabled.
  // @param xy_fixed a point in the fixed frame.
  // @param max_spread_m largest spread in metres (at most temporal.max_spread_m to be stricter than the gate).
  // @param quiet_frames frames since the cell was last gated that are needed, >= 0.
  // @return true when the cell of xy_fixed is steady.
  bool steady_at(const Eigen::Vector2d & xy_fixed, double max_spread_m, int quiet_frames) const;

private:
  struct Cell
  {
    std::deque<CellSample> history;  // per-frame samples (stamp, median, plane about the centre)
    std::optional<double> spread;    // of the last apply in which the cell was seen with min_frames
    bool output = false;             // the cell's points were output in the last apply
    int frames_since_gated = std::numeric_limits<int>::max();  // frames with min_frames values since the last gate
    // The current run of gated frames, to tell a turnover (a new surface replacing the old one in the history) from
    // noise: the level output before the run, frames_since_gated before it, and the range of the values that arrived
    // during it.
    std::optional<double> level;             // value output in the last apply in which the cell was output
    std::optional<double> level_before_run;  // level when the current gated run began
    int frames_since_gated_before_run = std::numeric_limits<int>::max();
    int run_frames = 0;
    double run_min = 0.0;
    double run_max = 0.0;
  };

  std::int64_t key_of(const Eigen::Vector2d & xy_fixed) const;

  TemporalConfig cfg_;
  std::unordered_map<std::int64_t, Cell> cells_;
  double newest_stamp_s_ = 0.0;
  bool has_stamp_ = false;
  std::size_t gated_cell_count_ = 0;
};

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__CELL_MEDIAN_HPP_
