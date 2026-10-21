// Per-cell class hysteresis in the fixed frame. Each frame every cell of the temporal grid (the CellMedianFilter grid:
// floor(x / cell_size_m), floor(y / cell_size_m) in the fixed frame) gets one vote, the strongest class it was observed
// with; a cell's stable class needs enter_frames of the last window_frames votes to enter and exit_frames to stay, so
// a label flipping frame to frame settles. Stable warning cells are grouped into warning areas whose polygons are
// built from the cell corners, so they do not move while the classes hold. The filter keeps no points: callers
// publish only this frame's points of stable lethal cells (no placeholder data).
#ifndef GROUND_ESTIMATOR_CPP__CELL_CLASS_HPP_
#define GROUND_ESTIMATOR_CPP__CELL_CLASS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/traversability.hpp"

namespace ground_estimator_cpp
{

enum class CellClass : uint8_t
{
  none,
  warning,
  obstacle,
  drop
};

// A warning area of stable warning cells, in the fixed frame.
struct StableArea
{
  std::vector<Eigen::Vector2d> polygon;  // convex hull of the cells' corners, counter-clockwise
  std::string kind;                      // most frequent recorded kind of the cells (ties: lexicographically smallest)
  double height_m = 0.0;                 // median of the cells' recorded heights
  double slope_deg = 0.0;                // median of the cells' recorded slopes; 0 when no cell has one
  std::string source;                    // "tof" when any cell was recorded from a tof observation, else "colour"
};

class CellClassFilter
{
public:
  // @param cfg the hysteresis settings (validated: 1 <= exit_frames <= enter_frames <= window_frames).
  // @param cell_size_m edge of a fixed-frame cell in metres (temporal.cell_size_m), > 0.
  CellClassFilter(const HysteresisConfig & cfg, double cell_size_m);

  // Start a frame: forget the previous frame's observations (the vote history is kept).
  void begin_frame();

  // Observe this frame's points of one class. A cell's label is the strongest it receives in the frame (drop or
  // obstacle over warning over none; between drop and obstacle the later observation wins). A cell labelled by points
  // records kind "drop" (drop) or "step" (obstacle, warning), source "tof", height_m the median base_link z of its
  // points of that class and no slope. CellClass::none is ignored.
  // @param points_base points in base_link; non-finite points are skipped.
  // @param cls the class of every point.
  // @param fixed_T_base pose of base_link in the fixed frame.
  void observe_points(
    const pcl::PointCloud<pcl::PointXYZ> & points_base, CellClass cls, const Eigen::Isometry3d & fixed_T_base);

  // Observe one warning area: label warning on the cells whose centre lies inside the polygon and on the cell of the
  // polygon centroid, and record the area's kind, height_m, slope_deg and source on them (unless the cell is labelled
  // lethal this frame, whose record comes from its points). A polygon with no vertex is ignored.
  // @param area_base the area, polygon in base_link (on the floor, z = 0).
  // @param fixed_T_base pose of base_link in the fixed frame.
  void observe_area(const WarningArea & area_base, const Eigen::Isometry3d & fixed_T_base);

  // End a frame: every tracked or observed cell gets one vote (none when not observed), the window keeps the last
  // window_frames votes, a cell whose window holds only none is forgotten, and the stable classes are updated.
  void end_frame();

  // @param xy_fixed a point in the fixed frame.
  // @return the stable class of its cell; none for an untracked cell.
  CellClass stable_at(const Eigen::Vector2d & xy_fixed) const;

  // @return one area per 8-connected component of stable warning cells, ordered by the component's smallest cell
  //         key (ix, then iy).
  std::vector<StableArea> warning_areas() const;

  // @param cls a class.
  // @return the number of tracked cells whose stable class is cls.
  std::size_t count(CellClass cls) const;

  // Forget every cell and this frame's observations.
  void reset();

private:
  using Key = std::pair<std::int32_t, std::int32_t>;  // (ix, iy)

  // What a cell records of its latest observation, read when it forms a warning area.
  struct Record
  {
    std::string kind;
    double height_m = 0.0;
    std::optional<double> slope_deg;
    std::string source;
  };

  // A cell's observations of the current frame.
  struct FrameObservation
  {
    CellClass label = CellClass::none;
    std::vector<double> obstacle_z, drop_z, warning_z;
    std::optional<Record> area;
  };

  // A tracked cell.
  struct Cell
  {
    std::deque<CellClass> votes;
    CellClass stable = CellClass::none;
    CellClass last_lethal = CellClass::obstacle;
    std::optional<Record> record;
  };

  Key key_of(const Eigen::Vector2d & xy_fixed) const;
  FrameObservation & observe(const Key & key, CellClass cls);

  HysteresisConfig cfg_;
  double cell_size_m_;
  std::map<Key, FrameObservation> frame_;
  std::map<Key, Cell> cells_;
};

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__CELL_CLASS_HPP_
