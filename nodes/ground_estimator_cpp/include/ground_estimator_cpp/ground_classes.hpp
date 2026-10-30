// The traversability classes every perceived surface carries (task T3): one class set shared by the planner (Nav2) and
// the 3D map (mcp_server map3d, web_ui), from one config of limits (TraversabilityConfig, BandsConfig). The enum is
// mirrored in shared/ros2_common/ground_classes.py; tests/test_ground_classes_contract.py pins both to each other.
// /ground/classified_points carries one class per measured point (x, y, z float32 and `class` uint8, fixed frame).
#ifndef GROUND_ESTIMATOR_CPP__GROUND_CLASSES_HPP_
#define GROUND_ESTIMATOR_CPP__GROUND_CLASSES_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "ground_estimator_cpp/cell_class.hpp"
#include "ground_estimator_cpp/config.hpp"

namespace ground_estimator_cpp
{

// The class of a surface point. The values are the wire format of the `class` field.
enum class GroundClass : std::uint8_t
{
  unknown = 0,    // measured, but no class is settled (cell not stable, spread-gated, in an occlusion shadow)
  floor = 1,      // within bands.floor_band_m of the floor plane
  step_up = 2,    // a climbable step up: above the floor band, at most the step up limit (slow down)
  step_down = 3,  // a climbable step down: below the floor band, at most the step down limit (slow down)
  ramp = 4,       // an incline up to traversability.max_slope_deg (slow down)
  obstacle = 5,   // too high a step or too steep (lethal)
  wall = 6,       // a lethal cell taller than edges.step_max_height_m: a wall or a tall object (lethal)
  drop = 7,       // deeper than the step down limit (lethal, no-go)
  overhang = 8    // above traversability.robot_height_m over a cell that is not blocked: ignored for driving
};

constexpr std::size_t GROUND_CLASS_COUNT = 9;

// How the planner treats a class (Nav2: lethal marks through STVL, slow-down areas through costmap_zones).
enum class ClassCost : std::uint8_t
{
  unknown,  // nothing claimed: neither free nor blocked
  free,
  slow,
  lethal,
  ignored  // never marked (overhang)
};

// @param c a class.
// @return its name, as in the Python mirror ("unknown", "floor", "step_up", ...).
const char * to_string(GroundClass c);

// @param c a class.
// @return the cost policy of the class.
ClassCost cost_of(GroundClass c);

// The limits of the classes, all from the one ground estimator config.
struct ClassLimits
{
  double floor_band_m = 0.0;       // bands.floor_band_m
  double max_step_up_m = 0.0;      // max_step_up_m(traversability)
  double max_step_down_m = 0.0;    // max_step_down_m(traversability)
  double wall_min_height_m = 0.0;  // edges.step_max_height_m: the tallest step-like object
  double robot_height_m = 0.0;     // traversability.robot_height_m
};

// @param c the config.
// @return its class limits.
ClassLimits class_limits(const Config & c);

// What the point's fixed-frame cell says.
struct PointCell
{
  CellClass cls = CellClass::none;  // the stable class of the cell (CellClassFilter::stable_at)
  std::string kind;                 // the warning kind recorded on a warning cell (step, drop, ramp, colour, no_return)
  double top_m = 0.0;               // the highest plane-relative height of the frame's points in an obstacle cell
};

// The class of one measured point.
// - An overhang: its raw height is above the robot height and its cell is not an obstacle cell.
// - Floor: |filtered height| within the floor band, whatever the cell.
// - An obstacle cell: wall when its top or the point is above wall_min_height_m, else obstacle (a point below the floor
//   band there is unknown).
// - A drop cell: drop below the floor band (unknown above it).
// - A warning cell: ramp (kind ramp), step_up above the floor band, step_down below it (kind step or drop); a colour or
//   no_return warning says nothing about the height, so unknown.
// - Any other cell: unknown off the floor band (a cell not stable yet, or too few points to be lethal).
// @param filtered_h_m plane-relative height after the temporal median (the cell's stable height).
// @param raw_h_m the point's own plane-relative height in this frame (a cell median mixes the levels under a table).
// @param cell the point's cell.
// @param limits the class limits.
// @return the class.
GroundClass point_class(double filtered_h_m, double raw_h_m, const PointCell & cell, const ClassLimits & limits);

// The PointCloud2 of /ground/classified_points: x, y, z float32 at 0, 4, 8, `class` uint8 at 12, 16 bytes per point,
// little endian, dense, height 1.
// @param points the points (in frame).
// @param classes one class per point (parallel; extra entries on either side are ignored).
// @param frame header frame id.
// @param stamp header stamp.
// @return the message.
sensor_msgs::msg::PointCloud2 to_classified_msg(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<GroundClass> & classes, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__GROUND_CLASSES_HPP_
