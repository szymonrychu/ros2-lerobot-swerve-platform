#include "ground_estimator_cpp/ground_classes.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ground_estimator_cpp
{

namespace
{
constexpr std::uint32_t CLASS_OFFSET = 12;
constexpr std::uint32_t POINT_STEP = 16;
constexpr std::uint32_t FLOAT_BYTES = 4;
}  // namespace

const char * to_string(GroundClass c)
{
  switch (c) {
    case GroundClass::unknown:
      return "unknown";
    case GroundClass::floor:
      return "floor";
    case GroundClass::step_up:
      return "step_up";
    case GroundClass::step_down:
      return "step_down";
    case GroundClass::ramp:
      return "ramp";
    case GroundClass::obstacle:
      return "obstacle";
    case GroundClass::wall:
      return "wall";
    case GroundClass::drop:
      return "drop";
    case GroundClass::overhang:
      return "overhang";
  }
  return "unknown";
}

ClassCost cost_of(GroundClass c)
{
  switch (c) {
    case GroundClass::unknown:
      return ClassCost::unknown;
    case GroundClass::floor:
      return ClassCost::free;
    case GroundClass::step_up:
      return ClassCost::slow;
    case GroundClass::step_down:
      return ClassCost::slow;
    case GroundClass::ramp:
      return ClassCost::slow;
    case GroundClass::obstacle:
      return ClassCost::lethal;
    case GroundClass::wall:
      return ClassCost::lethal;
    case GroundClass::drop:
      return ClassCost::lethal;
    case GroundClass::overhang:
      return ClassCost::ignored;
  }
  return ClassCost::unknown;
}

ClassLimits class_limits(const Config & c)
{
  ClassLimits l;
  l.floor_band_m = c.bands.floor_band_m;
  l.max_step_up_m = c.max_step_up_m();
  l.max_step_down_m = c.max_step_down_m();
  l.wall_min_height_m = c.edges.step_max_height_m;
  l.robot_height_m = c.traversability.robot_height_m;
  return l;
}

GroundClass point_class(double filtered_h_m, double raw_h_m, const PointCell & cell, const ClassLimits & limits)
{
  if (raw_h_m > limits.robot_height_m && cell.cls != CellClass::obstacle) {
    return GroundClass::overhang;
  }
  if (std::fabs(filtered_h_m) <= limits.floor_band_m) {
    return GroundClass::floor;
  }
  const bool above = filtered_h_m > 0.0;
  switch (cell.cls) {
    case CellClass::obstacle:
      if (!above) {
        return GroundClass::unknown;
      }
      return std::max(cell.top_m, filtered_h_m) > limits.wall_min_height_m ? GroundClass::wall : GroundClass::obstacle;
    case CellClass::drop:
      return above ? GroundClass::unknown : GroundClass::drop;
    case CellClass::warning:
      if (cell.kind == "ramp") {
        return GroundClass::ramp;
      }
      if (cell.kind == "step" || cell.kind == "drop") {
        return above ? GroundClass::step_up : GroundClass::step_down;
      }
      return GroundClass::unknown;
    case CellClass::none:
      return GroundClass::unknown;
  }
  return GroundClass::unknown;
}

sensor_msgs::msg::PointCloud2 to_classified_msg(
  const pcl::PointCloud<pcl::PointXYZ> & points, const std::vector<GroundClass> & classes, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame;
  msg.header.stamp = stamp;
  const std::size_t n = std::min(points.size(), classes.size());
  const char * names[] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = i * FLOAT_BYTES;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  sensor_msgs::msg::PointField cls;
  cls.name = "class";
  cls.offset = CLASS_OFFSET;
  cls.datatype = sensor_msgs::msg::PointField::UINT8;
  cls.count = 1;
  msg.fields.push_back(cls);
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(n);
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = POINT_STEP;
  msg.row_step = POINT_STEP * msg.width;
  msg.data.assign(static_cast<std::size_t>(msg.row_step), 0);
  for (std::size_t i = 0; i < n; ++i) {
    std::uint8_t * out = msg.data.data() + i * POINT_STEP;
    const float xyz[3] = {points[i].x, points[i].y, points[i].z};
    std::memcpy(out, xyz, sizeof(xyz));
    out[CLASS_OFFSET] = static_cast<std::uint8_t>(classes[i]);
  }
  return msg;
}

}  // namespace ground_estimator_cpp
