// The shared traversability classes (task T3): the class contract, the per-point rule and the classified cloud layout.
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "ground_estimator_cpp/cell_class.hpp"
#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/ground_classes.hpp"

namespace
{

using ground_estimator_cpp::CellClass;
using ground_estimator_cpp::class_limits;
using ground_estimator_cpp::ClassCost;
using ground_estimator_cpp::ClassLimits;
using ground_estimator_cpp::Config;
using ground_estimator_cpp::cost_of;
using ground_estimator_cpp::GroundClass;
using ground_estimator_cpp::point_class;
using ground_estimator_cpp::PointCell;

ClassLimits limits() { return class_limits(Config()); }

// A point of a cell with the given stable class, warning kind and top (the highest point of its obstacle cell).
PointCell cell(CellClass cls, const std::string & kind = "", double top_m = 0.0)
{
  PointCell c;
  c.cls = cls;
  c.kind = kind;
  c.top_m = top_m;
  return c;
}

}  // namespace

TEST(GroundClasses, ContractValuesNamesAndCosts)
{
  // tests/test_ground_classes_contract.py pins these to shared/ros2_common/ground_classes.py.
  const std::vector<std::pair<GroundClass, std::string>> expected = {
    {GroundClass::unknown, "unknown"},     {GroundClass::floor, "floor"}, {GroundClass::step_up, "step_up"},
    {GroundClass::step_down, "step_down"}, {GroundClass::ramp, "ramp"},   {GroundClass::obstacle, "obstacle"},
    {GroundClass::wall, "wall"},           {GroundClass::drop, "drop"},   {GroundClass::overhang, "overhang"}};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(static_cast<int>(expected[i].first), static_cast<int>(i));
    EXPECT_EQ(ground_estimator_cpp::to_string(expected[i].first), expected[i].second);
  }
  EXPECT_EQ(ground_estimator_cpp::GROUND_CLASS_COUNT, expected.size());
  EXPECT_EQ(cost_of(GroundClass::unknown), ClassCost::unknown);
  EXPECT_EQ(cost_of(GroundClass::floor), ClassCost::free);
  for (const GroundClass c : {GroundClass::step_up, GroundClass::step_down, GroundClass::ramp}) {
    EXPECT_EQ(cost_of(c), ClassCost::slow) << ground_estimator_cpp::to_string(c);
  }
  for (const GroundClass c : {GroundClass::obstacle, GroundClass::wall, GroundClass::drop}) {
    EXPECT_EQ(cost_of(c), ClassCost::lethal) << ground_estimator_cpp::to_string(c);
  }
  EXPECT_EQ(cost_of(GroundClass::overhang), ClassCost::ignored);
}

TEST(GroundClasses, LimitsComeFromTheOneConfig)
{
  const ClassLimits l = limits();
  EXPECT_DOUBLE_EQ(l.floor_band_m, 0.02);
  EXPECT_NEAR(l.max_step_up_m, 0.04, 1e-4);
  EXPECT_NEAR(l.max_step_down_m, 0.04, 1e-4);
  EXPECT_DOUBLE_EQ(l.wall_min_height_m, 0.2);  // edges.step_max_height_m: taller than any step
  EXPECT_DOUBLE_EQ(l.robot_height_m, 0.6);
  Config c;
  c.traversability.max_step_down_fraction = 0.25;
  EXPECT_NEAR(class_limits(c).max_step_down_m, 0.03, 1e-9);
  EXPECT_NEAR(class_limits(c).max_step_up_m, 0.04, 1e-4);
}

TEST(GroundClasses, FloorBandIsFloorWhateverTheCell)
{
  const ClassLimits l = limits();
  for (const CellClass c : {CellClass::none, CellClass::warning, CellClass::obstacle, CellClass::drop}) {
    EXPECT_EQ(point_class(0.0, 0.0, cell(c, "step", 0.1), l), GroundClass::floor);
    EXPECT_EQ(point_class(0.02, 0.02, cell(c, "ramp", 0.1), l), GroundClass::floor);
    EXPECT_EQ(point_class(-0.02, -0.02, cell(c, "drop", 0.1), l), GroundClass::floor);
  }
}

TEST(GroundClasses, WarningCellsAreClimbableStepsAndRamps)
{
  const ClassLimits l = limits();
  EXPECT_EQ(point_class(0.03, 0.03, cell(CellClass::warning, "step"), l), GroundClass::step_up);
  EXPECT_EQ(point_class(-0.03, -0.03, cell(CellClass::warning, "step"), l), GroundClass::step_down);
  EXPECT_EQ(point_class(-0.03, -0.03, cell(CellClass::warning, "drop"), l), GroundClass::step_down);
  EXPECT_EQ(point_class(0.05, 0.05, cell(CellClass::warning, "ramp"), l), GroundClass::ramp);
  EXPECT_EQ(point_class(-0.05, -0.05, cell(CellClass::warning, "ramp"), l), GroundClass::ramp);
  // A colour or no-return warning says nothing about a height off the floor.
  EXPECT_EQ(point_class(0.03, 0.03, cell(CellClass::warning, "colour"), l), GroundClass::unknown);
  EXPECT_EQ(point_class(0.03, 0.03, cell(CellClass::warning, "no_return"), l), GroundClass::unknown);
}

TEST(GroundClasses, LethalCellsAreObstaclesWallsAndDrops)
{
  const ClassLimits l = limits();
  EXPECT_EQ(point_class(0.05, 0.05, cell(CellClass::obstacle, "", 0.05), l), GroundClass::obstacle);
  EXPECT_EQ(point_class(0.03, 0.03, cell(CellClass::obstacle, "", 0.10), l), GroundClass::obstacle);  // its foot
  EXPECT_EQ(point_class(0.05, 0.05, cell(CellClass::obstacle, "", 0.25), l), GroundClass::wall);      // a tall cell
  EXPECT_EQ(point_class(0.5, 0.5, cell(CellClass::obstacle, "", 0.5), l), GroundClass::wall);
  EXPECT_EQ(point_class(-0.05, -0.05, cell(CellClass::obstacle, "", 0.1), l), GroundClass::unknown);
  EXPECT_EQ(point_class(-0.20, -0.20, cell(CellClass::drop), l), GroundClass::drop);
  EXPECT_EQ(point_class(-0.03, -0.03, cell(CellClass::drop), l), GroundClass::drop);  // the lip down to the lower floor
  EXPECT_EQ(point_class(0.05, 0.05, cell(CellClass::drop), l), GroundClass::unknown);
}

TEST(GroundClasses, AboveTheRobotIsAnOverhangUnlessTheCellIsBlocked)
{
  const ClassLimits l = limits();
  // A table top at 0.7 m over floor: the robot drives under it. The raw height decides (a cell median mixes levels).
  EXPECT_EQ(point_class(0.0, 0.7, cell(CellClass::none), l), GroundClass::overhang);
  EXPECT_EQ(point_class(0.7, 0.7, cell(CellClass::warning, "step"), l), GroundClass::overhang);
  EXPECT_EQ(point_class(0.7, 0.61, cell(CellClass::none), l), GroundClass::overhang);
  // Over a blocked (obstacle) cell the column is a wall up to the top.
  EXPECT_EQ(point_class(0.7, 0.7, cell(CellClass::obstacle, "", 0.55), l), GroundClass::wall);
  // Exactly at the robot height is not above it.
  EXPECT_EQ(point_class(0.6, 0.6, cell(CellClass::none), l), GroundClass::unknown);
}

TEST(GroundClasses, UnsettledCellsOffTheFloorAreUnknown)
{
  const ClassLimits l = limits();
  EXPECT_EQ(point_class(0.10, 0.10, cell(CellClass::none), l), GroundClass::unknown);
  EXPECT_EQ(point_class(-0.10, -0.10, cell(CellClass::none), l), GroundClass::unknown);
}

TEST(GroundClasses, ClassifiedCloudLayout)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  points.push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));
  points.push_back(pcl::PointXYZ(-1.0f, 0.5f, 0.25f));
  const std::vector<GroundClass> classes = {GroundClass::drop, GroundClass::overhang};
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;
  const auto msg = ground_estimator_cpp::to_classified_msg(points, classes, "odom", stamp);
  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.header.stamp.nanosec, 34u);
  EXPECT_EQ(msg.height, 1u);
  EXPECT_EQ(msg.width, 2u);
  EXPECT_TRUE(msg.is_dense);
  EXPECT_FALSE(msg.is_bigendian);
  ASSERT_EQ(msg.fields.size(), 4u);
  const std::vector<std::string> names = {"x", "y", "z", "class"};
  const std::vector<uint32_t> offsets = {0, 4, 8, 12};
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(msg.fields[i].name, names[i]);
    EXPECT_EQ(msg.fields[i].offset, offsets[i]);
    EXPECT_EQ(msg.fields[i].count, 1u);
  }
  EXPECT_EQ(msg.fields[0].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(msg.fields[3].datatype, sensor_msgs::msg::PointField::UINT8);
  EXPECT_EQ(msg.point_step, 16u);
  EXPECT_EQ(msg.row_step, 32u);
  ASSERT_EQ(msg.data.size(), 32u);
  float y = 0.0f;
  std::memcpy(&y, &msg.data[16 + 4], sizeof(float));
  EXPECT_FLOAT_EQ(y, 0.5f);
  EXPECT_EQ(msg.data[12], static_cast<uint8_t>(GroundClass::drop));
  EXPECT_EQ(msg.data[16 + 12], static_cast<uint8_t>(GroundClass::overhang));
}
