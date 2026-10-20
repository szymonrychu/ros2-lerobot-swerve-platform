// Candidate selection by signed height above the fitted floor plane. No traversability decision here.
#ifndef GROUND_ESTIMATOR_CPP__SEGMENTATION_HPP_
#define GROUND_ESTIMATOR_CPP__SEGMENTATION_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

enum class Band : uint8_t
{
  floor,
  above,
  below,
  none
};

// Band of a point.
// @param height signed height above the plane in metres.
// @param range horizontal distance from the base_link origin in metres.
// @param bands band limits and maximum range.
// @param floor_distance_m half thickness of the floor band (fit.distance_m).
// @return floor: |h| <= floor_distance_m; above: up to bands.obstacle_max_m; below: down to -bands.drop_max_m;
//         none otherwise or when range > bands.max_range_m.
Band classify(double height, double range, const BandsConfig & bands, double floor_distance_m);

// Points per band, in their base_link coordinates, with signed heights parallel to above and below.
struct Segmented
{
  pcl::PointCloud<pcl::PointXYZ> above, below, floor;
  std::vector<float> above_height, below_height;
};

// Split a cloud by band. Non-finite points are skipped.
// @param points_base cloud in base_link.
// @param plane fitted floor plane.
// @param bands band limits.
// @param floor_distance_m half thickness of the floor band.
// @return the clouds (x, y, z unchanged) and the signed heights of above and below.
Segmented segment(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const BandsConfig & bands,
  double floor_distance_m);

// PointCloud2 (x, y, z float32, dense) of a cloud; an empty cloud is a valid message with width 0.
// @param cloud the points.
// @param frame header frame id.
// @param stamp header stamp.
// @return the message.
sensor_msgs::msg::PointCloud2 to_cloud_msg(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::string & frame, const builtin_interfaces::msg::Time & stamp);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__SEGMENTATION_HPP_
