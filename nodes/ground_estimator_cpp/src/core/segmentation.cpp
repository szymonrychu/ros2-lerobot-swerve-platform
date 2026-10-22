#include "ground_estimator_cpp/segmentation.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

namespace ground_estimator_cpp
{

Band classify(double height, double range, const BandsConfig & bands, double floor_distance_m)
{
  if (range < bands.min_range_m || range > bands.max_range_m) {
    return Band::none;
  }
  if (std::fabs(height) <= floor_distance_m) {
    return Band::floor;
  }
  if (height > 0.0) {
    return height <= bands.obstacle_max_m ? Band::above : Band::none;
  }
  return height >= -bands.drop_max_m ? Band::below : Band::none;
}

Segmented segment(
  const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const BandsConfig & bands,
  double floor_distance_m)
{
  Segmented out;
  for (const auto & p : points_base) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const double h = height_above(plane, Eigen::Vector3d(p.x, p.y, p.z));
    switch (classify(h, std::hypot(p.x, p.y), bands, floor_distance_m)) {
      case Band::floor:
        out.floor.push_back(p);
        break;
      case Band::above:
        out.above.push_back(p);
        out.above_height.push_back(static_cast<float>(h));
        break;
      case Band::below:
        out.below.push_back(p);
        out.below_height.push_back(static_cast<float>(h));
        break;
      case Band::none:
        break;
    }
  }
  return out;
}

sensor_msgs::msg::PointCloud2 to_cloud_msg(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::string & frame, const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame;
  msg.header.stamp = stamp;
  msg.is_dense = true;
  return msg;
}

}  // namespace ground_estimator_cpp
