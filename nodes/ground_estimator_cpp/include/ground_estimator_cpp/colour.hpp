// Floor colour model in CIELAB and per-point labels of a coloured cloud: the colour tells floor from obstacle, including
// flat objects the depth cannot separate. The labels are not the traversability class (ground-traversability-rule).
#ifndef GROUND_ESTIMATOR_CPP__COLOUR_HPP_
#define GROUND_ESTIMATOR_CPP__COLOUR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

// Label of a point of the labelled cloud (the value of its uint8 field).
enum class Label : uint8_t
{
  floor = 0,
  above_floor = 1,
  below_floor = 2,
  floor_colour_mismatch = 3,
  unlabelled = 4
};

// CIELAB of an sRGB colour.
// @param rgb colour in R, G, B order (0..255).
// @return L (0..100), a, b.
cv::Vec3f rgb_to_lab(const cv::Vec3b & rgb);

// Running Gaussian (mean and covariance in CIELAB) of the floor colour.
class FloorColourModel
{
public:
  // Blend a batch of floor samples into the model: mean and covariance move by weight forgetting towards the batch
  // (the first batch initialises the model); the sample count only grows.
  // @param rgb_floor_samples floor colours in R, G, B order; empty is a no-op.
  // @param forgetting weight of the batch, 0 < v <= 1.
  // @param min_floor_samples samples needed before ready().
  void update(const std::vector<cv::Vec3b> & rgb_floor_samples, double forgetting, int min_floor_samples);

  // @return true once at least min_floor_samples samples were seen (as of the last update).
  bool ready() const { return ready_; }

  // @return total samples seen.
  std::size_t samples() const { return count_; }

  // Mahalanobis distance of a colour to the model.
  // @param rgb colour in R, G, B order.
  // @return the distance (unitless); 0 when the model has no samples.
  double mahalanobis(const cv::Vec3b & rgb) const;

private:
  Eigen::Vector3d mean_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d cov_inv_ = Eigen::Matrix3d::Identity();
  std::size_t count_ = 0;
  bool ready_ = false;
};

// Point counts per label.
struct LabelStats
{
  std::size_t floor = 0, above = 0, below = 0, mismatch = 0, unlabelled = 0;
  std::optional<double> mismatch_ratio;  // mismatch / (floor + mismatch); nullopt without floor-band points
};

// A floor_colour_mismatch point with its height above the plane.
struct MismatchPoint
{
  Eigen::Vector3d point;
  double height = 0.0;
};

// Label every point. Band by classify() (geometry wins: above and below stay so whatever their colour); floor-band
// points first update the model, then those with model ready and mahalanobis > colour.max_mahalanobis are
// floor_colour_mismatch. Points outside every band or non-finite are unlabelled. colour.enabled false gives the
// geometric labels only and leaves the model untouched.
// @param cloud_base coloured cloud in base_link.
// @param plane fitted floor plane.
// @param bands band limits.
// @param colour colour settings.
// @param floor_distance_m half thickness of the floor band.
// @param model floor colour model, updated.
// @param labels output, one label per point.
// @return the counts.
LabelStats label_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud_base, const Plane & plane, const BandsConfig & bands,
  const ColourConfig & colour, double floor_distance_m, FloorColourModel & model, std::vector<Label> & labels);

// The floor_colour_mismatch points with their heights (input of colour_warning_areas).
// @param cloud_base the labelled cloud.
// @param labels labels from label_cloud, parallel to the cloud.
// @param plane the plane used for the labels.
// @return the points.
std::vector<MismatchPoint> mismatch_points(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud_base, const std::vector<Label> & labels, const Plane & plane);

// PointCloud2 with x, y, z (float32), rgb (float32 packed as head_rgbd_cpp) and label (uint8).
// @param cloud the points.
// @param labels labels parallel to the cloud.
// @param frame header frame id.
// @param stamp header stamp.
// @return the message.
sensor_msgs::msg::PointCloud2 to_labelled_msg(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const std::vector<Label> & labels, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__COLOUR_HPP_
