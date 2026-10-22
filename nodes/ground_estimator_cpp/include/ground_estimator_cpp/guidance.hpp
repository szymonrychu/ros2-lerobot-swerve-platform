// The guidance frame (design D3, D5, D12): an upsampled depth image plus its RGB, turned into floor-relative depth
// samples and no-return floor hits in base_link. Pure core code, no ROS node wiring.
#ifndef GROUND_ESTIMATOR_CPP__GUIDANCE_HPP_
#define GROUND_ESTIMATOR_CPP__GUIDANCE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/plane_fit.hpp"

namespace ground_estimator_cpp
{

// A plumb_bob camera calibration. Pixel (u, v) integer coordinates are pixel centres (OpenCV convention).
struct CameraModel
{
  cv::Matx33d K = cv::Matx33d::zeros();
  cv::Vec<double, 5> D = cv::Vec<double, 5>::zeros();
  int width = 0;
  int height = 0;

  bool operator==(const CameraModel & o) const;
  bool operator!=(const CameraModel & o) const { return !(*this == o); }
};

// The calibration of a CameraInfo.
// @param info the camera info (plumb_bob; d has 5 coefficients, or none for an undistorted camera).
// @return the model, or nullopt for an all-zero K, a non-plumb_bob model, an unusable d or a zero size.
std::optional<CameraModel> camera_model_of(const sensor_msgs::msg::CameraInfo & info);

// The normalised rays (x, y with z = 1, undistorted with cv::undistortPoints) of the sampling grid: every
// step-th pixel centre (u = i * step, v = j * step). Built once per model and step.
class RayGrid
{
public:
  // An empty grid; ensure() builds it.
  RayGrid() = default;

  // @param model the camera model (size > 0).
  // @param step_px grid stride in pixels (>= 1).
  // @throws std::invalid_argument on a non-positive size or step.
  RayGrid(const CameraModel & model, int step_px);

  // Rebuild only when the model or the step differ from the ones the grid was built from.
  // @param model the camera model.
  // @param step_px grid stride in pixels.
  // @return true when the grid was rebuilt.
  bool ensure(const CameraModel & model, int step_px);

  // @return true when the grid was built from this model and step.
  bool matches(const CameraModel & model, int step_px) const;

  // @return grid columns / rows (0 for an empty grid).
  int cols() const { return cols_; }
  int rows() const { return rows_; }
  int step() const { return step_; }
  // @return the model the grid was built from.
  const CameraModel & model() const { return model_; }
  // @return the normalised (x, y) of grid node (i, j), pixel (i * step, j * step).
  const cv::Vec2d & ray(int i, int j) const { return rays_[static_cast<std::size_t>(j) * cols_ + i]; }

private:
  CameraModel model_;
  int step_ = 0;
  int cols_ = 0;
  int rows_ = 0;
  std::vector<cv::Vec2d> rays_;
};

// One RGB-D pair, decoded and checked.
struct GuidanceFrame
{
  int64_t stamp_ns = 0;
  std::string frame_id;
  cv::Mat depth_mm;  // CV_16UC1, z along the optical axis in mm, 0 = no return
  cv::Mat bgr;       // CV_8UC3, same size as depth_mm
  CameraModel model;
};

// Decode a depth image, its camera info and its JPEG.
// @param depth the depth image (16UC1).
// @param info its camera info (size equal to the depth).
// @param rgb the JPEG (same size as the depth).
// @param why when not null, set to the reason on rejection.
// @return the frame, or nullopt for: a depth that is not 16UC1 or shorter than its size, an unusable camera info, a
// camera info size different from the depth, a JPEG that does not decode, a JPEG size different from the depth.
std::optional<GuidanceFrame> decode_guidance(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & info,
  const sensor_msgs::msg::CompressedImage & rgb, std::string * why);

// The samples of one frame, in base_link; colours are R G B.
struct GuidanceSamples
{
  pcl::PointCloud<pcl::PointXYZ> points;
  std::vector<float> heights;  // signed height above the plane, one per point
  std::vector<cv::Vec3b> colours;
  pcl::PointCloud<pcl::PointXYZ> no_return;  // floor points the ToF should have seen
  std::vector<cv::Vec3b> no_return_colours;
};

// Sample the frame on the ray grid.
// A pixel with depth z gives the point ray * z moved to base_link, with its height above the plane, when its range
// from the camera is <= max_range_m. A pixel without depth gives a no-return sample when its ray meets the plane in
// front of the camera, the hit is within [no_return.min_range_m, max_range_m] horizontally of the camera, and the hit
// projected into the ToF image lands at least no_return.tof_margin_px inside it. Other pixels give nothing.
// @param frame the decoded frame.
// @param grid the grid built from frame.model (a differing model gives no samples).
// @param base_T_cam camera optical frame in base_link.
// @param plane the floor plane in base_link.
// @param tof the ToF camera model.
// @param tof_T_base base_link in the ToF optical frame.
// @param config the guidance settings.
// @return the samples.
GuidanceSamples sample_guidance(
  const GuidanceFrame & frame, const RayGrid & grid, const Eigen::Isometry3d & base_T_cam, const Plane & plane,
  const CameraModel & tof, const Eigen::Isometry3d & tof_T_base, const GuidanceConfig & config);

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__GUIDANCE_HPP_
