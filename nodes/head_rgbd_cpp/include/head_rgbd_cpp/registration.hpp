// Registration of the ToF depth into the RGB camera (design D7), without ROS types: a ray table precomputed once from
// the ToF calibration (cv::undistortPoints), the extrinsic transform, a plumb_bob projection into the RGB intrinsics
// scaled to the output size, and a footprint splat (the rectangle spanned by the pixel's projected corners, clipped to
// the image) keeping the nearest depth per output pixel (z-buffer). Distortion is removed on the ToF side (rays) and
// applied on the RGB side (projection), so the output lines up with the raw, unrectified RGB image.
//
// Pixel convention (OpenCV): pixel (u, v) covers [u - 0.5, u + 0.5] x [v - 0.5, v + 0.5], its centre is (u, v).
#ifndef HEAD_RGBD_CPP__REGISTRATION_HPP_
#define HEAD_RGBD_CPP__REGISTRATION_HPP_

#include <Eigen/Geometry>
#include <cstddef>
#include <opencv2/core.hpp>

#include "head_rgbd_cpp/config.hpp"

namespace head_rgbd_cpp
{

// Largest change of aspect ratio scale_model accepts, in pixels of the requested size.
constexpr double MAX_SCALE_ASPECT_ERROR_PX = 1.0;
// Largest ToF/output depth in mm a CV_16UC1 pixel holds.
constexpr double MAX_DEPTH_MM = 65535.0;

// A pinhole camera with plumb_bob distortion (k1, k2, p1, p2, k3), as in sensor_msgs/CameraInfo. The skew K(0, 1) is
// ignored, as by cv::projectPoints.
struct PinholeModel
{
  cv::Matx33d K;         // camera matrix [fx 0 cx; 0 fy cy; 0 0 1], pixels
  cv::Vec<double, 5> D;  // plumb_bob coefficients k1, k2, p1, p2, k3
  int width = 0;         // image width, pixels
  int height = 0;        // image height, pixels
};

// Counters of one register_depth call.
struct RegistrationStats
{
  std::size_t valid_points = 0;     // ToF pixels with a depth inside [range.min_m, range.max_m]
  std::size_t behind_points = 0;    // valid points skipped: the point or a corner at z <= 0 in the RGB frame
  std::size_t folded_points = 0;    // valid points skipped: a corner beyond the radius where the RGB distortion folds
  std::size_t outside_points = 0;   // valid points whose clipped footprint was empty (outside the RGB image)
  std::size_t splatted_points = 0;  // valid points whose footprint was written into the z-buffer
  std::size_t filled_pixels = 0;    // output pixels holding a depth (non-zero) after the call
};

// The model of the same camera at another image size: fx, cx scaled by width / m.width and fy, cy by
// height / m.height with the pixel-centre convention (c' = (c + 0.5) * s - 0.5); D unchanged.
// @param m the model at its calibrated size.
// @param width the new image width in pixels (> 0).
// @param height the new image height in pixels (> 0).
// @return the scaled model.
// @throws std::invalid_argument on a non-positive size or when the aspect ratio changes by more than
//   MAX_SCALE_ASPECT_ERROR_PX pixels.
PinholeModel scale_model(const PinholeModel & m, int width, int height);

// The pixel of a normalised image point (x, y) = (X / Z, Y / Z), with plumb_bob distortion: the closed form of
// cv::projectPoints (no skew).
// @param m the camera model.
// @param x, y the undistorted normalised coordinates.
// @return the distorted pixel position (u, v).
cv::Point2d project_normalised(const PinholeModel & m, double x, double y);

// The pixel of a 3D point in a camera's optical frame, with plumb_bob distortion: the closed form of cv::projectPoints
// (no skew).
// @param m the camera model.
// @param p the point in the camera's optical frame, metres; p.z() must be > 0.
// @return the distorted pixel position (u, v).
cv::Point2d project_point(const PinholeModel & m, const Eigen::Vector3d & p);

// The squared normalised radius (x^2 + y^2 of the undistorted image point) beyond which the radial plumb_bob
// distortion of m stops increasing with the radius (folds back), searched up to a 10.0 normalised radius;
// std::numeric_limits<double>::infinity() when it is monotonic over that whole range. Points beyond it would be
// projected back into the image at a wrong place by the polynomial, so register_depth skips them.
// @param m the camera model.
// @return the squared normalised fold radius.
double fold_radius_squared(const PinholeModel & m);

// Normalised rays (x, y, with z = 1) of every pixel centre and every pixel corner of a camera, built once with
// cv::undistortPoints. A 3D point of a pixel at depth z (along the optical axis) is (x * z, y * z, z).
class RayTable
{
public:
  // Undistort every pixel centre and every pixel corner of m.
  // @param m the camera model (width, height > 0).
  // @throws std::invalid_argument on a non-positive size.
  explicit RayTable(const PinholeModel & m);

  // @return the image width, pixels.
  int width() const { return model_.width; }
  // @return the image height, pixels.
  int height() const { return model_.height; }
  // @return the camera model the table was built from.
  const PinholeModel & model() const { return model_; }

  // The ray of a pixel centre.
  // @param u column, 0 .. width - 1.
  // @param v row, 0 .. height - 1.
  // @return the normalised (x, y) of pixel (u, v).
  const cv::Vec2d & ray(int u, int v) const { return centres_(v, u); }

  // The ray of a pixel corner: corner (x, y) is the image point (x - 0.5, y - 0.5), so pixel (u, v) has the corners
  // (u, v), (u + 1, v), (u, v + 1) and (u + 1, v + 1).
  // @param x corner column, 0 .. width.
  // @param y corner row, 0 .. height.
  // @return the normalised (x, y) of the corner.
  const cv::Vec2d & corner(int x, int y) const { return corners_(y, x); }

  // @return every centre ray, height x width.
  const cv::Mat_<cv::Vec2d> & centres() const { return centres_; }
  // @return every corner ray, (height + 1) x (width + 1).
  const cv::Mat_<cv::Vec2d> & corners() const { return corners_; }

private:
  PinholeModel model_;
  cv::Mat_<cv::Vec2d> centres_;
  cv::Mat_<cv::Vec2d> corners_;
};

// The integer pixel rectangle covered by a footprint [u_min, u_max] x [v_min, v_max] (continuous pixel coordinates):
// every pixel whose centre lies inside it, clipped to the image. Footprints of neighbouring pixels share their
// corners, so the covered pixels tile the projected surface with no gap. Empty (cv::Rect()) when the footprint is
// fully outside the image on any side (including u_max < 0 and u_min >= width), covers no pixel centre, or has a NaN
// bound; depth_image_proc's fill_upsampling_holes misses the u_max < 0 / u_min >= width cases.
// @param u_min left bound, pixels.
// @param v_min top bound, pixels.
// @param u_max right bound, pixels.
// @param v_max bottom bound, pixels.
// @param width image width, pixels.
// @param height image height, pixels.
// @return the clipped rectangle, inside [0, width) x [0, height), or an empty one.
cv::Rect clip_footprint(double u_min, double v_min, double u_max, double v_max, int width, int height);

// Register a ToF depth image into an RGB camera, writing into a preallocated image (which may be a view into a larger
// Mat: nothing outside it is written). Every ToF pixel with a depth z inside [range.min_m, range.max_m] becomes the
// point ray * z, is transformed with T_rgb_tof and skipped when it or a pixel corner lands at z <= 0 or a corner lies
// beyond the RGB fold radius; its four corner points (same depth) are projected into rgb_out with plumb_bob
// distortion and the clipped bounding rectangle of the projected corners is filled with the transformed point's z
// (mm, rounded), keeping the minimum per output pixel. Pixels where no point landed are 0.
// @param tof_depth_mm the ToF depth, CV_16UC1, rays.width() x rays.height(), mm, 0 invalid.
// @param rays the ray table of the ToF model.
// @param T_rgb_tof the transform from the ToF optical frame to the RGB optical frame (p_rgb = T_rgb_tof * p_tof).
// @param rgb_out the RGB model at the output size.
// @param range the valid depth range of the ToF depth.
// @param out the output, CV_16UC1, rgb_out.width x rgb_out.height; overwritten.
// @param stats optional counters of the call.
// @throws std::invalid_argument on a wrong type or size of tof_depth_mm or out.
void register_depth_into(
  const cv::Mat & tof_depth_mm, const RayTable & rays, const Eigen::Isometry3d & T_rgb_tof,
  const PinholeModel & rgb_out, const DepthConfig & range, cv::Mat & out, RegistrationStats * stats = nullptr);

// register_depth_into a newly allocated output.
// @param tof_depth_mm the ToF depth, CV_16UC1, rays.width() x rays.height(), mm, 0 invalid.
// @param rays the ray table of the ToF model.
// @param T_rgb_tof the transform from the ToF optical frame to the RGB optical frame.
// @param rgb_out the RGB model at the output size.
// @param range the valid depth range of the ToF depth.
// @param stats optional counters of the call.
// @return the registered depth, CV_16UC1, rgb_out.width x rgb_out.height, z in the RGB optical frame in mm, 0 empty.
// @throws std::invalid_argument on a wrong type or size of tof_depth_mm.
cv::Mat register_depth(
  const cv::Mat & tof_depth_mm, const RayTable & rays, const Eigen::Isometry3d & T_rgb_tof,
  const PinholeModel & rgb_out, const DepthConfig & range, RegistrationStats * stats = nullptr);

}  // namespace head_rgbd_cpp

#endif  // HEAD_RGBD_CPP__REGISTRATION_HPP_
