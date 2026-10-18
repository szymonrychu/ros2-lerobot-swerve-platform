// Synthetic scenes for the registration, motion and upsampling tests: infinite planes and axis-aligned boxes in a world
// frame, ray-cast into any PinholeModel at any camera pose. Renders the ToF depth (CV_16UC1 mm, as the bridge
// publishes it), the ground-truth depth seen by the RGB camera (CV_64FC1 metres), the index of the primitive hit per
// pixel and a grey guide image (per-primitive albedo), plus the default head models and the rig helpers (RGB = ToF
// identity, RGB offset from the ToF).
//
// Conventions: poses are T_world_cam (camera optical frame -> world; optical frame x right, y down, z forward). The
// world frame defaults to the ToF optical frame, so scenes are written in ToF coordinates.
#ifndef HEAD_RGBD_CPP__TEST__CORE__SYNTHETIC_SCENE_HPP_
#define HEAD_RGBD_CPP__TEST__CORE__SYNTHETIC_SCENE_HPP_

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

#include "head_rgbd_cpp/registration.hpp"

namespace head_rgbd_cpp::test
{

// Default head ToF: Arducam ToF, 240x180, fx about 191 (design D7).
constexpr int TOF_WIDTH = 240;
constexpr int TOF_HEIGHT = 180;
constexpr double TOF_F = 191.0;
// Default RGB mapping stream: Camera Module 3 at 576x324, fx about 440; the default output (divisor 2) is 288x162.
constexpr int RGB_MAPPING_WIDTH = 576;
constexpr int RGB_MAPPING_HEIGHT = 324;
constexpr double RGB_MAPPING_F = 440.0;
constexpr int RGB_OUT_WIDTH = 288;
constexpr int RGB_OUT_HEIGHT = 162;
// A ray hit closer than this (metres along the ray direction scaled to z = 1) is ignored.
constexpr double MIN_HIT_T = 1e-9;
// No primitive was hit.
constexpr int NO_HIT = -1;

// An infinite plane n . x = n . point, in the world frame.
struct Plane
{
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  double albedo = 0.5;  // grey level of render_grey, [0, 1]
};

// An axis-aligned box [min, max] in the world frame.
struct Box
{
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  double albedo = 0.8;  // grey level of render_grey, [0, 1]
};

// Planes and boxes. Primitive indices: planes first (0 .. planes.size() - 1), then boxes.
struct Scene
{
  std::vector<Plane> planes;
  std::vector<Box> boxes;
};

// A ray-cast image: z in the camera optical frame per pixel and the primitive hit.
struct Render
{
  cv::Mat depth_m;  // CV_64FC1, metres, 0 where nothing was hit
  cv::Mat index;    // CV_32SC1, primitive index, NO_HIT where nothing was hit
};

// A ToF-RGB camera pair: models, poses in the world and the registration transform between them.
struct Rig
{
  PinholeModel tof;
  PinholeModel rgb;               // at the output size
  Eigen::Isometry3d T_world_tof;  // ToF pose
  Eigen::Isometry3d T_world_rgb;  // RGB pose
  // @return p_rgb = T_rgb_tof() * p_tof.
  Eigen::Isometry3d T_rgb_tof() const { return T_world_rgb.inverse() * T_world_tof; }
};

// A distortion-free model with the principal point at the image centre.
// @param f focal length fx = fy, pixels.
// @param width image width, pixels.
// @param height image height, pixels.
// @return the model.
inline PinholeModel make_model(double f, int width, int height)
{
  PinholeModel m;
  m.K = cv::Matx33d(f, 0.0, (width - 1) / 2.0, 0.0, f, (height - 1) / 2.0, 0.0, 0.0, 1.0);
  m.D = cv::Vec<double, 5>(0.0, 0.0, 0.0, 0.0, 0.0);
  m.width = width;
  m.height = height;
  return m;
}

// @return the default ToF model, 240x180, f 191, no distortion.
inline PinholeModel default_tof_model() { return make_model(TOF_F, TOF_WIDTH, TOF_HEIGHT); }

// @return the default RGB mapping model, 576x324, f 440, no distortion.
inline PinholeModel default_rgb_mapping_model()
{
  return make_model(RGB_MAPPING_F, RGB_MAPPING_WIDTH, RGB_MAPPING_HEIGHT);
}

// @return the default RGB output model: the mapping model scaled to 288x162 (f 220).
inline PinholeModel default_rgb_out_model()
{
  return scale_model(default_rgb_mapping_model(), RGB_OUT_WIDTH, RGB_OUT_HEIGHT);
}

// A pose translated by (x, y, z) metres with no rotation.
// @param x, y, z the translation, metres.
// @return the pose.
inline Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

// RGB = ToF: the same model at the same pose, so registration is the identity.
// @param m the model of both cameras.
// @return the rig.
inline Rig identity_rig(const PinholeModel & m)
{
  return Rig{m, m, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
}

// The ToF at the world origin and the RGB at T_tof_rgb (its pose in the ToF optical frame).
// @param tof the ToF model.
// @param rgb the RGB model at the output size.
// @param T_tof_rgb the RGB pose in the ToF optical frame, e.g. translation(0.05, 0, 0) for an RGB 5 cm right.
// @return the rig.
inline Rig offset_rig(const PinholeModel & tof, const PinholeModel & rgb, const Eigen::Isometry3d & T_tof_rgb)
{
  return Rig{tof, rgb, Eigen::Isometry3d::Identity(), T_tof_rgb};
}

// A plane facing the camera (normal -z) at depth z in the world frame.
// @param z depth, metres.
// @param albedo grey level, [0, 1].
// @return the plane.
inline Plane fronto_parallel_plane(double z, double albedo = 0.5)
{
  return Plane{Eigen::Vector3d(0.0, 0.0, z), Eigen::Vector3d(0.0, 0.0, -1.0), albedo};
}

// A plane through (0, 0, z) tilted by angle_deg about the world y axis.
// @param z depth on the optical axis, metres.
// @param angle_deg tilt, degrees (0 is fronto-parallel).
// @param albedo grey level, [0, 1].
// @return the plane.
inline Plane slanted_plane(double z, double angle_deg, double albedo = 0.5)
{
  const double a = angle_deg * CV_PI / 180.0;
  return Plane{Eigen::Vector3d(0.0, 0.0, z), Eigen::Vector3d(std::sin(a), 0.0, -std::cos(a)), albedo};
}

// The ray parameter t of the nearest hit of a ray with a plane (point = origin + t * dir), or infinity.
inline double intersect(const Plane & plane, const Eigen::Vector3d & origin, const Eigen::Vector3d & dir)
{
  const double denom = plane.normal.dot(dir);
  if (std::abs(denom) < 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  const double t = plane.normal.dot(plane.point - origin) / denom;
  return t > MIN_HIT_T ? t : std::numeric_limits<double>::infinity();
}

// The ray parameter t of the nearest hit of a ray with a box from outside it (slab method), or infinity.
inline double intersect(const Box & box, const Eigen::Vector3d & origin, const Eigen::Vector3d & dir)
{
  double t_near = -std::numeric_limits<double>::infinity();
  double t_far = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(dir[axis]) < 1e-12) {
      if (origin[axis] < box.min[axis] || origin[axis] > box.max[axis]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double t0 = (box.min[axis] - origin[axis]) / dir[axis];
    double t1 = (box.max[axis] - origin[axis]) / dir[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_near = std::max(t_near, t0);
    t_far = std::min(t_far, t1);
  }
  if (t_near > t_far || t_near <= MIN_HIT_T) {
    return std::numeric_limits<double>::infinity();
  }
  return t_near;
}

// The nearest hit of a ray in the scene.
// @param scene the scene.
// @param origin the ray origin, world frame.
// @param dir the ray direction, world frame (any length; t is in units of it).
// @return (t, primitive index), (infinity, NO_HIT) when nothing is hit.
inline std::pair<double, int> cast(const Scene & scene, const Eigen::Vector3d & origin, const Eigen::Vector3d & dir)
{
  double best_t = std::numeric_limits<double>::infinity();
  int best = NO_HIT;
  int index = 0;
  for (const auto & plane : scene.planes) {
    const double t = intersect(plane, origin, dir);
    if (t < best_t) {
      best_t = t;
      best = index;
    }
    ++index;
  }
  for (const auto & box : scene.boxes) {
    const double t = intersect(box, origin, dir);
    if (t < best_t) {
      best_t = t;
      best = index;
    }
    ++index;
  }
  return {best_t, best};
}

// Ray-cast the scene into every pixel centre of a camera (rays undistorted with the model's distortion).
// @param scene the scene.
// @param m the camera model.
// @param T_world_cam the camera pose.
// @return z in the camera optical frame and the primitive index per pixel.
inline Render render(const Scene & scene, const PinholeModel & m, const Eigen::Isometry3d & T_world_cam)
{
  const RayTable rays(m);
  Render out{
    cv::Mat(m.height, m.width, CV_64FC1, cv::Scalar(0.0)), cv::Mat(m.height, m.width, CV_32SC1, cv::Scalar(NO_HIT))};
  const Eigen::Vector3d origin = T_world_cam.translation();
  for (int v = 0; v < m.height; ++v) {
    for (int u = 0; u < m.width; ++u) {
      const cv::Vec2d & r = rays.ray(u, v);
      // The camera ray (x, y, 1): t is the z of the hit in the camera frame.
      const Eigen::Vector3d dir = T_world_cam.linear() * Eigen::Vector3d(r[0], r[1], 1.0);
      const auto [t, index] = cast(scene, origin, dir);
      if (index != NO_HIT) {
        out.depth_m.at<double>(v, u) = t;
        out.index.at<int>(v, u) = index;
      }
    }
  }
  return out;
}

// A depth in metres as the bridge publishes it: CV_16UC1 mm, rounded, 0 where nothing was hit or outside the range.
// @param depth_m CV_64FC1 metres, 0 no hit.
// @param min_m, max_m the range kept, metres.
// @return the depth in mm.
inline cv::Mat to_depth_mm(const cv::Mat & depth_m, double min_m = 0.0, double max_m = 65.535)
{
  cv::Mat out(depth_m.size(), CV_16UC1, cv::Scalar(0));
  for (int v = 0; v < depth_m.rows; ++v) {
    for (int u = 0; u < depth_m.cols; ++u) {
      const double z = depth_m.at<double>(v, u);
      if (z > 0.0 && z >= min_m && z <= max_m) {
        out.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(z * 1000.0));
      }
    }
  }
  return out;
}

// The ToF depth image of a scene: CV_16UC1 mm, 0 where nothing was hit or the depth is above 65.535 m.
// @param scene the scene.
// @param tof the ToF model.
// @param T_world_tof the ToF pose.
// @return the depth.
inline cv::Mat render_tof_depth_mm(const Scene & scene, const PinholeModel & tof, const Eigen::Isometry3d & T_world_tof)
{
  return to_depth_mm(render(scene, tof, T_world_tof).depth_m);
}

// The ground-truth depth an RGB camera sees: CV_64FC1 z in the RGB optical frame, metres, 0 where nothing was hit.
// @param scene the scene.
// @param rgb the RGB model (at the output size).
// @param T_world_rgb the RGB pose.
// @return the depth.
inline cv::Mat render_rgb_truth_m(const Scene & scene, const PinholeModel & rgb, const Eigen::Isometry3d & T_world_rgb)
{
  return render(scene, rgb, T_world_rgb).depth_m;
}

// A grey guide image: the albedo of the primitive hit per pixel, CV_32FC1 in [0, 1], 0 where nothing was hit.
// @param scene the scene.
// @param m the camera model.
// @param T_world_cam the camera pose.
// @return the image.
inline cv::Mat render_grey(const Scene & scene, const PinholeModel & m, const Eigen::Isometry3d & T_world_cam)
{
  const Render r = render(scene, m, T_world_cam);
  cv::Mat grey(m.height, m.width, CV_32FC1, cv::Scalar(0.0F));
  const auto n_planes = static_cast<int>(scene.planes.size());
  for (int v = 0; v < m.height; ++v) {
    for (int u = 0; u < m.width; ++u) {
      const int index = r.index.at<int>(v, u);
      if (index == NO_HIT) {
        continue;
      }
      const double albedo = index < n_planes ? scene.planes[static_cast<size_t>(index)].albedo
                                             : scene.boxes[static_cast<size_t>(index - n_planes)].albedo;
      grey.at<float>(v, u) = static_cast<float>(albedo);
    }
  }
  return grey;
}

// The world point a camera pixel's ray hits, with the hit primitive index.
// @param scene the scene.
// @param rays the ray table of the camera model (built once by the caller).
// @param T_world_cam the camera pose.
// @param u, v the pixel.
// @return (world point, index); index NO_HIT when nothing is hit.
inline std::pair<Eigen::Vector3d, int> hit_point(
  const Scene & scene, const RayTable & rays, const Eigen::Isometry3d & T_world_cam, int u, int v)
{
  const cv::Vec2d & r = rays.ray(u, v);
  const Eigen::Vector3d dir = T_world_cam.linear() * Eigen::Vector3d(r[0], r[1], 1.0);
  const auto [t, index] = cast(scene, T_world_cam.translation(), dir);
  if (index == NO_HIT) {
    return {Eigen::Vector3d::Zero(), NO_HIT};
  }
  return {T_world_cam.translation() + t * dir, index};
}

// True when a world point is the first thing a camera centre sees along the line to it (nothing occludes it).
// @param scene the scene.
// @param cam_origin the camera centre, world frame.
// @param point the point, world frame.
// @param tolerance_m how much nearer than the point an occluder must be, metres.
// @return whether the point is visible.
inline bool visible(
  const Scene & scene, const Eigen::Vector3d & cam_origin, const Eigen::Vector3d & point, double tolerance_m = 1e-6)
{
  const Eigen::Vector3d dir = point - cam_origin;
  const double length = dir.norm();
  const auto hit = cast(scene, cam_origin, dir / length);
  return hit.second == NO_HIT || hit.first >= length - tolerance_m;
}

// True when a world point lies in front of a camera and projects inside its image (within the pixel area
// [-0.5, width - 0.5) x [-0.5, height - 0.5)).
// @param m the camera model.
// @param T_world_cam the camera pose.
// @param point the point, world frame.
// @return whether the camera images the point.
inline bool in_view(const PinholeModel & m, const Eigen::Isometry3d & T_world_cam, const Eigen::Vector3d & point)
{
  const Eigen::Vector3d p = T_world_cam.inverse() * point;
  if (p.z() <= 0.0) {
    return false;
  }
  const cv::Point2d px = project_point(m, p);
  return px.x >= -0.5 && px.x < m.width - 0.5 && px.y >= -0.5 && px.y < m.height - 0.5;
}

}  // namespace head_rgbd_cpp::test

#endif  // HEAD_RGBD_CPP__TEST__CORE__SYNTHETIC_SCENE_HPP_
