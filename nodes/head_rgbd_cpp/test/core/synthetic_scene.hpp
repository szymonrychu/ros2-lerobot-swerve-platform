// Synthetic scenes for the upsampling tests: infinite planes and axis-aligned boxes in a world frame, ray-cast into a
// distortion-free pinhole camera at any pose. Renders the ground-truth depth a camera sees (CV_64FC1 metres), the same
// depth as CV_16UC1 mm, the index of the primitive hit per pixel and a grey guide image (per-primitive albedo), plus
// the default RGB mapping and output models.
//
// Conventions: poses are T_world_cam (camera optical frame -> world; optical frame x right, y down, z forward).
#ifndef HEAD_RGBD_CPP__TEST__CORE__SYNTHETIC_SCENE_HPP_
#define HEAD_RGBD_CPP__TEST__CORE__SYNTHETIC_SCENE_HPP_

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

namespace head_rgbd_cpp::test
{

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

// A distortion-free pinhole camera: focal lengths and principal point in pixels, image size in pixels.
struct PinholeModel
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int width = 0;
  int height = 0;
};

// A distortion-free model with the principal point at the image centre.
// @param f focal length fx = fy, pixels.
// @param width image width, pixels.
// @param height image height, pixels.
// @return the model.
inline PinholeModel make_model(double f, int width, int height)
{
  return PinholeModel{f, f, (width - 1) / 2.0, (height - 1) / 2.0, width, height};
}

// The model of the same camera at another image size, with the pixel-centre convention (c' = (c + 0.5) * s - 0.5).
// @param m the model.
// @param width, height the new image size, pixels.
// @return the scaled model.
inline PinholeModel scale_model(const PinholeModel & m, int width, int height)
{
  const double sx = static_cast<double>(width) / m.width;
  const double sy = static_cast<double>(height) / m.height;
  return PinholeModel{m.fx * sx, m.fy * sy, (m.cx + 0.5) * sx - 0.5, (m.cy + 0.5) * sy - 0.5, width, height};
}

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

// Ray-cast the scene into every pixel centre of a camera.
// @param scene the scene.
// @param m the camera model.
// @param T_world_cam the camera pose.
// @return z in the camera optical frame and the primitive index per pixel.
inline Render render(const Scene & scene, const PinholeModel & m, const Eigen::Isometry3d & T_world_cam)
{
  Render out{
    cv::Mat(m.height, m.width, CV_64FC1, cv::Scalar(0.0)), cv::Mat(m.height, m.width, CV_32SC1, cv::Scalar(NO_HIT))};
  const Eigen::Vector3d origin = T_world_cam.translation();
  for (int v = 0; v < m.height; ++v) {
    for (int u = 0; u < m.width; ++u) {
      // The camera ray (x, y, 1): t is the z of the hit in the camera frame.
      const Eigen::Vector3d ray((u - m.cx) / m.fx, (v - m.cy) / m.fy, 1.0);
      const auto [t, index] = cast(scene, origin, T_world_cam.linear() * ray);
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

}  // namespace head_rgbd_cpp::test

#endif  // HEAD_RGBD_CPP__TEST__CORE__SYNTHETIC_SCENE_HPP_
