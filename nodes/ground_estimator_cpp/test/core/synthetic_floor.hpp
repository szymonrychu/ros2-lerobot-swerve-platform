// Deterministic synthetic ToF scenes in base_link for the ground estimator tests (part ground-synthetic-scene).
// A Scene is a set of surfaces; render() casts the rays of an image-like grid (240x180 sensor footprint subsampled
// by a decimation) from a sensor origin, keeps the nearest hit per ray, adds Gaussian noise and a uniform depth-like
// bias along the ray, and returns the cloud. Same arguments and seed give the same cloud.
#ifndef GROUND_ESTIMATOR_CPP__TEST__SYNTHETIC_FLOOR_HPP_
#define GROUND_ESTIMATOR_CPP__TEST__SYNTHETIC_FLOOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <vector>

namespace ground_test
{

constexpr int SENSOR_WIDTH = 240;
constexpr int SENSOR_HEIGHT = 180;
constexpr int DEFAULT_DECIMATION = 4;
constexpr double DEG = M_PI / 180.0;
constexpr double DEFAULT_SIGMA_M = 0.003;

// Where the rays start and how they fan out (head ToF: forward, looking down).
struct Sensor
{
  Eigen::Vector3d origin{0.15, 0.0, 0.45};
  double centre_down_deg = 30.0;  // pitch of the optical axis below the horizon
  double h_fov_deg = 70.0;
  double v_fov_deg = 50.0;
};

// A surface returns the ray parameter t > 0 of its hit, or nullopt.
using Surface = std::function<std::optional<double>(const Eigen::Vector3d &, const Eigen::Vector3d &)>;
using Region = std::function<bool(const Eigen::Vector3d &)>;

struct Scene
{
  std::vector<Surface> surfaces;
};

// Unit up normal of a floor tilted by roll about x then pitch about y (radians in the arguments of the callers).
inline Eigen::Vector3d tilted_normal(double roll_deg, double pitch_deg)
{
  const Eigen::Matrix3d r = (Eigen::AngleAxisd(roll_deg * DEG, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(pitch_deg * DEG, Eigen::Vector3d::UnitY()))
                              .toRotationMatrix();
  return r * Eigen::Vector3d::UnitZ();
}

// Infinite plane through a point, optionally limited to a region of the plane.
inline Surface plane_surface(const Eigen::Vector3d & point, const Eigen::Vector3d & normal, Region region = nullptr)
{
  const Eigen::Vector3d n = normal.normalized();
  return [point, n, region](const Eigen::Vector3d & o, const Eigen::Vector3d & dir) -> std::optional<double> {
    const double denom = n.dot(dir);
    if (std::fabs(denom) < 1e-9) {
      return std::nullopt;
    }
    const double t = n.dot(point - o) / denom;
    if (t <= 1e-6) {
      return std::nullopt;
    }
    if (region && !region(o + t * dir)) {
      return std::nullopt;
    }
    return t;
  };
}

// Axis-aligned box (slab test).
inline Surface box_surface(const Eigen::Vector3d & centre, const Eigen::Vector3d & size)
{
  const Eigen::Vector3d lo = centre - 0.5 * size;
  const Eigen::Vector3d hi = centre + 0.5 * size;
  return [lo, hi](const Eigen::Vector3d & o, const Eigen::Vector3d & dir) -> std::optional<double> {
    double t_near = 1e-6;
    double t_far = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) {
      if (std::fabs(dir[i]) < 1e-12) {
        if (o[i] < lo[i] || o[i] > hi[i]) {
          return std::nullopt;
        }
        continue;
      }
      double t0 = (lo[i] - o[i]) / dir[i];
      double t1 = (hi[i] - o[i]) / dir[i];
      if (t0 > t1) {
        std::swap(t0, t1);
      }
      t_near = std::max(t_near, t0);
      t_far = std::min(t_far, t1);
      if (t_near > t_far) {
        return std::nullopt;
      }
    }
    return t_near;
  };
}

// Floor tilted by (roll, pitch) in degrees and passing through (0, 0, height).
inline Scene floor_scene(double roll_deg = 0.0, double pitch_deg = 0.0, double height_m = 0.0)
{
  Scene scene;
  scene.surfaces.push_back(plane_surface(Eigen::Vector3d(0.0, 0.0, height_m), tilted_normal(roll_deg, pitch_deg)));
  return scene;
}

// Floor limited to x < edge_x (use with add_step_up or add_drop_off, which add the part beyond the edge).
inline Scene floor_until(double edge_x, double height_m = 0.0)
{
  Scene scene;
  scene.surfaces.push_back(plane_surface(
    Eigen::Vector3d(0.0, 0.0, height_m), Eigen::Vector3d::UnitZ(),
    [edge_x](const Eigen::Vector3d & p) { return p.x() < edge_x; }));
  return scene;
}

// Obstacle box of the given centre and size.
inline void add_box(Scene & scene, const Eigen::Vector3d & centre, const Eigen::Vector3d & size)
{
  scene.surfaces.push_back(box_surface(centre, size));
}

// Flat top at height_m over the half plane x >= edge_x, with the riser at the edge. Pair with floor_until(edge_x).
inline void add_step_up(Scene & scene, double edge_x, double height_m)
{
  scene.surfaces.push_back(plane_surface(
    Eigen::Vector3d(edge_x, 0.0, height_m), Eigen::Vector3d::UnitZ(),
    [edge_x](const Eigen::Vector3d & p) { return p.x() >= edge_x; }));
  scene.surfaces.push_back(plane_surface(
    Eigen::Vector3d(edge_x, 0.0, 0.0), -Eigen::Vector3d::UnitX(),
    [height_m](const Eigen::Vector3d & p) { return p.z() >= 0.0 && p.z() <= height_m; }));
}

// Lower floor (depth_m below the main one) beyond the edge line x >= edge_x. Pair with floor_until(edge_x).
inline void add_drop_off(Scene & scene, double edge_x, double depth_m)
{
  scene.surfaces.push_back(plane_surface(
    Eigen::Vector3d(edge_x, 0.0, -depth_m), Eigen::Vector3d::UnitZ(),
    [edge_x](const Eigen::Vector3d & p) { return p.x() >= edge_x; }));
}

// Vertical wall across the view at x = x_m.
inline void add_wall(Scene & scene, double x_m)
{
  scene.surfaces.push_back(plane_surface(Eigen::Vector3d(x_m, 0.0, 0.0), -Eigen::Vector3d::UnitX()));
}

// Horizontal table top at height_m over the rectangle [x0, x1] x [y0, y1].
inline void add_table_top(Scene & scene, double height_m, double x0, double x1, double y0, double y1)
{
  scene.surfaces.push_back(plane_surface(
    Eigen::Vector3d(0.0, 0.0, height_m), Eigen::Vector3d::UnitZ(),
    [=](const Eigen::Vector3d & p) { return p.x() >= x0 && p.x() <= x1 && p.y() >= y0 && p.y() <= y1; }));
}

// Cast the sensor grid on the scene.
// @param scene the surfaces.
// @param sigma_m Gaussian noise (per axis) in metres.
// @param bias_m shift of every point along its ray away from the sensor (depth-like bias, the ToF long bias).
// @param seed noise seed.
// @param decimation pixel step of the 240x180 footprint (4 gives 60x45 rays).
// @param sensor ray origin and field of view.
// @return the cloud in base_link (rays that hit nothing give no point).
inline pcl::PointCloud<pcl::PointXYZ> render(
  const Scene & scene, double sigma_m = DEFAULT_SIGMA_M, double bias_m = 0.0, unsigned seed = 1,
  int decimation = DEFAULT_DECIMATION, const Sensor & sensor = Sensor())
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, sigma_m);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int v = 0; v < SENSOR_HEIGHT; v += decimation) {
    for (int u = 0; u < SENSOR_WIDTH; u += decimation) {
      const double az = ((u + 0.5) / SENSOR_WIDTH - 0.5) * sensor.h_fov_deg * DEG;
      const double down = sensor.centre_down_deg * DEG + ((v + 0.5) / SENSOR_HEIGHT - 0.5) * sensor.v_fov_deg * DEG;
      const Eigen::Vector3d dir(std::cos(down) * std::cos(az), std::cos(down) * std::sin(az), -std::sin(down));
      double best = std::numeric_limits<double>::infinity();
      for (const auto & surface : scene.surfaces) {
        const auto t = surface(sensor.origin, dir);
        if (t && *t < best) {
          best = *t;
        }
      }
      if (!std::isfinite(best)) {
        continue;
      }
      Eigen::Vector3d p = sensor.origin + (best + bias_m) * dir;
      if (sigma_m > 0.0) {
        p += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
      }
      cloud.push_back(pcl::PointXYZ(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
    }
  }
  return cloud;
}

}  // namespace ground_test

#endif  // GROUND_ESTIMATOR_CPP__TEST__SYNTHETIC_FLOOR_HPP_
