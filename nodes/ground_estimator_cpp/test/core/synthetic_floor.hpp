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
#include <cstddef>
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

// Wedge ramp standing on the z = 0 floor (part ramp-synthetic-scene): an incline rising along +x from the floor at
// x0 to height_m at x0 + length_m over y in [y0, y1], a vertical back face at x0 + length_m and triangular side
// faces at y0 and y1. Use alone or on floor_scene() (the ramp hides the floor under it).
// @param scene the scene to add the surfaces to.
// @param x0 x of the foot of the incline in metres.
// @param length_m horizontal length of the incline along x in metres (> 0).
// @param height_m height of the top edge in metres (> 0); the slope is atan(height_m / length_m).
// @param y0 lower y of the ramp in metres.
// @param y1 upper y of the ramp in metres (> y0).
inline void add_ramp(Scene & scene, double x0, double length_m, double height_m, double y0, double y1)
{
  const double x1 = x0 + length_m;
  const Region under_incline = [=](const Eigen::Vector3d & p) {
    return p.x() >= x0 && p.x() <= x1 && p.z() >= 0.0 && p.z() <= (p.x() - x0) / length_m * height_m;
  };
  scene.surfaces.push_back(plane_surface(
    Eigen::Vector3d(x0, 0.0, 0.0), Eigen::Vector3d(-height_m, 0.0, length_m),
    [=](const Eigen::Vector3d & p) { return p.x() >= x0 && p.x() <= x1 && p.y() >= y0 && p.y() <= y1; }));
  scene.surfaces.push_back(plane_surface(
    Eigen::Vector3d(x1, 0.0, 0.0), Eigen::Vector3d::UnitX(),
    [=](const Eigen::Vector3d & p) { return p.y() >= y0 && p.y() <= y1 && p.z() >= 0.0 && p.z() <= height_m; }));
  for (const double y_side : {y0, y1}) {
    scene.surfaces.push_back(plane_surface(Eigen::Vector3d(0.0, y_side, 0.0), Eigen::Vector3d::UnitY(), under_incline));
  }
}

// Sample the scene on a regular xy grid from straight above: one vertical ray per grid node over [x0, x1] x [y0, y1]
// (offset by a quarter step like grid_cloud), keeping the highest hit. Nodes over nothing give no point; noise-free.
// @param scene the surfaces.
// @param x0 lower x in metres.
// @param x1 upper x in metres.
// @param y0 lower y in metres.
// @param y1 upper y in metres.
// @param step_m grid step in metres.
// @return the hits in base_link.
inline pcl::PointCloud<pcl::PointXYZ> sample_grid(
  const Scene & scene, double x0, double x1, double y0, double y1, double step_m)
{
  constexpr double RAY_START_Z_M = 10.0;
  const Eigen::Vector3d down = -Eigen::Vector3d::UnitZ();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (double x = x0 + 0.25 * step_m; x < x1; x += step_m) {
    for (double y = y0 + 0.25 * step_m; y < y1; y += step_m) {
      const Eigen::Vector3d origin(x, y, RAY_START_Z_M);
      double best = std::numeric_limits<double>::infinity();
      for (const auto & surface : scene.surfaces) {
        const auto t = surface(origin, down);
        if (t && *t < best) {
          best = *t;
        }
      }
      if (std::isfinite(best)) {
        const Eigen::Vector3d p = origin + best * down;
        cloud.push_back(pcl::PointXYZ(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
      }
    }
  }
  return cloud;
}

// Seeded sparse sampling: keep each point independently with probability fraction (sparse ToF returns).
// @param in the cloud.
// @param fraction probability in 0..1 that a point is kept.
// @param seed sampling seed; the same seed keeps the same points.
// @return the kept points, in input order.
inline pcl::PointCloud<pcl::PointXYZ> keep_fraction(
  const pcl::PointCloud<pcl::PointXYZ> & in, double fraction, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> pick(0.0, 1.0);
  pcl::PointCloud<pcl::PointXYZ> out;
  for (const auto & p : in) {
    if (pick(rng) < fraction) {
      out.push_back(p);
    }
  }
  return out;
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

// Regular grid of points at height z_m over [x0, x1] x [y0, y1] (offset by a quarter step so no point sits on a
// round-number edge).
inline pcl::PointCloud<pcl::PointXYZ> grid_cloud(
  double x0, double x1, double y0, double y1, double step_m, double z_m = 0.0)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (double x = x0 + 0.25 * step_m; x < x1; x += step_m) {
    for (double y = y0 + 0.25 * step_m; y < y1; y += step_m) {
      cloud.push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z_m)));
    }
  }
  return cloud;
}

// Seeded noise on z: Gaussian sigma per point, plus a fraction of points replaced by a uniform offset in
// [-outlier_range_m, +outlier_range_m] (instead of the Gaussian one).
inline pcl::PointCloud<pcl::PointXYZ> add_z_noise(
  const pcl::PointCloud<pcl::PointXYZ> & in, double sigma_m, double outlier_fraction, double outlier_range_m,
  unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, sigma_m);
  std::uniform_real_distribution<double> uniform(-outlier_range_m, outlier_range_m);
  std::uniform_real_distribution<double> pick(0.0, 1.0);
  pcl::PointCloud<pcl::PointXYZ> out = in;
  for (auto & p : out) {
    p.z += static_cast<float>(pick(rng) < outlier_fraction ? uniform(rng) : gauss(rng));
  }
  return out;
}

// Mixed-pixel tail of a box (part mixed-pixel-scene): a ToF pixel straddling a depth edge averages the near edge and
// the far floor, so its point lands on the ray from the sensor through the box's far top edge, between the edge and
// the floor behind. Adds count points (seeded): each picks a point on the segment edge_a..edge_b, then a ray
// parameter uniform between start_fraction of the way from the edge to the floor plane z = floor_z_m and the floor,
// and appends the point to cloud. The first start_fraction of the way is left out: a pixel that close to the edge
// reads as the edge itself (a height within the sensor noise of the box top), not as a tail.
// Returns the index of the first appended point so callers can tell tail points from the rest.
inline std::size_t add_mixed_pixel_tail(
  pcl::PointCloud<pcl::PointXYZ> & cloud, const Eigen::Vector3d & sensor_origin, const Eigen::Vector3d & edge_a,
  const Eigen::Vector3d & edge_b, double floor_z_m, int count, unsigned seed, double start_fraction = 0.3)
{
  const std::size_t first = cloud.size();
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  for (int i = 0; i < count; ++i) {
    const Eigen::Vector3d edge = edge_a + unit(rng) * (edge_b - edge_a);
    const Eigen::Vector3d ray = edge - sensor_origin;
    if (ray.z() >= 0.0 || edge.z() <= floor_z_m) {
      continue;
    }
    const double t_floor = (floor_z_m - sensor_origin.z()) / ray.z();  // ray parameter of the floor behind the edge
    const double t = 1.0 + (start_fraction + unit(rng) * (1.0 - start_fraction)) * (t_floor - 1.0);  // 1.0 = edge
    const Eigen::Vector3d p = sensor_origin + t * ray;
    cloud.push_back(pcl::PointXYZ(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
  }
  return first;
}

}  // namespace ground_test

#endif  // GROUND_ESTIMATOR_CPP__TEST__SYNTHETIC_FLOOR_HPP_
