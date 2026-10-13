// Robot body obstacles and rigid placement for the arm self-collision kernels (port of mcp_server/body_model.py and
// self_collision.rigid). Every expression keeps the operation order of the numpy original, and the .cpp files of this
// family turn off floating-point contraction (no fused multiply-add), so a value gets the same bits as in numpy.
#pragma once

#include <array>

namespace collision {

using Vec3 = std::array<double, 3>;

// Golden-section search of a capsule axis (body_model.GOLDEN_RATIO, GOLDEN_ITERATIONS).
constexpr int GOLDEN_ITERATIONS = 48;

// A convex body obstacle in base_link: an axis-aligned box (BoxObstacle) or a vertical cylinder (CylinderObstacle).
struct Obstacle {
  bool box = true;
  Vec3 centre{};  // box centre (lo + hi) / 2
  Vec3 half{};    // box half size (hi - lo) / 2
  double cx = 0.0, cy = 0.0, radius = 0.0;
  double mid = 0.0, half_height = 0.0;  // cylinder (z_min + z_max) / 2 and (z_max - z_min) / 2

  // Exact signed distance of a point (m, negative inside).
  double signed_distance(const Vec3 &p) const;
};

Obstacle make_box(const Vec3 &lo, const Vec3 &hi);
Obstacle make_cylinder(double cx, double cy, double radius, double z_min, double z_max);

// body_model.capsule_signed_distance with radius 0: the smallest signed distance along the axis a -> b.
double axis_distance(const Obstacle &obstacle, const Vec3 &a, const Vec3 &b);

// A rigid transform r @ p + t with r row-major (self_collision.rigid: element by element, left to right).
struct Rigid {
  double r[3][3];
  Vec3 t;

  Vec3 apply(const Vec3 &p) const;
};

Rigid make_rigid(const double *rotation, const double *translation);

}  // namespace collision
