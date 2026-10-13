// Obstacle signed distances, the capsule axis search and rigid placement (see body_geometry.hpp).
#include "collision/body_geometry.hpp"
#include "collision/fp_contract.hpp"

#include <algorithm>
#include <cmath>

namespace collision {

namespace {

const double GOLDEN_RATIO = (std::sqrt(5.0) - 1.0) / 2.0;

}  // namespace

Obstacle make_box(const Vec3 &lo, const Vec3 &hi) {
  Obstacle o;
  o.box = true;
  for (int i = 0; i < 3; ++i) {
    o.centre[i] = (lo[i] + hi[i]) / 2.0;
    o.half[i] = (hi[i] - lo[i]) / 2.0;
  }
  return o;
}

Obstacle make_cylinder(double cx, double cy, double radius, double z_min, double z_max) {
  Obstacle o;
  o.box = false;
  o.cx = cx;
  o.cy = cy;
  o.radius = radius;
  o.mid = (z_min + z_max) / 2.0;
  o.half_height = (z_max - z_min) / 2.0;
  return o;
}

double Obstacle::signed_distance(const Vec3 &p) const {
  if (box) {
    double q[3];
    for (int i = 0; i < 3; ++i) q[i] = std::fabs(p[i] - centre[i]) - half[i];
    const double m0 = std::max(q[0], 0.0), m1 = std::max(q[1], 0.0), m2 = std::max(q[2], 0.0);
    const double sq0 = m0 * m0, sq1 = m1 * m1, sq2 = m2 * m2;
    const double outside = std::sqrt((sq0 + sq1) + sq2);
    const double inside = std::min(std::max(std::max(q[0], q[1]), q[2]), 0.0);
    return outside + inside;
  }
  const double radial = std::hypot(p[0] - cx, p[1] - cy) - radius;
  const double vertical = std::fabs(p[2] - mid) - half_height;
  const double outside = std::hypot(std::max(radial, 0.0), std::max(vertical, 0.0));
  const double inside = std::min(std::max(radial, vertical), 0.0);
  return outside + inside;
}

double axis_distance(const Obstacle &obstacle, const Vec3 &a, const Vec3 &b) {
  const Vec3 d{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  auto at = [&](double t) {
    const double s0 = d[0] * t, s1 = d[1] * t, s2 = d[2] * t;
    return obstacle.signed_distance(Vec3{a[0] + s0, a[1] + s1, a[2] + s2});
  };
  double lo = 0.0, hi = 1.0;
  double x1 = hi - GOLDEN_RATIO * (hi - lo);
  double x2 = lo + GOLDEN_RATIO * (hi - lo);
  double f1 = at(x1), f2 = at(x2);
  for (int i = 0; i < GOLDEN_ITERATIONS; ++i) {
    if (f1 < f2) {  // the minimum lies in [lo, x2]
      hi = x2;
      x2 = x1;
      x1 = hi - GOLDEN_RATIO * (hi - lo);
      f2 = f1;
      f1 = at(x1);
    } else {
      lo = x1;
      x1 = x2;
      x2 = lo + GOLDEN_RATIO * (hi - lo);
      f1 = f2;
      f2 = at(x2);
    }
  }
  return std::min(std::min(f1, f2), std::min(at(0.0), at(1.0)));
}

Rigid make_rigid(const double *rotation, const double *translation) {
  Rigid out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) out.r[i][j] = rotation[i * 3 + j];
    out.t[i] = translation[i];
  }
  return out;
}

Vec3 Rigid::apply(const Vec3 &p) const {
  Vec3 out;
  for (int i = 0; i < 3; ++i) {
    const double a = p[0] * r[i][0], b = p[1] * r[i][1], c = p[2] * r[i][2];
    out[i] = ((a + b) + c) + t[i];
  }
  return out;
}

}  // namespace collision
