// Body obstacles and the pruned clearance pass of the reach kernels (see body.hpp); every step mirrors
// mcp_server/body_model.py and SelfCollisionChecker.bounded_clearances, so a verdict or a reported clearance equals
// the Python one.
#if defined(__clang__)
// No fused multiply-add: the sums round like numpy's (GCC in ISO C++ mode does not contract either).
#pragma STDC FP_CONTRACT OFF
#endif

#include "ik_reach/body.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ik_reach {

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();
const double GOLDEN_RATIO = (std::sqrt(5.0) - 1.0) / 2.0;
constexpr int GOLDEN_ITERATIONS = 48;

// self_collision.rigid: rotation @ point + translation, summed element by element in the same order.
Vec3 rigid(const Mat4 &frame, const double *p) {
  Vec3 out{};
  for (int r = 0; r < 3; ++r) out[r] = p[0] * frame[r][0] + p[1] * frame[r][1] + p[2] * frame[r][2] + frame[r][3];
  return out;
}

}  // namespace

double Obstacle::signed_distance(const Vec3 &p) const {
  if (box) {
    double q[3];
    for (int i = 0; i < 3; ++i) q[i] = std::abs(p[i] - centre[i]) - half[i];
    double sq[3];
    for (int i = 0; i < 3; ++i) {
      const double m = std::max(q[i], 0.0);
      sq[i] = m * m;
    }
    const double outside = std::sqrt(sq[0] + (sq[1] + sq[2]));
    const double inside = std::min(std::max(std::max(q[0], q[1]), q[2]), 0.0);
    return outside + inside;
  }
  const double radial = std::hypot(p[0] - cx, p[1] - cy) - radius;
  const double mid = (z_min + z_max) / 2.0;
  const double vertical = std::abs(p[2] - mid) - (z_max - z_min) / 2.0;
  const double outside = std::hypot(std::max(radial, 0.0), std::max(vertical, 0.0));
  const double inside = std::min(std::max(radial, vertical), 0.0);
  return outside + inside;
}

double capsule_axis_distance(const Obstacle &obstacle, const Vec3 &a, const Vec3 &b) {
  const Vec3 d = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  auto at = [&](double t) { return obstacle.signed_distance({a[0] + d[0] * t, a[1] + d[1] * t, a[2] + d[2] * t}); };
  double lo = 0.0, hi = 1.0;
  double x1 = hi - GOLDEN_RATIO * (hi - lo);
  double x2 = lo + GOLDEN_RATIO * (hi - lo);
  double f1 = at(x1), f2 = at(x2);
  for (int it = 0; it < GOLDEN_ITERATIONS; ++it) {
    const bool left = f1 < f2;  // the minimum lies in [lo, x2]
    if (left)
      hi = x2;
    else
      lo = x1;
    const double x2_new = left ? x1 : lo + GOLDEN_RATIO * (hi - lo);
    const double x1_new = left ? hi - GOLDEN_RATIO * (hi - lo) : x2;
    const double f_new = at(left ? x1_new : x2_new);
    const double f1_new = left ? f_new : f2;
    const double f2_new = left ? f1 : f_new;
    f1 = f1_new;
    f2 = f2_new;
    x1 = x1_new;
    x2 = x2_new;
  }
  return std::min(std::min(f1, f2), std::min(at(0.0), at(1.0)));
}

BodyModel::BodyModel(BodyParams params) : params_(std::move(params)) {
  for (auto &o : params_.obstacles) {
    if (o.box) {
      for (int i = 0; i < 3; ++i) {
        o.centre[i] = (o.lo[i] + o.hi[i]) / 2.0;
        o.half[i] = (o.hi[i] - o.lo[i]) / 2.0;
      }
    }
  }
  for (const auto &g : params_.groups) {
    if (g.part < 0 || g.part >= CHECKED_PART_COUNT || g.blocks < 0 || g.size < 1 ||
        g.points.size() != static_cast<std::size_t>(g.blocks) * g.size * 3 ||
        g.centres.size() != static_cast<std::size_t>(g.blocks) * 3 || g.radii.size() != static_cast<std::size_t>(g.blocks))
      throw std::invalid_argument("BodyModel: inconsistent point block group");
  }
}

Vec3 BodyModel::to_base_link(const Vec3 &p) const {
  const Mat3 &r = params_.rotation;
  Vec3 out{};
  for (int i = 0; i < 3; ++i) out[i] = p[0] * r[i][0] + p[1] * r[i][1] + p[2] * r[i][2] + params_.offset[i];
  return out;
}

SampleClearance BodyModel::clearance(const ArmModel &arm, const Joints &measured, double hard_limit) const {
  const auto &obstacles = params_.obstacles;
  const int m = static_cast<int>(obstacles.size());
  const double hm = params_.hard_margin_m;
  Frames frames{};
  arm.forward_all(arm.urdf_vector(arm.to_urdf(measured)), frames);
  // Fixed points (self_collision.fixed_points): capsule axis ends and the jaw points, base_link.
  std::array<std::array<Vec3, 2>, CAPSULE_COUNT> ends{};
  for (int c = 0; c < CAPSULE_COUNT; ++c) {
    const Mat4 &fa = frames[params_.capsules[c].frame_a];
    const Mat4 &fb = frames[params_.capsules[c].frame_b];
    ends[c][0] = to_base_link({fa[0][3], fa[1][3], fa[2][3]});
    ends[c][1] = to_base_link({fb[0][3], fb[1][3], fb[2][3]});
  }
  const Mat4 &tool = frames[params_.tool_frame];
  const Mat4 &grip = frames[params_.gripper_frame];
  const Vec3 &offset = arm.params().tool_offset;
  Vec3 tool_point{};
  for (int r = 0; r < 3; ++r)
    tool_point[r] = (tool[r][0] * offset[0] + tool[r][1] * offset[1] + tool[r][2] * offset[2]) + tool[r][3];
  // JawModel.moving_tip at the closed gripper angle: rotate (closed_tip - pivot) about the axis, add the pivot.
  const double angle = params_.jaw_sign * (params_.jaw_closed_rad - params_.jaw_closed_rad);
  const double c = std::cos(angle), s = std::sin(angle);
  const Vec3 &ax = params_.jaw_axis;
  Vec3 v{};
  for (int i = 0; i < 3; ++i) v[i] = params_.jaw_closed_tip[i] - params_.jaw_pivot[i];
  const double along = v[0] * ax[0] + v[1] * ax[1] + v[2] * ax[2];
  const Vec3 cross = {ax[1] * v[2] - ax[2] * v[1], ax[2] * v[0] - ax[0] * v[2], ax[0] * v[1] - ax[1] * v[0]};
  Vec3 tip{};
  for (int i = 0; i < 3; ++i) tip[i] = (v[i] * c + cross[i] * s + ax[i] * along * (1.0 - c)) + params_.jaw_pivot[i];
  Vec3 moving{};
  for (int r = 0; r < 3; ++r) moving[r] = (grip[r][0] * tip[0] + grip[r][1] * tip[1] + grip[r][2] * tip[2]) + grip[r][3];
  const std::array<Vec3, 3> jaw = {to_base_link({tool[0][3], tool[1][3], tool[2][3]}), to_base_link(tool_point),
                                   to_base_link(moving)};
  // Block centres in base_link.
  const int groups = static_cast<int>(params_.groups.size());
  std::vector<std::vector<Vec3>> centres(groups);
  for (int g = 0; g < groups; ++g) {
    const BlockGroup &bg = params_.groups[g];
    centres[g].resize(bg.blocks);
    for (int b = 0; b < bg.blocks; ++b) centres[g][b] = to_base_link(rigid(frames[bg.frame], &bg.centres[3 * b]));
  }
  // Pass 1: bounds, and the jaw points exactly.
  std::vector<std::array<double, CHECKED_PART_COUNT>> out(m);
  for (auto &row : out) row.fill(INF);
  std::vector<std::array<double, CAPSULE_COUNT>> cap_lo(m);
  std::vector<std::vector<std::vector<double>>> block_lo(m, std::vector<std::vector<double>>(groups));
  double soft_up = INF, hard_up = INF;
  for (int k = 0; k < m; ++k) {
    const Obstacle &o = obstacles[k];
    for (int cpi = 0; cpi < CAPSULE_COUNT; ++cpi) {
      const Vec3 &a = ends[cpi][0], &b = ends[cpi][1];
      const double radius = params_.capsules[cpi].radius;
      const double dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
      const double half = std::sqrt(dx * dx + (dy * dy + dz * dz)) / 2.0;
      const Vec3 mid = {(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0};
      cap_lo[k][cpi] = o.signed_distance(mid) - half - radius;
      soft_up = std::min(soft_up, std::min(o.signed_distance(a), o.signed_distance(b)) - radius);
    }
    for (int g = 0; g < groups; ++g) {
      const BlockGroup &bg = params_.groups[g];
      auto &lows = block_lo[k][g];
      lows.resize(bg.blocks);
      double upper = INF;
      for (int b = 0; b < bg.blocks; ++b) {
        const double at = o.signed_distance(centres[g][b]);
        lows[b] = at - bg.radii[b];
        upper = std::min(upper, at + bg.radii[b]);
      }
      if (bg.contact_only)
        hard_up = std::min(hard_up, upper + hm);
      else
        soft_up = std::min(soft_up, upper);
    }
    for (int p = JAW_PART_FIRST; p < CHECKED_PART_COUNT; ++p) {
      out[k][p] = o.signed_distance(jaw[p - JAW_PART_FIRST]);
      soft_up = std::min(soft_up, out[k][p]);
    }
  }
  hard_up = std::min(hard_up, soft_up);
  const bool closest = hard_limit < 0.0;
  const double soft_t = closest ? soft_up + PRUNE_SLACK_M : -INF;
  const double hard_t = (closest ? hard_up : hard_limit) + PRUNE_SLACK_M;
  const double need = std::max(soft_t, hard_t);
  // Pass 2: exact distances of what can matter.
  for (int k = 0; k < m; ++k)
    for (int cpi = 0; cpi < CAPSULE_COUNT; ++cpi)
      if (cap_lo[k][cpi] <= need)
        out[k][cpi] = (capsule_axis_distance(obstacles[k], ends[cpi][0], ends[cpi][1]) - 0.0) -
                      params_.capsules[cpi].radius;
  std::vector<std::array<double, CAPSULE_COUNT>> contact_min(m);
  for (auto &row : contact_min) row.fill(INF);
  for (int g = 0; g < groups; ++g) {
    const BlockGroup &bg = params_.groups[g];
    const double required = bg.contact_only ? hard_t - hm : need;
    const Mat4 &frame = frames[bg.frame];
    std::vector<Vec3> placed(bg.size);
    for (int b = 0; b < bg.blocks; ++b) {
      bool any = false;
      for (int k = 0; k < m && !any; ++k) any = block_lo[k][g][b] <= required;
      if (!any) continue;
      for (int i = 0; i < bg.size; ++i)
        placed[i] = to_base_link(rigid(frame, &bg.points[(static_cast<std::size_t>(b) * bg.size + i) * 3]));
      for (int k = 0; k < m; ++k) {
        if (!(block_lo[k][g][b] <= required)) continue;
        double found = INF;
        for (int i = 0; i < bg.size; ++i) found = std::min(found, obstacles[k].signed_distance(placed[i]));
        double &target = bg.contact_only ? contact_min[k][bg.part] : out[k][bg.part];
        target = std::min(target, found);
      }
    }
  }
  SampleClearance result;
  result.values.resize(m);
  result.parts.resize(m);
  for (int k = 0; k < m; ++k) {
    for (int p = 0; p < CAPSULE_COUNT; ++p) out[k][p] = std::min(out[k][p], contact_min[k][p] + hm);
    int best = 0;
    for (int p = 1; p < CHECKED_PART_COUNT; ++p)
      if (out[k][p] < out[k][best]) best = p;
    result.values[k] = out[k][best];
    result.parts[k] = best;
  }
  return result;
}

void BodyModel::closest(const ArmModel &arm, const Joints &measured, double &value, int &obstacle, int &part) const {
  if (params_.obstacles.empty()) {
    value = INF;
    obstacle = -1;
    part = -1;
    return;
  }
  const SampleClearance c = clearance(arm, measured, -1.0);
  int worst = 0;
  for (int k = 1; k < static_cast<int>(c.values.size()); ++k)
    if (c.values[k] < c.values[worst]) worst = k;
  value = c.values[worst];
  obstacle = worst;
  part = c.parts[worst];
}

double BodyModel::body_clearance(const ArmModel &arm, const Joints &measured) const {
  const double cap = params_.soft_margin_m;
  if (params_.obstacles.empty()) return cap;
  const SampleClearance c = clearance(arm, measured, cap);
  double best = INF;
  for (double v : c.values) best = std::min(best, v);
  return std::min(best, cap);
}

}  // namespace ik_reach
