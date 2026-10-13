// The ported ik_reach kernels (see reach.hpp); reach_map_build mirrors reach_map.build step for step.
#if defined(__clang__)
// No fused multiply-add: the sums round like numpy's (GCC in ISO C++ mode does not contract either).
#pragma STDC FP_CONTRACT OFF
#endif

#include "ik_reach/reach.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace ik_reach {

namespace {

constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

// numpy's float remainder (the sign of the divisor), as reach_map.wrap_deg uses it: angles to [-180, 180).
double wrap_deg(double angle) {
  double mod = std::fmod(angle + 180.0, 360.0);
  if (mod != 0.0) {
    if (mod < 0.0) mod += 360.0;
  } else {
    mod = 0.0;
  }
  return mod - 180.0;
}

Vec3 to_base_link(const BodyParams &body, const Vec3 &p) {
  Vec3 out{};
  for (int i = 0; i < 3; ++i)
    out[i] = p[0] * body.rotation[i][0] + p[1] * body.rotation[i][1] + p[2] * body.rotation[i][2] + body.offset[i];
  return out;
}

}  // namespace

PointReach point_reach(ArmModel &arm, const BodyModel &body, const Vec3 &target_arm, const std::vector<double> &pitches,
                       const Joints &seed_urdf) {
  PointReach out;
  const Vec3 no_extra{};
  for (double p : pitches)
    out.solutions.push_back(arm.inverse(target_arm[0], target_arm[1], target_arm[2], p, seed_urdf, no_extra, 0.0));
  for (const auto &sol : out.solutions) {
    if (!sol) continue;
    double value;
    int obstacle, part;
    body.closest(arm, *sol, value, obstacle, part);
    out.closest.emplace_back(value, obstacle, part);
  }
  return out;
}

ReachMapResult reach_map_build(const ArmModel &arm, const BodyModel &body, const ReachMapGrid &grid) {
  const auto &arm_params = arm.params();
  const auto &body_params = body.params();
  const int n0 = grid.planar_samples[0], n1 = grid.planar_samples[1], n2 = grid.planar_samples[2];
  const long total = static_cast<long>(n0) * n1 * n2;
  const int heights = static_cast<int>(grid.heights_m.size());
  const int radii = static_cast<int>(grid.radii_m.size());
  const int pitches = static_cast<int>(grid.pitches_deg.size());
  const int headings = static_cast<int>(grid.headings_deg.size());
  auto planar = [&](long index) {
    Joints measured{};
    measured[1] = grid.joint_grids[0][index / (static_cast<long>(n1) * n2)];
    measured[2] = grid.joint_grids[1][(index / n2) % n1];
    measured[3] = grid.joint_grids[2][index % n2];
    return measured;  // shoulder_pan and wrist_roll 0
  };
  // Forward sampling at pan 0 and binning to the nearest (r, z) cell and pitch.
  struct Binned {
    long index;
    int ri, zi, pi;
    double bearing, distance;
    long cell;
  };
  std::vector<Binned> binned;
  for (long i = 0; i < total; ++i) {
    const Joints measured = planar(i);
    const Mat4 frame = arm.forward_last(arm.urdf_vector(arm.to_urdf(measured)));
    Vec3 arm_point{};
    for (int r = 0; r < 3; ++r)
      arm_point[r] = frame[r][3] + (frame[r][0] * arm_params.tool_offset[0] + frame[r][1] * arm_params.tool_offset[1] +
                                    frame[r][2] * arm_params.tool_offset[2]);
    const double pitch_deg = pitch_of(frame[0][2], frame[1][2], frame[2][2]) * RAD_TO_DEG;
    const Vec3 point = to_base_link(body_params, arm_point);
    const double rx = point[0] - grid.pan_axis[0], ry = point[1] - grid.pan_axis[1];
    const double r = std::hypot(rx, ry);
    const long ri = static_cast<long>(std::nearbyint(r / grid.r_step_m)) - 1;
    const long zi = static_cast<long>(std::nearbyint(point[2] / grid.z_step_m)) - 1;
    int pi = 0;
    double gap = std::abs(pitch_deg - grid.pitches_deg[0]);
    for (int k = 1; k < pitches; ++k) {
      const double g = std::abs(pitch_deg - grid.pitches_deg[k]);
      if (g < gap) {
        gap = g;
        pi = k;
      }
    }
    if (ri < 0 || ri >= radii || zi < 0 || zi >= heights || !(gap <= grid.pitch_half_spacing_deg + 1e-9)) continue;
    const double distance = std::hypot(r - grid.radii_m[ri], point[2] - grid.heights_m[zi]);
    binned.push_back({i, static_cast<int>(ri), static_cast<int>(zi), pi, std::atan2(ry, rx) * RAD_TO_DEG, distance,
                      (ri * heights + zi) * pitches + pi});
  }
  std::vector<std::size_t> order(binned.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    const Binned &x = binned[a], &y = binned[b];
    if (x.cell != y.cell) return x.cell < y.cell;
    if (x.distance != y.distance) return x.distance < y.distance;
    return x.index < y.index;
  });
  ReachMapResult out;
  out.binned = static_cast<long>(binned.size());
  const std::size_t cells = static_cast<std::size_t>(headings) * radii * heights;
  out.mask.assign(cells, 0);
  out.best.assign(cells, std::numeric_limits<double>::quiet_NaN());
  const double hard = body_params.hard_margin_m;
  out.checked_per_heading.assign(headings, 0);
  for (int h = 0; h < headings; ++h) {
    long group_cell = -1;
    int rank = 0;
    for (std::size_t k : order) {
      const Binned &s = binned[k];
      const double pan_deg = wrap_deg(grid.pan_sign * (grid.headings_deg[h] - s.bearing));
      if (!(pan_deg >= grid.pan_lo_deg && pan_deg <= grid.pan_hi_deg)) continue;
      rank = s.cell == group_cell ? rank + 1 : 0;
      group_cell = s.cell;
      if (rank >= grid.samples_per_cell) continue;
      Joints measured = planar(s.index);
      measured[0] = pan_deg * DEG_TO_RAD;
      const double clearance = body.body_clearance(arm, measured);
      ++out.checked;
      ++out.checked_per_heading[h];
      if (!(clearance >= hard)) continue;
      ++out.clear;
      const std::size_t flat = (static_cast<std::size_t>(h) * radii + s.ri) * heights + s.zi;
      out.mask[flat] = static_cast<std::uint16_t>(out.mask[flat] | (1u << s.pi));
      out.best[flat] = std::fmax(out.best[flat], clearance);
    }
  }
  return out;
}

}  // namespace ik_reach
