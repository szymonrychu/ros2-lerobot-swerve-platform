// The two ported ik_reach kernels: the compute of arm_reach.point_reach (IK per pitch, body clearance of each
// solution) and of reach_map.build (forward sampling, binning, per-heading body check).
#pragma once

#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

#include "ik_reach/body.hpp"
#include "ik_reach/kinematics.hpp"

namespace ik_reach {

struct PointReach {
  std::vector<std::optional<Joints>> solutions;            // per pitch, measured
  std::vector<std::tuple<double, int, int>> closest;       // per solution found: clearance, obstacle, part
};

// arm_reach.point_reach compute: ArmKinematics.inverse(target, pitch, seed) for every pitch, then
// SelfCollisionChecker.closest of the solutions found.
PointReach point_reach(ArmModel &arm, const BodyModel &body, const Vec3 &target_arm, const std::vector<double> &pitches,
                       const Joints &seed_urdf);

struct ReachMapGrid {
  std::array<std::vector<double>, 3> joint_grids;  // shoulder_lift, elbow_flex, wrist_flex samples (measured rad)
  std::array<int, 3> planar_samples{};
  Vec3 pan_axis{};
  double pan_sign = 1.0;
  std::vector<double> headings_deg, radii_m, heights_m, pitches_deg;
  double r_step_m = 0.0, z_step_m = 0.0;
  double pan_lo_deg = 0.0, pan_hi_deg = 0.0;
  double pitch_half_spacing_deg = 0.0;
  int samples_per_cell = 1;
};

struct ReachMapResult {
  std::vector<std::uint16_t> mask;  // [H, R, Z]
  std::vector<double> best;         // [H, R, Z], NaN where no pitch works
  long binned = 0, checked = 0, clear = 0;
  std::vector<long> checked_per_heading;  // samples body checked at each heading
};

// reach_map.build compute (the grid comes from the Python side: pan geometry, headings, radii, heights).
ReachMapResult reach_map_build(const ArmModel &arm, const BodyModel &body, const ReachMapGrid &grid);

}  // namespace ik_reach
