// The SO101 waypoint IK of mcp_server.ik.ArmKinematics.inverse (position + approach pitch, heading refinement, extra
// seeds, tool-point fixed-point iterations) on the ikpy chain described by mcp_server.grasp.solver_geometry, with
// ikpy's residual (position, then the tool z axis) solved by the scipy TRF port (trf.hpp).
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <nanobind/nanobind.h>

namespace grasp {

using Vec3 = std::array<double, 3>;
using Mat4 = std::array<double, 16>;  // row-major homogeneous transform
using Joints = std::vector<double>;   // chain joints, in GeometryModel::joints order

// solver_geometry, parsed once per kernel call.
struct GeometryModel {
  int links = 0;                         // ikpy chain links, origin link included (index 0)
  std::vector<Mat4> origin;              // per link: T(origin) @ RPY(origin); identity for the origin link
  std::vector<Vec3> axis;                // per link: joint axis (revolute links)
  std::vector<bool> revolute;            // per link
  std::vector<int> active_links;         // link indices of the IK-active joints, in chain order
  std::vector<double> lower, upper;      // IK bounds of the active links (URDF limits minus margin)
  std::vector<std::string> joints;       // chain joint names (ARM_CHAIN_JOINTS)
  std::vector<int> index;                // per chain joint: its link index
  std::vector<double> offsets;           // per chain joint: urdf = measured + offset
  std::vector<double> limit_lo, limit_hi;  // per chain joint: URDF limits (overrides applied)
  int roll = -1, pan = -1, lift = -1, elbow = -1, wrist = -1;  // chain joint positions by role
  double margin = 0.0;
  Vec3 tool_offset{};
  double max_reach_m = 0.0;
  double pan_x = 0.0, pan_y = 0.0;
  double position_tolerance_m = 0.0;
  double pitch_tolerance_rad = 0.0;
  int heading_refinements = 0;
  int tool_iterations = 0;
  double tool_convergence_m = 0.0;
  std::vector<Vec3> extra_seeds;  // (shoulder_lift, elbow_flex, wrist_flex)
  std::size_t cache_size = 0;

  static GeometryModel parse(const nanobind::dict &geometry);
};

// One IK search on a model, sharing a Python solution cache (ArmKinematics.solutions: OrderedDict of key bytes ->
// joint dict) with the same keys and LRU policy as ArmKinematics.solve.
class ArmIk {
 public:
  ArmIk(const GeometryModel &model, nanobind::object cache);

  // ArmKinematics.inverse with an approach pitch (the planner always passes one): true and the measured joints, or
  // false and the UnreachableError message.
  bool inverse(const Vec3 &target, double pitch, const Joints &seed, const std::optional<Vec3> &extra,
               double heading_bias, Joints &out, std::string &error);

  int solves = 0;                  // ikpy runs (cache misses)
  double last_heading_bias = 0.0;  // ArmKinematics.last_heading_bias

 private:
  bool inverse_flange(const Vec3 &target, double pitch, const Joints &seed, double heading_bias,
                      Joints &out, std::string &error);
  Joints solve(const Vec3 &target, const Vec3 &orientation, const Joints &start, double roll);
  Mat4 forward_kinematics(const std::vector<double> &q) const;
  std::vector<double> urdf_vector(const Joints &urdf) const;
  Joints clamp_seed(const Joints &urdf) const;
  bool within_limits(const Joints &urdf) const;
  Vec3 tool_point(const Joints &measured, const std::optional<Vec3> &extra) const;

  const GeometryModel &model_;
  nanobind::object cache_;
  std::vector<nanobind::str> names_;
};

}  // namespace grasp
