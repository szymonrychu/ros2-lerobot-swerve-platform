// SO101 arm kinematics of mcp_server.ik.ArmKinematics in C++: forward kinematics of the ikpy chain and the IK
// (inverse / inverse_flange with the same seeds, heading refinement, acceptance tests and solution cache), solved by
// trf.hpp, a port of the scipy least_squares 'trf' method ikpy calls.
#pragma once

#include <array>
#include <cstddef>
#include <list>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ik_reach {

using Vec3 = std::array<double, 3>;
using Mat4 = std::array<std::array<double, 4>, 4>;

// The chain joints shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll (ik.ARM_CHAIN_JOINTS); the IK
// solves the first four (ik.IK_ACTIVE_JOINTS) and keeps wrist_roll.
constexpr double PI = 3.14159265358979323846;
constexpr int CHAIN_JOINTS = 5;
constexpr int ACTIVE_JOINTS = 4;
constexpr int WRIST_ROLL = 4;
constexpr int MAX_LINKS = 12;
using Joints = std::array<double, CHAIN_JOINTS>;
using LinkVector = std::array<double, MAX_LINKS>;
using Frames = std::array<Mat4, MAX_LINKS>;

// Constants of mcp_server/ik.py.
constexpr double POSITION_TOLERANCE_M = 0.002;
constexpr double PITCH_TOLERANCE_DEG = 3.0;
constexpr int HEADING_REFINEMENTS = 6;
constexpr int TOOL_ITERATIONS = 6;
constexpr std::size_t SOLUTION_CACHE_SIZE = 8192;
constexpr double TOOL_CONVERGENCE_M = 0.0005;
constexpr std::array<std::array<double, 3>, 5> EXTRA_SEEDS = {{
    {0.0, 0.0, 0.0},
    {-0.8, 0.8, 0.5},
    {0.5, -0.5, 1.0},
    {-1.2, 1.4, 0.0},
    {0.8, 0.8, -0.5},
}};

// The ikpy chain and the ArmKinematics parameters, as mcp_server.ik_reach_core reads them from an ArmKinematics.
struct ArmParams {
  std::vector<Mat4> link_fixed;  // per link: its matrix at joint angle 0 (origin translation and rpy)
  std::vector<Vec3> link_axis;   // per link: rotation axis (unused for fixed links)
  std::vector<bool> link_moves;  // per link: revolute joint
  std::array<int, CHAIN_JOINTS> joint_link{};  // link index of each chain joint
  Joints lower{};                // URDF limits (after overrides)
  Joints upper{};
  double margin = 0.0;           // kept from each limit
  Joints offsets{};              // urdf = measured + offset
  Vec3 tool_offset{};            // tool point in gripper_frame_link
  double max_reach_m = 0.0;
  double pan_axis_x = 0.0;
  double pan_axis_y = 0.0;
};

// Result of one inverse call.
struct InverseResult {
  std::optional<Joints> solution;  // measured joint angles, none when unreachable
};

class ArmModel {
 public:
  explicit ArmModel(ArmParams params);

  // Link count of the chain.
  int links() const { return static_cast<int>(params_.link_fixed.size()); }
  const ArmParams &params() const { return params_; }

  // Full link vector (URDF angles) of URDF chain joint angles.
  LinkVector urdf_vector(const Joints &urdf) const;
  // Frames of every link for a link vector (ikpy forward_kinematics, full_kinematics=True).
  void forward_all(const LinkVector &q, Frames &out) const;
  // Frame of the last link (gripper_frame_link).
  Mat4 forward_last(const LinkVector &q) const;

  Joints to_urdf(const Joints &measured) const;
  Joints to_measured(const Joints &urdf) const;

  // ArmKinematics.inverse: tool point at (x, y, z) in the arm base frame, optional approach pitch, URDF seed
  // (missing seed joints 0), extra tool offset. Updates last_heading_bias as the Python solver does.
  std::optional<Joints> inverse(double x, double y, double z, std::optional<double> pitch, const Joints &seed_urdf,
                                const Vec3 &extra_offset, double heading_bias);
  // ArmKinematics.inverse_flange (gripper_frame_link at the target).
  std::optional<Joints> inverse_flange(double x, double y, double z, std::optional<double> pitch,
                                       const Joints &seed_urdf, double heading_bias);

  double last_heading_bias = 0.0;  // approach heading - target bearing of the last pitched solve
  long solve_count = 0;            // solves run (cache hits excluded)

 private:
  Joints clamp_seed(const Joints &joints) const;
  bool within_limits(const Joints &urdf) const;
  // Tool point (base frame) and pitch of measured joints with an extra offset (forward()).
  Vec3 tool_point(const Joints &measured, const Vec3 &extra_offset, double *pitch) const;
  // One ikpy solve from a URDF start (inside bounds), cached.
  Joints solve(const Vec3 &target, const std::optional<Vec3> &orientation, const Joints &start, double roll);

  ArmParams params_;
  std::array<double, ACTIVE_JOINTS> lb_{};
  std::array<double, ACTIVE_JOINTS> ub_{};
  // LRU cache of solves keyed by the bytes of (target, orientation, initial link vector, roll, mode).
  std::list<std::string> order_;
  std::unordered_map<std::string, std::pair<Joints, std::list<std::string>::iterator>> cache_;
};

// pitch_of: angle of an approach vector below the horizontal plane.
double pitch_of(double ax, double ay, double az);

}  // namespace ik_reach
