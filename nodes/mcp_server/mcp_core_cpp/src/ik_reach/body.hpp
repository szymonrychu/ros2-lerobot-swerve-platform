// Body clearance of arm configurations for the reach kernels: the convex body obstacles of mcp_server/body_model.py
// (exact point signed distances, golden-section capsule search) and the pruned clearance pass of
// SelfCollisionChecker.bounded_clearances (mcp_server/self_collision.py) over the checked parts: three link capsules,
// the link collision box face points and gripper hull points in blocks with bounding spheres, and the three jaw points.
#pragma once

#include <array>
#include <vector>

#include "ik_reach/kinematics.hpp"

namespace ik_reach {

using Mat3 = std::array<std::array<double, 3>, 3>;

// A box (axis aligned, corners lo / hi) or a vertical cylinder, base_link.
struct Obstacle {
  bool box = true;
  Vec3 lo{}, hi{};           // box
  Vec3 centre{}, half{};     // box: derived like BoxObstacle.signed_distance
  double cx = 0.0, cy = 0.0;  // cylinder axis
  double radius = 0.0, z_min = 0.0, z_max = 0.0;
  double signed_distance(const Vec3 &p) const;
};

// The point blocks of one link (self_collision.PointBlocks): `size` points per block, link frame.
struct BlockGroup {
  int part = 0;           // index into CHECKED_PARTS
  int frame = 0;          // link frame index in the chain
  bool contact_only = false;
  int blocks = 0;
  int size = 0;
  std::vector<double> points;   // blocks * size * 3
  std::vector<double> centres;  // blocks * 3
  std::vector<double> radii;    // blocks
};

struct Capsule {
  int frame_a = 0;  // link frame of the axis start
  int frame_b = 0;  // link frame of the axis end
  double radius = 0.0;
};

// Everything SelfCollisionChecker needs besides the kinematics (body_model, mount, jaw, link geometry, margins).
struct BodyParams {
  Mat3 rotation{};       // arm mount yaw
  Vec3 offset{};         // arm mount position in base_link
  Vec3 jaw_closed_tip{};  // JawModel, gripper_link frame
  Vec3 jaw_pivot{};
  Vec3 jaw_axis{};
  double jaw_sign = 1.0;
  double jaw_closed_rad = 0.0;
  int tool_frame = 0;     // gripper_frame_link
  int gripper_frame = 0;  // gripper_link
  std::array<Capsule, 3> capsules{};
  std::vector<BlockGroup> groups;
  std::vector<Obstacle> obstacles;
  double hard_margin_m = 0.0;
  double soft_margin_m = 0.0;
};

// Checked parts in self_collision.CHECKED_PARTS order: three capsules, gripper body, jaw tip, tool point, moving tip.
constexpr int CHECKED_PART_COUNT = 7;
constexpr int CAPSULE_COUNT = 3;
constexpr int JAW_PART_FIRST = 4;
constexpr double PRUNE_SLACK_M = 1e-9;

// Hard clearance of one sample to every obstacle and the closest part (bounded_clearances outputs 0 and 1).
struct SampleClearance {
  std::vector<double> values;  // per obstacle
  std::vector<int> parts;      // per obstacle, index into CHECKED_PARTS
};

class BodyModel {
 public:
  explicit BodyModel(BodyParams params);
  const BodyParams &params() const { return params_; }

  // bounded_clearances of one configuration (measured joints, closed gripper). With hard_limit < 0 the hard values
  // are exact up to the sample's smallest hard clearance and the soft band guarantee is kept (closest()); otherwise
  // they are exact up to hard_limit and nothing is evaluated for the soft band (clear_of, the reach map).
  SampleClearance clearance(const ArmModel &arm, const Joints &measured, double hard_limit) const;
  // SelfCollisionChecker.closest of one configuration: (clearance, obstacle index, part index).
  void closest(const ArmModel &arm, const Joints &measured, double &value, int &obstacle, int &part) const;
  // reach_map.body_clearance of one configuration: smallest hard clearance, saturated at soft_margin_m.
  double body_clearance(const ArmModel &arm, const Joints &measured) const;

 private:
  Vec3 to_base_link(const Vec3 &p) const;
  BodyParams params_;
};

// The signed distance of a capsule axis a -> b (radius 0) to an obstacle, body_model.capsule_signed_distance.
double capsule_axis_distance(const Obstacle &obstacle, const Vec3 &a, const Vec3 &b);

}  // namespace ik_reach
