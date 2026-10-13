// Arm self-collision kernels (port of mcp_server/self_collision.py SelfCollisionChecker: clearances_of,
// bounded_clearances, path; reference bodies in mcp_server/reference/collision.py).
//
// Parts are indexed as CHECKED_PARTS: the capsules first (0 .. C-1), then the parts held as points, the last J of
// which are the single jaw points. Point groups (LINK_BLOCKS) are the box face / hull points of one link in blocks of
// equal size with a bounding sphere each (link frame). A sample's geometry (SampleGeometry) is its capsule axis ends
// and jaw points in base_link plus the pose of every block link in the arm frame.
#pragma once

#include <cstdint>
#include <vector>

#include "collision/body_geometry.hpp"

namespace collision {

// Points of one link in blocks: points[b * size + i], the first `count` are the unpadded points.
struct PointGroup {
  int part = 0;
  int link = 0;
  bool contact_only = false;
  int blocks = 0;
  int size = 0;
  int count = 0;
  std::vector<Vec3> points;
  std::vector<Vec3> centres;
  std::vector<double> radii;
};

// One sample's geometry: pointers into the stacked numpy arrays.
struct SampleView {
  const double *capsules;      // (C, 2, 3) axis ends, base_link
  const double *jaws;          // (J, 3) jaw points, base_link
  const double *rotations;     // (L, 3, 3) link rotations, arm frame
  const double *translations;  // (L, 3) link translations, arm frame
};

// Placed part points of one sample (the full computation's input): capsule ends, held points per part (capsules: box
// face points, others: hull or jaw points) and contact-only points per capsule.
struct PlacedSample {
  std::vector<Vec3> ends;                  // 2 per capsule
  std::vector<std::vector<Vec3>> held;     // per part
  std::vector<std::vector<Vec3>> contact;  // per capsule
  std::vector<bool> has_contact;           // per capsule
};

// Exact distances evaluated, as SelfCollisionChecker.distance_counts counts them.
struct Counts {
  int64_t points = 0;
  int64_t capsules = 0;
};

// Per-sample, per-obstacle hard clearances and closest parts, then the soft ones (row-major (N, M)).
struct Clearances {
  std::vector<double> values;
  std::vector<int64_t> parts;
  std::vector<double> soft;
  std::vector<int64_t> soft_parts;

  explicit Clearances(size_t size = 0) : values(size), parts(size), soft(size), soft_parts(size) {}
};

// path() result: per sample the smallest soft clearance with its obstacle and part, and the first hard violation.
struct PathResult {
  std::vector<double> clearances;
  std::vector<int64_t> obstacles;
  std::vector<int64_t> parts;
  int64_t violation = -1;
  int64_t violation_obstacle = -1;
  int64_t violation_part = -1;
  double violation_clearance = 0.0;
  Counts counts;
};

class BodyChecker {
 public:
  BodyChecker(std::vector<Obstacle> obstacles, std::vector<double> capsule_radii, std::vector<PointGroup> groups,
              Rigid mount, int parts, int jaws, int links, double prune_slack, double escape_tolerance);

  size_t obstacle_count() const { return obstacles_.size(); }
  int capsule_count() const { return static_cast<int>(radii_.size()); }
  int part_count() const { return parts_; }
  int jaw_count() const { return jaws_; }
  int link_count() const { return links_; }

  // clearances_of on placed points: writes M values per output at row offset `row`.
  void full(const PlacedSample &sample, double hard_margin, Clearances &out, size_t row, Counts &counts) const;
  // Place every group point of a sample (geometry) for full().
  PlacedSample place(const SampleView &sample) const;
  // bounded_clearances of one sample; hard_limit (M values) or nullptr for the sample's smallest hard clearance.
  void bounded(const SampleView &sample, double hard_margin, const double *hard_limit, bool soft, Clearances &out,
               size_t row, Counts &counts) const;
  // path(): pruned (or full) clearances, the soft band of all parts or only `moving` (indices of jaw parts), the
  // per-sample worst soft clearance and the first sample breaking the hard limit.
  PathResult path(const std::vector<SampleView> &samples, double hard_margin, bool allow_escape, bool prune,
                  const std::vector<int64_t> &moving) const;

 private:
  Vec3 to_base_link(const Rigid &link, const Vec3 &p) const { return mount_.apply(link.apply(p)); }
  Rigid link_pose(const SampleView &sample, int link) const;
  Vec3 jaw(const SampleView &sample, int part) const;

  std::vector<Obstacle> obstacles_;
  std::vector<double> radii_;
  std::vector<PointGroup> groups_;
  Rigid mount_;
  int parts_;
  int jaws_;
  int links_;
  double prune_slack_;
  double escape_tolerance_;
  size_t total_blocks_;
};

}  // namespace collision
