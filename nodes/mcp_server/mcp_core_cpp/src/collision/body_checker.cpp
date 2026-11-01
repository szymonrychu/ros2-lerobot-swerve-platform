// Arm self-collision kernels (see body_checker.hpp); each function mirrors its reference in
// mcp_server/reference/collision.py step by step, so pruned and full results agree bit for bit.
#include "collision/body_checker.hpp"
#include "collision/fp_contract.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace collision {

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();

// numpy min / argmin of a row (first index of the smallest value; 0 for a row of inf).
std::pair<double, int64_t> row_min(const double *row, int count) {
  double best = row[0];
  int64_t index = 0;
  for (int i = 1; i < count; ++i) {
    if (row[i] < best) {
      best = row[i];
      index = i;
    }
  }
  return {best, index};
}

Vec3 at(const double *data, size_t index) { return Vec3{data[index * 3], data[index * 3 + 1], data[index * 3 + 2]}; }

}  // namespace

BodyChecker::BodyChecker(std::vector<Obstacle> obstacles, std::vector<double> capsule_radii,
                         std::vector<PointGroup> groups, Rigid mount, int parts, int jaws, int links,
                         double prune_slack, double escape_tolerance)
    : obstacles_(std::move(obstacles)),
      radii_(std::move(capsule_radii)),
      groups_(std::move(groups)),
      mount_(mount),
      parts_(parts),
      jaws_(jaws),
      links_(links),
      prune_slack_(prune_slack),
      escape_tolerance_(escape_tolerance),
      total_blocks_(0) {
  const int capsules = capsule_count();
  if (capsules + jaws_ >= parts_) {
    throw std::invalid_argument("parts must hold the capsules, a point part and the jaws");
  }
  for (const PointGroup &g : groups_) {
    if (g.part < 0 || g.part >= parts_ - jaws_ || g.link < 0 || g.link >= links_) {
      throw std::invalid_argument("point group with an invalid part or link");
    }
    if (g.contact_only && g.part >= capsules) {
      throw std::invalid_argument("contact-only points belong to a capsule");
    }
    const size_t blocks = static_cast<size_t>(g.blocks);
    if (g.blocks <= 0 || g.size <= 0 || g.count <= 0 || g.count > g.blocks * g.size ||
        g.points.size() != blocks * g.size || g.centres.size() != blocks || g.radii.size() != blocks) {
      throw std::invalid_argument("point group with inconsistent blocks");
    }
    total_blocks_ += g.blocks;
  }
}

Rigid BodyChecker::link_pose(const SampleView &sample, int link) const {
  return make_rigid(sample.rotations + link * 9, sample.translations + link * 3);
}

Vec3 BodyChecker::jaw(const SampleView &sample, int part) const {
  return at(sample.jaws, static_cast<size_t>(part - (parts_ - jaws_)));
}

PlacedSample BodyChecker::place(const SampleView &sample) const {
  const int capsules = capsule_count();
  PlacedSample out;
  out.ends.reserve(capsules * 2);
  for (int c = 0; c < capsules * 2; ++c) out.ends.push_back(at(sample.capsules, c));
  out.held.assign(parts_, {});
  out.contact.assign(capsules, {});
  out.has_contact.assign(capsules, false);
  std::vector<Rigid> poses;
  poses.reserve(links_);
  for (int l = 0; l < links_; ++l) poses.push_back(link_pose(sample, l));
  for (const PointGroup &g : groups_) {
    std::vector<Vec3> &target = g.contact_only ? out.contact[g.part] : out.held[g.part];
    if (g.contact_only) out.has_contact[g.part] = true;
    for (int i = 0; i < g.count; ++i) target.push_back(to_base_link(poses[g.link], g.points[i]));
  }
  for (int j = 0; j < jaws_; ++j) out.held[parts_ - jaws_ + j].push_back(jaw(sample, parts_ - jaws_ + j));
  return out;
}

void BodyChecker::full(const PlacedSample &sample, double hard_margin, Clearances &out, size_t row,
                       Counts &counts) const {
  const int capsules = capsule_count();
  const size_t m = obstacles_.size();
  std::vector<double> clear(m * parts_);
  auto nearest = [](const Obstacle &o, const std::vector<Vec3> &points) {
    double best = INF;
    for (const Vec3 &p : points) best = std::min(best, o.signed_distance(p));
    return best;
  };
  for (size_t k = 0; k < m; ++k) {
    const Obstacle &o = obstacles_[k];
    double *c = &clear[k * parts_];
    for (int p = 0; p < capsules; ++p) {
      c[p] = axis_distance(o, sample.ends[2 * p], sample.ends[2 * p + 1]) - radii_[p];
      counts.capsules += 1;
      if (!sample.held[p].empty()) {
        c[p] = std::min(c[p], nearest(o, sample.held[p]));
        counts.points += static_cast<int64_t>(sample.held[p].size());
      }
    }
    for (int p = capsules; p < parts_; ++p) {
      c[p] = nearest(o, sample.held[p]);
      counts.points += static_cast<int64_t>(sample.held[p].size());
    }
  }
  for (size_t k = 0; k < m; ++k) {
    const auto [value, part] = row_min(&clear[k * parts_], parts_);
    out.soft[row + k] = value;
    out.soft_parts[row + k] = part;
  }
  for (int p = 0; p < capsules; ++p) {
    if (!sample.has_contact[p]) continue;
    for (size_t k = 0; k < m; ++k) {
      const double contact = nearest(obstacles_[k], sample.contact[p]);
      counts.points += static_cast<int64_t>(sample.contact[p].size());
      clear[k * parts_ + p] = std::min(clear[k * parts_ + p], contact + hard_margin);
    }
  }
  for (size_t k = 0; k < m; ++k) {
    const auto [value, part] = row_min(&clear[k * parts_], parts_);
    out.values[row + k] = value;
    out.parts[row + k] = part;
  }
}

void BodyChecker::bounded(const SampleView &sample, double hard_margin, const double *hard_limit, bool soft,
                          Clearances &out, size_t row, Counts &counts) const {
  const int capsules = capsule_count();
  const size_t m = obstacles_.size();
  std::vector<Rigid> poses;
  poses.reserve(links_);
  for (int l = 0; l < links_; ++l) poses.push_back(link_pose(sample, l));
  // Block centres in base_link, all groups in order.
  std::vector<Vec3> centres;
  centres.reserve(total_blocks_);
  for (const PointGroup &g : groups_) {
    for (int b = 0; b < g.blocks; ++b) centres.push_back(to_base_link(poses[g.link], g.centres[b]));
  }
  std::vector<double> clear(m * parts_, INF);
  std::vector<double> cap_lo(m * capsules);
  std::vector<double> block_lo(m * total_blocks_);
  double soft_up = INF, hard_up = INF;
  // Pass 1: bounds (and the jaw points, single points evaluated exactly).
  for (size_t k = 0; k < m; ++k) {
    const Obstacle &o = obstacles_[k];
    for (int p = 0; p < capsules; ++p) {
      const Vec3 a = at(sample.capsules, 2 * p), b = at(sample.capsules, 2 * p + 1);
      const double d0 = b[0] - a[0], d1 = b[1] - a[1], d2 = b[2] - a[2];
      const double s0 = d0 * d0, s1 = d1 * d1, s2 = d2 * d2;
      const double half = std::sqrt((s0 + s1) + s2) / 2.0;
      const Vec3 mid{(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0};
      cap_lo[k * capsules + p] = (o.signed_distance(mid) - half) - radii_[p];
      soft_up = std::min(soft_up, std::min(o.signed_distance(a), o.signed_distance(b)) - radii_[p]);
    }
    size_t block = 0;
    for (const PointGroup &g : groups_) {
      double upper = INF;
      for (int b = 0; b < g.blocks; ++b, ++block) {
        const double distance = o.signed_distance(centres[block]);
        block_lo[k * total_blocks_ + block] = distance - g.radii[b];
        upper = std::min(upper, distance + g.radii[b]);
      }
      if (g.contact_only) {
        hard_up = std::min(hard_up, upper + hard_margin);
      } else {
        soft_up = std::min(soft_up, upper);
      }
    }
    for (int p = parts_ - jaws_; p < parts_; ++p) {
      clear[k * parts_ + p] = o.signed_distance(jaw(sample, p));
      soft_up = std::min(soft_up, clear[k * parts_ + p]);
      counts.points += 1;
    }
  }
  hard_up = std::min(hard_up, soft_up);
  const double soft_t = soft ? soft_up + prune_slack_ : -INF;
  std::vector<double> hard_t(m), need(m);
  for (size_t k = 0; k < m; ++k) {
    hard_t[k] = (hard_limit != nullptr ? hard_limit[k] : hard_up) + prune_slack_;
    need[k] = std::max(soft_t, hard_t[k]);
  }
  // Pass 2: exact distances of what can matter.
  for (size_t k = 0; k < m; ++k) {
    for (int p = 0; p < capsules; ++p) {
      if (cap_lo[k * capsules + p] <= need[k]) {
        clear[k * parts_ + p] =
            axis_distance(obstacles_[k], at(sample.capsules, 2 * p), at(sample.capsules, 2 * p + 1)) - radii_[p];
        counts.capsules += 1;
      }
    }
  }
  std::vector<double> contact_min(m * capsules, INF);
  std::vector<Vec3> placed;
  size_t block = 0;
  for (const PointGroup &g : groups_) {
    for (int b = 0; b < g.blocks; ++b, ++block) {
      bool any = false;
      for (size_t k = 0; k < m && !any; ++k) {
        const double required = g.contact_only ? hard_t[k] - hard_margin : need[k];
        any = block_lo[k * total_blocks_ + block] <= required;
      }
      if (!any) continue;
      placed.clear();
      for (int i = 0; i < g.size; ++i) placed.push_back(to_base_link(poses[g.link], g.points[b * g.size + i]));
      for (size_t k = 0; k < m; ++k) {
        const double required = g.contact_only ? hard_t[k] - hard_margin : need[k];
        if (!(block_lo[k * total_blocks_ + block] <= required)) continue;
        double found = INF;
        for (const Vec3 &p : placed) found = std::min(found, obstacles_[k].signed_distance(p));
        counts.points += g.size;
        double &target = g.contact_only ? contact_min[k * capsules + g.part] : clear[k * parts_ + g.part];
        target = std::min(target, found);
      }
    }
  }
  for (size_t k = 0; k < m; ++k) {
    const auto [value, part] = row_min(&clear[k * parts_], parts_);
    out.soft[row + k] = value;
    out.soft_parts[row + k] = part;
    for (int p = 0; p < capsules; ++p) {
      clear[k * parts_ + p] = std::min(clear[k * parts_ + p], contact_min[k * capsules + p] + hard_margin);
    }
    const auto [hard, hard_part] = row_min(&clear[k * parts_], parts_);
    out.values[row + k] = hard;
    out.parts[row + k] = hard_part;
  }
}

PathResult BodyChecker::path(const std::vector<SampleView> &samples, double hard_margin, bool allow_escape,
                             bool prune, const std::vector<int64_t> &moving) const {
  const size_t n = samples.size(), m = obstacles_.size();
  for (int64_t part : moving) {
    if (part < parts_ - jaws_ || part >= parts_) throw std::invalid_argument("moving parts must be jaw points");
  }
  PathResult result;
  if (n == 0 || m == 0) throw std::invalid_argument("path needs at least one sample and one obstacle");
  Clearances c(n * m);
  std::vector<double> limit(m);
  auto hard_limit = [&]() {
    for (size_t k = 0; k < m; ++k) {
      const double start = c.values[k];
      limit[k] = (start >= hard_margin || !allow_escape) ? hard_margin : start - escape_tolerance_;
    }
  };
  if (prune) {
    const std::vector<double> margin(m, hard_margin);
    bounded(samples[0], hard_margin, margin.data(), true, c, 0, result.counts);
    hard_limit();
    for (size_t i = 1; i < n; ++i) bounded(samples[i], hard_margin, limit.data(), true, c, i * m, result.counts);
  } else {
    for (size_t i = 0; i < n; ++i) full(place(samples[i]), hard_margin, c, i * m, result.counts);
    hard_limit();
  }
  if (!moving.empty()) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t k = 0; k < m; ++k) {
        double best = INF;
        int64_t index = 0;
        for (size_t j = 0; j < moving.size(); ++j) {
          const double d = obstacles_[k].signed_distance(jaw(samples[i], static_cast<int>(moving[j])));
          if (j == 0 || d < best) {
            best = d;
            index = static_cast<int64_t>(j);
          }
        }
        c.soft[i * m + k] = best;
        c.soft_parts[i * m + k] = moving[index];
      }
    }
  }
  result.clearances.resize(n);
  result.obstacles.resize(n);
  result.parts.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const auto [value, worst] = row_min(&c.soft[i * m], static_cast<int>(m));
    result.clearances[i] = value;
    result.obstacles[i] = worst;
    result.parts[i] = c.soft_parts[i * m + worst];
  }
  for (size_t i = 0; i < n && result.violation < 0; ++i) {
    bool broken = false;
    for (size_t k = 0; k < m; ++k) broken = broken || c.values[i * m + k] < limit[k];
    if (!broken) continue;
    std::vector<double> masked(m);
    for (size_t k = 0; k < m; ++k) masked[k] = c.values[i * m + k] < limit[k] ? c.values[i * m + k] : INF;
    const int64_t k = row_min(masked.data(), static_cast<int>(m)).second;
    result.violation = static_cast<int64_t>(i);
    result.violation_obstacle = k;
    result.violation_part = c.parts[i * m + k];
    result.violation_clearance = c.values[i * m + k];
  }
  return result;
}

}  // namespace collision
