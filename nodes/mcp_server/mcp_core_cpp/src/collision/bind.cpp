// Python bindings of the collision kernel family (submodule mcp_core_cpp.collision): BodyChecker, the arm
// self-collision kernels of mcp_server.self_collision (ported: SelfCollisionChecker.path and .pose; see
// docs/collision.md). Arrays are float64 / int64 numpy arrays; non-contiguous inputs are copied on the way in.
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/vector.h>

#include "collision/body_checker.hpp"
#include "families.hpp"

namespace nb = nanobind;
using namespace nb::literals;

namespace {

using collision::BodyChecker;
using collision::Clearances;
using collision::Counts;
using collision::SampleView;

using InArray = nb::ndarray<const double, nb::c_contig, nb::device::cpu>;

// Obstacle table columns: kind (0 box, 1 vertical cylinder), then box lo xyz, hi xyz / cylinder x, y, radius, z_min,
// z_max, unused.
constexpr size_t OBSTACLE_COLUMNS = 7;

void require(bool ok, const std::string &what) {
  if (!ok) throw nb::value_error(what.c_str());
}

void require_shape(const InArray &a, std::initializer_list<int64_t> shape, const char *name) {
  require(a.ndim() == shape.size(), std::string(name) + ": wrong number of dimensions");
  size_t i = 0;
  for (int64_t want : shape) {
    require(want < 0 || static_cast<int64_t>(a.shape(i)) == want, std::string(name) + ": wrong shape");
    ++i;
  }
}

template <typename T>
nb::ndarray<nb::numpy, T> to_numpy(std::vector<T> &&values, std::initializer_list<size_t> shape) {
  auto *heap = new std::vector<T>(std::move(values));
  nb::capsule owner(heap, [](void *p) noexcept { delete static_cast<std::vector<T> *>(p); });
  return nb::ndarray<nb::numpy, T>(heap->data(), shape, owner);
}

collision::Vec3 vec(const double *p) { return collision::Vec3{p[0], p[1], p[2]}; }

std::vector<collision::Obstacle> obstacles_of(const InArray &table) {
  require_shape(table, {-1, static_cast<int64_t>(OBSTACLE_COLUMNS)}, "obstacles");
  std::vector<collision::Obstacle> out;
  for (size_t k = 0; k < table.shape(0); ++k) {
    const double *row = table.data() + k * OBSTACLE_COLUMNS;
    if (row[0] == 0.0) {
      out.push_back(collision::make_box(vec(row + 1), vec(row + 4)));
    } else if (row[0] == 1.0) {
      out.push_back(collision::make_cylinder(row[1], row[2], row[3], row[4], row[5]));
    } else {
      throw nb::value_error("obstacles: kind must be 0 (box) or 1 (vertical cylinder)");
    }
  }
  return out;
}

BodyChecker make_checker(const InArray &obstacles, const InArray &capsule_radii, const std::vector<int64_t> &parts,
                         const std::vector<int64_t> &links, const std::vector<bool> &contact_only,
                         const std::vector<int64_t> &counts, const std::vector<InArray> &points,
                         const std::vector<InArray> &centres, const std::vector<InArray> &radii,
                         const InArray &mount_rotation, const InArray &mount_offset, int part_count, int jaw_count,
                         int link_count, double prune_slack, double escape_tolerance) {
  require_shape(capsule_radii, {-1}, "capsule_radii");
  require_shape(mount_rotation, {3, 3}, "mount_rotation");
  require_shape(mount_offset, {3}, "mount_offset");
  const size_t g = parts.size();
  require(links.size() == g && contact_only.size() == g && counts.size() == g && points.size() == g &&
              centres.size() == g && radii.size() == g,
          "point groups: every group list needs one entry per group");
  std::vector<collision::PointGroup> groups;
  for (size_t i = 0; i < g; ++i) {
    require_shape(points[i], {-1, -1, 3}, "group points");
    const int64_t blocks = static_cast<int64_t>(points[i].shape(0));
    require_shape(centres[i], {blocks, 3}, "group centres");
    require_shape(radii[i], {blocks}, "group radii");
    collision::PointGroup group;
    group.part = static_cast<int>(parts[i]);
    group.link = static_cast<int>(links[i]);
    group.contact_only = contact_only[i];
    group.blocks = static_cast<int>(blocks);
    group.size = static_cast<int>(points[i].shape(1));
    group.count = static_cast<int>(counts[i]);
    const size_t total = points[i].shape(0) * points[i].shape(1);
    for (size_t j = 0; j < total; ++j) group.points.push_back(vec(points[i].data() + 3 * j));
    for (int64_t b = 0; b < blocks; ++b) {
      group.centres.push_back(vec(centres[i].data() + 3 * b));
      group.radii.push_back(radii[i].data()[b]);
    }
    groups.push_back(std::move(group));
  }
  std::vector<double> radius(capsule_radii.data(), capsule_radii.data() + capsule_radii.shape(0));
  try {
    return BodyChecker(obstacles_of(obstacles), std::move(radius), std::move(groups),
                       collision::make_rigid(mount_rotation.data(), mount_offset.data()), part_count, jaw_count,
                       link_count, prune_slack, escape_tolerance);
  } catch (const std::invalid_argument &e) {
    throw nb::value_error(e.what());
  }
}

// One SampleView per sample of the stacked geometry arrays (checked against the checker's sizes).
std::vector<SampleView> views(const BodyChecker &self, const InArray &capsules, const InArray &jaws,
                              const InArray &rotations, const InArray &translations) {
  const int64_t c = self.capsule_count(), j = self.jaw_count(), l = self.link_count();
  require_shape(capsules, {-1, c, 2, 3}, "capsules");
  const int64_t n = static_cast<int64_t>(capsules.shape(0));
  require_shape(jaws, {n, j, 3}, "jaws");
  require_shape(rotations, {n, l, 3, 3}, "rotations");
  require_shape(translations, {n, l, 3}, "translations");
  std::vector<SampleView> out;
  for (int64_t i = 0; i < n; ++i) {
    out.push_back(SampleView{capsules.data() + i * c * 6, jaws.data() + i * j * 3, rotations.data() + i * l * 9,
                             translations.data() + i * l * 3});
  }
  return out;
}

nb::tuple clearances_tuple(Clearances &&c, size_t n, size_t m, const Counts &counts) {
  return nb::make_tuple(to_numpy(std::move(c.values), {n, m}), to_numpy(std::move(c.parts), {n, m}),
                        to_numpy(std::move(c.soft), {n, m}), to_numpy(std::move(c.soft_parts), {n, m}),
                        counts.points, counts.capsules);
}

nb::tuple full(const BodyChecker &self, const InArray &capsules, const InArray &jaws, const InArray &rotations,
               const InArray &translations, double hard_margin) {
  const std::vector<SampleView> samples = views(self, capsules, jaws, rotations, translations);
  const size_t n = samples.size(), m = self.obstacle_count();
  Clearances c(n * m);
  Counts counts;
  {
    nb::gil_scoped_release release;
    for (size_t i = 0; i < n; ++i) self.full(self.place(samples[i]), hard_margin, c, i * m, counts);
  }
  return clearances_tuple(std::move(c), n, m, counts);
}

nb::tuple points_full(const BodyChecker &self, const std::vector<InArray> &capsules,
                      const std::vector<InArray> &contact, const std::vector<InArray> &others, double hard_margin) {
  const int nc = self.capsule_count(), np = self.part_count();
  require(capsules.size() == static_cast<size_t>(nc) && contact.size() == static_cast<size_t>(nc) &&
              others.size() == static_cast<size_t>(np - nc),
          "points_full: one array per capsule (capsules, contact) and per other part");
  require(!capsules.empty(), "points_full: no capsules");
  require_shape(capsules[0], {-1, -1, 3}, "capsule points");
  const int64_t n = static_cast<int64_t>(capsules[0].shape(0));
  for (const InArray &a : capsules) {
    require_shape(a, {n, -1, 3}, "capsule points");
    require(a.shape(1) >= 2, "capsule points: the first two rows are the axis ends");
  }
  for (const InArray &a : contact) require_shape(a, {n, -1, 3}, "contact points");
  for (const InArray &a : others) {
    require_shape(a, {n, -1, 3}, "part points");
    require(a.shape(1) >= 1, "part points: at least one point per part");
  }
  const size_t m = self.obstacle_count();
  Clearances c(static_cast<size_t>(n) * m);
  Counts counts;
  for (int64_t i = 0; i < n; ++i) {
    collision::PlacedSample s;
    s.held.assign(np, {});
    s.contact.assign(nc, {});
    s.has_contact.assign(nc, false);
    auto rows = [i](const InArray &a, size_t from, std::vector<collision::Vec3> &into) {
      const size_t k = a.shape(1);
      for (size_t r = from; r < k; ++r) into.push_back(vec(a.data() + (i * k + r) * 3));
    };
    for (int p = 0; p < nc; ++p) {
      const size_t k = capsules[p].shape(1);
      s.ends.push_back(vec(capsules[p].data() + (i * k) * 3));
      s.ends.push_back(vec(capsules[p].data() + (i * k + 1) * 3));
      rows(capsules[p], 2, s.held[p]);
      s.has_contact[p] = contact[p].shape(1) > 0;
      rows(contact[p], 0, s.contact[p]);
    }
    for (int p = nc; p < np; ++p) rows(others[p - nc], 0, s.held[p]);
    self.full(s, hard_margin, c, static_cast<size_t>(i) * m, counts);
  }
  return clearances_tuple(std::move(c), static_cast<size_t>(n), m, counts);
}

nb::tuple bounded(const BodyChecker &self, const InArray &capsules, const InArray &jaws, const InArray &rotations,
                  const InArray &translations, double hard_margin, const std::optional<InArray> &hard_limit,
                  bool soft) {
  const std::vector<SampleView> samples = views(self, capsules, jaws, rotations, translations);
  const size_t n = samples.size(), m = self.obstacle_count();
  if (hard_limit) require_shape(*hard_limit, {static_cast<int64_t>(n), static_cast<int64_t>(m)}, "hard_limit");
  Clearances c(n * m);
  Counts counts;
  {
    nb::gil_scoped_release release;
    for (size_t i = 0; i < n; ++i) {
      const double *limit = hard_limit ? hard_limit->data() + i * m : nullptr;
      self.bounded(samples[i], hard_margin, limit, soft, c, i * m, counts);
    }
  }
  return clearances_tuple(std::move(c), n, m, counts);
}

nb::tuple path(const BodyChecker &self, const InArray &capsules, const InArray &jaws, const InArray &rotations,
               const InArray &translations, double hard_margin, bool allow_escape, bool prune,
               const std::vector<int64_t> &moving) {
  const std::vector<SampleView> samples = views(self, capsules, jaws, rotations, translations);
  collision::PathResult r;
  try {
    nb::gil_scoped_release release;
    r = self.path(samples, hard_margin, allow_escape, prune, moving);
  } catch (const std::invalid_argument &e) {
    throw nb::value_error(e.what());
  }
  const size_t n = samples.size();
  return nb::make_tuple(to_numpy(std::move(r.clearances), {n}), to_numpy(std::move(r.obstacles), {n}),
                        to_numpy(std::move(r.parts), {n}), r.violation, r.violation_obstacle, r.violation_part,
                        r.violation_clearance, r.counts.points, r.counts.capsules);
}

}  // namespace

void register_collision(nb::module_ &m) {
  nb::class_<BodyChecker>(m, "BodyChecker",
                          "Arm self-collision clearances to the robot body (mcp_server.self_collision kernels).")
      .def(nb::new_(&make_checker), "obstacles"_a, "capsule_radii"_a, "group_parts"_a, "group_links"_a,
           "group_contact_only"_a, "group_counts"_a, "group_points"_a, "group_centres"_a, "group_radii"_a,
           "mount_rotation"_a, "mount_offset"_a, "part_count"_a, "jaw_count"_a, "link_count"_a, "prune_slack"_a,
           "escape_tolerance"_a,
           "Bind the body (obstacle table rows: kind 0 box lo xyz hi xyz / kind 1 cylinder x y radius z_min z_max 0), "
           "the capsule radii, the point groups (LINK_BLOCKS) and the arm mount.")
      .def_prop_ro("obstacle_count", &BodyChecker::obstacle_count)
      .def("full", &full, "capsules"_a, "jaws"_a, "rotations"_a, "translations"_a, "hard_margin"_a,
           "clearance_matrix of stacked sample geometry: (values, parts, soft, soft_parts, points, capsules).")
      .def("points_full", &points_full, "capsules"_a, "contact"_a, "others"_a, "hard_margin"_a,
           "clearances_of stacked part points: (values, parts, soft, soft_parts, points, capsules).")
      .def("bounded", &bounded, "capsules"_a, "jaws"_a, "rotations"_a, "translations"_a, "hard_margin"_a,
           "hard_limit"_a.none(), "soft"_a,
           "bounded_clearances (bounding-sphere pruning): (values, parts, soft, soft_parts, points, capsules).")
      .def("path", &path, "capsules"_a, "jaws"_a, "rotations"_a, "translations"_a, "hard_margin"_a, "allow_escape"_a,
           "prune"_a, "moving"_a,
           "path(): (clearances, obstacles, parts, violation, violation_obstacle, violation_part, "
           "violation_clearance, points, capsules); violation -1 when no sample breaks the hard limit.");
}
