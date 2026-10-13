// Python bindings of the ik_reach kernel family (submodule mcp_core_cpp.ik_reach): the arm and body models the
// mcp_server side builds once per ArmKinematics (ArmKinematics.core_model) and SelfCollisionChecker (arm_reach.core_body), and the two
// ported kernels point_reach (arm_reach.point_reach) and reach_map_build (reach_map.build).
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "families.hpp"
#include "ik_reach/body.hpp"
#include "ik_reach/kinematics.hpp"
#include "ik_reach/reach.hpp"

namespace nb = nanobind;
using namespace nb::literals;

namespace {

using ik_reach::Joints;
using ik_reach::Vec3;

template <std::size_t N>
std::array<double, N> fixed(const std::vector<double> &values, const char *name) {
  if (values.size() != N) throw std::invalid_argument(std::string(name) + ": expected " + std::to_string(N) + " values");
  std::array<double, N> out{};
  for (std::size_t i = 0; i < N; ++i) out[i] = values[i];
  return out;
}

ik_reach::ArmModel make_arm(const std::vector<double> &link_fixed, const std::vector<double> &link_axis,
                            const std::vector<bool> &link_moves, const std::vector<int> &joint_link,
                            const std::vector<double> &lower, const std::vector<double> &upper, double margin,
                            const std::vector<double> &offsets, const std::vector<double> &tool_offset,
                            double max_reach_m, const std::vector<double> &pan_axis_xy) {
  const std::size_t n = link_moves.size();
  if (link_fixed.size() != n * 16 || link_axis.size() != n * 3)
    throw std::invalid_argument("ArmModel: link_fixed needs 16 and link_axis 3 values per link");
  ik_reach::ArmParams p;
  for (std::size_t i = 0; i < n; ++i) {
    ik_reach::Mat4 m{};
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) m[r][c] = link_fixed[i * 16 + r * 4 + c];
    p.link_fixed.push_back(m);
    p.link_axis.push_back({link_axis[i * 3], link_axis[i * 3 + 1], link_axis[i * 3 + 2]});
  }
  p.link_moves = link_moves;
  if (joint_link.size() != ik_reach::CHAIN_JOINTS) throw std::invalid_argument("ArmModel: joint_link needs 5 links");
  for (int j = 0; j < ik_reach::CHAIN_JOINTS; ++j) p.joint_link[j] = joint_link[j];
  p.lower = fixed<5>(lower, "lower");
  p.upper = fixed<5>(upper, "upper");
  p.margin = margin;
  p.offsets = fixed<5>(offsets, "offsets");
  p.tool_offset = fixed<3>(tool_offset, "tool_offset");
  p.max_reach_m = max_reach_m;
  const auto axis = fixed<2>(pan_axis_xy, "pan_axis_xy");
  p.pan_axis_x = axis[0];
  p.pan_axis_y = axis[1];
  return ik_reach::ArmModel(std::move(p));
}

using CapsuleSpec = std::tuple<int, int, double>;
using GroupSpec = std::tuple<int, int, bool, int, int, std::vector<double>, std::vector<double>, std::vector<double>>;
using ObstacleSpec = std::tuple<std::string, std::vector<double>>;

ik_reach::BodyModel make_body(const std::vector<double> &rotation, const std::vector<double> &offset,
                              const std::vector<double> &jaw_closed_tip, const std::vector<double> &jaw_pivot,
                              const std::vector<double> &jaw_axis, double jaw_sign, double jaw_closed_rad,
                              int tool_frame, int gripper_frame, const std::vector<CapsuleSpec> &capsules,
                              const std::vector<GroupSpec> &groups, const std::vector<ObstacleSpec> &obstacles,
                              double hard_margin_m, double soft_margin_m) {
  ik_reach::BodyParams p;
  const auto rot = fixed<9>(rotation, "rotation");
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) p.rotation[r][c] = rot[r * 3 + c];
  p.offset = fixed<3>(offset, "offset");
  p.jaw_closed_tip = fixed<3>(jaw_closed_tip, "jaw_closed_tip");
  p.jaw_pivot = fixed<3>(jaw_pivot, "jaw_pivot");
  p.jaw_axis = fixed<3>(jaw_axis, "jaw_axis");
  p.jaw_sign = jaw_sign;
  p.jaw_closed_rad = jaw_closed_rad;
  p.tool_frame = tool_frame;
  p.gripper_frame = gripper_frame;
  if (capsules.size() != ik_reach::CAPSULE_COUNT) throw std::invalid_argument("BodyModel: three capsules expected");
  for (std::size_t c = 0; c < capsules.size(); ++c)
    p.capsules[c] = {std::get<0>(capsules[c]), std::get<1>(capsules[c]), std::get<2>(capsules[c])};
  for (const auto &g : groups) {
    ik_reach::BlockGroup bg;
    std::tie(bg.part, bg.frame, bg.contact_only, bg.blocks, bg.size, bg.points, bg.centres, bg.radii) = g;
    p.groups.push_back(std::move(bg));
  }
  for (const auto &[kind, values] : obstacles) {
    ik_reach::Obstacle o;
    if (kind == "box") {
      const auto v = fixed<6>(values, "box");
      o.box = true;
      o.lo = {v[0], v[1], v[2]};
      o.hi = {v[3], v[4], v[5]};
    } else if (kind == "cylinder") {
      const auto v = fixed<5>(values, "cylinder");
      o.box = false;
      o.cx = v[0];
      o.cy = v[1];
      o.radius = v[2];
      o.z_min = v[3];
      o.z_max = v[4];
    } else {
      throw std::invalid_argument("BodyModel: obstacle kind must be 'box' or 'cylinder', not " + kind);
    }
    p.obstacles.push_back(o);
  }
  p.hard_margin_m = hard_margin_m;
  p.soft_margin_m = soft_margin_m;
  return ik_reach::BodyModel(std::move(p));
}

nb::dict point_reach(ik_reach::ArmModel &arm, const ik_reach::BodyModel &body, const std::vector<double> &target,
                     const std::vector<double> &pitches, const std::vector<double> &seed_urdf,
                     double last_heading_bias) {
  arm.last_heading_bias = last_heading_bias;
  const long solves_before = arm.solve_count;
  const auto result = ik_reach::point_reach(arm, body, fixed<3>(target, "target"), pitches, fixed<5>(seed_urdf, "seed"));
  nb::list solutions;
  for (const auto &sol : result.solutions) {
    if (sol)
      solutions.append(nb::cast(std::vector<double>(sol->begin(), sol->end())));
    else
      solutions.append(nb::none());
  }
  nb::list closest;
  for (const auto &[value, obstacle, part] : result.closest) closest.append(nb::make_tuple(value, obstacle, part));
  nb::dict out;
  out["solutions"] = solutions;
  out["closest"] = closest;
  out["last_heading_bias"] = arm.last_heading_bias;
  out["solves"] = arm.solve_count - solves_before;
  return out;
}

nb::dict reach_map_build(const ik_reach::ArmModel &arm, const ik_reach::BodyModel &body,
                         const std::vector<std::vector<double>> &joint_grids, const std::vector<int> &planar_samples,
                         const std::vector<double> &pan_axis, double pan_sign, const std::vector<double> &headings_deg,
                         const std::vector<double> &radii_m, const std::vector<double> &heights_m,
                         const std::vector<double> &pitches_deg, double r_step_m, double z_step_m, double pan_lo_deg,
                         double pan_hi_deg, int samples_per_cell, double pitch_half_spacing_deg) {
  ik_reach::ReachMapGrid grid;
  if (joint_grids.size() != 3 || planar_samples.size() != 3)
    throw std::invalid_argument("reach_map_build: three planar joints expected");
  for (int j = 0; j < 3; ++j) {
    if (joint_grids[j].size() != static_cast<std::size_t>(planar_samples[j]) || planar_samples[j] < 1)
      throw std::invalid_argument("reach_map_build: joint grid sizes must equal planar_samples");
    grid.joint_grids[j] = joint_grids[j];
    grid.planar_samples[j] = planar_samples[j];
  }
  if (pitches_deg.empty() || pitches_deg.size() > 16)
    throw std::invalid_argument("reach_map_build: 1..16 pitches (uint16 mask)");
  grid.pan_axis = fixed<3>(pan_axis, "pan_axis");
  grid.pan_sign = pan_sign;
  grid.headings_deg = headings_deg;
  grid.radii_m = radii_m;
  grid.heights_m = heights_m;
  grid.pitches_deg = pitches_deg;
  grid.r_step_m = r_step_m;
  grid.z_step_m = z_step_m;
  grid.pan_lo_deg = pan_lo_deg;
  grid.pan_hi_deg = pan_hi_deg;
  grid.samples_per_cell = samples_per_cell;
  grid.pitch_half_spacing_deg = pitch_half_spacing_deg;
  ik_reach::ReachMapResult result;
  {
    nb::gil_scoped_release release;
    result = ik_reach::reach_map_build(arm, body, grid);
  }
  const std::size_t shape[3] = {headings_deg.size(), radii_m.size(), heights_m.size()};
  auto *mask = new std::uint16_t[result.mask.size()];
  std::copy(result.mask.begin(), result.mask.end(), mask);
  nb::capsule mask_owner(mask, [](void *p) noexcept { delete[] static_cast<std::uint16_t *>(p); });
  auto *best = new double[result.best.size()];
  std::copy(result.best.begin(), result.best.end(), best);
  nb::capsule best_owner(best, [](void *p) noexcept { delete[] static_cast<double *>(p); });
  nb::dict out;
  out["mask"] = nb::ndarray<nb::numpy, std::uint16_t, nb::ndim<3>>(mask, 3, shape, mask_owner);
  out["best"] = nb::ndarray<nb::numpy, double, nb::ndim<3>>(best, 3, shape, best_owner);
  out["binned"] = result.binned;
  out["checked"] = result.checked;
  out["clear"] = result.clear;
  out["checked_per_heading"] = result.checked_per_heading;
  return out;
}

}  // namespace

void register_ik_reach(nb::module_ &m) {
  nb::class_<ik_reach::ArmModel>(m, "ArmModel", "Forward kinematics and IK of mcp_server.ik.ArmKinematics")
      .def(nb::new_(&make_arm), "link_fixed"_a, "link_axis"_a, "link_moves"_a, "joint_link"_a, "lower"_a, "upper"_a,
           "margin"_a, "offsets"_a, "tool_offset"_a, "max_reach_m"_a, "pan_axis_xy"_a,
           "Chain links (row-major 4x4 matrix at angle 0, axis, revolute) and the ArmKinematics parameters.")
      .def(
          "inverse",
          [](ik_reach::ArmModel &arm, double x, double y, double z, std::optional<double> pitch,
             const std::vector<double> &seed_urdf, const std::vector<double> &extra_offset,
             double heading_bias) -> std::optional<std::vector<double>> {
            const auto sol = arm.inverse(x, y, z, pitch, fixed<5>(seed_urdf, "seed"),
                                         fixed<3>(extra_offset, "extra_offset"), heading_bias);
            if (!sol) return std::nullopt;
            return std::vector<double>(sol->begin(), sol->end());
          },
          "x"_a, "y"_a, "z"_a, "pitch"_a.none(), "seed_urdf"_a, "extra_offset"_a, "heading_bias"_a,
          "ArmKinematics.inverse: measured joints, or None when unreachable.")
      .def_rw("last_heading_bias", &ik_reach::ArmModel::last_heading_bias)
      .def_ro("solve_count", &ik_reach::ArmModel::solve_count);
  nb::class_<ik_reach::BodyModel>(m, "BodyModel", "Body obstacles, mount, jaw and link point blocks of the checker")
      .def(nb::new_(&make_body), "rotation"_a, "offset"_a, "jaw_closed_tip"_a, "jaw_pivot"_a, "jaw_axis"_a,
           "jaw_sign"_a, "jaw_closed_rad"_a, "tool_frame"_a, "gripper_frame"_a, "capsules"_a, "groups"_a,
           "obstacles"_a, "hard_margin_m"_a, "soft_margin_m"_a,
           "SelfCollisionChecker geometry: mount rotation (row-major 3x3) and offset, jaw model, frames, capsules, "
           "point block groups and obstacles ('box': lo + hi, 'cylinder': x, y, radius, z_min, z_max).")
      .def(
          "closest",
          [](const ik_reach::BodyModel &body, const ik_reach::ArmModel &arm, const std::vector<double> &measured) {
            double value;
            int obstacle, part;
            body.closest(arm, fixed<5>(measured, "joints"), value, obstacle, part);
            return std::make_tuple(value, obstacle, part);
          },
          "arm"_a, "measured"_a, "SelfCollisionChecker.closest of one configuration: (clearance, obstacle, part).")
      .def(
          "body_clearance",
          [](const ik_reach::BodyModel &body, const ik_reach::ArmModel &arm, const std::vector<double> &measured) {
            return body.body_clearance(arm, fixed<5>(measured, "joints"));
          },
          "arm"_a, "measured"_a, "reach_map.body_clearance of one configuration.");
  m.def("point_reach", &point_reach, "arm"_a, "body"_a, "target"_a, "pitches"_a, "seed_urdf"_a,
        "last_heading_bias"_a,
        "arm_reach.point_reach compute: {solutions (measured per pitch or None), closest ((clearance, obstacle, "
        "part) per solution), last_heading_bias, solves}.");
  m.def("reach_map_build", &reach_map_build, "arm"_a, "body"_a, "joint_grids"_a, "planar_samples"_a, "pan_axis"_a,
        "pan_sign"_a, "headings_deg"_a, "radii_m"_a, "heights_m"_a, "pitches_deg"_a, "r_step_m"_a, "z_step_m"_a,
        "pan_lo_deg"_a, "pan_hi_deg"_a, "samples_per_cell"_a, "pitch_half_spacing_deg"_a,
        "reach_map.build compute: {mask (uint16 [H, R, Z]), best (float64, NaN where nothing is clear), binned, "
        "checked, clear, checked_per_heading}.");
}
