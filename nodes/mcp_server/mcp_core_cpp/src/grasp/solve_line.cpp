#include "solve_line.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <nanobind/stl/string.h>

#include "arm_ik.hpp"
#include "py_format.hpp"

namespace nb = nanobind;

namespace grasp {
namespace {

constexpr double LIMIT_EPS = 1e-9;  // ArmKinematics.within_limits slack

double param(const nb::dict &params, const char *name) {
  if (!params.contains(name)) throw std::invalid_argument(std::string("solve_line: params lack ") + name);
  return nb::cast<double>(params[name]);
}

// GraspPlanner.pose_problem: limit margin, then the shoulder stall; empty when the pose is fine.
std::string pose_problem(const GeometryModel &model, const std::string &label, const Joints &joints,
                         double stall_lift, double stretched_elbow) {
  for (std::size_t j = 0; j < joints.size(); ++j) {
    const double urdf = joints[j] + model.offsets[j];
    if (!(model.limit_lo[j] + model.margin - LIMIT_EPS <= urdf && urdf <= model.limit_hi[j] - model.margin + LIMIT_EPS))
      return format("%s: joints outside the limits minus the %s rad margin", label.c_str(),
                    py_repr(model.margin).c_str());
  }
  const double lift = joints[static_cast<std::size_t>(model.lift)];
  const double elbow = joints[static_cast<std::size_t>(model.elbow)];
  if (lift > stall_lift && elbow <= stretched_elbow)
    return format("%s: stretched arm (elbow_flex %.2f rad) with shoulder_lift %.2f rad above %s rad would stall the shoulder",
                  label.c_str(), elbow, lift, py_repr(stall_lift).c_str());
  return {};
}

}  // namespace

nb::dict solve_line(const nb::dict &geometry, const std::string &label,
                    const std::optional<std::array<double, 3>> &start, const std::array<double, 3> &end,
                    double pitch, const nb::dict &seed, const std::optional<std::array<double, 3>> &extra,
                    double heading_bias, const nb::dict &params, std::optional<std::int64_t> budget_limit,
                    std::int64_t budget_used, nb::object cache) {
  const GeometryModel model = GeometryModel::parse(geometry);
  if (cache.is_none()) cache = nb::module_::import_("collections").attr("OrderedDict")();
  ArmIk ik(model, cache);
  ik.last_heading_bias = heading_bias;
  const double step = param(params, "interpolation_step_m");
  const double max_jump = param(params, "max_joint_jump_rad");
  const double stall_lift = param(params, "stall_shoulder_lift_rad");
  const double stretched_elbow = param(params, "stretched_elbow_max_rad");
  Joints prev(model.joints.size());
  for (std::size_t j = 0; j < prev.size(); ++j) {
    const char *name = model.joints[j].c_str();
    if (!seed.contains(name)) throw std::invalid_argument(std::string("solve_line: seed lacks ") + name);
    prev[j] = nb::cast<double>(seed[name]);
  }
  std::vector<std::array<double, 3>> points;
  if (start) {
    const std::array<double, 3> diff{end[0] - (*start)[0], end[1] - (*start)[1], end[2] - (*start)[2]};
    const double length = std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);
    const long n = std::max(1L, static_cast<long>(std::ceil(length / step)));
    for (long i = 1; i <= n; ++i) {
      std::array<double, 3> p{};
      for (std::size_t k = 0; k < 3; ++k) p[k] = (*start)[k] + diff[k] * static_cast<double>(i) / static_cast<double>(n);
      points.push_back(p);
    }
  } else {
    points.push_back(end);
  }
  std::optional<Vec3> offset;
  if (extra) offset = Vec3{(*extra)[0], (*extra)[1], (*extra)[2]};
  std::vector<Joints> samples;
  std::string problem;
  for (const auto &point : points) {
    if (budget_limit && budget_used + ik.solves >= *budget_limit) {
      problem = format("%s: IK budget of %lld solves spent (planning effort cap)", label.c_str(),
                       static_cast<long long>(*budget_limit));
      break;
    }
    Joints sol;
    std::string error;
    if (!ik.inverse({point[0], point[1], point[2]}, pitch, prev, offset, ik.last_heading_bias, sol, error)) {
      problem = label + " unreachable: " + error;
      break;
    }
    problem = pose_problem(model, label, sol, stall_lift, stretched_elbow);
    if (!problem.empty()) break;
    if (start) {
      double jump = 0.0;
      for (std::size_t j = 0; j < sol.size(); ++j) jump = std::max(jump, std::abs(sol[j] - prev[j]));
      if (jump > max_jump) {
        problem = format(
            "%s: joint jump %.2f rad between straight-line samples at (%.3f, %.3f, %.3f) exceeds max_joint_jump_rad %s",
            label.c_str(), jump, point[0], point[1], point[2], py_repr(max_jump).c_str());
        break;
      }
    }
    samples.push_back(sol);
    prev = sol;
  }
  nb::list out_samples;
  if (problem.empty()) {
    for (const Joints &s : samples) {
      nb::dict d;
      for (std::size_t j = 0; j < s.size(); ++j) d[model.joints[j].c_str()] = s[j];
      out_samples.append(d);
    }
  }
  nb::dict out;
  out["samples"] = out_samples;
  out["problem"] = problem.empty() ? nb::none() : nb::cast(problem);
  out["solves"] = ik.solves;
  out["heading_bias"] = ik.last_heading_bias;
  return out;
}

}  // namespace grasp
