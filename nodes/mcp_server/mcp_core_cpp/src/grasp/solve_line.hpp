// mcp_core_cpp.grasp.solve_line: the waypoint IK of the grasp planner (GraspPlanner.solve_point when start is None,
// GraspPlanner.line otherwise), with its pose checks, joint-jump check, IK budget and reason strings. The Python
// reference is mcp_server/reference/grasp.py solve_line_reference.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include <nanobind/nanobind.h>

namespace grasp {

// Returns {"samples": [joint dict, ...] (empty on a problem), "problem": str | None, "solves": int,
// "heading_bias": float}. cache is ArmKinematics.solutions (None: a fresh cache for this call).
nanobind::dict solve_line(const nanobind::dict &geometry, const std::string &label,
                          const std::optional<std::array<double, 3>> &start, const std::array<double, 3> &end,
                          double pitch, const nanobind::dict &seed,
                          const std::optional<std::array<double, 3>> &extra, double heading_bias,
                          const nanobind::dict &params, std::optional<std::int64_t> budget_limit,
                          std::int64_t budget_used, nanobind::object cache);

}  // namespace grasp
