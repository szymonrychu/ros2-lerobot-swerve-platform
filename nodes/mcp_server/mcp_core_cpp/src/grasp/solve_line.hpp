// mcp_core_cpp.grasp.solve_line: the waypoint IK of the grasp planner (GraspPlanner.solve_point when start is None,
// GraspPlanner.line otherwise), with its pose checks, joint-jump check, IK budget and reason strings, solved by the one
// arm IK of mcp_core_cpp (ik_reach::ArmModel, on ik_reach::trf::least_squares). The Python reference is
// mcp_server/reference/grasp.py solve_line_reference.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include <nanobind/nanobind.h>

#include "ik_reach/kinematics.hpp"

namespace grasp {

// Returns {"samples": [joint dict, ...] (empty on a problem), "problem": str | None, "solves": int (IK runs of this
// call, cache hits excluded), "heading_bias": float}. arm is ArmKinematics.core_model(); its solution cache and
// last_heading_bias carry over between calls as ArmKinematics' do.
nanobind::dict solve_line(ik_reach::ArmModel &arm, const std::string &label,
                          const std::optional<std::array<double, 3>> &start, const std::array<double, 3> &end,
                          double pitch, const nanobind::dict &seed,
                          const std::optional<std::array<double, 3>> &extra, double heading_bias,
                          const nanobind::dict &params, std::optional<std::int64_t> budget_limit,
                          std::int64_t budget_used);

}  // namespace grasp
