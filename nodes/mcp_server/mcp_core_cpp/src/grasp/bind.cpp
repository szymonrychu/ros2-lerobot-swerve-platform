// Python bindings of the grasp kernel family (submodule mcp_core_cpp.grasp).
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>

#include "families.hpp"
#include "solve_line.hpp"

namespace nb = nanobind;
using namespace nb::literals;

void register_grasp(nb::module_ &m) {
  m.def("solve_line", &grasp::solve_line, "geometry"_a, "label"_a, "start"_a.none(), "end"_a, "pitch"_a,
        "seed"_a, "extra"_a.none(), "heading_bias"_a, "params"_a, "budget_limit"_a.none(), "budget_used"_a,
        "cache"_a.none() = nb::none(),
        "Waypoint IK of the grasp planner: GraspPlanner.line from start to end (start None: GraspPlanner.solve_point "
        "at end), on the kinematics of mcp_server.grasp.solver_geometry. Returns samples, problem, solves (ikpy runs "
        "counted against the IK budget) and heading_bias (ArmKinematics.last_heading_bias after the call). cache is "
        "ArmKinematics.solutions (shared LRU of IK answers); None uses a fresh one.");
}
