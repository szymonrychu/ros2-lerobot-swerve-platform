// Register functions of the kernel families: each family's src/<family>/bind.cpp defines its register_<family>,
// which adds that family's kernels to the submodule mcp_core_cpp.<family> it is given.
#pragma once

#include <nanobind/nanobind.h>

void register_camera(nanobind::module_ &m);
void register_grasp(nanobind::module_ &m);
void register_ik_reach(nanobind::module_ &m);
void register_collision(nanobind::module_ &m);
void register_frontiers(nanobind::module_ &m);
