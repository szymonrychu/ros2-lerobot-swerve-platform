// mcp_core_cpp module definition: version, build_info() and one submodule per kernel family, filled by that family's
// register_<family> (src/<family>/bind.cpp).
#include <string>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "families.hpp"

namespace nb = nanobind;

namespace {

using RegisterFn = void (*)(nb::module_ &);

struct Family {
  const char *name;
  RegisterFn register_fn;
};

// Registration order; CMakeLists.txt MCP_CORE_FAMILIES lists the same directories.
const Family FAMILIES[] = {
    {"camera", register_camera},
    {"grasp", register_grasp},
    {"ik_reach", register_ik_reach},
    {"collision", register_collision},
    {"frontiers", register_frontiers},
};

std::string compiler() {
#if defined(__clang__)
  return std::string("clang ") + __clang_version__;
#elif defined(__GNUC__)
  return std::string("gcc ") + __VERSION__;
#else
  return "unknown";
#endif
}

}  // namespace

NB_MODULE(mcp_core_cpp, m) {
  m.doc() = "C++ compute kernels of mcp_server, one submodule per kernel family";
  m.attr("__version__") = MCP_CORE_VERSION;

  std::vector<std::string> registered;
  for (const Family &family : FAMILIES) {
    nb::module_ sub = m.def_submodule(family.name);
    family.register_fn(sub);
    registered.emplace_back(family.name);
  }

  m.def(
      "build_info",
      [registered]() {
        nb::dict info;
        info["version"] = MCP_CORE_VERSION;
        info["binding"] = std::string("nanobind ") + MCP_CORE_NANOBIND_VERSION;
        info["compiler"] = compiler();
        info["build_type"] = MCP_CORE_BUILD_TYPE;
        info["families"] = registered;
        return info;
      },
      "Build facts: version, binding library, compiler, build type and the registered families in order.");
}
