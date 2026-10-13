"""The mcp_core_cpp extension is built into this venv and registers every kernel family.

mcp_core_cpp is a uv path dependency of mcp_server (nodes/mcp_server/mcp_core_cpp, scikit-build-core + nanobind), so
``uv sync`` compiles it; these tests fail when the extension is missing, stale or a family register call was dropped.
"""

import importlib.metadata
from types import ModuleType

import mcp_core_cpp
from mcp_server.reference import FAMILIES

INFO_KEYS = {"version", "binding", "compiler", "build_type", "families"}


def test_version_matches_installed_distribution() -> None:
    """__version__ comes from the package version (pyproject.toml), the one uv.lock records."""
    assert mcp_core_cpp.__version__ == importlib.metadata.version("mcp-core-cpp")


def test_build_info_describes_the_build() -> None:
    """build_info() names the version, the binding library, the compiler, the build type and the families."""
    info = mcp_core_cpp.build_info()
    assert set(info) == INFO_KEYS
    assert info["version"] == mcp_core_cpp.__version__
    assert info["binding"].startswith("nanobind ")
    assert info["compiler"]
    assert info["build_type"]


def test_every_family_register_function_ran() -> None:
    """module.cpp calls register_<family> for the five families, each with its own submodule, in FAMILIES order."""
    assert FAMILIES == ("camera", "grasp", "ik_reach", "collision", "frontiers")
    assert mcp_core_cpp.build_info()["families"] == list(FAMILIES)
    for family in FAMILIES:
        assert isinstance(getattr(mcp_core_cpp, family), ModuleType), family
