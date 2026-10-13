"""Python references of the grasp kernel family (registry read by tests/regenerate_core_golden.py)."""

from mcp_server.reference import ReferenceKernel

# Kernel name -> reference; a port of a grasp kernel to mcp_core_cpp.grasp adds its entry here.
KERNELS: dict[str, ReferenceKernel] = {}
