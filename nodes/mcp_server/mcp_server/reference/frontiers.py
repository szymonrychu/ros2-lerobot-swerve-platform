"""Python references of the frontiers kernel family (registry read by tests/regenerate_core_golden.py)."""

from mcp_server.reference import ReferenceKernel

# Kernel name -> reference; a port of a frontiers kernel to mcp_core_cpp.frontiers adds its entry here.
KERNELS: dict[str, ReferenceKernel] = {}
