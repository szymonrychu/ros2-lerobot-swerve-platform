"""Python references of the collision kernel family (registry read by tests/regenerate_core_golden.py)."""

from mcp_server.reference import ReferenceKernel

# Kernel name -> reference; a port of a collision kernel to mcp_core_cpp.collision adds its entry here.
KERNELS: dict[str, ReferenceKernel] = {}
