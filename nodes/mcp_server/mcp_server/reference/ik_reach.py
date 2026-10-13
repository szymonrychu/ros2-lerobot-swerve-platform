"""Python references of the ik_reach kernel family (registry read by tests/regenerate_core_golden.py)."""

from mcp_server.reference import ReferenceKernel

# Kernel name -> reference; a port of a ik_reach kernel to mcp_core_cpp.ik_reach adds its entry here.
KERNELS: dict[str, ReferenceKernel] = {}
