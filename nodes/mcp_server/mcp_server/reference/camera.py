"""Python references of the camera kernel family (registry read by tests/regenerate_core_golden.py)."""

from mcp_server.reference import ReferenceKernel

# Kernel name -> reference; a port of a camera kernel to mcp_core_cpp.camera adds its entry here.
KERNELS: dict[str, ReferenceKernel] = {}
