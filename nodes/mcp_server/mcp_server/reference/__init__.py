"""Python reference implementations of the kernels ported to mcp_core_cpp, one module per kernel family.

Each family module (``camera``, ``grasp``, ``ik_reach``, ``collision``, ``frontiers``) holds a ``KERNELS`` registry:
kernel name -> ``ReferenceKernel``. A port registers the Python version of its kernel there (normally a thin call into
the existing mcp_server code it replaces), names the bench/inputs.py function that builds its input cases, and sets the
per-output tolerances. tests/regenerate_core_golden.py runs the reference on those cases and writes the golden that
tests/core_parity.py checks the C++ kernel against.
"""

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

# The kernel families, in the order mcp_core_cpp registers them (src/module.cpp).
FAMILIES = ("camera", "grasp", "ik_reach", "collision", "frontiers")
# Tolerance key used for every output that has no tolerance of its own.
DEFAULT_TOLERANCE_KEY = "default"


@dataclass(frozen=True)
class Tolerance:
    """Allowed difference of a numeric output: |actual - expected| <= atol + rtol * |expected| per element.

    Attributes:
        atol (float): Absolute tolerance.
        rtol (float): Relative tolerance (of the expected value).
    """

    atol: float = 0.0
    rtol: float = 0.0


@dataclass(frozen=True)
class ReferenceKernel:
    """One kernel's Python reference, its bench input cases and its parity tolerances.

    Attributes:
        run (Callable[..., Mapping[str, Any]]): The reference: called with one case's keyword arguments, returns the
            named outputs (numbers, strings, numpy arrays, lists, nested dicts).
        inputs (str): Name of the bench/inputs.py function returning the cases (list of keyword-argument dicts).
        tolerances (Mapping[str, Tolerance]): Per top-level output name; ``"default"`` covers the rest. An output with
            no entry and no default must match exactly.
    """

    run: Callable[..., Mapping[str, Any]]
    inputs: str
    tolerances: Mapping[str, Tolerance] = field(default_factory=dict)
