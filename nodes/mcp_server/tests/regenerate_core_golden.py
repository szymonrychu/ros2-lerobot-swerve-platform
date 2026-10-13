"""Regenerate the golden of one mcp_core_cpp kernel from its Python reference.

Looks the kernel up in ``mcp_server/reference/<family>.py`` (``KERNELS``), builds its input cases with the named
bench/inputs.py function (the T9 bench inputs: deployed client.yml config, recorded grasp cases, fixed-seed synthetic
data), runs the reference on every case and writes ``tests/data/core_golden/<family>/<kernel>.json`` with the
reference's tolerances. Run it when a kernel is first ported, and again only when the Python reference changes by
design; review the golden diff like code.

    cd nodes/mcp_server && uv run python -m tests.regenerate_core_golden --family grasp --kernel plan_segments
"""

import argparse
import importlib
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import bench.inputs
from mcp_server.reference import FAMILIES, ReferenceKernel
from tests.core_parity import GOLDEN_ROOT, Golden, GoldenCase, write_golden


def reference_kernel(family: str, kernel: str) -> ReferenceKernel:
    """The registered reference of a kernel.

    Args:
        family (str): Kernel family (one of FAMILIES).
        kernel (str): Kernel name in that family's KERNELS.

    Returns:
        ReferenceKernel: The registry entry.

    Raises:
        SystemExit: When the family has no such kernel.
    """
    kernels: Mapping[str, ReferenceKernel] = importlib.import_module(f"mcp_server.reference.{family}").KERNELS
    if kernel not in kernels:
        raise SystemExit(f"{family}: no reference kernel {kernel!r} (registered: {sorted(kernels) or 'none'})")
    return kernels[kernel]


def bench_cases(name: str) -> list[dict[str, Any]]:
    """Input cases from a bench/inputs.py function.

    Args:
        name (str): Function name in bench/inputs.py (ReferenceKernel.inputs).

    Returns:
        list[dict[str, Any]]: Keyword-argument dicts, one per case.

    Raises:
        SystemExit: When bench/inputs.py has no such function or it returns no cases.
    """
    build = getattr(bench.inputs, name, None)
    if build is None:
        raise SystemExit(f"bench/inputs.py has no input function {name!r}")
    cases = list(build())
    if not cases:
        raise SystemExit(f"bench/inputs.py {name}() returned no cases")
    return cases


def build_golden(family: str, kernel: str, reference: ReferenceKernel, cases: Sequence[dict[str, Any]]) -> Golden:
    """Run the reference on every case.

    Args:
        family (str): Kernel family.
        kernel (str): Kernel name.
        reference (ReferenceKernel): The reference and its tolerances.
        cases (Sequence[dict[str, Any]]): Keyword-argument dicts.

    Returns:
        Golden: Cases with the reference outputs.
    """
    return Golden(
        family=family,
        kernel=kernel,
        tolerances=dict(reference.tolerances),
        cases=[GoldenCase(inputs=dict(case), outputs=dict(reference.run(**case))) for case in cases],
    )


def main(argv: Sequence[str] | None = None) -> int:
    """Regenerate one kernel's golden.

    Args:
        argv (Sequence[str] | None): Arguments (default sys.argv[1:]).

    Returns:
        int: Exit status 0.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--family", required=True, choices=FAMILIES)
    parser.add_argument("--kernel", required=True)
    parser.add_argument("--root", type=Path, default=GOLDEN_ROOT, help="golden root (default tests/data/core_golden)")
    args = parser.parse_args(argv)
    reference = reference_kernel(args.family, args.kernel)
    golden = build_golden(args.family, args.kernel, reference, bench_cases(reference.inputs))
    path = write_golden(golden, args.root)
    print(f"wrote {path} ({len(golden.cases)} cases)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
