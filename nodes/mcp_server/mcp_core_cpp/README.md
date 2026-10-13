# mcp_core_cpp

C++ compute kernels of `mcp_server`, loaded in-process as the Python extension module `mcp_core_cpp`. mcp_server stays
a Python node and keeps every tool entry point; a measured hot kernel (see `../bench/REPORT.md`) moves here one family
at a time, and a golden parity test proves the C++ version equal to the Python one within stated tolerances.

- **Binding: nanobind** (3.1.0), chosen over pybind11 for smaller binaries, faster compiles on the Pi and lower call
  overhead; numpy arrays cross the boundary as `nb::ndarray` without copies.
- **Build: scikit-build-core** (1.1.0) driving CMake. `mcp_server` depends on it as a uv path source
  (`[tool.uv.sources] mcp-core-cpp = { path = "mcp_core_cpp" }`), so `uv sync` in `nodes/mcp_server` compiles it into
  the node's venv, on the Mac and on the Pi (the deploy's `uv sync --frozen --no-dev`).
- **Toolchain:** a C++17 compiler and the Python headers (`python3-dev`, present on the Pi). cmake (>= 3.26) and ninja
  are not needed on the machine: when none of a usable version is installed, scikit-build-core adds the PyPI `cmake` /
  `ninja` wheels to the build requirements (the Pi uses its system cmake 3.28; the Mac the PyPI wheels).
- **Rebuilds:** `[tool.uv] cache-keys` lists `pyproject.toml`, `CMakeLists.txt` and `src/**/*.{cpp,hpp}`, so `uv sync`
  rebuilds after any C++ change. The Pi deploy only runs `uv sync` when `nodes/mcp_server/pyproject.toml` or `uv.lock`
  changed, so **every C++ change bumps `version` in `pyproject.toml` and re-runs `uv lock`** (uv.lock records the
  version). `mcp_core_cpp.__version__` / `build_info()` on the robot then show which build is loaded.

## Layout

```
mcp_core_cpp/
  pyproject.toml          package metadata, build requirements, uv cache keys
  CMakeLists.txt          one nanobind module; collects src/module.cpp + src/<family>/*.cpp per family
  src/
    families.hpp          declarations of register_<family>(nanobind::module_&)
    module.cpp            NB_MODULE: __version__, build_info(), one submodule per family, calls register_<family>
    camera/bind.cpp       register_camera     -> mcp_core_cpp.camera
    grasp/bind.cpp        register_grasp      -> mcp_core_cpp.grasp (solve_line: waypoint IK on ik_reach::ArmModel)
    ik_reach/bind.cpp     register_ik_reach   -> mcp_core_cpp.ik_reach
    ik_reach/trf.hpp      the one least-squares solver (scipy TRF port), shared by every IK kernel
    ik_reach/kinematics.* the one arm IK (ik_reach::ArmModel), shared by ik_reach and grasp
    collision/bind.cpp    register_collision  -> mcp_core_cpp.collision
    frontiers/bind.cpp    register_frontiers  -> mcp_core_cpp.frontiers
```

Python API:

| Name | What it is |
|---|---|
| `mcp_core_cpp.__version__` | package version (`pyproject.toml`, the one uv.lock records) |
| `mcp_core_cpp.build_info()` | `{"version", "binding" ("nanobind 3.1.0"), "compiler", "build_type", "families"}`; `families` lists the families whose register function ran, in order |
| `mcp_core_cpp.<family>` | submodule holding that family's kernels |

The family list appears in three places that must agree: `MCP_CORE_FAMILIES` in `CMakeLists.txt`, `FAMILIES` in
`src/module.cpp` and `FAMILIES` in `mcp_server/reference/__init__.py` (`tests/test_core_build.py` checks the last two).

## Porting a kernel

A port touches only its own family: `src/<family>/`, the Python module(s) whose kernel it ports,
`mcp_server/reference/<family>.py`, its goldens, its parity test and `docs/<family>.md`.

The golden must always come from Python, never from the C++ kernel. So the original Python body is **moved** (not
called) into `mcp_server/reference/<family>.py`, and the Python entry point then calls the C++ kernel. Regenerating a
golden after the switch still runs the original Python code, never C++ against C++.

1. **Pick it from the profile.** Only kernels with verdict `port` in `../bench/report.json` (see `../bench/REPORT.md`)
   are ported. List every kernel of the family in `docs/<family>.md` with its median ms and either `ported` or
   `below threshold, not ported`.
2. **Move the Python body into the reference** `mcp_server/reference/<family>.py` and register it:

   ```python
   KERNELS: dict[str, ReferenceKernel] = {
       "plan_segments": ReferenceKernel(
           run=plan_segments_reference,       # keyword args -> dict of named outputs (arrays, numbers, strings)
           inputs="plan_segments_cases",      # function in bench/inputs.py returning list[dict] of keyword args
           tolerances={"segments": Tolerance(atol=1e-9), "default": Tolerance(atol=1e-12, rtol=1e-12)},
       ),
   }
   ```

   `plan_segments_reference` holds the original Python implementation, moved verbatim from the mcp_server module; the
   reference is used only by the golden generator. Add the input function to `bench/inputs.py` next to the bench's own
   inputs (deployed config, recorded grasp cases, fixed-seed data).
3. **Write the golden:** `uv run python -m tests.regenerate_core_golden --family grasp --kernel plan_segments` writes
   `tests/data/core_golden/grasp/plan_segments.json`. Commit it; regenerate only when the reference changes by design.
4. **Write the parity test first** (`tests/test_core_parity_<family>.py`) and see it fail (the kernel does not exist
   yet):

   ```python
   golden = core_parity.load_golden("grasp", "plan_segments")
   core_parity.assert_parity(golden, mcp_core_cpp.grasp.plan_segments)
   ```

   The kernel is called with each case's keyword arguments and returns a mapping of the same named outputs. The test
   also asserts `docs/<family>.md` matches `report.json` for the family.
5. **Implement** in `src/<family>/<kernel>.cpp` (+ `.hpp`), bind it in `src/<family>/bind.cpp` with
   `m.def("plan_segments", ..., "a"_a, ...)`. New `.cpp` files in the family directory are picked up by CMake's glob
   (`CONFIGURE_DEPENDS`); no CMakeLists change.
6. **Bump `version`** in `pyproject.toml`, run `uv lock` and `uv sync` in `nodes/mcp_server`.
7. **Switch the Python entry point** (same signature) to call the C++ kernel; the existing tool tests keep exercising
   it, and the bench re-run shows the kernel faster than its Python median.

## Parity workflow

- `tests/core_parity.py`: `load_golden(family, kernel)`, `check_parity(golden, kernel) -> ParityReport`,
  `assert_parity(golden, kernel)`, `compare_case(expected, actual, tolerances, case)`, `write_golden(golden, root)`.
- Goldens are JSON: format version, family, kernel, tolerances, and cases of `inputs` / `outputs`. numpy arrays keep
  dtype and shape (`{"__ndarray__": ..., "dtype": ..., "shape": ...}`) and load back as arrays.
- Comparison: nested mappings by key (missing / extra keys fail), lists of records element by element, numeric
  values and arrays per element with `|actual - expected| <= atol + rtol * |expected|` (NaN equals NaN only), strings,
  bools and None exactly. The tolerance is chosen by top-level output name, then `"default"`, else exact.
- Report: each compared array yields its worst element (the largest fraction of its tolerance used, with case, path,
  index, both values, error and the count outside tolerance). A shape, type, missing or extra difference is a
  structural failure. `ParityReport.worst` is reported for passing runs too, so the margin stays visible.
- `tests/test_core_parity_harness.py` proves the harness on a toy kernel pair (pass within tolerance, fail outside it
  with the worst element named, shape mismatch, missing output, records, NaN, the regenerate entry point).
- `tests/test_core_build.py` checks the extension imports, its version and that all five register functions ran.
- `tests/test_core_parity_ik_reach.py`: golden parity of `arm_reach.point_reach` and `reach_map.build` (goldens
  from `bench/inputs.py` `point_reach_cases` / `reach_map_cases`), `docs/ik_reach.md` against `bench/report.json`, the
  entry points not running the Python IK, each port faster than its Python median, and the version carrying ik_reach
  (0.2.0) locked in `uv.lock`.
- `tests/test_core_parity_grasp.py`: golden parity of `grasp.solve_line` (111 cases from `bench/inputs.py`
  `grasp_solve_line_cases`, run on `ArmKinematics.core_model()` of the case's geometry), the reference staying pure
  Python, `docs/grasp.md` against `bench/report.json`, the planner entry points calling the kernel (cache, budget),
  exactly one least-squares solver and one arm IK under `src/` (both in `ik_reach`), and the version carrying the
  grasp kernels (0.3.0) locked in `uv.lock`. `tests/test_ik.py::test_heading_refinement_survives_a_repeated_heading`: every pitch
  of a target whose heading refinement repeats a heading ends in a solution or `UnreachableError`.
  `tests/test_reach_map.py::test_reference_build_checks_bounded_batches`: the Python reference build (the golden
  source) body checks at most a batch at a time.
