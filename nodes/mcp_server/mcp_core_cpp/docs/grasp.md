# grasp kernel family

Kernels of the grasp family in `../../bench/report.json` (threshold 5 ms on the dev Mac), and what was ported.
`tests/test_core_parity_grasp.py` checks this table against the report.

| kernel | median ms | status |
|---|---:|---|
| `grasp.plan_auto` | 214.995 | ported |
| `jaw_crop.jaw_crop` | 0.106 | below threshold, not ported |
| `grasp.plan_scoop` | 0.010 | below threshold, not ported |
| `grasp_check.band_verdict` | 0.002 | below threshold, not ported |

## grasp.plan_auto: the waypoint IK in C++

`GraspPlanner.plan` spent about 70 % of its time in the waypoint IK of `realize`: `GraspPlanner.solve_point` (one
waypoint) and `GraspPlanner.line` (a straight tool-point line, one IK per `interpolation_step_m`), each sample an
`ArmKinematics.inverse` on ikpy + scipy `least_squares`. Both entry points keep their signatures and now call
`mcp_core_cpp.grasp.solve_line`:

```python
mcp_core_cpp.grasp.solve_line(
    arm,             # kin.core_model(): mcp_core_cpp.ik_reach.ArmModel, the one C++ arm IK (with its solution cache)
    label, start,    # start None: solve_point at end; else the line start -> end
    end, pitch, seed, extra, heading_bias,
    params,          # mcp_server.grasp.kernel_params(params)
    budget_limit, budget_used,
) -> {"samples": [joint dict, ...], "problem": str | None, "solves": int, "heading_bias": float}
```

| Source | What it is |
|---|---|
| `src/grasp/solve_line.{hpp,cpp}` | `solve_point` / `line` / `pose_problem`: interpolation, IK budget, limit-margin, shoulder-stall and joint-jump checks, the same reason strings |
| `src/grasp/py_format.{hpp,cpp}` | Python-identical `.Nf` formatting and float `repr` for the reasons |
| `src/grasp/bind.cpp` | `register_grasp`: binds `solve_line` |

**One solver.** The waypoint IK is `ik_reach::ArmModel::inverse` (`src/ik_reach/kinematics.{hpp,cpp}`: ikpy's FK and
residual, extra seeds, heading refinement with the secant step, tool-point iterations, the LRU solution cache) on
`ik_reach::trf::least_squares` (`src/ik_reach/trf.hpp`, the step-for-step port of scipy `least_squares(method="trf")`
with ikpy's defaults). The grasp family has no IK or least-squares code of its own; for its reasons `ArmModel::inverse`
takes an optional `std::string *reason` that gets the `UnreachableError` message `ArmKinematics.inverse` raises
(beyond reach, not finite, unreachable within joint limits with the closest miss, tool point not converged).
`tests/test_core_parity_grasp.py::test_mcp_core_cpp_has_one_least_squares_solver` fails if a second least-squares
solver or arm IK is defined anywhere under `src/`.

**Solution cache.** The kernel solves on `ArmKinematics.core_model()`, so it uses that model's C++ cache (same byte
keys and LRU size as `ArmKinematics.solutions`), which lasts as long as the model: `core_model()` rebuilds it when a
parameter it copies changes, and `ArmKinematics.clear_solutions()` clears both caches (a cold plan). The Python
`ArmKinematics.solutions` is not shared with the kernel; `solves` counts the model's IK runs, which the planner adds
to `kin.solve_count` for its `IkBudget`.

**Release at exit.** A planner's kinematics can outlive interpreter shutdown (a module-global planner, or one on the
stack of a daemon thread that was never stopped), and nanobind would then report its `ArmModel` as leaked.
`mcp_server.ik.release_core_models` (registered with `atexit`) drops the model of every live `ArmKinematics`; a
`core_model()` call after it raises `RuntimeError` instead of building a model nobody frees.

The original Python bodies live in `mcp_server/reference/grasp.py` (`solve_line_reference`, run on an ikpy chain
rebuilt by `kinematics_from_geometry` from the case's `mcp_server.grasp.solver_geometry`; the parity test runs the
kernel on that rebuilt kinematics' `core_model()`) and only write the golden `tests/data/core_golden/grasp/solve_line.json`: every
waypoint IK call of the recorded grasp scenarios (`tests/data/grasp_golden.json`) plus the failure paths (beyond
reach, out of the joint limits, spent budget, joint jump, shoulder stall, no tool offset). Inputs come from
`bench/inputs.py` `grasp_solve_line_cases`. The reference must stay pure Python (ikpy and scipy, never an
`mcp_core_cpp` kernel), so a regenerated golden never compares C++ with C++: `test_reference_runs_pure_python`
replays golden cases with every `mcp_core_cpp` family disabled.

Parity: joint samples and heading bias within 1e-9 rad (seen: about 1e-12), reasons identical, solve count within 1
(a fixed-point heading iteration can repeat a query bit-exactly in one implementation and an ulp off in the other,
turning one IK run into a cache hit). Only a pitched IK is ported: the planner always passes a pitch, and the
position-only problem is underdetermined (float noise moves its answer along the null space).

Bench (`inputs.grasp_plan_call(None)`, cold caches, dev Mac): 215.0 ms in Python, 22.0 ms with the C++ waypoint IK
on `ik_reach::ArmModel` (mcp_core_cpp 0.3.0, median of 15). The rest is mostly the robot body check
(`self_collision.path`), a kernel of the collision family.

## Below threshold

`jaw_crop.jaw_crop`, `grasp.plan_scoop` and `grasp_check.band_verdict` stay Python: each is well under the 5 ms
threshold (and `grasp.plan_scoop` exits before any IK for the recorded case).
