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
    geometry,        # mcp_server.grasp.solver_geometry(kin): ikpy chain links, limits, offsets, tool offset, ik constants
    label, start,    # start None: solve_point at end; else the line start -> end
    end, pitch, seed, extra, heading_bias,
    params,          # mcp_server.grasp.kernel_params(params)
    budget_limit, budget_used,
    cache=None,      # ArmKinematics.solutions (shared IK answer cache); None: a fresh one
) -> {"samples": [joint dict, ...], "problem": str | None, "solves": int, "heading_bias": float}
```

| Source | What it is |
|---|---|
| `src/grasp/trf.{hpp,cpp}` | step-for-step port of scipy `least_squares(method="trf")` with ikpy's defaults (2-point Jacobian, 1-sided at the bounds, exact trust-region solver with a one-sided Jacobi SVD) |
| `src/grasp/arm_ik.{hpp,cpp}` | `ArmKinematics.inverse` / `inverse_flange` / `solve`: ikpy's FK and residual (position, tool z axis), extra seeds, heading refinement with the secant step, tool-point iterations, the shared LRU cache with the same byte keys |
| `src/grasp/solve_line.{hpp,cpp}` | `solve_point` / `line` / `pose_problem`: interpolation, IK budget, limit-margin, shoulder-stall and joint-jump checks, the same reason strings |
| `src/grasp/py_format.{hpp,cpp}` | Python-identical `.Nf` formatting and float `repr` for the reasons |
| `src/grasp/bind.cpp` | `register_grasp`: binds `solve_line` |

The original Python bodies live in `mcp_server/reference/grasp.py` (`solve_line_reference`, run on an ikpy chain
rebuilt from the same geometry) and only write the golden `tests/data/core_golden/grasp/solve_line.json`: every
waypoint IK call of the recorded grasp scenarios (`tests/data/grasp_golden.json`) plus the failure paths (beyond
reach, out of the joint limits, spent budget, joint jump, shoulder stall, no tool offset). Inputs come from
`bench/inputs.py` `grasp_solve_line_cases`. The reference must stay pure Python (ikpy and scipy, never an
`mcp_core_cpp` kernel), so a regenerated golden never compares C++ with C++: `test_reference_runs_pure_python`
replays golden cases with every `mcp_core_cpp` family disabled.

Parity: joint samples and heading bias within 1e-9 rad (seen: about 1e-12), reasons identical, solve count within 1
(a fixed-point heading iteration can repeat a query bit-exactly in one implementation and an ulp off in the other,
turning one IK run into a cache hit). Only a pitched IK is ported: the planner always passes a pitch, and the
position-only problem is underdetermined (float noise moves its answer along the null space).

Bench (`inputs.grasp_plan_call(None)`, cold cache, dev Mac): 215.0 ms in Python, 78.4 ms with the C++ waypoint IK.
What remains is the robot body check (`self_collision.path`, about 80 % of it), a kernel of the collision family.

## Below threshold

`jaw_crop.jaw_crop`, `grasp.plan_scoop` and `grasp_check.band_verdict` stay Python: each is well under the 5 ms
threshold (and `grasp.plan_scoop` exits before any IK for the recorded case).
