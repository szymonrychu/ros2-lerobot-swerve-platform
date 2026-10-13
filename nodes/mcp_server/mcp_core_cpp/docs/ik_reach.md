# ik_reach kernel family

Kernels of `mcp_server.ik`, `arm_reach`, `reach_map`, `arm_geometry` and `sag`, as ranked by the bench
(`../../bench/report.json`, dev Mac, threshold 5 ms). Only the kernels above the threshold are ported; the C++ lives in
`../src/ik_reach/` and is registered by `register_ik_reach` as `mcp_core_cpp.ik_reach`.

| Kernel | Median ms | Status |
|---|---|---|
| `reach_map.build` | 1265.95 | ported |
| `arm_reach.point_reach` | 12.61 | ported |
| `ik.inverse` | 1.77 | below threshold, not ported |
| `ik.inverse_flange` | 0.67 | below threshold, not ported |
| `arm_geometry.box_surface_points` | 0.55 | below threshold, not ported |
| `sag.compensate` | 0.19 | below threshold, not ported |
| `sag.deflection` | 0.18 | below threshold, not ported |
| `ik.forward` | 0.02 | below threshold, not ported |
| `ik.link_frames` | 0.02 | below threshold, not ported |

Median ms is the Python median of the report. `tests/test_core_parity_ik_reach.py` checks this table against
`report.json` row for row.

## What runs in C++

- `arm_reach.point_reach` calls `mcp_core_cpp.ik_reach.point_reach`: `ArmKinematics.inverse` for every approach pitch
  (same seeds, heading refinement with its secant step, tool point iteration, acceptance tolerances and an 8192-entry
  solution cache of its own) and `SelfCollisionChecker.closest` of the solutions found. The suggestion text, envelope
  lookup and result dict stay in Python. The kernel writes the kinematics' `last_heading_bias` and `solve_count` back,
  as the Python IK does. `arm_reach.point_solutions` returns the per-pitch IK solutions of the same kernel.
- `reach_map.build` calls `mcp_core_cpp.ik_reach.reach_map_build`: forward sampling of the planar grid, binning to
  (r, z, pitch) cells, the ordering by cell, distance and index, the per-heading candidate choice and the body check of
  every chosen sample (`body_clearance`). Pan geometry, the grid, the key and the meta stay in Python. The GIL is
  released while it runs.
- The solver is a port of what ikpy calls: `scipy.optimize.least_squares` with method `trf` (bounded, `tr_solver`
  exact, `x_scale` 1, ftol = xtol = gtol = 1e-8, `max_nfev` 400) and the bounded 2-point finite difference Jacobian
  (`trf.hpp`); the SVD is a one-sided Jacobi. Forward kinematics multiplies the ikpy link matrices (each link's matrix
  at angle 0, read from ikpy, times its axis rotation).
- The body check mirrors `bounded_clearances`: bounding-sphere bounds per capsule and point block, exact
  golden-section capsule distances and exact point distances only where they can matter.
- The models are built once on the Python side: `ArmKinematics.core_model()` (chain, limits, margin, offsets, tool
  offset; rebuilt when one of them changes) and `arm_reach.core_body(checker)` (mount, jaw, capsules, `LINK_BLOCKS`,
  obstacles, margins; rebuilt when they change). Samples are taken with the gripper closed, as both kernels' Python
  versions do.

## Parity

The original Python bodies are `mcp_server/reference/ik_reach.py` (`point_reach`, `build`), used only by the golden
generator. Every golden case carries its robot model (`robot_inputs`: URDF, offsets, limits, tool point, jaw, mount,
body and margins), so a change of the deployed config does not invalidate the goldens.

```bash
cd nodes/mcp_server
uv run python -m tests.regenerate_core_golden --family ik_reach --kernel point_reach       # 73 cases
uv run python -m tests.regenerate_core_golden --family ik_reach --kernel reach_map_build   # 2 grids
uv run pytest tests/test_core_parity_ik_reach.py
```

- `point_reach`: the bench point, points at the lidar and the deck, out-of-reach points and 64 fixed-seed random points
  (16 with one pitch), two IK seeds, and a point whose pitched refinement repeats a heading. Tolerances: IK solutions
  1e-6 rad; every other output (reachable, collision-free, pitches, slowed, limiting obstacle, link and clearance,
  suggestion) exact. Worst solution difference on the goldens: 1.2e-8 rad.
- `reach_map_build`: the bench grid and a denser one with two samples per cell. Pitch mask and counts exact, clearance
  within 2e-5 m (one float16 step); measured identical.
- A wider sweep of 300 random points and seeds gave identical reach verdicts and results everywhere; the largest
  solution difference, 1.5e-6 rad, was an accepted solve with a non-zero residual (Jacobian condition about 1.7e5)
  where scipy itself stops on its gradient tolerance, so the solution is only defined to about that.

## Speed (dev Mac, bench inputs)

| Kernel | Python median ms | C++ median ms |
|---|---|---|
| `reach_map.build` | 1265.95 | 193.5 |
| `arm_reach.point_reach` (warm IK cache, as benched) | 12.61 | 0.31 |
| `arm_reach.point_reach`, cold, 300 random points | about 130 per point | about 1.3 per point |

`tests/test_core_parity_ik_reach.py` asserts each ported kernel beats both its report median and the Python reference
timed in the same run.
