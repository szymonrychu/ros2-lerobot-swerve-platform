# collision kernel family

Arm self-collision and footprint kernels of mcp_server (`self_collision.py`, `body_model.py`,
`footprint_geometry.py`). Only the kernels the profile (`../../bench/report.json`, threshold 5 ms on the dev Mac)
ranks above threshold are ported; `tests/test_core_parity_collision.py` checks this table against the report.

## Kernels in the report

Median ms is the report's median wall time of the Python kernel (dev Mac, 2026-10-13).

| Kernel | Median ms | Status |
|---|---|---|
| `self_collision.path` | 14.897 | ported |
| `self_collision.pose` | 6.430 | ported |
| `self_collision.geometry` | 2.333 | below threshold, not ported |
| `body_model.signed_distance` | 0.457 | below threshold, not ported |
| `footprint_geometry.shapes` | 0.439 | below threshold, not ported |
| `footprint_geometry.body_outline` | 0.243 | below threshold, not ported |

After the port (same bench, `bench/bench_kernels.py` with families `[collision]`, 31 repetitions, dev Mac):

| Kernel | Python median ms | C++ median ms |
|---|---|---|
| `self_collision.path` | 14.897 | 4.020 (2.3 of it is the forward kinematics of `geometry`, still Python) |
| `self_collision.pose` | 6.430 | 0.629 |

## What runs in C++

`mcp_core_cpp.collision.BodyChecker` (`src/collision/`), built once per `SelfCollisionChecker` from the body
obstacles, the capsule radii, the point blocks (`LINK_BLOCKS`) and the arm mount:

| Method | Python caller | What it computes |
|---|---|---|
| `path(...)` | `SelfCollisionChecker.path` | pruned (or full) clearances of every sample, the escape limits, the gripper-only soft band, the worst soft clearance per sample and the first hard violation |
| `full(...)` | `clearance_matrix` (so `pose`) | every distance of every part, no pruning |
| `bounded(...)` | `bounded_clearances` (so `closest`, `clear_of`, the reach map) | bounding-sphere pruned clearances |
| `points_full(...)` | `clearances_of` | `full` on part points already placed in base_link (`stacked_points`) |

Python keeps the forward kinematics (`geometry`, `part_points`: one `ArmKinematics.link_frames` pass per sample), the
`BodyReport` / `PoseClearance` assembly and the hint texts. Every entry point keeps its signature.

Interpreter exit: a checker still referenced at shutdown (a daemon thread that was never stopped, such as a motion
queue worker, or a module global) is never freed by Python, and nanobind would report its `BodyChecker` as leaked.
`self_collision.release_kernels` (atexit) drops the kernel of every live checker (`CHECKERS`, a weak set); a call on a
checker after that raises `RuntimeError`. The reach envelope additionally stops its running thread first
(`arm_reach.stop_running_envelopes`).

Sources:

- `body_geometry.{hpp,cpp}`: box and vertical cylinder signed distances, the golden-section capsule axis search
  (48 iterations, as `body_model.capsule_signed_distance`) and the element-by-element rigid placement.
- `body_checker.{hpp,cpp}`: the full, pruned and path computations, step for step as the references.
- `fp_contract.hpp`: no floating-point contraction (fused multiply-add) in these files, so each value rounds as numpy
  rounds it. Checked on 2026-10-13 with both compilers: Apple clang (dev Mac) and GCC 13.3.0 aarch64 on the client
  Pi 5 build (`-Wall -Wextra`, no warnings), where the C++ path and pose outputs are bit-identical to the Python
  reference run on the same machine for every golden input (3767 floats, 0 differing).
- `bind.cpp`: `register_collision`, the nanobind class.

## Parity

The original Python bodies (`path`, `pose`, `clearances_of`, `clearance_matrix`, `bounded_clearances`,
`pruned_path_clearances`, `capsule_distances`, `hard_limit`, `moving_soft_clearances`) are in
`mcp_server/reference/collision.py`. They are used only to write the goldens:

```bash
cd nodes/mcp_server
uv run python -m tests.regenerate_core_golden --family collision --kernel self_collision.path
uv run python -m tests.regenerate_core_golden --family collision --kernel self_collision.pose
```

Inputs (`bench/inputs.py`): `collision_path_cases` (the bench path, the joint lines between every pair of nine
scenario poses on the deployed config, lines held to the hard margin, unpruned lines, gripper-only motions and seeded
random lines; rejected, slowed-only and clear motions and an escape from inside the hard margin) and
`collision_pose_cases` (the scenario poses and 24 seeded random poses).

Tolerances: clearances `atol 1e-9` m; the rounded clearances in `summary` / `rejection` `atol 1e-4`; everything else
(verdicts, slowed samples, obstacle and part names, the violating sample) exact. On the dev Mac the kernel matches
the reference bit for bit (worst error 0).

The existing tests (`test_self_collision.py`, `test_arm_link_boxes.py`, `test_reach_map.py`) compare the pruned
kernel with the full one field for field, as before.
