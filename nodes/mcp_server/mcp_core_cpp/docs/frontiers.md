# frontiers kernel family

Kernels of `mcp_server/frontiers.py` profiled by `../../bench/report.json` (5 ms threshold on the dev Mac). Every
kernel with verdict `port` runs in C++ in `mcp_core_cpp.frontiers` (`../src/frontiers/`); the others stay Python.
`tests/test_core_parity_frontiers.py` checks this table against the report.

| kernel | median ms | status | C++ median ms |
|---|---:|---|---:|
| frontiers.tracker_update | 15.358 | ported | 9.410 |
| frontiers.geodesic_from | 13.838 | ported | 1.921 |
| frontiers.traversability | 6.940 | ported | 2.949 |
| frontiers.frontier_field | 5.052 | ported | 2.652 |
| frontiers.extract_frontiers | 3.313 | below threshold, not ported | - |
| frontiers.observation_pose | 0.203 | below threshold, not ported | - |

"median ms" is the Python median of the report; "C++ median ms" is the same bench (`bench.bench_kernels`, frontiers
family only, 15 repetitions, same Mac) after the port, timing the unchanged Python entry points.

## Ported kernels

| C++ function | Python entry point (signature unchanged) | What runs in C++ |
|---|---|---|
| `traversability(data, resolution, pass_clearance_m, free_threshold, occupied_threshold)` | `traversability(grid, pass_clearance_m)` | known box, clearance EDT (exact squared distances, Felzenszwalb lower envelope), traversable mask |
| `geodesic_from(trav, snapped_row, snapped_col, snap_m, downsample, resolution)` | `geodesic_from(trav, start_rc, downsample, resolution)` | coarse grid and 8-neighbour Dijkstra (bucket queue of width 1, valid as every edge costs at least 1); the 0.3 m start snap stays Python (`snap_to_traversable`, also used by `FrontierTracker.geodesic`) |
| `frontier_field(data, cells, labels, resolution)` | `frontier_field(grid, found)` | known box, label image, feature transform with scipy's tie-breaking (`distance_transform_edt(return_indices=True)`: column pass then its `_VoronoiFT` row pass), distances, nearest labels and cells |
| `tracker_update(data, cells, labels, resolution, pass_clearance_m, free_threshold, occupied_threshold)` | `FrontierTracker.update(grid)` | the traversability and frontier-field stages of one map version in one call, with their stage times (`timings_ms`); extraction (`extract_frontiers`, below threshold) and id matching (`match_ids`, scipy Hungarian on a handful of centroids) stay Python |

The GIL is released while a kernel runs. Results are new numpy arrays with the dtypes of the Python versions
(bool / float32 for traversability, float64 / int32 for the field, float64 for the geodesic).

## Parity

Original Python bodies: `mcp_server/reference/frontiers.py` (`traversability_reference`, `geodesic_from_reference`,
`frontier_field_reference`, `ReferenceFrontierTracker.update`). Goldens: `tests/data/core_golden/frontiers/<kernel>.json`
from the cases in `bench/inputs.py` (`frontier_*_cases`: a walled room with doorways, random clutter with mid-range
occupancy values and free space at the map border, an all-unknown and a one-cell map; geodesic starts inside, on a
wall, beyond the snap radius and off the map; tracker sequences that keep, merge, explore and re-number ids across
epoch resets and a resolution change).

| kernel | tolerance |
|---|---|
| traversability | exact (trav, clearance_m) |
| frontier_field | exact (distance, label, nearest cell) |
| tracker_update | exact (frontier cells, labels, ids, retired ids and reasons, centroids, trav, clearance, field) |
| geodesic_from | atol 1e-9 m, rtol 1e-12 (equal-length paths may be summed in another order) |

Regenerate a golden only when a reference changes by design:

```bash
cd nodes/mcp_server && uv run python -m tests.regenerate_core_golden --family frontiers --kernel tracker_update
```
