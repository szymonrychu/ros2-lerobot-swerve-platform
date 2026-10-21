# Ground detection robustness design

## Summary

The ground estimator (nodes/ground_estimator_cpp) and costmap_zones get eight fixes so the robot can drive on what they publish: a per-cell temporal spread gate, a median edge step, a robust ramp slope, warning-only colour clusters, multi-point lethal marks, per-cell class hysteresis in the odom frame with cell-aligned warning areas, a cell-keyed warning store in costmap_zones and smoother plane tracking. The ramp rule rises from 8 to 15 deg, bounded by the tilt_guard error limits, because the user calls the ~14 deg ramp safe; the 5 cm box stays an obstacle under the 1/3-wheel rule. A report script and a static runbook prove it on the user's scene with the robot parked.

## Intent and why

User, 2026-10-21: "There is still a lot of random noise on the ground/floor/obstacle/edge/etc detection. Fix it if it's too unreliable to properly let robot drive around." The live diagnosis (160 s and 200 s captures, robot static) says it is not safe to drive yet:

- far band: ToF cells at the edge of reliable returns (1.15-1.45 m in one capture, 1.65-2.2 m in another) read +4..+19 cm with 30-90 mm temporal std, giving ~190-210 lethal points per frame on empty floor through `lethal_unclustered`; floor cells 0.35-1.05 m have 4-20 mm std, the box top 3-6 mm;
- box: step region present in 84% of states, obstacle 100%; region median 3.3-3.5 cm but `edge_step_m` is the p90 of the edge band (box top near the edge reads 4-6 cm);
- ramp: present in 27% of states, obstacle 68% / warning 32%, measured slope p50 5.7, p90 7.2-7.6 deg against a true ~14 deg (sparse ToF on the black mat, cell std 13-21 mm);
- board: colour warning, slope 3.7-6.4 deg; a false drop warning in 10% of states; colour clusters can go lethal only through the noisy PCA slope (colour-warning slopes p50 6.0, p90 7.6, max 8.0);
- false drop warnings behind the box at (1.2-1.3, 0.1-0.3), -2.6 cm, in 25-31% of states (likely ToF multipath beside the box's white side);
- RGB-D labels flip in 30-59% of frames at some cells; the warning-area count changes in 186 of 300 states; costmap_zones keeps each message 3 s, so 30-60 areas are active at once and the masks republish at the 2 Hz cap;
- plane: height 6.7 cm (head geometry, fixed by head-chain-floor-plane-fit-a), std 6 mm, pitch std 0.45 deg, roll std 0.73 deg; 37% of fits rejected `offset_off_prior` (same geometry cause);
- not causes: the IMU prior (0 `normal_off_prior`), late TF while static (0 drops). The CPU epic is deployed (3.74 -> ~2.3 cores).

## Goals

- Empty floor: 0 lethal points at p95 of states; noisy far-band cells are unknown, never lethal, never a step or drop edge.
- The 5 cm box is an obstacle in >= 95% of states; a 3 cm sill stays a warning.
- The ~14 deg ramp is a warning in >= 90% of states, its slope within +-3 deg of 14.
- The board (colour mismatch) never gives a lethal state.
- Warning-area churn < 10% of states; costmap_zones republishes only when its active cell set changes.
- The runbook numbers are measured on the robot and recorded.

## Non-goals

- Driving the robot (it is charging); only head pan/tilt moves.
- The head chain geometry and the plane offset rejections (head-chain-floor-plane-fit-a, this epic's blocker).
- Changing the `/ground/warning_areas` or `/ground/state` JSON schemas, Nav2 layers, the collision monitor, the web UI or dashboards.

## Anti-goals

- A real obstacle that never becomes lethal, or a lethal mark that lags more than 0.3 s at 10 Hz (enter_frames 3).
- Placeholder data on any topic: a held lethal cell publishes only the points seen in it this frame.
- A ramp limit above the tilt_guard error limits.
- CPU regressions that undo client-cpu-budget-recovery-a (the new work is O(points + cells) per frame).

## Constraints

- C++ in nodes/ground_estimator_cpp, tested with `scripts/cpp-node-test.sh ground_estimator_cpp` (all gtests plus ament linters); core modules are picked up by the CMake glob.
- `config/ground_estimator.yaml` and the client.yml ground_estimator_cpp block stay equal (`tests/test_ground_estimator_deploy.py`); unknown keys are rejected; every key documented with meaning, unit and range.
- Python: type hints, pydantic config, constants on top, docstrings with types (CLAUDE.md).
- Deploy only with `scripts/deploy-nodes.sh`; the runbook is executed by the orchestrator, robot static.
- client.yml is also edited by head-chain-floor-plane-fit-a (in progress): the epic is blocked on it.

## Decisions

### D1: The 5 cm box stays an obstacle

The 1/3-wheel-diameter rule (0.04 m, user Q3 of the ground epic) stands; the user's box is exactly 5 cm and the user called it "around" the drivable height, so it is an obstacle and must be classified stably. Alternative: raise max_step_fraction so 5 cm rides; rejected, it overrides a standing user rule. (my call) Serves A2, A9.

### D2: max_slope_deg 15, bounded by the tilt_guard error limits

The user states the ~14 deg ramp is a safe skew, so `traversability.max_slope_deg` 8 -> 15 and the validation moves from `<= min(roll_warn_deg, pitch_warn_deg)` to `<= min(roll_error_deg, pitch_error_deg)` (15 / 20 today). Exactly 15 is a warning, 15.01 an obstacle. Alternative: bound by roll_error only (the brief's wording); min of both error limits is the same today and stays safe if pitch_error ever drops (my call). Serves A7, A9.

### D3: Per-cell temporal spread gate

In CellMedianFilter, a cell whose robust spread (1.4826 * MAD of its per-frame medians) exceeds `temporal.max_spread_m` (0.025) outputs no points: it is unknown, never lethal, never an edge, and recovers when it settles. Alternatives: a fixed range cap (rejected: the reliable range moved from 1.45 to 2.2 m between captures); window std (rejected: one bad frame would gate a steady box cell). MAD over std is my call. Serves A1.

### D4: Edge step from the median of the edge band

`edge_step_m` = p50 of |h| within edge_band_m of the edge (was p90), so the 5 cm box reads 5 cm and a 3 cm sill does not read 4+ cm on noise. Serves A2.

### D5: Per-cell class hysteresis in the fixed frame

A new CellClassFilter on the temporal cell grid (odom) votes once per frame per cell: lethal when lethal votes >= enter_frames of window_frames, staying lethal while >= exit_frames; warning likewise over warning plus lethal votes; enter_frames must be a strict majority. Defaults window 5, enter 3, exit 1 (my call): a new lethal mark lags 0.3 s at 10 Hz, a flipping cell settles. Alternatives: per-region hysteresis keyed by cluster matching (rejected: cluster identity is unstable frame to frame); time-based EMA of class scores (rejected: harder to reason about latency). Regions take their class from their cells (lethal when any cell is stable lethal). Serves A2, A5, A7.

### D6: Colour-mismatch clusters are warnings only

`colour_warning_areas` never produces an obstacle; the colour lethal path and `WarningArea::cls` are removed. Colour can still slow the robot, never block it; ToF height decides lethality. Serves A3.

### D7: Unclustered lethal points need company

An unclustered point above max_step_m is lethal only when at least `traversability.lethal_min_points` (2) such points of the same group lie within `edges.cluster_tolerance_m`; persistence over frames comes from D5. Serves A4.

### D8: Warning areas from stable warning cells

Warning areas are the 8-connected components of stable-warning cells, polygon = convex hull of the cell corners in odom, attributes from the cells' latest observations; the JSON schemas do not change. Identity is the cell set, so polygons are identical while the classes hold. Serves A5.

### D9: costmap_zones stores warnings per map cell

The 3 s TTL union of whole messages becomes a per-map-cell last-seen store; masks are rasterised from the active cells and republished only when the active cell set changes (rate cap kept). `ground_warning_ttl_s` stays 3.0, now per cell (my call). Serves A5.

### D10: fit.smoothing 0.5 -> 0.2

The plane estimate weights a new fit 0.2, after the spread gate lands (dependsOn). Serves A6.

### D11: Robust ramp slope with minimum support

The cluster slope is a robust fit (IRLS Huber or RANSAC) of plane-relative height over per-bin samples (`edges.slope_bin_m` 0.10, mean xy and median height per bin) instead of the PCA eigenvector over per-point positions, removing the staircase bias of the cell median. With fewer than `edges.slope_min_cells` (3) bins, or bins along a single line across the principal axis, `slope_known` is false, slope_deg is 0 and the cluster is classified on its step height only (kind step, never ramp). The estimator may differ if it passes the same synthetic tests (Left to the builder). (my call) Serves A7.

### D12: Runbook with a report script over topic_scraper captures

`scripts/ground_noise_report.py` plus a YAML of the user's scene ROIs and thresholds computes per-scene metrics from `scripts/topic_scraper_collect.py` captures of `/ground/state`; the runbook captures 2 minutes per scene with the head pointed at it. The orchestrator executes it after deploy and fills the Results table. Serves A8.

### D13: One config foundation, client.yml mirrored per task

New keys land in one foundation task (robust-config-keys) with their validation and the client.yml mirror; the two value changes (D2, D10) land in slope-and-smoothing-defaults with their test pins; client.yml edits are chained by dependsOn and the epic is blocked on head-chain-floor-plane-fit-a (client.yml overlap and the plane geometry the runbook needs). Serves A1, A6, A7.

## Research

| Lens | Ran or skipped | Adopted ideas | Rejected ideas |
|---|---|---|---|
| Live diagnosis (brief ground-robust.md, scratchpad/groundnoise captures) | ran | spread gate, p50 edge step, hysteresis, colour warning-only, multi-point lethal, cell-keyed costmap store, smoothing 0.2, robust slope (brief "Fixes to build" 1-8) | fixed range cap (brief fix 1: range moved between captures) |
| Code read (LSP/grep) | ran | cell grid shared with CellMedianFilter (`cell_median.cpp:cell_key`); `edges.cpp:compute_features` p90 and PCA slope; `traversability.cpp:lethal_unclustered`, `colour_warning_areas`; `costmap_zones/ground_warnings.py:GroundWarningStore` TTL union | per-region identity by cluster matching (`traversability.cpp` SAME_EDGE_M matching is exact per frame only) |
| External research | skipped: the causes are measured and the fixes are standard robust statistics (MAD, IRLS, N-of-M hysteresis) | | |
| architect-researcher | skipped: one viable approach per fix, all inside one node and one consumer, reversible by config | | |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief fix 1 | per-cell temporal spread gate, configurable, replaces a range cap | robust-config-keys, cell-spread-gate, node-stable-classes |
| brief fix 2 | edge step p50 plus per-cell class hysteresis | edge-step-and-ramp-slope, cell-class-hysteresis, node-stable-classes |
| brief fix 3 | colour clusters warning-only | lethal-point-gates |
| brief fix 4 | lethal_unclustered needs >= 2 points or persistence | robust-config-keys, lethal-point-gates, node-stable-classes |
| brief fix 5 | per-odom-cell hysteresis before areas; costmap keyed by cell, publish on change | cell-class-hysteresis, node-stable-classes, costmap-cell-keyed-warnings |
| brief fix 6 | fit.smoothing 0.5 -> 0.2 after the spread gate | slope-and-smoothing-defaults |
| brief fix 7 | max_slope_deg 15 vs tilt_guard error limits; robust slope, minimum support, hysteresis | robust-config-keys, slope-and-smoothing-defaults, edge-step-and-ramp-slope, cell-class-hysteresis |
| brief fix 8 | static runbook with the scene and thresholds | ground-noise-report-script, ground-robustness-runbook, ground-robustness-docs |
| brief D1, D2 | box obstacle, ramp warning at 15 deg | slope-and-smoothing-defaults, ground-robustness-docs |
| brief "Existing parts" | tests pin both sides of each threshold (6/10 deg) | slope-and-smoothing-defaults |
| brief "Concurrent" | order client.yml after head-chain-floor-plane-fit-a | epic blocker (README), robust-config-keys |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 The 5 cm box stays an obstacle | edge-step-and-ramp-slope, node-stable-classes, ground-robustness-docs |
| D2 max_slope_deg 15, bounded by the tilt_guard error limits | robust-config-keys, slope-and-smoothing-defaults, ground-robustness-docs |
| D3 Per-cell temporal spread gate | robust-config-keys, cell-spread-gate, node-stable-classes |
| D4 Edge step from the median of the edge band | edge-step-and-ramp-slope |
| D5 Per-cell class hysteresis in the fixed frame | robust-config-keys, cell-class-hysteresis, node-stable-classes |
| D6 Colour-mismatch clusters are warnings only | lethal-point-gates |
| D7 Unclustered lethal points need company | robust-config-keys, lethal-point-gates |
| D8 Warning areas from stable warning cells | cell-class-hysteresis, node-stable-classes |
| D9 costmap_zones stores warnings per map cell | costmap-cell-keyed-warnings |
| D10 fit.smoothing 0.5 -> 0.2 | slope-and-smoothing-defaults |
| D11 Robust ramp slope with minimum support | robust-config-keys, edge-step-and-ramp-slope, lethal-point-gates |
| D12 Runbook with a report script over topic_scraper captures | ground-noise-report-script, ground-robustness-runbook |
| D13 One config foundation, client.yml mirrored per task | robust-config-keys, slope-and-smoothing-defaults, costmap-cell-keyed-warnings |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `robust-config` | robust-config-keys | cell-spread-gate, edge-step-and-ramp-slope, lethal-point-gates, cell-class-hysteresis, slope-and-smoothing-defaults | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp` |
| `spread-gate` | cell-spread-gate | slope-and-smoothing-defaults, node-stable-classes | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp` |
| `robust-slope` | edge-step-and-ramp-slope | lethal-point-gates, node-stable-classes | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp` |
| `ramp-synthetic-scene` | edge-step-and-ramp-slope | node-stable-classes | `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp` |
| `lethal-gates` | lethal-point-gates | slope-and-smoothing-defaults, node-stable-classes | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp` |
| `cell-class-filter` | cell-class-hysteresis | node-stable-classes | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_class.hpp` |
| `ground-traversability-rule` | slope-and-smoothing-defaults | ground-robustness-docs | `nodes/ground_estimator_cpp/config/ground_estimator.yaml` |
| `stable-warning-areas` | node-stable-classes | ground-robustness-docs | `nodes/ground_estimator_cpp/src/ground_node.cpp` |

`ground-traversability-rule` is also registered in docs/plans/contracts.json as a cross-epic contract (consumer epic rgb-d-ground-estimation-and, whose runbook verifies the rule). Existing, unchanged: `ground-warning-areas` JSON (rgb-d-ground-estimation-and/ground-state-json, `state.hpp:to_warning_areas_string`), read by costmap-cell-keyed-warnings through `costmap_zones/ground_warnings.py:GroundWarningMessage`.

## Review focus

- cell_class.cpp: the enter/exit rule, lethal priority, that no remembered point is ever published (no placeholder data), deterministic area order and polygons.
- ground_node.cpp: lethal clouds only from this frame's points in stable lethal cells; warning areas through TF fixed -> warnings.frame; reset on plane loss.
- edges.cpp: the robust slope on sparse and misaligned bins; `slope_known` false never silently hides a steep, well-supported ramp.
- costmap_zones: the cell store expires per cell and never grows without bound; geometry change clears it.
- Config: yaml and client.yml stay equal; validation names every key.

## Left to the builder

- The robust slope estimator (IRLS Huber or RANSAC over per-bin samples, or another unbiased fit) as long as edge-step-and-ramp-slope's synthetic tests pass.
- The kd-tree or grid used for the lethal_min_points neighbour count.
- The costmap cell representation (boolean mask or flat-index set) and the raster helper shape.
- The NDJSON reader details of the report script, after reading topic_scraper_collect's output format.

## Assumptions and unknowns

- The far-band noise is temporal (30-90 mm std of cell values) rather than a steady bias; a steady bias above 4 cm would pass the spread gate. Confirmed by the runbook's empty-floor scene.
- The false drops behind the box are noisy enough to be gated or held below enter_frames; confirmed by the box scene churn and drop counts.
- The plane height and offset rejections are fixed by head-chain-floor-plane-fit-a before the runbook runs (blocker).
- The sparse-ramp synthetic model (30% of a 2 cm grid, 15 mm noise) matches the black mat; confirmed by the ramp scene slope error.

## Open questions

none

## Irreversible operations in scope

none (deploys through scripts/deploy-nodes.sh are reversible by redeploying the previous commit; the runbook does not drive the robot).
