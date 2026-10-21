---
title: Edge step and ramp slope
status: todo
repo: ros2-lerobot-sverve-platform
value: "The 5 cm box reads 5 cm at its edge and the 14 deg ramp reads about 14 deg instead of 6, so both are classified correctly."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp, nodes/ground_estimator_cpp/src/core/edges.cpp, nodes/ground_estimator_cpp/test/core/test_edges.cpp, nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [robust-config-keys]
owns: [robust-slope, ramp-synthetic-scene]
uses: [robust-config]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Edge step and ramp slope

## Requirement

Median edge step and a ramp slope that survives sparse returns and the temporal cell median (design D4, D11, brief fixes 2 and 7).

- `src/core/edges.cpp`, `include/ground_estimator_cpp/edges.hpp`: `Cluster::edge_step_m` becomes the 50th percentile (median) of |height| within `edge_band_m` of the edge line (was the 90th); the header comment says so.
- Robust slope: `Cluster::slope_deg` is no longer the PCA smallest-eigenvector angle. Default method (my call; the builder may pick another only if it passes every test below): bin the cluster points in base_link xy by `edges.slope_bin_m` (read from the `EdgesConfig` already passed to `extract_clusters`, so no caller changes), one sample per bin = (mean xy, median plane-relative height); fit h = a*x + b*y + c robustly over the samples (IRLS with a Huber weight, or RANSAC with an inlier threshold of `fit.distance_m`); `slope_deg = atan(hypot(a, b))` in degrees. The per-bin sampling removes the bias the per-cell median staircase gives a per-point fit.
- New `bool Cluster::slope_known`: true only when the cluster covers at least `edges.slope_min_cells` bins and those bins span at least two distinct bins along the cluster's principal axis; when false, `slope_deg` is 0.0 and documented as "not measurable" (task lethal-point-gates makes the rule skip the slope test for such clusters).
- `test/core/synthetic_floor.hpp`: add a ramp surface helper (`add_ramp(scene, x0, length_m, height_m, y0, y1)`) and a sparse-sampling helper (keep a seeded random fraction of a cloud) if they do not exist.
- Tests first in `test/core/test_edges.cpp`, seen failing first, fixed seeds:
  - sparse 14.1 deg ramp (25 cm long, 63 mm high, 0.25 m wide, x 1.0-1.25 m; 30% of a 2 cm grid kept; per-point noise sigma 15 mm) fed through `CellMedianFilter` for 10 frames, then `segment` and `extract_clusters`: over 30 seeds the ramp cluster's slope p50 is within +-1.5 deg of 14.1 and the p90 absolute error is <= 3 deg; run once with identity `fixed_T_base` and once with a misaligned one (0.03 m offset, 20 deg yaw) so the bins do not line up with the temporal cells;
  - a dense noise-free 25 deg ramp gives slope >= 23 deg; a flat 5 cm box top gives slope < 3 deg;
  - a cluster covering fewer than `slope_min_cells` bins has `slope_known == false` and slope 0;
  - edge step: a 5 cm box whose edge-band heights are spread 0.035-0.065 m gives edge_step_m 0.05 +- 0.005 (the p90 would be ~0.062); a 3 cm sill with +-1 cm noise gives edge_step_m < 0.04;
  - every existing test_edges test stays green or is updated to the p50 definition with a comment.

Interfaces: `Cluster::edge_step_m` (p50), `Cluster::slope_deg` (robust), `Cluster::slope_known` in edges.hpp (part robust-slope); ramp and sparse helpers in synthetic_floor.hpp (part ramp-synthetic-scene).

Notes: measured on the robot: ramp slope p50 5.7 deg, p90 7.2-7.6 deg against a true ~14 deg, ToF cell std 13-21 mm on the black mat. CellMedianFilter moves every point of a cell to the cell's median height, so a per-point fit over a ramp sees a staircase. Uses `EdgesConfig::slope_bin_m` and `EdgesConfig::slope_min_cells` (robust-config).

## Acceptance conditions

- a1: a sparse noisy 14.1 deg ramp through the cell median gives slope p50 within 1.5 deg and p90 error <= 3 deg, aligned and misaligned to the cells [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: edge_step_m is the median of the edge band: a 5 cm box with a 3.5-6.5 cm edge band reads 0.05 +- 0.005 m, a 3 cm sill reads below 0.04 m [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: clusters below slope_min_cells bins have slope_known false and slope 0 [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: Cluster carries slope_known [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp::slope_known]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp`
- `nodes/ground_estimator_cpp/src/core/edges.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edges.cpp`
- `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
