---
title: D435 edge and flying-pixel filtering
status: todo
repo: ros2-lerobot-sverve-platform
value: Box edges stop producing phantom obstacles while the floor stays intact
files: [ansible/group_vars/client.yml, tests/test_d435_edge_filter.py, nodes/bridges/realsense_d435/README.md, tests/README.md]
verify: "uv run pytest tests/test_d435_edge_filter.py -q && uv run poe test-ansible && uv run poe lint"
depends_on: [record-d435-ground-replay-fi]
model: opus
review: ""
verified_head: ""
points: 5
expected_minutes: 34
expected_tokens: 1191588
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# D435 edge and flying-pixel filtering

## Requirement

Carries the intent of drivable-ground-tof-filterin tof-flying-pixel-retune, tof-amplitude-gate and tof-filter-live-check (closed as superseded 2026-10-27): remove mixed/flying pixels at object edges from the D435 depth that feeds the obstacle cloud and ground estimation, while keeping the floor. Prefer librealsense post-processing filters exposed by realsense2_camera (spatial, temporal, hole_filling off, decimation as configured in d435-node-type-and-profile) set in the node config; add a filter stage in the obstacle-cloud path only if those cannot meet the numbers, and measure CPU on the Pi. Measure on the D435 recordings of record-d435-ground-replay-fi (home, box scenes), in a test that replays the recorded depth.

## Acceptance conditions

- a1: on the recorded D435 frames the filter keeps >= 95 % of the floor pixels 0.3-2.0 m in every scene [evidence: command:uv run pytest tests/test_d435_edge_filter.py -q]
- a2: >= 80 % of the mixed pixels at the box edges are removed, box-top pixels kept >= 95 % [evidence: command:uv run pytest tests/test_d435_edge_filter.py -q]
- a3: the realsense node CPU with the filters stays within the budget of d435-memory-and-cpu-budget [evidence: judgement]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_d435_edge_filter.py`
- `nodes/bridges/realsense_d435/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

1. Write tests/test_d435_edge_filter.py replaying the recorded D435 depth (record-d435-ground-replay-fi) with labelled floor, box-edge and box-top masks; it fails without filtering.
2. Choose realsense2_camera post-processing filters (spatial/temporal) in the node config; replay them offline (pyrealsense2 or an equivalent numpy model of the same filter) until a1-a2 hold.
3. Deploy, read the node CPU (node_cpu_compare.py) for a3; run the verify.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
