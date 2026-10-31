# Printable markers runbook implementation plan

## Agreed rung

Rung 3 (complete): items 1.1, 1.2, 1.3, 2.1, 2.2, 3.1. Gate modifications (plan.md Agreed scope): run after D435 S59
(calibration) and S84 (deploy); pose distances about 0.4 / 0.55 / 0.7 m for the 150 mm marker, a range shortfall
recorded as a finding with numbers; the zoom step is dropped. No threshold of the moved requirement is loosened
(<= 3 cm, <= 5 deg, decode + pose p95 <= 100 ms, CPU p95 <= 0.3 core). No code change in any node: marker_detector,
scripts/marker_bench.sh, poi_store, mcp_server `list_pois(marker_kind=...)` and the web UI facing arrows are already on
the D435 (client.yml:3685-3690, scripts/marker_bench.sh:6, nodes/marker_detector/README.md:48).

Offline delivers 1.1 and the computed half of 3.1. The live items 1.2, 1.3, 2.1, 2.2 and the measured half of 3.1 run
under "After merge (live, with the user)".

## Approach

Component mode, one worktree via workbench:plan-run, branch `plan/printable-markers-runbook`. One chunk, two steps, in
order. Each step is test-first: the new pins in `tests/test_markers_runbook.py` are run and seen failing on the
current docs, then the docs are rewritten until they pass. Only `docs/runbooks/printable-markers.md`, `docs/markers.md`,
`tests/test_markers_runbook.py` and `tests/README.md` change. The Results table keeps no invented numbers: every row
reads `-` until the live run.

## Chunk 1: re-base the runbook and the range section on the D435 (model: sonnet)

### S1: Runbook on the D435

- Files: `docs/runbooks/printable-markers.md`, `tests/test_markers_runbook.py` (new), `tests/README.md`
- Covers: 1.1
- Test first: `tests/test_markers_runbook.py` with
  - `test_runbook_reads_the_d435`: the runbook names `/head/d435/color` and contains none of `head_camera_cpp`,
    `/head_camera/`, `1152`, `head_camera_calibration.yaml`, `zoom` (case-insensitive).
  - `test_prerequisites_are_the_d435_calibration`: the Prerequisites section names `docs/calibration/` (the
    `*-d435` results of D435 S59), `docs/runbooks/d435-switchover.md` and `./scripts/d435_diag.sh`.
  - `test_pose_distances_are_in_d435_range`: the pose-accuracy step names 0.4, 0.55 and 0.7 m and 30 deg, keeps
    `3 cm` and `5 deg`, says a distance that does not decode is recorded as a finding, and no longer names `1.5 m`.
  - `test_thresholds_kept`: `100 ms`, `640x360`, `0.3 core` and `20 sightings` present.
  - `test_results_rows_match_steps`: the number of numbered steps equals the number of rows of the Results table, and
    each row's first cell starts with its step number.
  Run `uv run --frozen pytest tests/test_markers_runbook.py -q`; expect failures on the current runbook (it names
  head_camera_cpp, 1152x648, zoom and 1.5 m).
- Change: rewrite `docs/runbooks/printable-markers.md`:
  - Header: design pointer `docs/plans/done/printable-metadata-markers-t/plan.md` (the old `in-progress/.../design.md` path is gone); the
    detector reads the D435 colour `/head/d435/color/image_raw/compressed` + `/head/d435/color/camera_info` and the
    aligned depth `/head_rgbd/depth/image_raw`.
  - Prerequisites: D435 S59 done (calibration results committed under `docs/calibration/<date>-d435/`, head chain in
    `ansible/group_vars/client.yml`), D435 S84 deployed; `./scripts/d435_diag.sh --rates` shows the colour
    camera_info publishing; no agent session running.
  - Steps, renumbered 1-9: 1 Generate and print (unchanged); 2 Deploy (unchanged command); 3 Placeholder check: no
    marker in view gives nothing on `/markers/detections` or `/poi/command`, then stop `realsense_d435` (deploy-nodes
    disable or the diag script, not raw systemctl) and confirm nothing is published, restart it; 4 Benchmark
    `scripts/marker_bench.sh`, decode + pose p95 <= 100 ms per 640x360 frame, on a miss lower `process_hz` in
    client.yml, redeploy, record; 5 Pose accuracy of the 150 mm marker at 0.4, 0.55 and 0.7 m head-on and at 30 deg,
    limits <= 3 cm and <= 5 deg (localisation included), a distance that does not decode is recorded as a range
    finding with numbers; 6 Refinement and dedup (unchanged); 7 CPU 10 min Grafana p95 <= 0.3 core; 8 Agent
    `list_pois(marker_kind="charger")` with approach pose, web UI facing arrows; 9 fill "Range and fallback" in
    `docs/markers.md` with measured decode ranges per marker size, finding if too short for docking (AprilTag named,
    not built). The zoom step is gone.
  - Results: one line "Re-based on the D435 2026-10-31; the 2026-10-19 status (blocked on the CM3 head calibration) is
    retired by the D435 switchover", then a table with one row per step 1-9 (Date, Result, Numbers, Pass/fail all `-`).
  - `tests/README.md`: a `### test_markers_runbook.py` section listing each test with one line.
- Verify: `uv run --frozen pytest tests/test_markers_runbook.py -q && uv run poe lint` - expect `5 passed` and lint clean.

### S2: Range and fallback on the D435

- Files: `docs/markers.md`, `tests/test_markers_runbook.py`, `tests/README.md`
- Covers: 3.1 (the computed table; the measured ranges are filled live)
- Test first: add to `tests/test_markers_runbook.py`
  - `test_markers_doc_range_is_d435`: the "Range and fallback" section of `docs/markers.md` names `640` and contains
    no `887`, `1152` or `zoom` (case-insensitive); the AprilTag fallback line is still there.
  - `test_markers_doc_detection_is_d435`: the "Detection" section names `/head/d435/color` and contains no
    `/head_camera/` and no zoom path.
  Run the file; expect both new tests to fail on the current doc.
- Change: in `docs/markers.md`:
  - "Range and fallback": the D435 colour stream is 640x360 with f about 460 px (69 deg HFOV; the live run uses the
    deployed camera_info fx). Range at 4 px per module, on-axis, d = f x module / 4, one Stream column:
    60 mm v2 0.28 m, 60 mm v3 0.24 m, 100 mm v2 0.46 m, 100 mm v3 0.40 m, 150 mm v2 0.69 m, 150 mm v3 0.59 m,
    200 mm v3 0.79 m. Drop the zoom columns and the zoom sentence; keep the AprilTag fallback paragraph; "Measured field
    ranges: not yet measured" now points at runbook step 9 and the D435 S59 prerequisite.
  - "Detection" (my call: it describes the zoom path and `/head_camera/` topics the range section now contradicts):
    the stream path reads the D435 colour topic at 640x360; the zoom path bullet is removed; "Ambiguity and ToF"
    describes the aligned D435 depth (`depth_plane.*` keys as in client.yml:3703-3709) instead of `tof.*`.
  - `tests/README.md`: the two new tests listed.
- Verify: `uv run --frozen pytest tests/test_markers_runbook.py -q && uv run poe lint` - expect `7 passed`, lint clean.

## Files to delete

none

## Final verify

`uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible`

## After merge (live, with the user)

Gate: start only when D435 S59 (calibration) and S84 (final deploy) are done in
`docs/plans/in-progress/realsense-d435-head-switchov/implementation-plan.md` (or its done/ location); otherwise report
the named blocker. Robot unreachable: stop with the named blocker (CLAUDE.md). Deploys only through
`scripts/deploy-nodes.sh` after the ansible-deploy skill, logs in `.logs/`; check no agent session runs before an
mcp_server deploy. Motion: head pan/tilt in the verified range; base moves only as the user approves.

1. Test first: add `test_results_are_recorded` to `tests/test_markers_runbook.py` (every Results row has a date, a
   number cell other than `-`, and PASS or FAIL); run it, see it fail.
2. Run runbook steps 1-9 with the user (covers 1.2, 2.1, 2.2, 3.1 measured). Fill each Results row (1.3) and the
   "Measured field ranges" of `docs/markers.md` (3.1); a pose distance that does not decode, or a range too short for
   docking, is written as a finding with numbers.
3. If step 4 or 7 missed, the lowered `process_hz` goes into `ansible/group_vars/client.yml` with
   `uv run poe test-ansible`, and `./scripts/deploy-nodes.sh client marker_detector 2>&1 | tee .logs/deploy-marker-hz.log`.
4. `uv run --frozen pytest tests/test_markers_runbook.py -q && uv run poe lint`, update `tests/README.md`, commit
   `docs: printable markers runbook with results`, push origin main.
