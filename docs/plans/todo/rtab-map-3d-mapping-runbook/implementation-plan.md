# RTAB-Map 3D mapping runbook implementation plan

## Agreed rung

Rung 3 (complete): items 1.1, 1.2, 1.3, 2.1, 3.1. Gate modifications (plan.md Agreed scope): run after D435 S59
(calibration) and S84 (deploy), in the same robot session as D435 S72 (which already deploys rtabmap_mapping and
map3d_uplink); acceptance a3 becomes "D435 head chain applied, /head_rgbd publishing". No threshold of the moved
requirement is loosened (alignment_agreement >= 0.8, <= 0.15 m/s and <= 0.3 rad/s, caps >= 2x measured peak, uplink
total within `uplink_mbit_s_max`). No node code changes: the contract and launch are already on the D435
(nodes/rtabmap_mapping/config/map3d_topics.yaml:137-138, D435 plan's map3d step).

Offline delivers 1.1. The live items 1.2, 1.3, 2.1 and 3.1 run under "After merge (live, with the user)".

## Approach

Component mode, one worktree via workbench:plan-run, branch `plan/rtab-map-3d-mapping-runbook`. One chunk, one step,
test-first: the pins in `tests/test_map3d_runbook.py` are run and seen failing on the current runbook, then the runbook
is rewritten until they pass. Only `docs/runbooks/map3d-mapping.md`, `tests/test_map3d_runbook.py` and
`tests/README.md` change. The Results table keeps no invented numbers.

## Chunk 1: re-base the runbook on the D435 (model: sonnet)

### S1: Map3d runbook on the D435

- Files: `docs/runbooks/map3d-mapping.md`, `tests/test_map3d_runbook.py` (new), `tests/README.md`
- Covers: 1.1
- Test first: `tests/test_map3d_runbook.py` with
  - `test_runbook_has_no_cm3_or_tof_inputs`: none of `head_camera_calibration.yaml`, `tof_calibration.yaml`,
    `head_rgbd_extrinsics.yaml`, `arducam_tof`, `head_camera_cpp`, `camera_info_file`, `focus lock` in the runbook.
  - `test_head_chain_check_is_the_d435` (acceptance a3 as replaced): step 1 names `docs/calibration/` (the `*-d435`
    results of D435 S59), `./scripts/d435_diag.sh` and `/head_rgbd/depth/image_raw`, and says the pair publishes at
    the mapping rate.
  - `test_deploy_and_placeholder_name_the_d435`: the deploy step reads
    `./scripts/deploy-nodes.sh client head_rgbd_cpp map3d_uplink web_ui`, the placeholder step stops `realsense_d435`
    and expects `waiting_for_inputs`, the CPU step names `realsense_d435`, `head_rgbd_cpp` and `map3d_uplink`.
  - `test_thresholds_kept`: `0.8`, `0.15 m/s`, `0.3 rad/s`, `2x` and `budget_bytes` present.
  - `test_runs_with_d435_s72`: the header names D435 S59, S84 and S72 and `docs/runbooks/d435-switchover.md`.
  - `test_results_rows_match_steps`: numbered steps count equals Results rows, each row starting with its step number.
  Run `uv run --frozen pytest tests/test_map3d_runbook.py -q`; expect failures on the current runbook (CM3/ToF files,
  head_camera_cpp, arducam_tof).
- Change: rewrite `docs/runbooks/map3d-mapping.md`:
  - Header: design pointer `docs/plans/done/rtab-map-rgb-d-3d-mapping-on/plan.md`; runs after D435 S59 and S84, in the
    robot session of D435 S72 (which already deploys rtabmap_mapping and map3d_uplink;
    `docs/runbooks/d435-switchover.md`). Motion limits unchanged.
  - Step 1 Head chain check: D435 S59 results committed (`git ls-files 'docs/calibration/*-d435/*'` non-empty) and its
    head chain in `ansible/group_vars/client.yml`; `./scripts/d435_diag.sh --rates` clean; with a subscriber on
    `/head_rgbd/depth/image_raw` the depth, RGB and both camera_infos publish at the mapping rate (~2 Hz). No focus
    lock (the D435 has none).
  - Step 2 Deploy: `client head_rgbd_cpp map3d_uplink web_ui`, `server rtabmap_mapping map3d_manager`,
    `server monitoring`, each `2>&1 | tee .logs/<name>.log`; skip whatever S72 already deployed in the same session;
    mcp_server / claude_agent only with no agent session.
  - Steps 3, 4 unchanged (map a room, save and restore; alignment >= 0.8, graph correction).
  - Step 5 CPU / memory: client CPU of `realsense_d435`, `head_rgbd_cpp`, `map3d_uplink` before/after enabling the
    uplink; server unchanged; caps >= 2x the measured peak.
  - Step 6 WiFi unchanged (measured `budget_bytes`, `tests/test_map3d_topics.py`, README Topics table).
  - Step 7 Placeholder: stop `realsense_d435` (deploy-nodes disable or the diag script): no pair, no depth to the
    server, `/map3d/status` `waiting_for_inputs`; restart it.
  - Results: one line "Re-based on the D435 2026-10-31; the 2026-10-19 run stopped at the CM3/ToF calibration
    precondition, retired by the D435 switchover", then a table with one row per step 1-7, every cell `-`.
  - `tests/README.md`: a `### test_map3d_runbook.py` section listing each test with one line.
- Verify: `uv run --frozen pytest tests/test_map3d_runbook.py tests/test_map3d_topics.py -q && uv run poe lint` -
  expect all passed (6 new plus the existing map3d topic tests), lint clean.

## Files to delete

none

## Final verify

`uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible`

## After merge (live, with the user)

Gate: start only when D435 S59 (calibration) and S84 (final deploy) are done in
`docs/plans/in-progress/realsense-d435-head-switchov/implementation-plan.md` (or its done/ location); run in the same
robot session as D435 S72. Robot unreachable: stop with the named blocker (CLAUDE.md). Deploys only through
`scripts/deploy-nodes.sh` after the ansible-deploy skill, logs in `.logs/`.

1. Test first: add `test_results_are_recorded` to `tests/test_map3d_runbook.py` (every Results row has a date and PASS
   or FAIL, step 4 an `alignment_agreement` number, steps 5 and 6 numbers); run it, see it fail.
2. Run runbook steps 1-4 and 7 (covers 1.2, 1.3) and step 5 (2.1); fill Results.
3. Caps (2.1): if a peak x2 exceeds its cap, raise `memory_max` / `cpu_quota` in `ansible/group_vars/server.yml`,
   `uv run poe test-ansible`, `./scripts/deploy-nodes.sh server rtabmap_mapping map3d_manager 2>&1 | tee .logs/deploy-server-caps.log`.
4. WiFi (3.1): step 6; replace each `budget_bytes` in `nodes/rtabmap_mapping/config/map3d_topics.yaml` with the measured
   value (comment `measured <date>`); `uv run --frozen pytest tests/test_map3d_topics.py -q` stays green
   (`test_bandwidth_within_budget`); if the measured total exceeds `uplink_mbit_s_max`, keep the budget and record it as
   a finding with numbers instead of raising it. Update the `nodes/rtabmap_mapping/README.md` Topics and "Bandwidth
   budget (D3)" tables; `./scripts/deploy-nodes.sh server rtabmap_mapping 2>&1 | tee .logs/deploy-server-rtabmap.log`.
5. `uv run --frozen pytest tests/test_map3d_runbook.py tests/test_map3d_topics.py -q && uv run poe lint`, update
   `tests/README.md`, commit `docs: 3D mapping runbook and measured budget`, push origin main.
