---
title: Head remount floor runbook
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The robot is re-zeroed, recalibrated and shown to see a flat floor and a clear costmap, with the numbers written down."
files: [docs/runbooks/head-remount-floor-check.md, tests/test_head_remount_runbook.py, docs/runbooks/ground-estimation.md, docs/runbooks/head-rgbd-fusion.md, docs/head-rgbd-fusion-methodology.md, MEMORY.md, tests/README.md]
verify: "uv run pytest tests/test_head_remount_runbook.py -q && uv run poe lint"
depends_on: [head-pan-inverted, tof-radial-to-z-depth, tof-calibrated-k-deploy, ground-temporal-wiring, tof-mark-band-ten-cm]
owns: []
uses: [tof-depth-z, head-pan-sign, tof-mark-band, ground-cell-median]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-depth-fix-and-ground-noi--head-remount-floor-runbook
branch: tof-depth-fix-and-ground-noi--head-remount-floor-runbook
verified_head: ""
run: ""
provides: []
---

# Head remount floor runbook

## Requirement

Robot runbook and docs for the head remount fix (design D7, D8; approval A5).

- New `docs/runbooks/head-remount-floor-check.md` with sections `## Preconditions` (robot free to move, not on blocks; flat floor; ChArUco board on the floor in front; check for a running agent session before deploying mcp_server), `## Deploy` (only `scripts/deploy-nodes.sh`, output tee'd to `.logs/`: client arducam_tof, head_rgbd_calibration (apply unit), base_servos, ground_estimator_cpp, nav2_bringup, mcp_server; the ansible-deploy skill gives the exact command), `## Head pan direction` (a +0.3 rad pan command turns the head left, CCW from above; the user confirms), `## Head re-zero` (Calibrate page, Servo torque and neutral section, per docs/runbooks/servo-torque-neutral.md), `## Head RGB-D recalibration` (Calibrate page, per docs/runbooks/head-rgbd-fusion.md; with z-depth the plane_bias_mm_median should be near 0, within +-10 mm (my call), the LimitsConfig plane_max_bias_mm 40 still applies), `## Floor numbers` (topic_scraper_api workflow per CLAUDE.md: plane height 0 +-1 cm, plane_vs_base < 0.6 deg, zero drop and obstacle points on the flat floor over 60 s, ground_temporal_cells non-zero), `## Costmap` (local costmap clear on an empty floor for 30 s; the board and a basket still marked where taller than 10 cm), `## Results` (filled from the robot run: date, numbers, pass/fail per check).
- `docs/runbooks/ground-estimation.md` links the new runbook; `docs/runbooks/head-rgbd-fusion.md` and `docs/head-rgbd-fusion-methodology.md` state that /tof/depth/image_raw is z-depth (radial SDK distance divided by the ray norm of the calibrated K) and what that does to the plane bias.
- `MEMORY.md` records the decision: SDK depth is radial, converted to z in arducam_tof with the calibrated K; head_pan servo mounted reversed; ground per-cell temporal median; ToF obstacle band 10 cm; rule unchanged.
- Tests first: `tests/test_head_remount_runbook.py` pins the sections, the deploy command form (`scripts/deploy-nodes.sh`, never ansible-playbook), the floor thresholds (+-1 cm, 0.6 deg), the 0.10 band equal to nav2_params.yaml tof_mark, the temporal defaults equal to config/ground_estimator.yaml, every scripts/ path named exists, and the MEMORY entry; `tests/README.md` lists it.
- After the merge and deploy, the runbook is executed on the robot and its Results section filled with the measured numbers (the user confirms the pan direction and runs the Calibrate page steps; the agent drives the checks).

## Acceptance conditions

- a1: the runbook has every section and its thresholds, band and temporal defaults equal the shipped config [evidence: test:tests/test_head_remount_runbook.py::test_runbook_values_equal_the_shipped_config]
- a2: the runbook deploys only through scripts/deploy-nodes.sh and every scripts/ path it names exists [evidence: test:tests/test_head_remount_runbook.py::test_runbook_deploys_only_through_deploy_nodes]
- a3: MEMORY.md has the decision and tests/README.md lists the new test [evidence: test:tests/test_head_remount_runbook.py::test_memory_and_tests_readme]
- a4: root tests and lint pass [evidence: command:uv run pytest tests -q && uv run poe lint]
- a5: the runbook was executed on the robot after deploy and Results records plane 0 +-1 cm, plane_vs_base < 0.6 deg, no drops or obstacles on the flat floor, the costmap clear on an empty floor [evidence: judgement]

## Files

- `docs/runbooks/head-remount-floor-check.md`
- `tests/test_head_remount_runbook.py`
- `docs/runbooks/ground-estimation.md`
- `docs/runbooks/head-rgbd-fusion.md`
- `docs/head-rgbd-fusion-methodology.md`
- `MEMORY.md`
- `tests/README.md`

## Interfaces

Uses `tof-depth-z`, `head-pan-sign`, `tof-mark-band`, `ground-cell-median` (names, values and keys in the docs). Keep docs/runbooks/ground-estimation.md sections (owned by rgb-d-ground-estimation-and/ground-final-suite-runbook): only a link line is added.

## Expected outcome

`uv run pytest tests/test_head_remount_runbook.py -q` passes; after merge and deploy the Results section holds the robot numbers.

## Steps

- Write tests/test_head_remount_runbook.py; run it, see FAIL (no runbook).
- Write docs/runbooks/head-remount-floor-check.md, the link and z-depth notes, the MEMORY.md entry and the tests/README.md row; rerun, see PASS; `uv run pytest tests -q && uv run poe lint`.
- Commit `docs: head remount floor check runbook`.
- After merge: deploy through scripts/deploy-nodes.sh, run the runbook with the user, fill Results, commit `docs: head remount floor check results`.

## Notes

Pan check amplitude 0.3 rad, head only (servo safety). topic_scraper_api must be enabled in group_vars and deployed before the scraper workflow (CLAUDE.md). Ask before deploying mcp_server if an agent session runs.
