---
title: Calib robot runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head chain measured against the arm is applied on the robot and checked with real numbers."
files: [docs/runbooks/head-arm-calibration.md]
verify: "grep -q '^## Results' docs/runbooks/head-arm-calibration.md"
depends_on: [calib-docs-and-test-index]
owns: []
uses: []
model: ""
estimate: 90
actual: ""
review: skip
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib robot runbook

## Requirement

Verify the whole feature on the robot with the user and record real numbers (design D13; approvals H7, H9).
Prerequisite: the Hardware section of `docs/runbooks/head-arm-calibration.md` is recorded.

- Full suite green first: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible`,
  plus `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q`, `cd nodes/mcp_server && uv run --frozen
  pytest tests -q`, `cd nodes/web_ui && uv run --frozen pytest tests -q`, `cd nodes/web_ui/frontend && npx tsc --noEmit
  && npm test && npm run lint && npm run build`; read every line.
- Extend the runbook with the procedure and a `## Results` section, then execute it, filling in date, numbers and
  pass/fail per step; logs in `.logs/`. Telemetry only through topic_scraper_api / `scripts/topic_scraper_collect.py`
  and the web UI, never rclpy probes on the client (MEMORY):
  1. Deploy: invoke the ansible-deploy skill; check no agent session is running (deploying mcp_server kills it; ask
     first); `./scripts/deploy-nodes.sh client head_rgbd_calibration head_state_publisher mcp_server web_ui
     base_servos 2>&1 | tee .logs/deploy-head-arm-calib.log`; services start without `ModuleNotFoundError`.
  2. Tilt range: `set_head_angles` tilt 30, 40, 50 and the recorded limit at pan 0, slowly; nothing touches; record.
  3. Head from arm on the Calibrate page with the user watching (robot docked, no driving): Move to base pose (the user
     confirms), the user clips the board, tiers 0 and 1, then ask the user before "Allow elbow and shoulder moves"
     (CLAUDE.md servo safety); record views, arm poses, skipped reasons, RMS, held-out RMS and board error, every
     parameter with sigma, its change from the applied floor chain and the floor cross-check; the user removes the
     board and confirms Park.
  4. Apply a passed result; the units restart; check TF with topic_scraper_api (enable it first, CLAUDE.md) and the web
     UI head pose; `/ground/state` at home still within 0.01 m height (the floor fit must not regress).
  5. Live check of the yaw and x/y: arm at the base pose without the board, `head_look_at` on the tool point; the tool
     point's pixel offset from the image centre in the head camera image (web UI), converted to deg, before (CAD
     chain) and after; record.
  6. Note in the runbook that the arm joint offset solve is epic arm-joint-offsets-from-the-g.
- The task ends with the runbook committed with real numbers, or with a named blocker (robot offline, hardware not
  done, user OK not given, battery gate refusal) per CLAUDE.md.

## Acceptance conditions

- a1: the runbook has a Results section with real numbers [evidence: doc-section:docs/runbooks/head-arm-calibration.md#Results]
- a2: the applied result passed every limit and the floor check held on the robot [evidence: judgement]
- a3: every big-joint motion had the user's OK [evidence: judgement]

## Files

- `docs/runbooks/head-arm-calibration.md`

## Interfaces

- `docs/runbooks/head-arm-calibration.md` `## Results`.

## Expected outcome

The head chain measured against the arm is applied on the robot and checked with real numbers.

## Steps

- Run the full suite; deploy; run the procedure with the user; record; commit `docs(runbooks): head from arm calibration results`.
- Run `grep -q '^## Results' docs/runbooks/head-arm-calibration.md`; expect exit 0.

## Notes

- Mirror `docs/runbooks/head-floor-fit.md` (preconditions, deploy, UI steps, acceptance checks, results table).
- Design: `ticket.py path head-and-arm-calibration-fro design` (decisions D1-D16, review focus).
