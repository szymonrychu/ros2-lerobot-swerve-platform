# Per-pose quality hints in the RGB-D calibration capture record

## Summary

Asked: per-pose quality hints in the head RGB-D calibration: a capture-time hint (few corners) per pose, and after Compute each pose's RGB, ToF and stereo reprojection error with outliers flagged; shown in the web_ui Calibrate pose list. Hints only, no pruning.
Shipped: all 5 tasks, merged (5f9f96b1) and deployed (head_rgbd_calibration, web_ui). Live check 2026-10-22 01:03 passed for hints and compute (r-20261022-010256, 29 poses with errors, no outliers).
Not live-verified: the outlier path (pose 007 had already been deleted by the user; outlier flagging was exercised only by synthetic data).
Verdict: pass (2026-10-22). A root test (tests/test_jetson_retired.py) failed after merge; fixed on main (90e642da).

## Planned vs built

Verifier head = `tasks[].verification.head` from hints-run1-return.json. Red = implementer's own `tests.red`; `tdd-revert` results (failedOnBase true) are not quoted as red. Files = `filesChanged` in the return (last-round list; unknown where empty).

| Task | Status | Rounds | Continuations | Files (return) | Verifier head | Verification tail | Red (implementer) |
|---|---|---|---|---|---|---|---|
| per-pose-errors-and-outliers | built | 2 | unknown | tests/test_head_rgbd_calib_quality.py (+ calibrate.py, extrinsics.py per the verification) | 331a7ac9 | lint ok, exit 0, 141 files formatted; a1-a10 met | none recorded |
| capture-corner-hints | built | 2 | unknown | tests/test_head_rgbd_calib_hints.py, node test_store.py, test_app.py | 9258c6ea | exit 0; 67 node tests, 138 full node suite; lint ok | none recorded |
| compute-result-pose-errors | built | 2 | unknown | node tests/test_compute.py | e45812cd | 136 passed, 1 warning; lint ok | none recorded |
| calibrate-pose-list-hints | built | 1 | unknown | rgbdApi.ts, rgbdApi.test.ts, RgbdCapture.tsx, RgbdCapture.test.tsx | 44356aba | vitest 117 passed; tsc clean; eslint 0 warnings | `poseErrors is not a function`; 2 files failed, 6 tests failed |
| hints-docs-and-deploy | built (live check done after merge, see rulings) | 1 | unknown | unknown (empty in return; READMEs per commits cb31508e, 8c540d68) | 8c540d68 | root lint ok, 143 files formatted | none recorded (docs task) |

Approvals (`approvals.json`; the return's A4 was `blocked`, cleared by the ruling):

| Id | Text | Decisions | Tasks | Status |
|---|---|---|---|---|
| A1 | Per-pose RGB/ToF/stereo errors, outlier flag max(3 x median, limit), report.md "delete pose NNN and Compute again" | D1-D4 | per-pose-errors-and-outliers, compute-result-pose-errors | delivered |
| A2 | Capture-time corner hints in pose.json and session API | D5 | capture-corner-hints | delivered |
| A3 | Calibrate page pose list shows hints and errors, flagged pose highlighted | D1, D6 | calibrate-pose-list-hints | delivered (browser rendering of the red border not verified) |
| A4 | Tests, READMEs, tests/README.md, deploy via deploy-nodes.sh | D7 | all 5 tasks | delivered (return: blocked until merge to main and push; merged 90e642da, deployed 2026-10-21 23:2x) |
| G1 | Few-corners hint below half of 20 inner corners (10) | D8 | capture-corner-hints | delivered |

## Deviations and rulings

- hints-docs-and-deploy: accepted (rulings.json). The task could not deploy or run the live check from its branch (deploy pulls origin/main), so the orchestrator merged, deployed and ran the live check. Live: GET session returned hints (few corners on 9/8/6-corner poses); compute r-20261022-010256 passed (rgb 0.130, tof 0.216, stereo 0.200 px; 29 RGB/ToF views, 28 stereo pairs), no outliers, pose 007 already deleted. Cost if wrong: the outlier flag and the "delete pose NNN" wording in failed_limits were never seen on a real failing session, only on synthetic data.
- Root test failure after merge: tests/test_jetson_retired.py (retired-word allowlist) failed on the new `stereo` calibration wording. a5fc01fb added a one-line allowlist entry and was committed with the test still failing because of a pipeline exit-code mistake (the failure was masked); corrected in 90e642da (allowlist for the RGB-D per-pose stereo wording). Cost: a red commit on main between a5fc01fb and 90e642da; nothing pushed.
- Declined findings upheld: capture-corner-hints verification claims "a1/a3-a6 not met" contradicted by the tree (8 root hint tests and 67 store/app tests passed); per-pose-errors-and-outliers claims a1, a3-a7 unmet carried no evidence.
- forwardMerged: none.

## Duplicates reconciled

none.

## Discovered and blocked

- Outlier path on real data (pose 007, 66.6 px) not exercised live. Who can clear it: the operator, on a session that holds a bad pose.
- Browser rendering of the highlighted pose row (red border, warning colours) not verified. Who: the user.
- Full suites: node and frontend were run on the integrated branch per the README Review (root 1982, head_rgbd_calibration 141, frontend 1225 passed), not by the per-task verifiers.
- Process gaps: tests/README.md and node README initially lacked entries for test_head_rgbd_calib_hints.py and capture hints (fixed by cb31508e, 2bb4a6d9, 8c540d68); the deploy-then-verify task ordering needed the integrated branch on origin main.
- No `escalated`, `discovered` or `misattributed` entries.

## Research that changed the plan

none.

## Cost

Agents by role, continuations, audits and wall-clock: unknown (single plan-execute run; rounds 1-2 per task).

| Task | Estimate (min) | Actual (min) |
|---|---|---|
| per-pose-errors-and-outliers | 60 | unknown |
| capture-corner-hints | 55 | unknown |
| compute-result-pose-errors | 40 | unknown |
| calibrate-pose-list-hints | 55 | unknown |
| hints-docs-and-deploy | 45 | unknown |

README Time: Estimate 255 min | Actual 0 min | Remaining 45 min (actuals were never filled in).

## MRs and deploy

No MRs: epic merged locally as 5f9f96b1 (main), root-test fix 90e642da; not pushed by this record's author. Deploy log .logs/deploy-client-hints.log; head_rgbd_calibration and web_ui deployed to the client. Live evidence .logs/hints-live-session.json and .logs/hints-live-result.json.

## Lessons routed to

- Retired-word allowlist broke on new domain wording after merge: the integrated-branch run must include the root suite before the merge is declared done (mechanical): make the merge step run `uv run pytest tests/` and fail on non-zero.
- The red commit a5fc01fb came from a pipeline exit-code mistake (masked failure): add `set -o pipefail` or run the test without piping to `tail`/`tee` in the merge script (mechanical, hook or script lint).
- Deploy task ordered before merge cannot ship code (deploy pulls origin/main): a plan-lint rule that a deploy/live-check task depends on the merge step, not on task branches (mechanical, plan template check).
- Verifier "not met" claims without evidence needed manual rebuttal twice: a judgement call, edit the plan-execute reviewer prompt to require a file:line quote per unmet condition.
