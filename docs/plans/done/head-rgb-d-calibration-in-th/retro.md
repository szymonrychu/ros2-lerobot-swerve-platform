# Head RGB-D calibration in the web UI Calibrate page retro

## Review

Migrated from the epic format.

## What went well

none recorded in the epic format

## What went wrong or could improve

none recorded in the epic format

## Recommendations

none

## Delivery record (migrated)

### Summary

Asked: capture, review, compute and apply the head RGB-D calibration from the web UI Calibrate page, all on the robot. Shipped: robot-side head_rgbd_calibration node, shared calibration core in shared/ros2_common, web_ui proxy and Calibrate page section (live capture, pose list, sessions, results, apply, download), head nodes and URDF reading the applied calibration, Ansible deploy, runbook. Verdict: pass (2026-10-20), E2E on the robot. 26 of 26 tasks marked done. Short-form record: no plan-execute return value was supplied, so per-task review rounds, continuations, files, verifier heads and verification tails are unknown.

### Planned vs built

| task | status | review rounds | continuations | files | verifier head | verification tail |
|---|---|---|---|---|---|---|
| calib-core-calibrate-run | done | unknown | unknown | unknown | unknown | unknown |
| calib-core-config-charuco | done | unknown | unknown | unknown | unknown | unknown |
| calib-core-extrinsics-urdf | done | unknown | unknown | unknown | unknown | unknown |
| calib-core-frames-intrinsics | done | unknown | unknown | unknown | unknown | unknown |
| calib-core-retire-shims | done | unknown | unknown | unknown | unknown | unknown |
| calib-core-shared-decoders | done | unknown | unknown | unknown | unknown | unknown |
| calib-node-ansible-deploy | done | unknown | unknown | unknown | unknown | unknown |
| calib-node-apply-download | done | unknown | unknown | unknown | unknown | unknown |
| calib-node-cgroup-tier-and-d | done | unknown | unknown | unknown | unknown | unknown |
| calib-node-compute-job | done | unknown | unknown | unknown | unknown | unknown |
| calib-node-http-api | done | unknown | unknown | unknown | unknown | unknown |
| calib-node-live-capture | done | unknown | unknown | unknown | unknown | unknown |
| calib-node-scaffold-config | done | unknown | unknown | unknown | unknown | unknown |
| calib-node-session-store | done | unknown | unknown | unknown | unknown | unknown |
| calib-pose-counter-in-core | done | unknown | unknown | unknown | unknown | unknown |
| head-nodes-use-applied-calib | done | unknown | unknown | unknown | unknown | unknown |
| head-urdf-tof-joint-override | done | unknown | unknown | unknown | unknown | unknown |
| jetson-retirement-allowlist | done | unknown | unknown | unknown | unknown | unknown |
| retire-ssh-live-counter | done | unknown | unknown | unknown | unknown | unknown |
| rgbd-api-contract-frontend | done | unknown | unknown | unknown | unknown | unknown |
| rgbd-api-contract-node | done | unknown | unknown | unknown | unknown | unknown |
| rgbd-calib-frontend-api | done | unknown | unknown | unknown | unknown | unknown |
| rgbd-calib-runbook-and-docs | done | unknown | unknown | unknown | unknown | unknown |
| rgbd-capture-section-ui | done | unknown | unknown | unknown | unknown | unknown |
| rgbd-sessions-and-results-ui | done | unknown | unknown | unknown | unknown | unknown |
| web-ui-rgbd-calib-proxy | done | unknown | unknown | unknown | unknown | unknown |

Per-task status is from the task files (`status: done`). Verifier heads are not in any file I was given, so every task is `unverified` by the record rules except the five in rulings.json, which were accepted by ruling (see Deviations and rulings), not by a verifier head.

Approvals (from `approvals.json`; no plan-execute approvals return supplied, so status is the ledger's `approved`, not `delivered`):

| id | text | decisions | tasks | status |
|---|---|---|---|---|
| A1 | Results live on the robot state dir; Apply makes head nodes and URDF tof_joint use them without redeploy; survives deploys; Download returns YAMLs | D3 D6 D7 D8 D12 | calib-node-session-store, calib-node-compute-job, calib-node-apply-download, calib-node-http-api, head-urdf-tof-joint-override, calib-node-ansible-deploy, head-nodes-use-applied-calib, rgbd-sessions-and-results-ui | unknown (E2E: apply of r-20261020-092536 worked, head_rgbd publishes pairs) |
| A2 | Everything on the robot; live pose count, status and thumbnails | D1 D3 D4 D10 | calib-pose-counter-in-core, calib-node-live-capture, calib-node-http-api, web-ui-rgbd-calib-proxy, rgbd-calib-frontend-api, rgbd-capture-section-ui | unknown (E2E: 38 poses captured) |
| A3 | Pose list: thumbnails with corners, delete, coverage hint, past sessions | D3 D4 D5 D9 D10 | calib-node-session-store, calib-node-live-capture, calib-node-compute-job, calib-node-http-api, web-ui-rgbd-calib-proxy, rgbd-calib-frontend-api, rgbd-capture-section-ui, rgbd-sessions-and-results-ui | unknown |
| A4 | Compute on the robot from stored poses, own cgroup, niced child | D1 D5 | calib-node-scaffold-config, calib-node-compute-job, calib-node-ansible-deploy | unknown (E2E: compute ran on the robot) |
| A5 | Core reused in shared/ros2_common; CLI kept; live.py, live_relay.py, capture.live removed with retirement test | D2 | calib-core-config-charuco, calib-core-frames-intrinsics, calib-core-extrinsics-urdf, calib-core-calibrate-run, calib-pose-counter-in-core, retire-ssh-live-counter, calib-core-retire-shims | unknown |
| A6 | IMU section keeps working on the same page | D9 D10 | web-ui-rgbd-calib-proxy, rgbd-capture-section-ui, head-nodes-use-applied-calib | unknown (not exercised in the E2E notes) |
| A7 | camera_info_file/tof_calibration_file point at applied state files; runbook steps 2-3 describe web UI flow; overlap noted in design.md | D7 D11 | head-nodes-use-applied-calib, calib-node-ansible-deploy, rgbd-calib-runbook-and-docs | unknown |

E2E on the robot 2026-10-20: session calib-20261020-083900, 38 poses, RGB 0.10 px, ToF 0.28 px, stereo 0.28 px RMS; applied r-20261020-092536; head_rgbd publishes pairs (0 drops, valid ratio 0.42).

### Deviations and rulings

- Plane-bias limit 15 mm -> 40 mm, plus rejection of background-contaminated plane pairs (33a73511). What: first compute on the E2E session failed, plane bias 19.9 mm > 15 mm. Why: the check measured the ToF's own depth error (reads 0-40 mm long at 0.2-0.4 m; black squares read about 2x white), so 15 mm refused a good calibration. Ruling by the user. Cost if wrong: a bad plane fit within 40 mm would be accepted; mitigated by the background-pair rejection and the limit being config (`plane_max_bias_mm`, ce334203).
- Verifier crash on `--prev-head unknown` for calib-node-scaffold-config (rulings.json, 2026-10-19). Accepted after the orchestrator re-ran the task verify on HEAD 52434e47: 16 passed, lint clean, 14 passed (ci_workflow, node_lint_coverage), clean tree. Cost if wrong: that task has no independent verifier head.
- calib-core-shared-decoders, calib-node-cgroup-tier-and-d, rgbd-api-contract-frontend, rgbd-api-contract-node accepted by ruling (2026-10-20T09:26:57Z): merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot (capture, compute, apply, head_rgbd pairs). Cost if wrong: no per-task verifier head; live E2E covers the integrated result only.
- Integration fixes after the merges: 1736bd7b and ce334203 (tier tests, plane_max_bias_mm config, fixture allowlist).
- forwardMerged: unknown (no return value supplied).

### Duplicates reconciled

none recorded. The overlap with head-rgb-d-fusion-of-camera (rgbd-final-suite-and-runbook) is noted in design.md.

### Discovered and blocked

- Depth overlay shifts right for close objects (parallax from the ToF depth error). Deferred to a depth correction in the obstacle work; the user confirmed the overlay is otherwise mostly right. Who can clear it: the obstacle-work epic owner (Szymon Richert).
- declined / cannotVerify: unknown (no return value supplied).

### Research that changed the plan

ToF depth error measured on the robot (0-40 mm long at 0.2-0.4 m, black squares about 2x white) changed the plane-bias limit from 15 mm to 40 mm and added background-contaminated pair rejection (33a73511).

### Cost

Agents and wall-clock: unknown. README Time: estimate 1410 min, actual 0 min (not filled), remaining 175 min. Task files carry `estimate` only; every `actual` is empty.

### MRs and deploy

No MR/PR: merged directly on main (merges f42c78bb, e892b6d7, 2a94085f; fixes 1736bd7b, ce334203, 33a73511). Not pushed by this record. Deploy: applied and exercised on the robot 2026-10-20 (E2E above); deploy log not seen.

### Lessons routed to

- Plane-depth limits must be checked against measured sensor error on the real device: add a test that the compute path rejects background-contaminated pairs and that the bias limit comes from config (done in 33a73511 and ce334203; no further check proposed).
- Verifier tooling crashed on `--prev-head unknown`: add a check in the verifier script that handles an unknown prev-head (mechanical; owner is the claude-infra-plugin).
- Task `actual` minutes were never filled: add a lint in ticket.py that refuses `done` with empty `actual` (mechanical).
