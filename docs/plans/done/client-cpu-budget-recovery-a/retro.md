# Client CPU budget recovery after overload retro

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

Asked: win back ~1.0-1.2 of the Pi 5 client's 4 cores with eight user-picked savings (IMU 25 Hz, EventsExecutor in five nodes, head camera 15 fps, raw subscriptions in marker_detector and mcp_server, idle filter_node feedback, costmap_zones changed-masks, Alloy 15 s), plus a runbook.
Shipped: all eight items, the runbook, and a second pass added after measurement (nav2 bonds off, 3 unused servers removed, mcp_server feeds node). 17 task files: 15 code tasks built and verified, 1 runbook task, 1 discovered task rejected (folded into the runbook).
Measured (docs/runbooks/client-cpu-budget.md ## Results): 3.74 -> 3.31 (after the 8 items) -> 2.29 cores; idle 0-1 % -> 41 %; load 25-42 -> 5.0.
Verdict: pass (2026-10-22). Not measured: EKF rate; `failed to get costmap frame (odom)` re-check after a drive.

### Planned vs built

Verifier head = `tasks[].verification.head` from the plan-execute returns (run1 / tail / pass2). Rounds from the returns. Red = implementer's own `tests.red` run; "none recorded" means the return carries no red run (a `tdd-revert` result is not quoted as red). Files = `filesChanged` in the return, which lists only the last round's files for several tasks (unknown for the rest).

| Task | Status | Rounds | Continuations | Files (return) | Verifier head | Verification tail | Red (implementer) |
|---|---|---|---|---|---|---|---|
| imu-publish-rate-25-hz | built | 1 | unknown | client.yml, tests/test_client_cpu_load.py, 3 READMEs | b600cf08 | targeted pytest + lint + test-ansible, exit 0 | 2 failed, 5 passed (`assert 50 == 25`; `KeyError: imu_baseline_alpha`) |
| swerve-events-executor | built | 2 | unknown | tests/test_executor_select.py | c97cbcb4 | 215 passed; lint ok | none recorded |
| laser-filter-events-executor | built | 2 | unknown | tests/test_executor_select.py | 14060dff | lint ok, exit 0 | none recorded |
| marker-events-executor | built | 2 | unknown | tests/test_executor_select.py | f54ef8b7 | 159 passed; lint ok | none recorded |
| filter-node-events-executor | built | 3 | unknown | unknown (empty in return) | 2d652035 | 74 passed; lint ok | 4 failed, 64 passed, 6 errors (the verifier ran the red against merge-base files) |
| feetech-events-executor | built | 3 | unknown | tests/test_executor_select.py | b8759ed3 | 442 passed; lint ok | `ModuleNotFoundError: std_srvs`, 7 passed, 5 errors (import error, not the feature absence; weak red) |
| client-executor-config | built | 2 | unknown | tests/test_nav_stack_config.py | d12cc3f5 | 1416 deselected, exit 0; lint + test-ansible exit 0 | none recorded |
| head-camera-15-fps | built | 2 | unknown | head_camera_cpp README | 7fb71ada | targeted tests, lint, test-ansible ok | none recorded |
| marker-detector-raw-image | built | 1 | unknown | node.py, test_node_raw.py, ros_stubs.py, README | 42ba340f | 162 passed; lint ok | `no attribute 'deserialize_message'`, 3 errors |
| filter-node-idle-feedback | built | 2 | unknown | node.py, test_feedback_takeover.py, README | b3cebea6 | node tests + lint + test_client_cpu_load + test-ansible ok | 1 failed (`At index 0 diff: 1 != 0`); processGap: no red for the client.yml test (client.yml edited first) |
| mcp-raw-imu-and-head-states | built | 1 | unknown | ros_iface.py, staleness.py, test_raw_monitor_feeds.py, README | 950f3009 | 93 passed; lint ok | `cannot import name 'QueuedMonitorFeed'`, 1 error |
| costmap-zones-changed-masks | built (ruled) | 2 | unknown | costmap_zones README | 929e877b | 205 passed; ruff + vulture ok | none recorded |
| alloy-scrape-interval-15-s | built | 3 | unknown | tests/README, web_ui README, claude_agent README | c8e4973a | 124 targeted tests passed; lint ok; test-ansible not run by reviewer | none recorded (declined: "cannot be seen from the final tree") |
| cpu-budget-runbook | built (ruled) | 0 in run1 (dependency-failed, unverified), 2 in tail | unknown | docs/runbooks/client-cpu-budget.md, scripts/node_cpu_compare.yaml | de074240 (tail) | 3 passed; lint ok | none recorded |
| mcp-server-raw-feed-subscrip (pass 2) | built | 2 | unknown | ros_iface.py, test_feeds_executor.py, README | f78c5b90 | 2839 passed (4:08); lint ok | 1 failed, 8 passed (`test_follower_joint_states_subscribe_and_destroy_on_the_feeds_node`) |
| nav2-bonds-off-unused-server (pass 2) | built, orchestrator re-verified | 3 | unknown | nav2_bringup README, nav2_params.yaml (+ navigation_launch.py, client.yml per commits) | 5b9350fa | orchestrator rerun of verify-task.py after the launch-file replacement: 319 passed (-k nav), lint ok, test-ansible ok | none recorded |
| local-filters-transform-tole | not built (rejected, folded into runbook) | - | - | - | none: unverified | - | - |

Approvals (`approvals.json`; status from the returns, A7/A9 per the orchestrator's rulings):

| Id | Text | Decisions | Tasks | Status |
|---|---|---|---|---|
| A1 | /imu/data at 25 Hz, consumers checked | D1 | imu-publish-rate-25-hz | delivered |
| A2 | EventsExecutor in five node groups behind `executor:` switch | D2 | swerve-, laser-filter-, marker-, filter-node-, feetech-events-executor, client-executor-config | delivered |
| A3 | head_camera_cpp 15 fps | D3 | head-camera-15-fps | delivered |
| A4 | marker_detector raw head image/camera_info | D4 | marker-detector-raw-image | delivered |
| A5 | filter_node throttled feedback while idle | D5 | filter-node-idle-feedback | delivered |
| A6 | mcp_server raw /imu/data and /head/joint_states | D6 | mcp-raw-imu-and-head-states | delivered |
| A7 | costmap_zones publishes only changed masks; odom frame error cause found | D7 | costmap-zones-changed-masks | delivered (return said `escalated`, gate-repeat; ruled false positive; cause is TF timeliness outside costmap_zones, fix conditional, see below) |
| A8 | Alloy 15 s, rate windows widened | D8 | alloy-scrape-interval-15-s | delivered |
| A9 | runbook executed by the orchestrator after merge | D9 | cpu-budget-runbook | delivered (return said `escalated`; ruled false positive; results written in e1fd716b) |
| G1 | mcp_server decodes every queued IMU sample at the monitor tick | D10 | mcp-raw-imu-and-head-states | delivered |

The second-pass tasks (mcp-server-raw-feed-subscrip, nav2-bonds-off-unused-server) have no ledger entry: the approvals returns of the tail and pass2 runs list none for them (pass2 `approvals` is empty). Status: unknown.

### Deviations and rulings

- costmap-zones-changed-masks: gate-repeat escalation on "parking wording" at nodes/costmap_zones/README.md lines 122/124. Accepted as false positive: the lines state the measured root cause (TF timeliness under load, filters outside costmap_zones). Cost if wrong: a deferral hides an unfixed cause of the odom frame errors; mitigated by the conditional runbook check.
- local-filters-transform-tole (discovered task): rejected. Raising the local filters' transform_tolerance 0.2 -> 0.5 is conditional on the errors persisting after the CPU fixes; folded into cpu-budget-runbook as a check. Cost if wrong: the errors persist until someone runs the check; the Results section says it is still to be re-checked after a drive.
- cpu-budget-runbook: gate-repeat escalation on runbook lines 3, 46 and yaml line 31. Accepted as false positive: the runbook is executed by the orchestrator after merge by design (A9), the transform_tolerance step is deliberate, `switch_time: 1` is a placeholder per file convention. Cost if wrong: a runbook that proves nothing; offset by the results actually measured (3.74 -> 2.29).
- Second pass added after measurement: after 8 items the total was only 3.31 cores (mcp_server 0.59, nav2 0.58). Added nav2 bonds off plus 3 unused servers removed, and the mcp_server feeds node. Why: nav2 crashed twice with bond-reset segfaults (22:58, 23:36), the second triggered by an orchestrator-run rates probe. Cost if wrong: nav2 runs without bond supervision (`bond_timeout 0.0`, container `on_exit=Shutdown`, systemd `Restart=always`); a hung server would no longer be detected by bonds.
- nav2-bonds-off-unused-server: the implementer's navigation_launch.py differed from the installed 1.3.13 file in ~370 lines (e.g. the `use_composition` default). Replaced by the installed file plus the two intended edits (5b9350fa, with 491148fc / 092ef461 in the history) and re-verified by the orchestrator. Cost if wrong: an unintended nav2 launch change on the robot; the orchestrator's diff check and rerun is the only guard (the first verifier pass had missed it).
- Declined findings upheld by the plan-execute reviewers: nav2 verification claims a1/a2/a4 "unmet" (they are met in other files); mcp-server-raw-feed full-suite and `__main__` wiring test.
- forwardMerged: none in any of the three returns.

### Duplicates reconciled

none (all three returns: `duplicates: []`).

### Discovered and blocked

- Local filters' transform_tolerance (0.2) may need raising to 0.5 if `failed to get costmap frame (odom)` persists. Rejected as a task, kept as a runbook check; the orchestrator re-checks after a drive.
- EKF `/odometry/filtered` ~30 Hz not measured (web_ui does not forward it; topic_scraper_api left disabled). Who can clear it: the orchestrator or the user, by enabling topic_scraper_api temporarily.
- cannotVerify, unchanged by rulings: rclpy EventsExecutor live behaviour of the six nodes and of the mcp_server feeds node (shutdown from another thread, runtime-created subscriptions / InvalidHandle risk; rclpy not installed locally, tests mock it). Partly settled by the live deploy and the 2.29-core measurement, but no explicit executor check recorded. Who can clear it: the orchestrator on the robot.
- test-ansible / poe lint not run by the independent reviewer for alloy-scrape-interval-15-s and client-executor-config (the verify-task script did run them for the latter).
- Grafana panels at the 15 s interval on the deployed host: not checked in the files I read. Unknown.
- Process gaps from the returns: filter-node-events-executor red evidence reconstructed by the verifier, not the implementer; tests/README.md did not list nodes/filter_node/tests/test_feedback_takeover.py at review time; nav2 round-2 edit to tests/README.md unnamed.
- Nothing in `undelivered`, `blocked` or `misattributed` in any return.

### Research that changed the plan

- Measurement after the first eight items (3.31 cores, nav2 0.58, mcp_server 0.59) and two nav2 bond-reset segfaults (22:58, 23:36) added the second pass: nav2 bonds off, 3 servers removed, mcp_server feeds node. Source: docs/runbooks/client-cpu-budget.md ## Results.
- rclpy Jazzy source: `EventsExecutor.spin_once` honours its timeout (runs `events_queue_.Run(end)`), so control-loop pacing is unchanged (filter-node-events-executor process gap).

### Cost

Agents by role, continuations, audits: unknown (not in the inputs beyond per-task rounds: 1-3). Three plan-execute runs (run1, tail, pass2); wall-clock unknown.

| Task | Estimate (min) | Actual (min) |
|---|---|---|
| alloy-scrape-interval-15-s | 60 | unknown |
| feetech-events-executor | 60 | unknown |
| imu-publish-rate-25-hz | 60 | unknown |
| marker-events-executor | 40 | unknown |
| swerve-events-executor | 45 | unknown |
| client-executor-config | 40 | unknown |
| costmap-zones-changed-masks | 75 | unknown |
| cpu-budget-runbook | 60 | unknown |
| filter-node-events-executor | 45 | unknown |
| filter-node-idle-feedback | 75 | unknown |
| head-camera-15-fps | 45 | unknown |
| laser-filter-events-executor | 40 | unknown |
| local-filters-transform-tole | none | unknown |
| marker-detector-raw-image | 50 | unknown |
| mcp-raw-imu-and-head-states | 75 | unknown |
| mcp-server-raw-feed-subscrip | 75 | unknown |
| nav2-bonds-off-unused-server | 60 | unknown |

README Time: Estimate 905 min | Actual 0 min | Remaining 60 min (actuals were never filled in).

### MRs and deploy

No MRs: task branches merged locally into main (e.g. e6092364, 60273ff6, feb3c541, c4bebd57, 9a54af81..491148fc, ccaed407, 4a19d85a); not pushed by this record's author. Deploy: client --all, monitoring client and server, then nav2_bringup and mcp_server (README Review). Results commit e1fd716b. nav2 active since 00:46 with bonds off.

### Lessons routed to

- Reviewer must diff a replaced upstream file against the installed one (nav2 launch file: ~370 unintended lines passed one verifier): add a test in tests/test_nav_stack_config.py that pins navigation_launch.py to the installed file minus a named edit list (mechanical).
- Parking-wording gate flags README prose describing root causes and runbooks executed after merge (3 false positives): scope the gate to added code/TODO markers or let tasks pre-declare "runbook executed post-merge"; the rule is a judgement call, edit the plan-execute parking checklist.
- Rates probes against a saturated nav2 caused a segfault: a runbook rule, not a check - edit docs/runbooks/client-cpu-budget.md to say no `ros2 topic hz`-style probes while the client is saturated or before bonds are off (judgement).
- Missing red evidence on 8 of 15 code tasks (tdd-revert not accepted): add a verify-task requirement that `tests.red` is non-empty for any task adding a test file (mechanical, plan-execute check).
- Test files added without a tests/README.md entry (filter_node test_feedback_takeover.py): add a root test that every tests/ and nodes/*/tests file is listed in tests/README.md (mechanical).
