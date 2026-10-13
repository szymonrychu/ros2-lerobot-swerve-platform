# C++ nodes record

Plan: docs/plans/2026-10-13-cpp-nodes-plan.md (tasks.json, approvals.json, design.md alongside). Commits: `git log 04f3af7..HEAD` on main (44 commits, 4af78ac..9ff83fd).

## Summary

Asked: cut client CPU by moving hot nodes to C++ where it pays. Shipped: C++ scaffolding (ros2_common_cpp, docker test runner, CI job, Ansible local-source colcon build); stereo_depth_cpp and map_guard_cpp cut over with the Python nodes kept present and disabled; the mcp_core_cpp nanobind extension with five kernel families (camera, grasp, ik_reach, collision, frontiers) behind golden parity tests; web_ui and mcp_server on-demand subscriptions; a 10 Hz topic_tools throttle with dual-rate joint states; uvc MJPEG pass-through. All 15 planned tasks (T1-T15) are on main, plus follow-up tasks T11b, T12b and three on-demand and two throttle tasks. CPU (15-min Mimir windows, cores): map_guard 0.33 to 0.045; web_ui 0.615 to 0.157; mcp_server 0.657 to 0.522; gripper_uvc 0.17 to 0.071; total of scraped nodes 2.51 to 1.90. Verdict: delivered, with stereo_depth_cpp idle pending hardware and calibration (see Deferred). Several tasks ended on procedural not-clean stops, not defects; the files were read and the work landed (see rulings).

## Planned vs built

Six workflow runs: we08aquxr (run 1, T1-T15), wt1ksrlnr (run 2, T8, T10-T15), wwzv4zkze (run 3, T11-T15), wgw661vg4 (run 4, T11b and T12 rework), wvfkulza2 (run 5, on-demand), w2amqc3b2 (run 6, throttle). Rounds are "initial + continuation" from the run that last touched the task. Verifier head is `tasks[].verification.head` of that last run; it is a branch SHA and differs from the SHA landed on main where the merge rewrote it. Verification tail is the verifier's own evidence. Status "landed" means the code is on main; "clean" is the workflow's own flag.

| Task | Status | Rounds | Files | Verifier head | Verification tail |
|---|---|---|---|---|---|
| T1 scaffolding (ros2_common_cpp, cpp-node-test.sh, CI, poe) | landed, clean | 1 + 2 (continuation stopped "blocked" on merge/T8 ordering) | 19 | 4af78ac | pytest 23 passed; poe lint ok; no CI amd64 run observed |
| T2 Ansible colcon local-source build, _cpp registration | landed, clean after continuation | 2 + 2 (first stop "blocked": deploy only) | 0 in last run | 8a9c236 | lint/syntax chain exit 0; pytest summary line not captured in the tail |
| T3 stereo_depth_cpp | landed, not clean (procedural) | 1 + 1 | 26 (+2) | 03a2f4e | 84 gtests, 0 failures; fixtures up to date |
| T4 map_guard_cpp | landed, not clean (procedural, review-shallow) | 3 + 1 | 15 | c150386 | cpp-node-test map_guard_cpp PASS; fixtures up to date |
| T5 web_ui lazy serialization, JPEG pass-through | landed, not clean (review-shallow) | 3 + 1 | 7 (+9) | e3f950e | web_ui tests, ruff, vulture ok; exit 0 |
| T6 uvc_camera MJPEG pass-through | landed, clean | 1 + 1 | 8 | 28b3bfb | 64 passed; lint ok |
| T7 mcp_server latest-only state cache | landed, clean | 1 + 2 (no-progress stop, procedural) | 6 | 4545b7b | lint ok, 154 files formatted |
| T8 cut-over and node_cpu_compare | landed, clean (run 2) | 1 | 15 | f12cb74 | pytest summary line not captured; C++ suites not run on that host |
| T9 mcp_server kernel bench and ranked report | landed, not clean (round-cap, procedural) | 1 + 3 | 9 | c23284d | 65 passed; lint ok |
| T10 mcp_core_cpp scaffold and parity harness | landed, clean after continuation | 1 + 2 | 27 | 0228d97 | 18 passed; lint ok |
| T11 camera family | landed via T11 + T11b; T11b clean | T11 1; T11b 1 + 2 | 14; 7 | T11 d536514; T11b 09ea908 | T11b: camera and frontier parity 33 passed; lint ok |
| T12 grasp family | landed, clean after rebuild on ik_reach solver | run 3: 3 (round-cap); run 4: 1 + 1 | 2; 22; 5 | d150298 | full suite 2413 passed per audit; one least_squares definition |
| T13 ik_reach family | landed, clean | 2 | 5 | f342fe7 | lint ok; parity tests green |
| T14 collision family | landed, clean after continuation | 3 (round-cap) + 1 | 0; 3 | 8d8a011 | full suite 2361 passed per audit, no nanobind leak lines |
| T15 frontiers family | landed, not clean (review-shallow, procedural) | 1 | 12 | 99b0243 | lint ok; parity tests green |
| T1 (follow-up) web_ui on-demand subscriptions | landed (e4003bf), not clean (round-cap, procedural) | 3 | 0 in last run | dc13468 | web_ui 698 passed per audit |
| T2 (follow-up) mcp_server on-demand | landed (fe0d138), return "blocked" on deploy | 1 | 0 | c56674b | 2432 passed per audit |
| T3 (follow-up) drop 20 Hz monitor timer | landed (93d85d3), clean | 0 | 4 | f0de53d | 2421 passed per audit |
| T1 (throttle) topic_tools throttle nodes | landed (9b1ec52), clean | 1 | 8 | 9b1ec52 | pytest summary line not captured; chain exited 0 |
| T2 (throttle) mcp_server dual-rate joint states | landed (23bb462, 9ff83fd), not clean (round-cap, procedural) | 3 | 0 | 9ff83fd | mcp_server 2464 passed, root 1116 passed per audit |

No task has an empty `verification.head`: none is marked unverified in the final runs. In run 1, T8 and T10-T15 were unverified (dependency-failed, 0 rounds); each was re-run and has a head above. The red evidence (`tests.red`) is null for T2 (run 1), T14 (run 3), the follow-up T1/T2 and the throttle T2 in the returns; the verifiers or scope audits re-ran the red proof in scratch copies for the follow-up T1 and throttle T2 (reverting sources to the merge-base failed the new tests) and the record relies on those, not on the implementers. T1 (scaffolding) and T11 report cannot-verify for a red step visible in a single commit.

Files that contradict summaries: the tails for T2, T8 and throttle T1 say the pytest summary line was not captured, so those rows rest on exit 0 and the audits' own re-runs, not a quoted summary. The follow-up T1 verification evidence notes the pipe hid the exit code.

### Approvals

Returns of the final run touching each entry. K1 and K3 each reappear in later runs with different text (follow-up work); those rows are listed separately.

| Id | Text (short) | Decisions | Tasks | Status |
|---|---|---|---|---|
| K1 | Scaffolding, cheap fixes (web_ui lazy serialization, uvc pass-through, mcp state cache), CPU before/after | D1, D4 | T1, T2, T5, T6, T7, T8 | delivered (run 1 returned blocked on T8 and unmerged branches; T8 delivered in run 2; all landed on main) |
| K2 | stereo_depth_cpp and map_guard_cpp side by side, Python kept disabled | D1, D2, D3 | T3, T4, T8 | delivered (run 1 blocked on T8; T8 delivered in run 2, f12cb74). stereo_depth_cpp idle, see Deferred |
| K3 | mcp_core_cpp kernels, mcp_server stays Python, master2master not rewritten | D5 | T9-T15 (+T11b, T12 rework) | delivered (run 4 return). Runs 1-3 returned escalated |
| K1 (follow-up, run 5) | web_ui and mcp_server stop paying for unused traffic | not in ledger file | T1, T2, T3 | return: blocked (deploy of mcp_server needed merge and a go-ahead because deploying mcp_server kills a running agent session); landed and deployed by the orchestrator |
| K1 (follow-up, run 6) | mcp_server full rate only while moving, 10 Hz copy otherwise | not in ledger file | T1, T2 | return: escalated (round-cap, procedural); landed by the orchestrator, 9b1ec52..9ff83fd |

The follow-up K1 rows are not in `2026-10-13-cpp-nodes-approvals.json`; their ids and text come from the run returns.

## Deviations and rulings

1. Scope limits (user). web_ui and mcp_server stay Python; the mcp_core_cpp extension is allowed. master2master is not rewritten (disabled). swerve, filter and the servo bridges are deferred pending explicit approval. Cost if wrong: a missed CPU gain on those nodes; reversible, nothing was rewritten.
2. Procedural not-clean stops cascaded through rounds 1 to 3 (review-shallow, round-cap, no-progress, blocked-on-deploy). Why: reviewers did not read named files, or the implementer report lacked red evidence; T2 stopped only on deploy, which the plan assigns to the orchestrator; dependent tasks (T8, T10-T15) then reported dependency-failed with 0 rounds. Ruling: land the verified work each time and re-run the remainder. Cost if wrong: an unreviewed defect on main; mitigated by the scope audits and full-suite re-runs (2361, 2413, 2464 passed) and the CPU measurement.
3. Merge ordering. Branches were not merged by the workflow; the orchestrator merged them, so verifier heads differ from main SHAs. Version bumps were centralised: one mcp_core_cpp bump plus lock after the ports (0.2.0, then 0.2.1, 0.3.0), because the Pi deploy rebuilds only on a uv.lock change (f342fe7, 85cec3e, 41ea284). Cost if wrong: a stale extension on the Pi.
4. Deploy fixes (rulings): libturbojpeg0 renamed to libturbojpeg on noble (71bf382); one retry after a transient DNS drop; the throttle-topic check initially missed the discovery env and was fixed. Cost if wrong: failed deploys; each was re-run to success.
5. CPU profile (docs/plans/2026-10-13-cpp-nodes-briefs/webui-mcp-cpu-debug.md, a7842a5) showed web_ui and mcp_server cost was executor wakeups, not serialization. Follow-up runs 5 and 6 added on-demand subscriptions and the 10 Hz topic_tools throttle with dual-rate joint states. Motion-critical topics stay at full rate. Cost if wrong: stale data for a motion-critical consumer; pinned by tests (MOTION_CRITICAL_TOPICS, 9ff83fd), not yet seen on a live arm motion except as the orchestrator confirmed.
6. forwardMerged: none in any of the six returns.

## Duplicates reconciled

- Config reading (T1 vs T4): map_guard_cpp had its own scalar reader (classify/as_bool/as_int/Reader) next to the ros2_common_cpp config helpers. Kept: the ros2_common_cpp helpers; map_guard_cpp's config reader now goes through them (f12cb74).
- Least-squares solver (T12 vs T13): two independent TRF and arm IK ports (grasp::least_squares_trf, ik_reach::trf::least_squares). Kept: T13's ik_reach solver; T12 was rebuilt on it (1e3adda), exactly one least_squares definition, pinned by a test.
- Occupancy thresholds (T11): defined in grid.hpp and perception.py. Kept: perception.py as the single source, passed into map_counts and map_crop (15c6298).
- printf-style format helper built twice inside T12 (ik_reach and grasp): flagged by the run 4 audit; the grasp copy went with the solver removal (the audit saw one remaining solver, not the helper; the helper was not re-read here: unknown).
- Python export of the ikpy chain (T12 vs T13): flagged in run 3; resolved by the same rebuild.

## Deferred and blocked

- stereo_depth_cpp is idle until the right camera (cam1) probes (hardware) and calibration runs. Clears: the user.
- Python map_guard and stereo_depth stay present but disabled, for rollback. Clears: the user, when the C++ nodes are trusted.
- swerve, filter and the servo bridges: not rewritten, pending explicit approval. Clears: the user.
- graphify rebuild: reported as a process gap by T1, T3 and T11 (graphify-out/ is orchestrator-owned). Not confirmed done in the files read: unknown. Clears: the orchestrator.
- Declined and cannot-verify items left standing in the returns, not settled by a ruling: C++ CI job on an amd64 runner (only arm64 Mac observed); apt package availability on arm64 taken from the README; GCC 13.3 aarch64 warning-clean compile; cold plan bench 22.0 ms vs 78.4 ms (implementer claim, not re-run by the verifier); topic_tools throttle adopting the source QoS on Jazzy (tests check config only); real rclpy behaviour of creating and destroying subscriptions mid-spin (tests use a fake node); ros2_node_deploy hashes only pyproject.toml and uv.lock, so a C++ change to mcp_core_cpp needs a version bump (Ansible role edit outside T10). Clears: the orchestrator or user on the robot; the deploy-hash gap needs a plan decision.

## Research that changed the plan

- T9 ranked profile (bench report.json, 8e83b7f): port-verdict kernels in all five families (camera 3, grasp 1, ik_reach 2, collision 2, frontiers 4). Change: T11-T15 ported only those kernels.
- CPU profile (docs/plans/2026-10-13-cpp-nodes-briefs/webui-mcp-cpu-debug.md, a7842a5): cost was executor wakeups, not serialization. Change: follow-up on-demand subscriptions, 20 Hz monitor timer removed, 10 Hz throttle with dual-rate joint states instead of a web_ui or mcp_server rewrite.
- Parity checking (c1586d5): the Pi's numpy/IK differs from the Mac by about 1e-10, so the collision golden regeneration compares inputs within 1e-9; GCC 13.3 aarch64 bit-exact parity is recorded in docs/collision.md.

## Cost

Agents per workflow run: 98, 17, 41, 19, 18, 14 (207 total). Tokens per run: 6,185,895; 1,140,061; 3,497,174; 1,120,033; 1,168,760; 984,580 (about 14.1M). Scope audits: one per run (six); two returned complete=false (runs 1 and 2) and were resolved by the re-runs. Continuations: T1, T2, T3, T4, T5, T6, T7, T9, T10, T14, T11b, T12 each had at least one re-run entry in the returns. Wall-clock: main commits span 2026-10-13 08:40 +0200 (base 04f3af7) to 17:59 +0200 (9ff83fd); run outputs finished 10:37 to 18:28. Planning time before the first run: unknown.

## MRs and deploy

No MRs or PRs: work was merged to main by the orchestrator and pushed (project convention, single-agent push to main). Remote git@github.com:szymonrychu/ros2-lerobot-swerve-platform.git; local main tracks origin/main. CI state of the pushed commits: unknown, not read. Deploy: performed by the orchestrator per rulings; the three deploy issues above were fixed. Deploy logs under .logs/ were not read for this record. The CPU numbers in the Summary come from the orchestrator's Mimir windows and were not re-measured here; the T8 switch_time placeholder was fail-closed until deploy.

## Lessons routed to

- Reviewers declared "review-shallow" because named files were not read, and implementers omitted `tests.red`: add a check to the plan-execute return validator that rejects a task without `tests.red` unless a TDD exception is named, and one that fails a review round that lists no read of a file the diff touches. Mechanical.
- Procedural stops cascaded into dependency-failed for six tasks: change the plan-execute rule so a dependent is dispatched when its dependency is delivered with only procedural stops; this is a workflow rule edit (judgement call), routed to workbench:plan-execute.
- Duplicates (config reader, TRF solver, occupancy thresholds) came from per-family or per-task file ownership: add a test that counts definitions (one `least_squares`, no threshold constants in `grid.hpp`); they already exist for the solver and thresholds. For a shared helper the owning plan should name the helper in the task requirements (judgement call, start-development planning guidance).
- Deploy package name error (libturbojpeg0): extend the existing test that cross-checks `_cpp` apt_packages with package.xml to also check names with `apt-cache policy` in a docker noble image, or a lint on known renamed packages. Mechanical.
- Deploy-hash gap (ros2_node_deploy ignores mcp_core_cpp sources): add a test that fails when an extension source changes without a version bump and lock change. Mechanical.
- Deploying mcp_server kills a running agent session (project memory): the plan-execute return left it as blocked; add a precheck in `scripts/deploy-nodes.sh` that refuses to deploy mcp_server while a claude_agent session is active. Mechanical.
