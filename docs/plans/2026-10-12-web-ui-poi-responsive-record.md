# web-ui-poi-responsive record

## Summary

Asked: POI tab replacing IMU, kind/zone map colours, phases dropdown, responsive web UI, reliable agent POIs, Arm tab with jog and leader_bridge, generous agent time budgets (A1-A7, 40 tasks).
Shipped: 61 commits on main (6af89a2..d5510f3, 139 files, +9754/-809) over three plan-execute rounds plus orchestrator integration. Deployed to the client (failed=0).
Verdict: all 40 tasks landed. 39 have an independent verifier head; T39 is unverified (already on main, checked by checklist only). Open: live leader teleop move (user), no browser visual check, worktree cleanup.

## Planned vs built

Rounds = review rounds of the main run entry / continuation entry ("-r1"), 0 = none. Head = latest `verification.head` (the verifier's SHA, a worktree branch head unless noted). Files = largest `filesChanged` count across that task's entries. Tail = last line of verifier evidence. Status "built" = landed on main per rulings and `git log`.

| Task | Status | Rounds | Files | Verifier head | Verification tail |
|---|---|---|---|---|---|
| T1 layout foundation | built (stopped round-cap in R1, ruled landed) | 1/3 | 5 | 7719a0f | vitest 18 passed, eslint clean, tsc clean |
| T2 poiItems | built (no-progress in R1) | 0/2 | 2 | baa742f | 11 passed, eslint exit 0 |
| T3 kind colours | built | 1 | 3 | b4383fe | 22 passed, eslint exit 0 |
| T4 smaller photo wedge | built (round-cap in R1) | 3 | 0 in final entry | a1e9d5b | 8 passed, eslint exit 0 |
| T5 colour legend | built | 1 | 3 | c3554ef | 2 passed, eslint, tsc clean |
| T6 PoiTab | built (round-cap in R1) | 0/3 | 2 | f39c61f | 10 passed, eslint exit 0 |
| T7 IMU tab removal | built (round-cap in R1) | 1/3 | 12 | c7ad6a2 | ansible-lint 0 failures |
| T8 show on map popup | built in R2 (dependency-failed in R1) | 1/1 | 9 | b83dffa | 24 passed, eslint, tsc clean |
| T9 phases dropdown | built | 1 | 3 | 3493766 | lint no errors |
| T10 app shell per layout class | built in R2 | 1 | 3 | 3ad0314 | 3 passed, lint, tsc clean |
| T11 map toolbar | built in R2 | 2 | 3 | 72632a9 | 10 passed, lint clean |
| T12 POI panels on OverlayPanel | built in R2 | 1 | 5 | cd68647 | 8 passed, lint clean |
| T13 PhotoPanel | built in R2 | 0 | 2 | 0077376 | 8 passed, lint exit 0 |
| T14 PhotoList | built in R2 | 0 | 2 | dbf0979 | 10 passed, lint exit 0 |
| T15 GraspPanel | built in R2 | 2 | 1 | 4cab198 | 14 passed, lint clean |
| T16 size migrations and guard test | built in R2 | 1 | 10 | a0ae29d | exit 0; vitest summary lines not captured in the tail |
| T17 poi_store clear by statuses | built | 1 | 4 | ce5d313 | ruff, vulture clean |
| T18 mcp_server clear route filter | built (no-progress in R1) | 1/2 | 7 | 7c12c25 | ruff, vulture clean |
| T19 agent_poi_reset | built (no-progress in R1) | 1/2 | 6 | 93dae54 | ruff, vulture clean |
| T20 tool descriptions | built (round-cap in R1) | 0/3 | 2 | 59ce7b6 | ruff, vulture clean |
| T21 Map marking prompt | built (R1 continuation merged T17 fix) | 1/3 | 3 | e528f4c | ruff, vulture clean |
| T22 AutoPoiMarker | built (round-cap in R1) | 1/3 | 7 | 234d870 | ruff, vulture clean |
| T23 poi_hint nudge | built after integration (R1 blocked on T19/T24/T25 fields) | 1/2 | 7 | bb12b6b | ruff, vulture clean; A5 config test re-run on main |
| T24 complete_phase hint | built (review-shallow in R1) | 1/1 | 5 | 2a25438 | ruff, vulture clean |
| T25 POI review follow-up | built (no-progress in R1) | 1/3 | 6 | 723a7af | ruff, vulture clean |
| T26 client.yml D8 config | delivered by ruling (content on main at 489770a; no-progress in R2) | 2 | 0 | 489770a (main, not a branch) | ansible-lint 0 failures, exit 0 |
| T27 ArmTab cameras | built | 1/1 | 4 | 6c6d8b7 | exit 0 on rerun |
| T28 arm tab type | built in R3 (round-cap, complete=false in R3 continuation) | 1/3 | 15 | 9b5de3c | ansible-lint 0 failures; chain ran to end |
| T29 armCamera poses | built (no-progress in R1) | 0/2 | 2 | 9d463b5 | 8 passed; exit code not echoed, lint ran under && |
| T30 Arm 3D view | built in R3 | 2/1 | 4 | a4035c4 | exit 0; React ThemeProvider stack trace in ArmView.test.tsx output, tests passed |
| T31 mcp_server jog | built | 1/1 | 7 | 45ad588 | ruff, vulture clean |
| T32 take/release services | built (review-shallow in R1) | 1/1 | 6 | 1c4126c | ruff, vulture clean |
| T33 web_ui arm routes | built in R3 (no-progress) | 1/3 | 6 | 89e579d | 45 files formatted, vulture clean |
| T34 jog panel | built in R3 (blocked on robot check) | 1/1 | 5 | 56b0500 | vitest 628 passed, lint clean, build ok |
| T35 leader_bridge node | built (review-shallow, round-cap in R1) | 1/3 | 8 | abc1e35 | ruff, vulture clean |
| T36 leader_bridge Ansible deploy | built, client only (blocked in R3 on robot check) | 1/1 | 0 in final entry | 7391de7 | test-ansible exit 0 |
| T37 per-task time budget | built (round-cap, blocked in R1) | 3/1 | 9 | c1c35ce | ruff, vulture clean |
| T38 prompt planning section | delivered by ruling; ui2-t38 duplicate not merged (T40 design kept) | 2/3 | 3 | 7ab4511 | 65 passed, ruff clean |
| T39 Ansible caps and ceiling | unverified (on main; checklist only, dependency-failed in all rounds) | 0 | n/a | none | n/a |
| T40 job header budget chip | built (landed 63669e1) | 2/1 | 3 | 7064fe6 | 71 passed, eslint clean |

Red evidence: `tests.red` was absent in the final entry of T37 (R1), T6, T7, T21, T22, T24, T32, T34, T36 (continuation entries). Those tasks' earlier entries carry red output; T26 and T38 have none that predates the code (the work landed earlier).

Approvals (from `approvals.json` and the final return of the last round that touched each entry):

| Id | Text | Decisions | Tasks | Status |
|---|---|---|---|---|
| A1 | POI tab replaces IMU; unified list with show-on-map popup | D1-D4 | T2, T6, T7, T8 | delivered (R2 audit) |
| A2 | Marker colours by kind and zone; smaller photo triangles | D5 | T3, T4, T5 | delivered (R1 audit landed: true) |
| A3 | Phases dropdown | D6 | T9 | delivered |
| A4 | Responsive overhaul | D7 | T1, T10-T16 | delivered (R2) |
| A5 | Reliable agent POIs | D8 | T17-T26 | delivered by ruling (R2 escalated T26 on procedural TDD only) |
| A6 | Arm tab, jog, take/release/home, leader_bridge | D9 | T27-T36 | delivered after orchestrator robot checks; leader live move pending (user) |
| A7 | Generous limits and per-task timer | D10 | T37-T40 | delivered by ruling; T39 unverified |

## Deviations and rulings

1. R1 stops: T7 and T35 stopped review-shallow, T37 hit the round cap; 17 dependents cascaded as dependency-failed. Why: code was verified. Landed 23 branches (main 489770a, which also deduplicates auto_poi settings after rebasing T21-T23). Cost if wrong: unreviewed shallow-review code on main; mitigated by the later scope audits.
2. R2: T26 and T38 stalled on a procedural TDD finding although content was already on main (489770a); ruled delivered. Landed 63669e1 (ui2-t16 with T8, T10-T16, plus T40). Duplicate ui2-t38 not merged, T40 design kept. Cost if wrong: T26/T38 have no red run predating the code; T26 red was shown by mutating client.yml.
3. R3: residual findings were procedural (TDD proof) or orchestrator-level (merge plus robot check). Landed d5510f3 (T28, T30, T33, T34, T36). T39 was already on main and verified by checklist. Cost if wrong: T39 has no verifier head.
4. T36 planned both hosts; main carries leader_bridge in client.yml only (commit 60d7421 "on the client"; server.yml has 0 references). No ruling covers this beyond "deploy on the client". Cost if wrong: server-side bridge missing.
5. forwardMerged: R1 T17 (ce5d313) into T21 (e528f4c) and T31 (45ad588). R2 T8 (ui2-t8-map-popup-focus) into T10 3ad0314, T11 72632a9, T12 cd68647, T13 0077376, T14 dbf0979, T15 4cab198 and T16.

## Duplicates reconciled

- Kind colour rule: T3 `markerColors`, T6 `PoiTab.swatchColor`, T8 `MapPopup.kindColor` repeated it; T16 made `poi/style.ts:fillColor` the single rule (commit 0df8d54, popup kind tag in 879ab11).
- client.yml auto_poi pinning: three tests (49d5fb6, e82ea37, e021702) assert the same dict, plus a claude_agent test; kept (T26 reported as duplicate, none removed).
- ui2-t38 vs T40 design for the budget chip: T40 kept, ui2-t38 not merged.
- t21/t22/t23 auto_poi/agent POI settings duplicated by rebasing: deduplicated in 489770a.

## Deferred and blocked

- Live leader teleop move: leader_bridge endpoints discovered across hosts and active_source was leader, but no samples were seen (the leader publishes on change and was still). User must move the leader arm.
- Browser visual check: not done; superseded by the declutter round that restyles everything. Orchestrator/user.
- Worktree cleanup: ui-*, ui2-*, ui3-* kept; branch deletion needs the user's OK.
- Verifier `declined` and `cannotVerify` items stand unless noted: red-before-green order for T1, T3, T28, T30 (T28: 879ab11 left test_map3d_backend failing, fixed in 8d82c80); live rendering on real devices (dvh, blur, safe-area, drei popup, camera follow, marker distinguishability); T8 "showing N of total" wording above 1000 photos; T28 live web_ui loading client.yml (covered by the deploy, failed=0). Owner: user for visual items.
- T39: unverified, no independent verifier run.

## Research that changed the plan

none recorded in the inputs I was given.

## Cost

Three plan-execute runs (R1 59 entries incl. 19 continuations, R2 20, R3 11; R1 rounds=2, R2 rounds=2, R3 rounds=2). Agent counts by role, audits and wall-clock per run: unknown. Wall-clock of main commits: 2026-10-12 11:39 to 15:08 (+0200).

## MRs and deploy

No MR/PR URLs given; work landed directly on main (head d5510f3). Pipeline state: unknown. Deploy: client web_ui, mcp_server, claude_agent, poi_store and leader_bridge, failed=0. Tabs: map_nav, agent_chat, arm, camera, poi. Robot checks: gripper jog via /api/arm take, jog, release passed (-0.147 to -0.084 to -0.138 rad, other joints held, active_source none after release). leader_bridge: endpoints discovered across hosts, active_source leader, no samples (leader still).

## Lessons routed to

- Fresh worktrees lack node_modules (mechanical): add a worktree-setup step or a pre-verify hook that runs `npm ci` in `nodes/web_ui/frontend`; the orchestrator pre-installed them in R2 and R3.
- Plugin REVERT_RECIPE used `git checkout`, which the guard hook blocks (mechanical): add a lint/test over the recipe text that fails on blocked commands; another agent is fixing it.
- Procedural TDD finding stalls no-change tasks and cascades to dependents (mechanical): the verifier should accept an `alreadyOnBase` result (task content present at the merge-base with a mutation-based red) instead of raising a high TDD finding; add a check in plan-execute for zero-diff tasks.
- Review-shallow/round-cap cascades (judgement call): edit start-development so a stopped-but-verified task lets dependents run on its branch, not fail.
- T36 planned two hosts and shipped one (judgement call): the scope audit should compare plan file lists with the merged diff per host.

## Orchestrator rulings added after the record

- T36 "both hosts": leader_bridge runs only on the client. This is deliberate. The client subscribes to the server's /leader/joint_states over DDS: on 2026-10-12 `ros2 topic info -v` showed the server publisher and the client leader_bridge subscription discovered across hosts. A server-side copy would only duplicate the relay. Ruled delivered.
- T4, T6, T7 red evidence: the final entries lack tests.red, but each task's verifier passed on its HEAD, and the TDD findings were handled in their fix rounds (revert-proof runs). Ruled accepted. The process gap is the plugin's REVERT_RECIPE, which another agent is fixing.
- T39: delivered on main through T37's remediation, confirmed by the round-3 merger checklist (all five caps in client.yml, matching config.py defaults, pinned by tests/test_claude_agent_config.py).
