# Web UI head tab record

Plan: docs/plans/2026-10-13-web-ui-head-tab-plan.md (tasks, approvals and design alongside).

## Summary

Asked: an arducam_tof driver node (real ToF depth/amplitude topics, deployed disabled, SDK installed by Ansible) and a Head tab in web_ui (head RGB, greyscale ToF depth with optional amplitude, disabled pan/tilt block).
Shipped: all 7 planned tasks' content is on main (b5ebaf3..34677ed, 17 commits, 39 files, +2797/-55). Root tests: 1003 passed. Deployed to client (web_ui, arducam_tof), failed=0.
Workflow state: `finished: false`; 4 of 7 tasks ran and stopped not-clean, 3 never ran. No second round was run; the missing content was found already delivered by the landed branches (ruling 1).
Verdict: delivered, with ToF hardware behaviour unverified. arducam_tof is deployed disabled pending the user.

## Planned vs built

Review rounds are from the plan-execute return. Continuations: not reported (unknown). Verifier head is `tasks[].verification.head`. The return also holds fix-round entries (T1-r1, T3-r1, T4-r1, T5-r1); their heads are listed in the last column.

| Task | Status | Rounds | Continuations | Files | Verifier head | Verification tail |
|---|---|---|---|---|---|---|
| T1 ToF driver | built, stopped blocked (not clean) | 1 (+3 in T1-r1, round-cap) | unknown | nodes/bridges/arducam_tof (bridge.py, tests/test_bridge.py, driver, README), READMEs | 11977813 (r1: ae83e2d5) | 51 passed; lint, vulture clean |
| T2 register node | built inside T1 (e406e21: ci.yml matrix, lint-nodes, lint-all-nodes.sh); task never ran (dependency-failed) | 0 | unknown | ci.yml, root lint config | unverified | scope audit re-ran test_node_lint_coverage and test_ci_workflow: 9 passed |
| T3 Ansible deploy | built, stopped round-cap (not clean) | 3 (+2 in T3-r1, clean) | unknown | tests/test_arducam_tof_config.py, tests/README.md, ansible group_vars/role/SDK task | 7b035c92 (r1: b8a9e53d) | 93 passed, 1 skipped (driver not in tree); r1: 94 passed; test-ansible exit 0 |
| T4 head backend | built, clean | 1 (+3 in T4-r1, round-cap) | unknown | web_ui config.py, bridge.py, msg_serializer.py, __main__.py + tests; r1 added default.yaml, client.yml, READMEs | f7c075aa (r1: c0e0972c) | 65 passed; lint, vulture clean |
| T5 head UI | built, clean | 1 (+3 in T5-r1, round-cap) | unknown | frontend HeadTab.tsx, HeadTab.test.tsx, App.tsx, tabSelection.ts, types.ts, tabsDesign.test.ts | c2a68a05 | npm test, lint, build exit 0 (test summary line not captured) |
| T6 head config | content delivered on landed branches (head tab in default.yaml, client.yml, WEB_UI_TABS pinned); task never ran (dependency-failed) | 0 | unknown | nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, tests | unverified | none from the task; root suite 1003 passed after landing |
| T7 docs | content delivered on landed branches (node README, nodes index, web_ui/ansible/tests READMEs); task never ran (dependency-failed) | 0 | unknown | READMEs | unverified | none from the task |

Files over summaries: T1 summary says tests added test_bridge.py::test_camera_that_fails_after_start...; red re-run by the reviewer (pre-fix bridge.py: 2 failed, 49 passed). T4 and T5 implementers recorded no red; reviewers re-ran it (T4: 14 failed, 51 passed against b5ebaf3 files; T5: 2 failed, 22 passed with sources at b5ebaf3). T3 red was a driver-config import error, not an assertion on behaviour. T5 test summary line was never captured by the verifier.

### Approvals

| Id | Text | Decisions | Tasks | Status |
|---|---|---|---|---|
| C1 | arducam_tof driver node (real ToF topics, no placeholder frames) plus Ansible deploying it disabled and installing the SDK | D1 | T1, T2, T3, T7 | workflow return: escalated. Content delivered at 0b12a79 (ruling 1); deployed disabled. ToF hardware use blocked on the user |
| C2 | Head tab: head RGB, greyscale ToF depth with optional amplitude, disabled pan/tilt placeholder publishing nothing, wired into backend, configs, pinned tests, docs | D2 | T4, T5, T6, T7 | workflow return: escalated. Content delivered at 0b12a79 (ruling 1); deployed, tabs map_nav, agent_chat, arm, head, camera, poi |

The scope audit marked both C1 and C2 `landed: true` with `missing: []` against the unmerged branches.

## Deviations and rulings

1. T1, T3, T4, T5 stopped not-clean (blocked, round-cap, TDD-proof process findings). T2, T6, T7 never ran (dependency-failed). Branches were landed at 0b12a79. Inspection found T2 (e406e21 inside t1), T6 and T7 already delivered by the landed branches; the CLAUDE.md node count is not pinned, so no edit was needed. No second round ran. Cost if wrong: an unrun task hides a gap. Mitigation: root suite 1003 passed and the client deploy succeeded; T2/T6/T7 have no independent verifier head.
2. Docs duplicated by t1 and t4 were deduped to one copy each (see Duplicates). Cost if wrong: a README states a contract differing from the code. Reviewer found the two copies differ (different Head table rows), so the merge conflicted in nodes/web_ui/README.md.
3. Scrape target (orchestrator ruling): Alloy scrapes only enabled nodes. The arducam_tof target was removed while the node is disabled; tests pin "a target exists exactly when the node is enabled" (ecfc93c, 34677ed). Why: a disabled target reads as down. Cost if wrong: enabling the node without adding the target leaves it unmonitored; it is in the enable steps below.
4. forwardMerged: none.

## Duplicates reconciled

- arducam_tof node README and nodes/README entry: t1 be6fb56 and t4 cba7769 (same 75-line file). One copy kept.
- Head-tab docs in web_ui, tests and ansible READMEs: t1 ae83e2d versus t4 b704e81/c0e0972. Deduped to one copy each (ruling 2).
- hd-t2-tof-register worktree duplicated e406e21 from t1; not landed separately.
- The workflow's own `duplicates` field was empty; the scope audit's list is the one above.

## Deferred and blocked

- Hardware check of the ToF (user): plug in the ToF, confirm camera_index (cam1 is 1?) and range units (SDK RANGE in mm or m; depth_data in mm; amplitude_data/confidence_data read from the 0.1.24 .pyi stub only).
- Enable arducam_tof (user, after the above): set enabled true, add the Alloy scrape target, deploy arducam_tof, check the Head tab.
- cannotVerify, open: numpy dependency guard live check (service start without ModuleNotFoundError: numpy), needs hardware; live apt behaviour of the Arducam repo on noble arm64 beyond the install of arducam-tof-sdk-dev 0.1.24 reported at deploy; browser responsive behaviour of HeadTab; amplitude payload key jpeg_b64 against a live message; full web_ui suite claim of 653/655 passing was a reviewer-run figure, not the verifier's.
- cannotVerify, settled by later evidence: T3 config keys against T1's pydantic model (test_arducam_tof_config runs unconditionally at 0b12a79); integrated run of that test unskipped.
- declined by reviewers (no ruling needed): same topic shared by a head tab and another tab type (speculative); layout style.
- Process gaps left: graphify rebuild not run for T1 (graphify-out/); red-before-green order cannot be seen in single commits for T1, T4, T5.

## Research that changed the plan

None recorded in the workflow return.

## Cost

57 agents, 365 tool calls, 2,953,246 tokens; 2 workflow rounds. Roles per agent, continuations, audits count and wall-clock: unknown (not in the return). One scope audit ran.

## MRs and deploy

No MR or PR: work landed directly on main per project convention (34677ed head). Pipeline: none. Deploy: client web_ui and arducam_tof, failed=0. Tabs: map_nav, agent_chat, arm, head, camera, poi. arducam-tof-sdk-dev 0.1.24 installed on noble arm64. ros2-arducam_tof is disabled.

## Lessons routed to

- Reviewers re-ran red evidence for T1, T4, T5 because implementers left tests.red empty: add a check in plan-execute that a task with `wroteTestFirst` has a non-empty `tests.red` before verification (mechanical).
- Parallel tasks T1 and T4 both wrote the same docs: add a plan-lint check that no file appears in two tasks' file lists (mechanical).
- Dependent tasks (T2, T6, T7) were marked dependency-failed because upstream stopped round-cap even though their content was delivered: edit plan-execute docs so dependency-failed is distinguished from upstream-stopped-with-delivered-code (judgement call).
- Enabled-only scrape rule is pinned by tests (ecfc93c, 34677ed); no further routing.
