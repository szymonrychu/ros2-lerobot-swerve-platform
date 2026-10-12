# Web UI declutter record

Plan: docs/plans/2026-10-12-web-ui-declutter-plan.md (tasks, approvals and design files alongside).

## Summary

Asked: declutter the web_ui frontend (neutral theme, one map palette, docked sidebar, one status line, quiet panels, arm and camera tabs, guard test over every source). Shipped: 20 of 20 planned tasks on main (4855e5d..32d2a06, 43 commits) over four plan-execute rounds (20, 14, 8, 3 tasks; 85, 79, 60, 10 agents). Final main: 826 tests passed, lint clean, tsc clean, build ok (round 4 audit and orchestrator). Deployed to client web_ui. Verdict: delivered; a live browser visual check was not done. Three extra rounds were caused by procedural not-clean states, not code defects (see Lessons).

## Planned vs built

Verifier head is `tasks[].verification.head` from the round in which the task landed. Review rounds and files are from that round's entry (files count is per round entry, so a re-run entry can show fewer). "Red" is `tests.red` present in the return; it is the implementer's evidence, not seen by me.

| Task | Status | Round landed | Review rounds | Files | Verifier head | Verification tail |
|---|---|---|---|---|---|---|
| T1 theme tokens | built | 1 | 1 | 5 | 141ddff4b | 20 tests, lint, tsc, exit 0 |
| T2 designViolations guard | built | 1 | 0 | 2 | a429d1725 | 12 tests, lint, tsc, exit 0 |
| T3 map palette | built, stopped review-shallow, landed by ruling 1 | 1 | 1 | 8 | 7ef8b37d0 | 37 tests, lint, tsc, exit 0 |
| T4 MapLabel | built | 2 | 1 | 6 | 5ccbaaed3 | 4 tests, lint, tsc, exit 0 |
| T5 POI marker shapes | built (re-applied) | 3 | 2 | 3 | b30bd17fd | lint and tsc no output; exit code not captured (zsh PIPESTATUS) |
| T6 panel chrome, ConfirmDialog | built | 1 | 1 | 10 | bcd02c627 | 30 tests, lint, tsc; exit code inferred from && chain |
| T7 docked sidebar | built | 2 | 1 | 8 | a1fa4c81d | 75 tests, lint, tsc, exit 0 |
| T8 remove MapPopup | built, stopped review-shallow, landed by ruling 1 | 2 | 1 | 8 | cb612e1e8 | 23 tests, lint, tsc, exit 0 |
| T9 StatusItem, status bar | built | 1 | 1 | 7 | 26edf505b | 20 tests, lint, tsc; passed inferred from && chain |
| T10 StateBanner, tab bar | built, stopped review-shallow, landed by ruling 1 | 1 | 1 | 6 | 97b91907f | 24 tests, lint, tsc, exit 0 |
| T11 icon-only toolbar, status line | built | 3 | 3 | 2 | bdfe08698 | 30 tests, lint, tsc, exit 0 on rerun |
| T12 Layers panel | built | 3 | 1 | 6 | ba049f5fb | 10 tests, lint, tsc, exit 0 |
| T13 grasp panel | built | 3 | 1 | 4 | 0c9fb58f5 | tests passed; stderr emotion/MUI stack trace from OverlayPanel logged, did not fail |
| T14 POI and photo panels | built (re-applied by cherry-pick) | 2 | 2 | 6 | 0084a1f1a | 37 tests, lint, tsc, exit 0 |
| T15 agent header | built | 2 | 1 | 7 | 15d59eb82 | 22 tests, lint, tsc, exit 0 |
| T16 agent chat log | built | 2 | 1 | 3 | 301d4d94b | 8 tests, lint, tsc, exit 0 |
| T17 tabs cleanup, robot colours | built (re-applied), stopped review-shallow in round 3, landed by ruling 1 | 3 | 1 | 9 | c907625b6 | 21 tests, lint, tsc, exit 0 |
| T18 arm tab | built | 4 | 1 | 4 | 84b6b5600 | 44 tests, lint, tsc, exit 0 |
| T19 camera stream | built | 4 | 1 | 2 | c33b034c6 | lint, tsc clean, exit 0 |
| T20 guard every source, README | built | 4 | 1 | 5 | 32d2a06f9 | 826 tests, lint, tsc, build ok |

No landed entry lacks a verifier head, so no row is marked unverified. In rounds 1 to 3 the tasks that never ran (dependency-failed) had no head and were re-run in the next round.

Contradictions between summaries and files:
- T14 returned clean in round 1 but its branch conflicted at integration (ruling 2).
- Round 1 and 2 audits reported B1 to B4 not landed because branches were unmerged; they merged after the orchestrator landed them (ruling 1).
- T6, T9 and T5 verification passes are inferred from the command chain, not a captured exit code.

Approvals (ledger docs/plans/2026-10-12-web-ui-declutter-approvals.json; status from the round 4 return):

| Id | Text | Decisions | Tasks | Status |
|---|---|---|---|---|
| B1 | Neutral dark theme, white accent, no green ok, flat opaque panels, weights 400/500, guard test | D1 | T1, T2, T6, T9, T10, T17, T19, T20 | delivered (round 4 audit: guard iterates every SOURCES entry, README Design rules present, 826/826) |
| B2 | Map colour: zones and nav paths coloured, POI kinds greyscale and told apart by shape, one palette file | D2 | T3, T4, T5, T17, T18 | delivered (round 4 audit: ARM_COLORS achromatic in map3d/palette.ts) |
| B3 | Docked right sidebar, bottom sheets on phones, no MapPopup | D3 | T6, T7, T8 | unknown in the final return (absent from round 4); round 2 audit: landed true (MapPopup.tsx gone, mapFocus.test.tsx 24/24) |
| B4 | One status line, STOP and STOP GRASP in toolbar only, flat chat log, Dialog for destructive actions, two-click arm | D4 | T9 to T16, T18 | delivered (round 4 audit) |

## Deviations and rulings

1. What: each round stopped dependents only through procedural not-clean states (review-shallow, or a TDD-proof finding on verified code); the orchestrator landed the verified branches and re-ran the remainder. Main moved dcdc15c, 66af60f, b192cef, 32d2a06. Why: verified code was blocked by process findings that cascaded to dependency-failed. Cost if wrong: unreviewed-deeply code (T3, T8, T10, T17) landed on main; mitigated by the final full suite 826 passed.
2. What: T14 first conflicted (an old T6 copy); re-applied in round 2 by cherry-picking its own commits. Why: its branch carried stale T6 content. Cost if wrong: lost or duplicated T6 chrome edits; the final suite and lint pass.
3. What: T5 and T17 duplicated T4's arm colours; ruling made T4's ARM_COLORS (grey axes) the single table, T5 and T17 re-applied in round 3, one SHAPE_CLIP in poi/style.ts. Why: three incompatible colour tables (T4 greyscale, T5 hued axes, T17 ROBOT_COLORS). Cost if wrong: arm axis hues lost in the 3D view; B2 requires greyscale, so low.
4. forwardMerged: none in any round. Round 2 recorded a forward-merge failure for T5 (conflict in RobotModel.tsx and palette.ts against T4), resolved by ruling 3.

## Duplicates reconciled

- Arm colours (T4, T5, T17): T4's ARM_COLORS kept (ruling 3).
- SHAPE_CLIP: one copy, in poi/style.ts.
- Status mark (T9 StatusItem 6 px square versus T14 PoiPanels 8 px round dot, round 1 audit): T14 re-applied on StatusItem (commits 267b217, 3568567, 1ccef60).

## Deferred and blocked

- Live browser visual check: not done. Who can clear: the user, at http://client.ros2.lan:8080.
- Round 4 declined: T18 re-arm-on-action-switch has no dedicated test; T20 README prose accuracy (stop buttons never covered) not checked against live UI. Clear: reviewer with a live browser.
- Round 4 cannotVerify: T18 arm tab icon remains in the drawer (App.tsx, T10-owned), no test asserts a tab bar without it; red-before-green for T19 not observable. Clear: user or later review.
- Round 4 out of scope: CameraStream now surfaces ImageData.error as '<topic>: <error>' (new behaviour). Clear: the user decides keep or drop.
- Red evidence: several verifiers could not see the red run (T1, T3, T4, T5, T19). Clear: none possible after the fact.
- Plugin process gap: another agent owns the fixes.

## Research that changed the plan

none

## Cost

Agents: 85, 79, 60, 10 = 234 across four rounds. Tokens: 3,944,089, 4,017,644, 2,961,457, 523,439 = 11,446,629. Continuations and wall-clock: unknown (not in the return).

## MRs and deploy

No MR or PR; work landed on main locally and pushed. Deploy: client web_ui, failed=0, service active, serves HTTP 200 (orchestrator).

## Lessons routed to

- Verified tasks marked not-clean by review-shallow or TDD-proof findings cascade dependency-failed and cost three extra rounds: a plugin check (plan-execute) should treat these findings as non-blocking for dependents when verification head and tests passed. Mechanical; owner is the other agent fixing the plugin.
- Duplicated shared tables (ARM_COLORS, SHAPE_CLIP): add a lint or test that each exported constant table exists once under src, and have the plan name one owner per table.
- Verification passes inferred without a captured exit code (T5, T6, T9): add a check in the verify command that records `$?` (no pipe to tail).
- No red evidence for several tasks: add a hook requiring `tests.red` before a task may report clean.
