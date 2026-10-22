---
title: Web UI no return areas
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The operator sees where the ToF is blind but the camera sees something
files: [nodes/web_ui/frontend/src/map3d/groundLayer.ts, nodes/web_ui/frontend/src/map3d/groundLayer.test.ts, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/map3d && npx tsc --noEmit && npm run lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--web-ui-no-return-areas
branch: drivable-ground-tof-filterin--web-ui-no-return-areas
verified_head: ""
run: ""
provides: []
---

# Web UI no return areas

## Requirement

The web_ui 3D ground layer shows no_return areas and accepts null heights (design D5, D6, cross-epic contract ground-warning-no-return).

- `frontend/src/map3d/groundLayer.ts`: `GroundWarningArea.height_m: number | null`, `slope_deg: number | null`. Warning polygons of kind `no_return` get their own colour `GROUND_COLORS.noReturnFill` (distinct from warningFill) and a legend entry "No ToF return (caution)". Any readout of an area's height or slope shows "n/a" for null.
- Tests first (seen failing) in `frontend/src/map3d/groundLayer.test.ts`: `it('draws no_return areas in their own colour with a legend entry')` and `it('reads null area heights and slopes as n/a')`; existing tests unchanged.
- README `nodes/web_ui/README.md`: the Ground layer paragraph names the no_return areas and their colour.

## Acceptance conditions

- a1: no_return areas get their own colour and legend entry [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts::draws no_return areas in their own colour with a legend entry]
- a2: null heights and slopes read as n/a [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts::reads null area heights and slopes as n/a]
- a3: frontend tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/map3d/groundLayer.ts`
- `nodes/web_ui/frontend/src/map3d/groundLayer.test.ts`
- `nodes/web_ui/README.md`

## Interfaces

`frontend/src/map3d/groundLayer.ts`: `GroundWarningArea` with nullable `height_m` and `slope_deg`, and `GROUND_COLORS.noReturnFill` plus its legend row (contract ground-warning-no-return).

## Expected outcome

`cd nodes/web_ui/frontend && npx vitest run src/map3d` PASS with the two new tests; `npx tsc --noEmit` and `npm run lint` clean.

## Steps

- write the two tests; `npx vitest run src/map3d/groundLayer.test.ts` FAILS
- implement; the command PASSES; tsc and lint PASS
- README; commit `feat(web_ui): show no_return ground areas`

## Notes

Pick a colour distinct from the warning and colour-mismatch fills (for example purple); keep the legend order of `groundLayer.ts`. The frontend build ships with the web_ui deploy (orchestrator).
