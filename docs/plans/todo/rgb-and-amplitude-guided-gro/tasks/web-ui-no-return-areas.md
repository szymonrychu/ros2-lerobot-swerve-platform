---
title: Web UI no return areas
status: todo
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
worktree: ""
branch: ""
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
