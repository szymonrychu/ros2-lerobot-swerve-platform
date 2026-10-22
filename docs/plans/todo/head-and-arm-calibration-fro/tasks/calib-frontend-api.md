---
title: Calib frontend API
status: todo
repo: ros2-lerobot-sverve-platform
value: "The Calibrate page talks to the head-from-arm routes through one typed client checked against the node's fixtures."
files: [nodes/web_ui/frontend/src/calibrate/headArmApi.ts, nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts, nodes/web_ui/frontend/src/calibrate/headFloorApi.ts, nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts src/calibrate/headFloorApi.test.ts && npx tsc --noEmit && npm run lint"
depends_on: [head-arm-sequence-http-api]
owns: []
uses: [head-arm-calib-api]
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

# Calib frontend API

## Requirement



## Acceptance conditions



## Files

- `nodes/web_ui/frontend/src/calibrate/headArmApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
