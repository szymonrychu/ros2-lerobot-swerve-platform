---
title: "tests/README.md (repo root, around line 425) describes the feetech_servos tests but does not list test_bus_budget.py"
status: done
repo: ros2-lerobot-sverve-platform
value: "CLAUDE.md requires tests/README.md to list each test file; it is outside this task's named files"
files: [tests/README.md]
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# tests/README.md (repo root, around line 425) describes the feetech_servos tests but does not list test_bus_budget.py

## Requirement

tests/README.md (repo root, around line 425) describes the feetech_servos tests but does not list test_bus_budget.py

Where: tests/README.md

Why: CLAUDE.md requires tests/README.md to list each test file; it is outside this task's named files

## Acceptance conditions

- a1: resolved without a code change in this task [evidence: judgement]

## Files

- `tests/README.md`

## Interfaces

- None.

## Expected outcome

- Resolved.

## Steps

- None.

## Notes


Discovered in task `bus-budget-test` during run `split-servo-buses-arm-alone`.

Blocked by the dedup check:
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'deploy-head-camera-playbook' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'final-suite-and-runbook' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'head-camera-docs' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'head-camera-node-type' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'mcp-head-camera-topic' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'retire-pi-cameras' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'web-ui-head-camera-topics' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'named-maps-docs-and-memory' of in-progress epic 'named-maps-with-pois-and-rel'


## Resolution

Folded into final-suite-and-runbook via the run instructions (add the tests/README.md row for test_bus_budget.py).
