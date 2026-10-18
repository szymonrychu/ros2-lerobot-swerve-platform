---
title: "nodes/web_ui/README.md Head row (line 14) and Overview row (line 15) still say stereo/Jetson"
status: todo
repo: ros2-lerobot-sverve-platform
value: "Stale Jetson/stereo docs belong to the Head tab and camera re-point tasks, not this task."
files: [nodes/web_ui/README.md]
verify: "! grep -n -i -E 'stereo|jetson|overview_camera|fused' nodes/web_ui/README.md"
depends_on: [head-tab-tof-topics, remove-overview-tab, drop-fused-depth-topic]
owns: []
uses: []
model: sonnet
estimate: "15"
actual: ""
review: "false"
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# nodes/web_ui/README.md Head row (line 14) and Overview row (line 15) still say stereo/Jetson

## Requirement

nodes/web_ui/README.md Head row (line 14) and Overview row (line 15) still say stereo/Jetson

Where: nodes/web_ui/README.md:14-15

Why: Stale Jetson/stereo docs belong to the Head tab and camera re-point tasks, not this task.

## Acceptance conditions

- a1: nodes/web_ui/README.md names no stereo, Jetson, /overview_camera or fused-depth topic in its tab table or Head section [evidence: command: ! grep -n -i -E 'stereo|jetson|overview_camera|fused' nodes/web_ui/README.md]
- a2: the Head row describes the head RGB camera plus ToF depth/amplitude; the Overview row is gone [evidence: judgement]


## Files

- `nodes/web_ui/README.md:14-15`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes


Discovered in task `calibrate-page-imu-only` during run `remove-the-jetson-coprocesso`.

