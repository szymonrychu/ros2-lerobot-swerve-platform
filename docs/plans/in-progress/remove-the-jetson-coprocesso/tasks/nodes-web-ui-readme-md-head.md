---
title: "nodes/web_ui/README.md Head row (line 14) and Overview row (line 15) still say stereo/Jetson"
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Stale Jetson/stereo docs belong to the Head tab and camera re-point tasks, not this task."
files: [nodes/web_ui/README.md]
verify: "! grep -n -i -E 'stereo|jetson|overview_camera|fused' nodes/web_ui/README.md"
depends_on: [head-tab-tof-topics, remove-overview-tab, drop-fused-depth-topic]
owns: []
uses: []
model: sonnet
estimate: 15
actual: ""
review: false
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--nodes-web-ui-readme-md-head
branch: remove-the-jetson-coprocesso--nodes-web-ui-readme-md-head
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

- None (documentation only).

## Expected outcome

- nodes/web_ui/README.md tab table and Head section describe the head RGB camera plus ToF; no Overview row, no stereo/Jetson/fused wording.

## Steps

- After head-tab-tof-topics, remove-overview-tab and drop-fused-depth-topic land, fix the remaining stereo/Jetson/overview/fused wording in nodes/web_ui/README.md (rows 14-15, Head section).
- Run `! grep -n -i -E 'stereo|jetson|overview_camera|fused' nodes/web_ui/README.md`: exit 0 (no match).
- Docs only (TDD exception: documentation); commit `docs(web_ui): README head and overview rows`.

## Notes


Discovered in task `calibrate-page-imu-only` during run `remove-the-jetson-coprocesso`.

