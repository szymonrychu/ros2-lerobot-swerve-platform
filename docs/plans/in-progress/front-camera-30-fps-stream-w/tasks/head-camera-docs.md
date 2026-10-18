---
title: Head camera docs
status: todo
repo: ros2-lerobot-sverve-platform
value: "Anyone can read how the head camera node works, deploys and is verified."
files: [nodes/head_camera_cpp/README.md, nodes/README.md, ansible/README.md, .claude/skills/ansible-deploy/SKILL.md, MEMORY.md, tests/test_head_camera_deploy.py, tests/README.md]
verify: "uv run pytest tests/test_head_camera_deploy.py tests/test_cpp_node_deploy.py -q && uv run poe lint"
depends_on: [head-camera-ros-wiring, agent-zoom-prompt-and-docs, head-tab-click-to-zoom]
owns: []
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--head-camera-docs
branch: front-camera-30-fps-stream-w--head-camera-docs
verified_head: ""
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# Head camera docs

## Requirement

- `nodes/head_camera_cpp/README.md` complete: purpose (head RGB camera of the pan/tilt head, replaces camera_ros/pi_cameras), topics with types and QoS, `## Zoom interface` (mirrors zoom_json.hpp: request, image, result, statuses, stamp pairing, timing), state machine phases and failure handling, camera_info policy (D3), configuration table, metrics, build/test, deploy, the libcamera fork dependency, and that `head_camera_optical_frame` gets a TF when the pan/tilt head is built (non-goal of this epic).
- `nodes/README.md`: `head_camera_cpp/` entry; the `pi_cameras/` entry marked retired. `ansible/README.md`: the pi_cameras paragraph becomes the head_camera_cpp paragraph (node type, libcamera build unchanged, tier, config, playbook step, tests) plus a line on the pi_cameras retirement and camera_ros purge; the memory table row. `.claude/skills/ansible-deploy/SKILL.md`: the pi_cameras row replaced by head_camera_cpp (and pi_cameras retired). `MEMORY.md`: dated (2026-10-18) decision entry: head camera = Camera Module 3 + ToF on the pan/tilt head, head_camera_cpp owns it through libcamera, 1152x648 at 30 fps, zoom contract, pi_cameras retired, camera value `front` kept in mcp_server.
- Tests first, seen failing: `tests/test_head_camera_deploy.py::test_docs_describe_head_camera_cpp` (each doc names head_camera_cpp; the node README has `## Zoom interface` and `## Configuration`; SKILL and nodes README mark pi_cameras retired). `tests/README.md` lists it.

## Acceptance conditions

- a1: README, nodes README, ansible README, SKILL and MEMORY describe head_camera_cpp and the pi_cameras retirement [evidence: test: tests/test_head_camera_deploy.py::test_docs_describe_head_camera_cpp]
- a2: the node README documents the zoom interface [evidence: doc-section: nodes/head_camera_cpp/README.md#Zoom interface]
- a3: MEMORY.md records the head camera decision [evidence: command: grep -q head_camera_cpp MEMORY.md]
- a4: root lint passes [evidence: command: uv run poe lint]

## Files

- `nodes/head_camera_cpp/README.md`
- `nodes/README.md`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `MEMORY.md`
- `tests/test_head_camera_deploy.py`
- `tests/README.md`

## Interfaces

- Documentation only; `test_docs_describe_head_camera_cpp` pins it.

## Expected outcome

Docs and memory describe the head camera node, its zoom contract and the retirement.

## Steps

- Add the docs test; run `uv run pytest tests/test_head_camera_deploy.py -q`; see it fail.
- Write the docs; run the verify command; commit `docs: head_camera_cpp node, zoom interface, pi_cameras retired`.

## Notes

- CLAUDE.md audience: mid-level Python developer with ROS 2 experience.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on agent-zoom-prompt-and-docs, head-tab-click-to-zoom

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on head-tab-click-to-zoom
