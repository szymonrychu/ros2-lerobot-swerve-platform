---
title: Docs and memory sweep
status: done
repo: ros2-lerobot-sverve-platform
value: "Project memory, the deploy skill and the remaining READMEs and comments describe the robot without the Jetson."
files: [MEMORY.md, .claude/skills/ansible-deploy/SKILL.md, ansible/README.md, nodes/pi_cameras/README.md, tests/test_pi_cameras_deploy.py, nodes/README.md, ansible/group_vars/client.yml, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint"
depends_on: [head-and-arm-tabs-frontend]
owns: []
uses: []
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--docs-and-memory-sweep
branch: remove-the-jetson-coprocesso--docs-and-memory-sweep
verified_head: 50c2b9d97c9bd64dfe6ad4c92aa1b821b74ac0b9
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: [a1]
process_gaps: []
---

# Docs and memory sweep

## Requirement

- `MEMORY.md`: the "Stereo depth (IMX219 pair, client)" section is marked superseded, and a new dated decision entry (2026-10-18) records: the Jetson is removed from the robot; stereo_depth / stereo_depth_cpp sources are deleted (retired entries kept for the uninstall); the Head tab shows the Arducam ToF depth + amplitude with no RGB; `/front_camera` feeds the Arm tab and mcp_server `front`; the Overview tab and stereo calibration are gone; the client keeps chrony with internet pools; jetson_bridge_cpp is `present: false` with colcon purge; the `jetson_link` tag removes the client link state. The stale "the ToF never came up" claim is corrected (ToF frame capture on the Pi 5 kernel is tracked separately).
- `.claude/skills/ansible-deploy/SKILL.md`: no `jetson` deploy target or Jetson node row; the stereo_depth / stereo_depth_cpp rows say retired (D7, sources deleted); the overview_camera row names no Jetson bridge; a jetson_bridge_cpp row says retired (`present: false`, colcon purge) and that `client jetson_link` removes the link state.
- `ansible/README.md`: the retired stereo nodes paragraph (~382) no longer says the Jetson replaces them; no live Jetson description remains outside retirement notes.
- `nodes/pi_cameras/README.md`: `/front_camera` is consumed by the web_ui Front and Arm tabs and the mcp_server `front` camera; the line about `/overview_camera` being the Jetson bridge's (~20) and the only-consumer claim (~5) are corrected.
- `tests/test_pi_cameras_deploy.py`: the docstring (~4-5) names no Jetson; `test_web_ui_shows_the_previews_not_the_overview_camera_names` still passes.
- `nodes/README.md`: the realsense line (~20) no longer says "replaced by the Jetson camera path".
- `client.yml`: the comments above the retired overview_camera, stereo_depth and stereo_depth_cpp entries and node types and above pi_cameras (~1140) no longer mention the Jetson and still name D7 (`test_stereo_depth_config.py` and `test_pi_cameras_retired.py` pass).
- `tests/README.md`: descriptions still naming the Jetson outside retirement-test rows are updated (e.g. the tier description ~207).

## Acceptance conditions

- a1: MEMORY.md has a 2026-10-18 Jetson-removal decision entry and marks the stereo section superseded [evidence: judgement]
- a2: the deploy skill names no jetson deploy target [evidence: command: ! grep -n -E 'deploy-nodes.sh jetson|jetson base|depthd' .claude/skills/ansible-deploy/SKILL.md]
- a3: pi_cameras README, nodes README and the pi_cameras deploy test name no Jetson [evidence: command: ! grep -n -i jetson nodes/pi_cameras/README.md nodes/README.md tests/test_pi_cameras_deploy.py]
- a4: retired-entry comments in client.yml name no Jetson and the D7 retirement tests pass [evidence: command: uv run pytest tests/test_stereo_depth_config.py tests/test_pi_cameras_retired.py tests/test_pi_cameras_deploy.py -q]
- a5: root suite and lint pass [evidence: command: uv run pytest tests/ -q && uv run poe lint]

## Files

- `MEMORY.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `ansible/README.md`
- `nodes/pi_cameras/README.md`
- `tests/test_pi_cameras_deploy.py`
- `nodes/README.md`
- `ansible/group_vars/client.yml`
- `tests/README.md`

## Interfaces


- Docs and comments only; no schema or behaviour change.

## Expected outcome


MEMORY.md, the deploy skill and the READMEs describe the robot without the Jetson; the root suite is green.

## Steps


- Run `uv run pytest tests/test_stereo_depth_config.py tests/test_pi_cameras_retired.py -q` to see what the README/comment assertions pin before editing.
- Edit each file; run the verify command.
- Commit `docs: describe the robot without the Jetson`.

## Notes


- Doc-only edits (exception to test-first: comment/doc edit); the D7 retirement tests guard the README/comment facts they pin.

## Ruling

Accepted by the orchestrator (2026-10-18): verified and merged; the gate-repeat was a parking-wording flag on phrases the requirement mandates; root 1117 passed.
