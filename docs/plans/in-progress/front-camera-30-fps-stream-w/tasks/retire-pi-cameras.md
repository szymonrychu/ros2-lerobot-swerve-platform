---
title: Retire pi_cameras
status: escalated
repo: ros2-lerobot-sverve-platform
value: No camera_ros node competes for the head camera and no dead pi_cameras code remains.
files: [ansible/group_vars/client.yml, nodes/pi_cameras, tests/test_pi_cameras_config.py, tests/test_pi_cameras_deploy.py, tests/test_pi_cameras_retired.py, tests/test_jetson_retired.py, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible"
depends_on: [deploy-head-camera-playbook]
owns: []
uses: [head-camera-node-type]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--retire-pi-cameras
branch: front-camera-30-fps-stream-w--retire-pi-cameras
verified_head: ""
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# Retire pi_cameras

## Requirement

- `ansible/group_vars/client.yml` (D9): the `pi_cameras` ros2_nodes entry becomes `present: false` with no `config` (comment: retired, the Camera Module 3 is the head camera served by head_camera_cpp; names D9 of this epic); the `pi_cameras` node type keeps only `deploy_mode`, `node_launch_command` and `colcon_purge: [{package: camera_ros, workspace: /opt/ros2-ws}]` (libcamera is not purged: head_camera_cpp builds against it; the comment says so).
- `nodes/pi_cameras/` deleted entirely (launch, config, README, both patches; the libcamera patch lives on at `nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch`). `tests/test_pi_cameras_config.py` and `tests/test_pi_cameras_deploy.py` deleted.
- `tests/test_pi_cameras_retired.py` (tests first, seen failing): pi_cameras entry present false without config; type keys exactly `deploy_mode`, `node_launch_command`, `colcon_purge`, the purge list names camera_ros only; only head_camera_cpp builds libcamera and nobody builds camera_ros; `nodes/pi_cameras` and the two deleted test files do not exist; the client playbook still deploys pi_cameras so the uninstall runs; the docstring covers both retirements.
- `tests/test_jetson_retired.py`: the `pinned-build` allowlist entry for `nodes/pi_cameras/patches/**` is removed (the directory is gone). `tests/README.md`: the deleted files' sections removed, test_pi_cameras_retired rows updated.

## Acceptance conditions

- a1: pi_cameras is present false, uninstall-only, and purges camera_ros but not libcamera [evidence: test: tests/test_pi_cameras_retired.py]
- a2: only head_camera_cpp builds libcamera and nothing builds camera_ros [evidence: test: tests/test_pi_cameras_retired.py]
- a3: nodes/pi_cameras and its two test files are deleted [evidence: command: test ! -e nodes/pi_cameras && test ! -e tests/test_pi_cameras_config.py && test ! -e tests/test_pi_cameras_deploy.py]
- a4: the root suite, lint and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `nodes/pi_cameras`
- `tests/test_pi_cameras_config.py`
- `tests/test_pi_cameras_deploy.py`
- `tests/test_pi_cameras_retired.py`
- `tests/test_jetson_retired.py`
- `tests/README.md`

## Interfaces

- Retired `pi_cameras` entry and uninstall-only type in client.yml; `colcon_purge` (Jetson epic key, `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml`).

## Expected outcome

The next client deploy of pi_cameras uninstalls its unit and purges camera_ros; the repo holds no pi_cameras sources.

## Steps

- Rewrite `tests/test_pi_cameras_retired.py`; run `uv run pytest tests/test_pi_cameras_retired.py -q`; see it fail.
- Edit client.yml; `git rm -r nodes/pi_cameras tests/test_pi_cameras_config.py tests/test_pi_cameras_deploy.py`; fix the jetson allowlist and tests/README.md.
- Run the verify command; commit `feat(ansible): retire pi_cameras, purge camera_ros`.

## Notes

- Pattern: the overview_camera retirement in the same test file (D7 of the earlier epic).
- The web_ui pi_cameras tab is removed by web-ui-head-camera-topics; docs naming pi_cameras are updated by head-camera-docs.
