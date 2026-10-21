---
title: ToF calibrated K deploy
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The robot converts depth with the applied ToF calibration, and a new calibration reaches the conversion on Apply."
files: [ansible/group_vars/client.yml, ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2, tests/test_head_rgbd_calibration_deploy.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [tof-radial-to-z-depth, head-pan-inverted]
owns: []
uses: [tof-depth-z]
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-depth-fix-and-ground-noi--tof-calibrated-k-deploy
branch: tof-depth-fix-and-ground-noi--tof-calibrated-k-deploy
verified_head: ""
run: ""
provides: []
---

# ToF calibrated K deploy

## Requirement

Deploy the calibrated ToF K to arducam_tof (design D2; approval A1).

- `ansible/group_vars/client.yml` arducam_tof config: `tof_calibration_file: /var/lib/ros2/head_rgbd_calibration/applied/tof_calibration.yaml` and `depth_is_radial: true`, each with a one-line comment (why, and that Apply on the Calibrate page writes the file).
- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`: the try-restart line also restarts `ros2-arducam_tof.service`, so an applied calibration reaches the depth conversion; the comment names it.
- Tests first (red seen) in `tests/test_head_rgbd_calibration_deploy.py`: `test_apply_units_content` expects arducam_tof in the try-restart list; new `test_arducam_tof_reads_the_applied_tof_calibration` asserts the client.yml arducam_tof config names the same applied tof_calibration.yaml as head_rgbd_cpp's `tof_calibration_file` and sets `depth_is_radial: true`.
- A config key the node does not know fails at startup (extra=forbid), so this task runs after tof-radial-to-z-depth.
- `ansible/README.md` and `tests/README.md` updated for the changed unit and the new test.

## Acceptance conditions

- a1: the apply service restarts arducam_tof together with the head nodes [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_apply_units_content]
- a2: arducam_tof reads the same applied tof_calibration.yaml as head_rgbd_cpp, with depth_is_radial true [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_arducam_tof_reads_the_applied_tof_calibration]
- a3: root tests, root lint and ansible lint pass [evidence: command:uv run pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`
- `tests/test_head_rgbd_calibration_deploy.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces

Uses `tof-depth-z` (config keys `tof_calibration_file`, `depth_is_radial` of arducam_tof). Keep the apply path unit, the seed tasks (force: false) and the head_rgbd_cpp `tof_calibration_file`.

## Expected outcome

`uv run pytest tests/test_head_rgbd_calibration_deploy.py -q` passes; `uv run poe test-ansible` clean.

## Steps

- Edit tests/test_head_rgbd_calibration_deploy.py; run it, see FAIL.
- Add the two keys to the arducam_tof config block in client.yml and arducam_tof to the try-restart line of the apply service template; rerun, see PASS.
- `uv run poe lint && uv run poe test-ansible`; update ansible/README.md and tests/README.md.
- Commit `feat(ansible): arducam_tof reads the applied ToF calibration`.

## Notes

The unit name pattern is `ros2-<node name>.service` (see the existing try-restart line).
