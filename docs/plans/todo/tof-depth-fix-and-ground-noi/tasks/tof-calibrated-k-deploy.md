---
title: ToF calibrated K deploy
status: todo
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
worktree: ""
branch: ""
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
