---
title: Calibration apply restart units
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Applying a calibration restarts exactly the nodes that read it.
files: [ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2, ansible/playbooks/tasks/head_rgbd_calibration_setup.yml, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, ansible/group_vars/client.yml, tests/test_head_rgbd_calibration_deploy.py]
verify: "uv run pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint && uv run poe test-ansible && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py -q"
depends_on: [d435-intrinsics-check-comput, floor-sweep-on-d435-depth, head-arm-calibration-on-d435]
model: opus
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Calibration apply restart units

## Requirement

Goal G8. Design D10: "the apply restart units become head_rgbd_cpp + head_state_publisher" and the ToF/CM3 applied files are retired.

- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2` and the calibration node `restart_units` (config defaults, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, client.yml block): `ros2-head_rgbd_cpp.service`, `ros2-head_state_publisher.service` (head_camera_cpp and arducam_tof readers of applied files are retired).
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`: the seeding of `head_camera_calibration.yaml`, `tof_calibration.yaml`, `head_rgbd_extrinsics.yaml` and `tof_joint.json` into `applied/` is removed. Seeding of `head_chain.json` (if present) is kept. Existing files on the robot are not deleted.
- `tests/test_head_rgbd_calibration_deploy.py`: `test_apply_units_content` (new units) and `test_seed_copies_never_overwrite_and_need_existing_source` (only head_chain.json) are updated. `test_arducam_tof_reads_the_applied_tof_calibration` is removed: no longer accurate, since arducam_tof is retired. New `test_no_tof_or_head_camera_applied_files_are_seeded`.

## Acceptance conditions

- AC1: an Apply restarts head_rgbd_cpp and head_state_publisher only [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_apply_units_content]
- AC2: no ToF or head-camera applied file is seeded [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_no_tof_or_head_camera_applied_files_are_seeded]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_calibration_deploy.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update and add the deploy tests; run `uv run pytest tests/test_head_rgbd_calibration_deploy.py -q`; expect failures.
- Edit the template, the setup task, the config defaults and client.yml; rerun; expect pass.
- Run the verify command; commit `chore(ansible): calibration apply restarts the D435 consumers only`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
