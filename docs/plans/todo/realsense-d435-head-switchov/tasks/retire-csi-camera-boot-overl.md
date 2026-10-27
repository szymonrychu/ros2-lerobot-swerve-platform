---
title: Retire CSI camera boot overlays
status: todo
repo: ros2-lerobot-sverve-platform
value: The Pi boots without overlays for cameras that are no longer attached.
files: [ansible/playbooks/tasks/pi_camera_boot_config.yml, ansible/playbooks/tasks/arducam_tof_sdk.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_pi_camera_boot_config.py, tests/test_pi_cameras_retired.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_pi_camera_boot_config.py tests/test_pi_cameras_retired.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [retire-arducam-tof-in-ansibl, retire-head-camera-in-ansibl]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Retire CSI camera boot overlays

## Requirement

Goal G9: head ToF and head RGB retired. Design D11: "Then their source dirs, the CSI boot overlays, the ToF SDK task ... are deleted." The D435 is USB; no CSI camera remains.

- `ansible/playbooks/tasks/pi_camera_boot_config.yml`: the `dtoverlay=imx708,cam0` and `dtoverlay=arducam-pivariety,cam1` lines are removed from the client's boot config (lineinfile `state: absent`). `camera_auto_detect` stays off. The reboot-only-on-change rule is unchanged.
- `ansible/playbooks/tasks/arducam_tof_sdk.yml` is deleted together with its include in `ansible/playbooks/deploy_nodes_client.yml`. The boot config include keeps running under the retired node tags so their uninstall deploy also cleans the boot config.
- `tests/test_pi_camera_boot_config.py`: `test_auto_detect_is_off_and_both_overlays_are_written` and `test_client_playbook_includes_the_boot_config_under_both_camera_node_tags` are no longer accurate. They are replaced by `test_camera_overlays_are_removed` and `test_boot_config_runs_with_the_retired_camera_tags`. `tests/test_pi_cameras_retired.py` gains `test_arducam_sdk_task_is_gone` (git ls-files). `ansible/README.md` "CSI camera boot config" paragraph and tests/README.md updated.

## Acceptance conditions

- AC1: the boot config removes both CSI camera overlays and keeps auto-detect off [evidence: test:tests/test_pi_camera_boot_config.py::test_camera_overlays_are_removed]
- AC2: the Arducam SDK task file is no longer tracked or included [evidence: test:tests/test_pi_cameras_retired.py::test_arducam_sdk_task_is_gone]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/playbooks/tasks/pi_camera_boot_config.yml`
- `ansible/playbooks/tasks/arducam_tof_sdk.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_pi_camera_boot_config.py`
- `tests/test_pi_cameras_retired.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the new tests and drop the two stale ones; run `uv run pytest tests/test_pi_camera_boot_config.py tests/test_pi_cameras_retired.py -q`; expect failures.
- Edit the boot config task, delete the SDK task and its include, update ansible/README.md; rerun; expect pass.
- Run the verify command; commit `chore(ansible): remove the CSI camera overlays and the ToF SDK install`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
