---
title: Calib node cgroup tier and docs
status: done
repo: ros2-lerobot-sverve-platform
value: "The robot unit tiers stay complete with the new node in the background tier, and the test docs list every new test file."
files: [tests/test_unit_cgroups.py, ansible/group_vars/client.yml, tests/README.md, shared/ros2_common/head_rgbd_calibration/__init__.py]
verify: "uv run --frozen pytest tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe lint-ansible"
depends_on: [jetson-retirement-allowlist]
owns: []
uses: []
model: sonnet
estimate: 25
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-node-cgroup-tier-and-d
branch: head-rgb-d-calibration-in-th--calib-node-cgroup-tier-and-d
verified_head: "46271e5d"
run: ""
provides: []
---

# Calib node cgroup tier and docs

## Requirement

Close the integration gaps found by the epic's full-suite run and scope audit.

- `tests/test_unit_cgroups.py::test_every_client_type_is_tiered` fails on the epic branch (passes on main): the new `head_rgbd_calibration` node type is in no tier. Put it in the background tier (work the robot can lose: capture and compute must never starve robot nodes; memory feedback "cgroups guarantee headroom") by adding it to that tier's types in the test and setting its `ros2_node_type_defaults` cgroup keys in `ansible/group_vars/client.yml` to the background tier's weights; keep its own memory cap. Whole `tests/test_unit_cgroups.py` passes.
- `tests/README.md` lists and describes `tests/test_head_rgbd_calibration_deploy.py` (each test or test group), like the other entries.
- `shared/ros2_common/head_rgbd_calibration/__init__.py` docstring names every module of the package (config, charuco, frames, intrinsics, extrinsics, calibrate, poses, urdf_handoff, urdf_text, and any other present).

## Acceptance conditions

- a1: every client node type is tiered, head_rgbd_calibration in the background tier [evidence: test: tests/test_unit_cgroups.py::test_every_client_type_is_tiered]
- a2: tier weights hold for the new type [evidence: test: tests/test_unit_cgroups.py::test_tiers_have_expected_weights]
- a3: tests/README.md describes test_head_rgbd_calibration_deploy.py [evidence: command: grep -q test_head_rgbd_calibration_deploy tests/README.md]

## Files

- `tests/test_unit_cgroups.py`
- `ansible/group_vars/client.yml`
- `tests/README.md`
- `shared/ros2_common/head_rgbd_calibration/__init__.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

tests/test_unit_cgroups.py passes in full with head_rgbd_calibration in the background tier; tests/README.md describes test_head_rgbd_calibration_deploy.py; the core package docstring names every module.

## Steps

- Run `uv run --frozen pytest tests/test_unit_cgroups.py -q` and see test_every_client_type_is_tiered fail.
- Add the type to the background tier (test and client.yml weights), add the README entry and the docstring; run the verify command (all pass) and commit `fix: background cgroup tier for head_rgbd_calibration, test docs`.

## Ruling

- 2026-10-20 accepted: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
