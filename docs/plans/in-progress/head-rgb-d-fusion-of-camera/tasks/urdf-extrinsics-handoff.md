---
title: URDF extrinsics handoff
status: done
repo: ros2-lerobot-sverve-platform
value: "The measured camera spacing lands in the head URDF that every TF consumer already reads, so there is one geometry source."
files: [scripts/head_rgbd_calibration/urdf_handoff.py, tests/test_head_rgbd_urdf_handoff.py, nodes/web_ui/tests/test_head_urdf.py]
verify: "uv run pytest tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration && cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q && uv run --frozen poe lint"
depends_on: [rgbd-extrinsic-calibration]
owns: [head-rgbd-urdf-handoff]
uses: [head-rgbd-extrinsics-file]
model: sonnet
estimate: 60
actual: ""
review: ""
branch: "worktree branch of head-rgb-d-fusion-of-camera--urdf-extrinsics-handoff (HEAD 9bd313f2)"
verified_head: 9bd313f28afb6b932b6ff1d47915769cbc9dcff4
run: head-rgb-d-fusion-of-camera
provides: []
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--urdf-extrinsics-handoff
acs_for_audit: [a2]
process_gaps: []
---

# URDF extrinsics handoff

## Requirement

- `scripts/head_rgbd_calibration/urdf_handoff.py` (D4):
  - `read_chain(urdf_text: str) -> HeadChain`: the fixed joints `head_camera_joint`, `head_camera_optical_joint`, `tof_joint`, `tof_optical_joint` (origin xyz and rpy) and their parents/children, as parsed by `xml.etree.ElementTree`; raises naming a missing joint.
  - `tof_joint_origin(chain: HeadChain, extrinsic: ExtrinsicResult) -> Origin`: the `tof_joint` origin (relative to `head_link`) such that `T(head_camera_optical_frame <- tof_optical_frame)` computed through the URDF equals the calibrated transform: `T_head_tof = T_head_cam * T_cam_camopt * T_camopt_tofopt * inv(T_tof_tofopt)` (scipy `Rotation` for rpy conversion, URDF fixed-axis rpy convention).
  - `rewrite_tof_joint(urdf_text: str, origin: Origin, note: str) -> str`: replaces only the `xyz`/`rpy` attributes of the `<origin>` inside `<joint name="tof_joint">` (text edit, every other byte unchanged, comments kept) and puts `note` (calibration date and stereo RMS) in a comment directly above that joint, replacing a previous calibration comment rather than stacking them.
  - `check_drift(chain, extrinsic) -> Drift`: translation (mm) and rotation (deg) between the URDF's current transform and the calibration.
- `nodes/web_ui/tests/test_head_urdf.py` (pan/tilt epic test): the exact ToF origin assertion becomes: ToF y > RGB y, camera-to-ToF spacing within 0.05 +- 0.015 m, `tof_joint` origin |x| <= 0.05 m and |rpy| <= 0.1 rad; every structural check (chain, axes, limits, optical rpy, post) stays exact.
- Tests first in `tests/test_head_rgbd_urdf_handoff.py` on a copy of a URDF fixture with the pan/tilt chain (read from `nodes/web_ui/urdf/head.urdf` when it exists, else an inline fixture with the same joints), seen failing: rewriting with a known extrinsic and re-reading the chain reproduces the extrinsic within 1e-6 m / 1e-6 rad; the diff between input and output touches only the tof_joint origin line and the calibration comment; a second rewrite replaces the comment; a missing joint raises; drift is zero after a rewrite and equals the injected offset before.

## Acceptance conditions

- a1: the rewritten URDF chain reproduces the calibrated ToF-to-RGB transform [evidence: test: tests/test_head_rgbd_urdf_handoff.py]
- a2: only the tof_joint origin and its calibration comment change [evidence: test: tests/test_head_rgbd_urdf_handoff.py]
- a3: drift between URDF and calibration is reported without writing [evidence: test: tests/test_head_rgbd_urdf_handoff.py]
- a4: the head URDF test accepts calibrated values within tolerance and keeps the structural checks [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q]

## Files

- `scripts/head_rgbd_calibration/urdf_handoff.py`
- `tests/test_head_rgbd_urdf_handoff.py`
- `nodes/web_ui/tests/test_head_urdf.py`

## Interfaces

- `scripts/head_rgbd_calibration/urdf_handoff.py:read_chain`, `tof_joint_origin`, `rewrite_tof_joint`, `check_drift` (part `head-rgbd-urdf-handoff`).

## Expected outcome

Given an extrinsics file, the head URDF's tof_joint is rewritten so TF reproduces the calibration exactly.

## Steps

- Write the tests and relax the web_ui URDF test; run `uv run pytest tests/test_head_rgbd_urdf_handoff.py -q`; see it fail.
- Implement urdf_handoff.py; rerun until green; run the full verify command.
- Commit `feat(scripts): write calibrated ToF extrinsic into the head URDF`.

## Notes

- The head URDF is created by the pan/tilt epic (blocker); its chain: head_link -> head_camera_joint / tof_joint -> optical joints with rpy (-pi/2, 0, -pi/2).
- After the runbook rewrites the URDF, head_state_publisher (robot_state_publisher) must be redeployed to publish the new TF.
