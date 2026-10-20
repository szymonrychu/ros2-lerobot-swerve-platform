---
title: URDF chain FK core
status: todo
repo: ros2-lerobot-sverve-platform
value: "One tested forward-kinematics chain for the head and the arm, so both fits and the pose checks agree on the geometry."
files: [shared/ros2_common/head_rgbd_calibration/chain.py, tests/test_head_arm_calib_chain.py]
verify: "uv run --frozen pytest tests/test_head_arm_calib_chain.py -q && uv run poe lint"
depends_on: []
owns: [urdf-chain-fk]
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# URDF chain FK core

## Requirement

A numpy forward-kinematics chain over URDF text, shared by the head fit, the arm fit and the arm pose sequence (owns `urdf-chain-fk`, design D3, D4).

- New `shared/ros2_common/head_rgbd_calibration/chain.py` (numpy and scipy `Rotation`; no ikpy, no ROS):
  - `UrdfChain.from_text(urdf_text: str) -> UrdfChain`: parses joints (`fixed`, `revolute`, `continuous`; any other type raises ValueError naming the joint), parents, origins and axes.
  - `UrdfChain.transform(link: str, positions: dict[str, float], origins: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] | None = None, offsets: dict[str, float] | None = None) -> np.ndarray`: 4x4 pose of `link` in the URDF root. Movable joint angle = `positions[name] + offsets.get(name, 0.0)` (urdf_angle = measured_angle + offset, as `nodes/mcp_server/mcp_server/ik.py::ArmKinematics`). `origins` replaces a joint's URDF origin (xyz, rpy) and nothing else. A movable joint on the path missing from `positions` raises KeyError naming it (never 0). Unknown link raises KeyError.
  - `UrdfChain.joint_names(link) -> list[str]` (root to link), `UrdfChain.origin(joint) -> tuple[xyz, rpy]`, `UrdfChain.limits(joint) -> tuple[float, float] | None`.
  - `pose_from_xyz_rpy(xyz, rpy) -> np.ndarray`, `xyz_rpy_from_pose(T) -> tuple[xyz, rpy]` (fixed-axis xyz, the URDF convention, `urdf_handoff.RPY_SEQUENCE`).
  - `arm_in_robot_base(base: BaseInBaseLink | dict) -> np.ndarray`: transform from the arm URDF root (`base_link` of so101_arm.urdf) to robot base_link (x, y, z, yaw).
- New `tests/test_head_arm_calib_chain.py` on the repo URDFs (`nodes/web_ui/urdf/head.urdf`, `nodes/web_ui/urdf/so101_arm.urdf`):
  - head camera optical frame at pan = tilt = 0 equals the hand-computed product of the URDF origins (mount 0.15 0.10 0.31, tilt 0.04, camera 0.02 -0.025 0, optical rpy -1.5708 0 -1.5708);
  - head_pan +0.5 rotates the camera about +z (CCW from above); head_tilt + pitches down (the optical z axis gains a negative base_link z component);
  - an origin override changes exactly that joint; offsets add to measured; a missing joint raises KeyError naming it;
  - so101 gripper_link at zero joints equals the product of origins; xyz/rpy round trip within 1e-12;
  - `arm_in_robot_base` with the default gauge maps the arm root to (0.0592, -0.05, 0.100).

## Acceptance conditions

- a1: the head camera pose at zero equals the URDF origin product [evidence: test:tests/test_head_arm_calib_chain.py::test_head_camera_pose_at_zero_matches_urdf_origins]
- a2: pan and tilt signs follow the URDF conventions [evidence: test:tests/test_head_arm_calib_chain.py::test_pan_left_and_tilt_down_signs]
- a3: overrides replace one joint origin and offsets add to measured angles [evidence: test:tests/test_head_arm_calib_chain.py::test_origin_override_and_offsets]
- a4: a missing joint position raises naming the joint [evidence: test:tests/test_head_arm_calib_chain.py::test_missing_joint_raises]
- a5: the arm root maps into robot base_link by the gauge [evidence: test:tests/test_head_arm_calib_chain.py::test_arm_in_robot_base_default_gauge]
- a6: UrdfChain exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/chain.py::UrdfChain]
- a7: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/chain.py`
- `tests/test_head_arm_calib_chain.py`

## Interfaces

- `chain.py::UrdfChain.from_text`, `UrdfChain.transform`, `joint_names`, `origin`, `limits`; `pose_from_xyz_rpy`, `xyz_rpy_from_pose`, `arm_in_robot_base` (part `urdf-chain-fk`).

## Expected outcome

One tested forward-kinematics chain for the head and the arm, so both fits and the pose checks agree on the geometry.

## Steps

- Tests first (red), implement, run the verify command.
- Run `uv run --frozen pytest tests/test_head_arm_calib_chain.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

