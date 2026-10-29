"""Single source of truth: the MCP camera -> arm transform equals the calibration library's, with an applied chain."""

import math
from pathlib import Path

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base
from ros2_common.head_rgbd_calibration.urdf_text import Origin, apply_overrides, head_chain_json

from mcp_server.camera_point import CAMERA_FRAMES, frames_of_point
from mcp_server.camera_scene import build_scene, camera_setup, head_parent_transform
from mcp_server.config import McpServerConfig
from mcp_server.head import AppliedHeadKinematics, HeadController
from mcp_server.head_kinematics import CAMERA_FRAME, DEPTH_FRAME, HeadKinematics

from .fakes import FakeHeadBackend

COLOUR_INFO = {"hfov_deg": 69.4, "width": 640, "height": 360}
D435_MOUNT = {"parent_frame": "d435_color_frame", "x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
ARM_BASE = {"x": 0.0592, "y": -0.05, "z": 0.100, "yaw": 0.0}
# The applied chain: every joint a calibration writes, moved off the repo URDF by realistic fit-sized amounts.
APPLIED = {
    "head_mount_joint": Origin((0.151, 0.012, 0.309), (0.004, -0.021, 0.026)),
    "head_pan": Origin((0.003, -0.002, 0.018), (0.0, 0.0, 0.0)),
    "head_tilt": Origin((0.0, 0.0, 0.0412), (0.011, 0.083, 0.0)),
    "d435_joint": Origin((0.021, 0.0, 0.012), (0.002, 0.0, -0.011)),
}
POSES = [(-1.3, -0.4), (-0.6, 0.1), (0.0, 0.0), (0.0, 0.5236), (0.4, 0.2), (0.9, 0.7), (1.4, -0.3), (-0.2, 0.78)]
ROT_ATOL = 1e-7  # head.urdf writes the optical rotation as 1.5707963, camera_scene uses pi/2: 2.7e-8 rad apart
TRANS_ATOL = 1e-9
URDF_PATH = McpServerConfig().head.urdf_path
RANDOM_POINTS = np.random.default_rng(7).uniform(-0.6, 0.6, size=(6, 3))


def applied_dir(tmp_path: Path) -> Path:
    directory = tmp_path / "applied"
    directory.mkdir()
    (directory / "head_chain.json").write_text(head_chain_json(APPLIED, "synthetic", "calib-1", "r-1"))
    return directory


def mcp_config(applied: Path) -> McpServerConfig:
    return McpServerConfig.model_validate(
        {
            "head": {"applied_dir": str(applied)},
            "cameras": {"front": {"intrinsics": COLOUR_INFO, "mount": D435_MOUNT}},
            "arm": {"base_in_base_link": ARM_BASE},
        }
    )


def mcp_head(cfg: McpServerConfig, pan: float, tilt: float) -> HeadController:
    backend = FakeHeadBackend({"head_pan": pan, "head_tilt": tilt})
    return HeadController(backend, AppliedHeadKinematics(cfg.head.urdf_path, cfg.head.applied_dir), cfg.head)


def calibration_chain(applied: Path) -> UrdfChain:
    text, warnings = apply_overrides(URDF_PATH.read_text(), applied)
    assert warnings == []
    return UrdfChain.from_text(text)


def calibration_t_arm_optical(chain: UrdfChain, frame: str, pan: float, tilt: float) -> np.ndarray:
    """The model the head-from-arm fit solved: camera pose in base_link from the chain, base_link in the arm root."""
    t_base_camera = chain.transform(frame, {"head_pan": pan, "head_tilt": tilt})
    return np.linalg.inv(arm_in_robot_base(ARM_BASE)) @ t_base_camera


@pytest.mark.parametrize(("pan", "tilt"), POSES)
def test_scene_camera_to_arm_equals_the_calibration_chain(tmp_path: Path, pan: float, tilt: float) -> None:
    applied = applied_dir(tmp_path)
    cfg = mcp_config(applied)
    head = mcp_head(cfg, pan, tilt)
    scene = build_scene(camera_setup(cfg, "front"), cfg, head_parent_transform(cfg, head))
    assert scene.t_arm_ref is not None
    mcp = scene.t_arm_ref @ scene.t_ref_optical
    expected = calibration_t_arm_optical(calibration_chain(applied), CAMERA_FRAME, pan, tilt)
    assert np.allclose(mcp[:3, :3], expected[:3, :3], atol=ROT_ATOL, rtol=0)
    assert np.allclose(mcp[:3, 3], expected[:3, 3], atol=TRANS_ATOL, rtol=0)


@pytest.mark.parametrize("frame", [CAMERA_FRAME, DEPTH_FRAME])
@pytest.mark.parametrize(("pan", "tilt"), POSES)
def test_head_kinematics_link_pose_equals_the_calibration_chain_to_1e9(
    tmp_path: Path, frame: str, pan: float, tilt: float
) -> None:
    applied = applied_dir(tmp_path)
    kin = AppliedHeadKinematics(URDF_PATH, applied).current()
    expected = calibration_chain(applied).transform(frame, {"head_pan": pan, "head_tilt": tilt})
    assert np.allclose(kin.link_pose(pan, tilt, frame), expected, atol=TRANS_ATOL, rtol=0)


@pytest.mark.parametrize("frame", list(CAMERA_FRAMES))
@pytest.mark.parametrize(("pan", "tilt"), POSES)
def test_frames_of_point_matches_the_calibration_chain_to_1e9(
    tmp_path: Path, frame: str, pan: float, tilt: float
) -> None:
    applied = applied_dir(tmp_path)
    cfg = mcp_config(applied)
    kin = AppliedHeadKinematics(URDF_PATH, applied).current()
    t_arm_base = np.linalg.inv(arm_in_robot_base(ARM_BASE))
    expected = calibration_t_arm_optical(calibration_chain(applied), frame, pan, tilt)
    for point in RANDOM_POINTS:
        got = frames_of_point(kin.link_pose(pan, tilt, frame), t_arm_base, point)
        want = (expected @ np.append(point, 1.0))[:3]
        assert got.camera == pytest.approx(point, abs=1e-12)
        assert got.arm is not None and np.allclose(got.arm, want, atol=TRANS_ATOL, rtol=0)
        want_base = (
            calibration_chain(applied).transform(frame, {"head_pan": pan, "head_tilt": tilt}) @ np.append(point, 1)
        )[:3]
        assert np.allclose(got.base_link, want_base, atol=TRANS_ATOL, rtol=0)
    assert cfg.arm.base_in_base_link is not None


def test_frames_of_point_without_an_arm_offset_has_no_arm_point() -> None:
    frames = frames_of_point(np.eye(4), None, np.array([0.1, 0.2, 0.3]))
    assert frames.arm is None and frames.base_link == pytest.approx([0.1, 0.2, 0.3])


def test_repo_chain_without_an_applied_dir_also_agrees(tmp_path: Path) -> None:
    kin = HeadKinematics(URDF_PATH)
    chain = UrdfChain.from_text(URDF_PATH.read_text())
    for pan, tilt in POSES:
        expected = chain.transform(CAMERA_FRAME, {"head_pan": pan, "head_tilt": tilt})
        assert np.allclose(kin.link_pose(pan, tilt, CAMERA_FRAME), expected, atol=TRANS_ATOL, rtol=0)


def test_the_arm_mount_the_tools_use_is_the_one_the_fit_uses() -> None:
    """arm.base_in_base_link (mcp_server default) is the same pose arm_in_robot_base builds for the head-arm fit."""
    default = McpServerConfig().arm.base_in_base_link
    assert default is not None
    assert (default.x, default.y, default.z, default.yaw) == (
        ARM_BASE["x"],
        ARM_BASE["y"],
        ARM_BASE["z"],
        ARM_BASE["yaw"],
    )
    assert math.isclose(default.z, McpServerConfig().arm.arm_base_height_m)
