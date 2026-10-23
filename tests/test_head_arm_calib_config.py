"""Tests of the head-from-arm fit config, the board corner geometry and the view types."""

from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.charuco import Detection, board_corners_3d, board_outline, detect, make_board
from ros2_common.head_rgbd_calibration.config import (
    BaseInBaseLink,
    BoardConfig,
    BoardMountConfig,
    HeadArmFitConfig,
    HeadArmFitLimits,
)
from ros2_common.head_rgbd_calibration.views import ARM_JOINTS, HEAD_JOINTS, ArmBoardView, require_joints

from tests.rgbd_synthetic import board_poses, render_rgb, rgb_camera

ROOT = Path(__file__).resolve().parents[1]
MARKERS_YAML = ROOT / "scripts" / "markers_generate.yaml"
CLIENT_YML = ROOT / "ansible" / "group_vars" / "client.yml"
SQUARE_M = 0.040
PROJECTION_TOLERANCE_PX = 0.5


def client_arm() -> dict:
    """The mcp_server arm block of client.yml."""
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    node = next(n for n in nodes if n["name"] == "mcp_server")
    return yaml.safe_load(node["config"])["arm"]


def test_head_arm_fit_config_defaults() -> None:
    cfg = HeadArmFitConfig()
    assert cfg.board_mount == BoardMountConfig(xyz_init=(0, 0, 0), rpy_init=(0, 0, 0), outline_margin_m=0.01)
    assert cfg.fit_pan_axis is True
    assert cfg.limits == HeadArmFitLimits(
        rms_px=3.0,
        holdout_rms_px=4.0,
        holdout_board_mm=4.5,
        max_sigma_mm=2.0,
        max_sigma_deg=0.2,
        max_correlation=0.99,
        min_views=40,
        min_arm_poses=16,
        min_pan_span_deg=30.0,
        min_tilt_span_deg=10.0,
        holdout_every=5,
        max_floor_delta_mm=10.0,
        max_floor_delta_deg=1.5,
        max_start_delta_mm=30.0,
        max_start_delta_deg=5.0,
        max_pan_axis_delta_mm=50.0,
    )
    assert cfg.huber_f_scale_px == 1.0
    assert cfg.floor_prior_weight == 10.0  # user decision 2026-10-23 (replaces D49 prior off)
    assert cfg.base_in_base_link == BaseInBaseLink(x=0.0592, y=-0.05, z=0.100, yaw=0.0)


def test_board_is_the_existing_charuco_board() -> None:
    spec = yaml.safe_load(MARKERS_YAML.read_text())["charuco"]
    board = HeadArmFitConfig().board
    assert board == BoardConfig(**{k: spec[k] for k in BoardConfig.model_fields})


def test_unknown_keys_and_bad_joint_set_rejected() -> None:
    with pytest.raises(ValidationError):
        HeadArmFitConfig(bogus=1)
    with pytest.raises(ValidationError):
        HeadArmFitLimits(bogus=1)
    with pytest.raises(ValidationError, match="wrist_roll"):
        HeadArmFitConfig(joint_offsets_rad={"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 0, "wrist_flex": 0})
    with pytest.raises(ValidationError, match="gripper"):
        HeadArmFitConfig(joint_offsets_rad={**HeadArmFitConfig().joint_offsets_rad, "gripper": 0.0})
    with pytest.raises(ValidationError):
        HeadArmFitLimits(holdout_every=1)


def test_arm_gauge_matches_client_yml() -> None:
    arm = client_arm()
    cfg = HeadArmFitConfig()
    assert cfg.joint_offsets_rad == arm["joint_offsets_rad"]
    assert cfg.base_in_base_link.model_dump() == arm["base_in_base_link"]


def test_board_corners_3d_geometry() -> None:
    corners = board_corners_3d(BoardConfig())
    assert corners.shape == (20, 3)
    assert np.allclose(corners[:, 2], 0)
    assert np.allclose(corners[1] - corners[0], [SQUARE_M, 0, 0])
    assert np.allclose(corners[5] - corners[0], [0, SQUARE_M, 0])


def test_board_corners_3d_match_detection() -> None:
    cfg = BoardConfig()
    camera = rgb_camera()
    pose = board_poses(1, camera, cfg, distance_m=(0.4, 0.5), max_tilt_deg=15)[0]
    detection = detect(render_rgb(camera, pose, cfg), make_board(cfg))
    assert detection is not None
    corners = board_corners_3d(cfg)[detection.ids]
    projected, _ = cv2.projectPoints(corners, pose.rvec, pose.t, camera.k, camera.d)
    error = np.linalg.norm(projected.reshape(-1, 2) - detection.points, axis=1)
    assert error.max() < PROJECTION_TOLERANCE_PX


def test_board_outline_spans_outer_squares_plus_margin() -> None:
    cfg = BoardConfig()
    outline = board_outline(cfg, 0.01)
    assert outline.shape == (4, 3)
    assert np.allclose(outline.min(axis=0), [-0.01, -0.01, 0])
    assert np.allclose(outline.max(axis=0), [6 * SQUARE_M + 0.01, 5 * SQUARE_M + 0.01, 0])
    assert np.allclose(board_outline(cfg, 0.0).max(axis=0), [0.24, 0.2, 0])


def test_require_joints_names_missing() -> None:
    detection = Detection(ids=np.zeros(0, dtype=np.int32), points=np.zeros((0, 2), dtype=np.float32))
    joints = dict.fromkeys((*HEAD_JOINTS, *ARM_JOINTS), 0.0)
    require_joints(ArmBoardView("v1", 0, 0, joints, detection))
    del joints["head_tilt"], joints["wrist_roll"]
    with pytest.raises(ValueError, match=r"v2.*head_tilt.*wrist_roll"):
        require_joints(ArmBoardView("v2", 0, 0, joints, detection))
