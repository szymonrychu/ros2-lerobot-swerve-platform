"""Tests for the compute step: a synthetic session written through the store, then computed."""

import json
from datetime import datetime
from functools import cache
from pathlib import Path

import numpy as np
import pytest
import yaml
from ros2_common.head_rgbd_calibration.calibrate import detect_tof
from ros2_common.head_rgbd_calibration.charuco import detect, make_board
from ros2_common.head_rgbd_calibration.frames import CameraInfoData, StampedImage
from ros2_common.head_rgbd_calibration.poses import CapturedPose
from ros2_common.head_rgbd_calibration.urdf_handoff import read_chain

from head_rgbd_calibration.compute import compute_result, main
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import SessionStore
from tests.pose_factory import BOARD_CFG, RGB_STAMP_NS, SELECTION, TOF_STAMP_NS, load_synthetic

NOW = datetime(2026, 10, 19, 12, 30, 45)
LENS = 1.5
POSES = 18
DISTANCE_M = (0.3, 0.45)
TILT_DEG = 20.0
URDF_FILE = Path(__file__).resolve().parents[3] / "nodes" / "web_ui" / "urdf" / "head.urdf"
SDK_K = [191.0, 0.0, 120.0, 0.0, 191.0, 90.0, 0.0, 0.0, 1.0]
SDK_P = [191.0, 0.0, 120.0, 0.0, 0.0, 191.0, 90.0, 0.0, 0.0, 0.0, 1.0, 0.0]


@cache
def synthetic_poses() -> tuple[CapturedPose, ...]:
    """Poses of one board seen by both synthetic cameras, spread over distance and tilt."""
    syn = load_synthetic()
    rgb_cam, tof_cam, true = syn.rgb_camera(), syn.tof_camera(), syn.default_extrinsic(0.05, 1.5)
    board = make_board(BOARD_CFG)
    poses = []
    candidates = syn.board_poses(
        POSES + 8, rgb_cam, BOARD_CFG, seed=11, distance_m=DISTANCE_M, max_tilt_deg=TILT_DEG, lateral_fraction=0.1
    )
    for i, pose in enumerate(candidates):
        tof_pose = true.tof_pose(pose)
        rgb = syn.render_rgb(rgb_cam, pose, BOARD_CFG, seed=i)
        amplitude = syn.render_tof_amplitude(tof_cam, tof_pose, BOARD_CFG, seed=i)
        rgb_det, tof_det = detect(rgb, board), detect_tof(amplitude, board, SELECTION)
        if rgb_det is None or tof_det is None:
            continue
        depth = syn.render_depth(tof_cam, tof_pose, BOARD_CFG)
        poses.append(
            CapturedPose(
                number=len(poses) + 1,
                rgb=StampedImage(RGB_STAMP_NS, rgb),
                rgb_raw=None,
                amplitude=StampedImage(TOF_STAMP_NS, amplitude),
                depth=StampedImage(TOF_STAMP_NS, depth),
                rgb_detection=rgb_det,
                tof_detection=tof_det,
            )
        )
    assert len(poses) >= POSES
    return tuple(poses[:POSES])


def make_cfg(tmp_path: Path, **limits: float) -> NodeConfig:
    return NodeConfig.model_validate(
        {"state_dir": str(tmp_path / "state"), "head_urdf_file": str(URDF_FILE), "limits": limits}
    )


def make_session(store: SessionStore, count: int, tof_info: bool = True) -> str:
    sid = store.create_session(NOW, LENS)
    if tof_info:
        store.set_tof_info(sid, CameraInfoData(240, 180, "plumb_bob", np.zeros(5), np.array(SDK_K), np.array(SDK_P)))
    for pose in synthetic_poses()[:count]:
        store.add_pose(sid, pose)
    return sid


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return make_cfg(tmp_path)


@pytest.fixture
def store(cfg: NodeConfig) -> SessionStore:
    return SessionStore(Path(cfg.state_dir), cfg)


def test_passing_result_and_files(store: SessionStore, cfg: NodeConfig) -> None:
    sid = make_session(store, POSES)
    rid = store.new_result_id(NOW)
    result = compute_result(store, sid, rid, cfg, NOW)
    out = store.result_dir(sid, rid)
    assert result["passed"] is True, result
    assert json.loads((out / "result.json").read_text()) == result
    assert {p.name for p in out.iterdir()} == {
        "head_camera_calibration.yaml",
        "tof_calibration.yaml",
        "head_rgbd_extrinsics.yaml",
        "tof_joint.json",
        "head.urdf",
        "report.md",
        "result.json",
    }
    assert [p.name for p in out.parent.iterdir()] == [rid]
    assert result["id"] == rid and result["session"] == sid
    assert result["failed_limits"] == [] and result["lens_position"] == LENS
    assert result["pose_ids"] == store.pose_ids(sid)
    assert result["rgb_views"] == POSES and result["tof_views"] == POSES and result["stereo_pairs"] == POSES
    for key in ("rgb_rms_px", "tof_rms_px", "stereo_rms_px", "plane_bias_mm"):
        assert isinstance(result[key], float)
    assert yaml.safe_load((out / "head_camera_calibration.yaml").read_text())["camera_name"] == "head_camera"
    assert yaml.safe_load((out / "tof_calibration.yaml").read_text())["camera_name"] == "tof"
    meta = yaml.safe_load((out / "head_rgbd_extrinsics.yaml").read_text())
    assert meta["session"] == sid and meta["lens_position"] == LENS and "board" in meta and "tof_sdk_diff_pct" in meta
    assert sid in (out / "report.md").read_text()


def test_tof_joint_json_and_head_urdf_agree(store: SessionStore, cfg: NodeConfig) -> None:
    sid = make_session(store, POSES)
    rid = store.new_result_id(NOW)
    compute_result(store, sid, rid, cfg, NOW)
    out = store.result_dir(sid, rid)
    joint = json.loads((out / "tof_joint.json").read_text())
    assert set(joint) == {"xyz", "rpy", "note"}
    chain = read_chain((out / "head.urdf").read_text())
    np.testing.assert_allclose(chain.tof.origin.xyz, joint["xyz"], atol=1e-6)
    np.testing.assert_allclose(chain.tof.origin.rpy, joint["rpy"], atol=1e-6)
    assert "stereo RMS" in joint["note"]


def test_limits_exceeded_gives_failed_result_with_report(tmp_path: Path) -> None:
    cfg = make_cfg(tmp_path, rgb_max_rms_px=1e-6)
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = make_session(store, POSES)
    rid = store.new_result_id(NOW)
    result = compute_result(store, sid, rid, cfg, NOW)
    out = store.result_dir(sid, rid)
    assert result["passed"] is False
    assert result["failed_limits"] and "limits.rgb_max_rms_px" in result["failed_limits"][0]
    assert result["rgb_rms_px"] > 0
    assert (out / "report.md").is_file() and (out / "result.json").is_file()
    assert not list(out.glob("*.yaml"))


def test_too_few_poses_gives_error_and_no_yaml(store: SessionStore, cfg: NodeConfig) -> None:
    sid = make_session(store, 3)
    rid = store.new_result_id(NOW)
    result = compute_result(store, sid, rid, cfg, NOW)
    out = store.result_dir(sid, rid)
    assert result["passed"] is False and result["error"]
    assert [p.name for p in out.iterdir()] == ["result.json"]
    assert json.loads((out / "result.json").read_text())["error"] == result["error"]


def test_missing_tof_info_gives_error(store: SessionStore, cfg: NodeConfig) -> None:
    sid = make_session(store, POSES, tof_info=False)
    result = compute_result(store, sid, store.new_result_id(NOW), cfg, NOW)
    assert result["passed"] is False and "camera_info" in result["error"]


def test_main_runs_one_computation(tmp_path: Path) -> None:
    cfg = make_cfg(tmp_path)
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = make_session(store, POSES)
    config_path = tmp_path / "node.yaml"
    config_path.write_text(yaml.safe_dump(cfg.model_dump()))
    rid = store.new_result_id(NOW)
    assert main(["--config", str(config_path), "--session", sid, "--result", rid]) == 0
    assert json.loads((store.result_dir(sid, rid) / "result.json").read_text())["passed"] is True
