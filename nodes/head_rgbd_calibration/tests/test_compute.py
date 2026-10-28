"""Tests for the compute step: a synthetic D435 intrinsics check session written through the store, then computed."""

import dataclasses
import json
from datetime import datetime
from functools import cache
from pathlib import Path

import numpy as np
import pytest
import yaml
from ros2_common.head_rgbd_calibration.charuco import detect, make_board
from ros2_common.head_rgbd_calibration.frames import CameraInfoData, StampedImage

from head_rgbd_calibration.compute import compute_result, main
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import ColourDepthStill, SessionStore
from tests.contract import assert_matches_fixture
from tests.pose_factory import BOARD_CFG, STAMP_NS, factory_info, load_synthetic

NOW = datetime(2026, 10, 19, 12, 30, 45)
VIEWS = 20
DISTANCE_M = (0.4, 0.9)
FOCAL_ERROR = 1.03
PLANE_OFFSET_MM = 15
RESULT_FILES = {"result.json", "report.md"}


@cache
def synthetic_stills() -> tuple[ColourDepthStill, ...]:
    """Stills of the synthetic board spread over distance and tilt, with exact aligned depth and the true K."""
    syn = load_synthetic()
    camera = syn.rgb_camera()
    board = make_board(BOARD_CFG)
    stills = []
    for i, pose in enumerate(syn.board_poses(VIEWS, camera, BOARD_CFG, seed=3, distance_m=DISTANCE_M)):
        rgb = syn.render_rgb(camera, pose, BOARD_CFG, seed=i)
        detection = detect(rgb, board)
        if detection is None:
            continue
        stamp = STAMP_NS + i
        stills.append(
            ColourDepthStill(
                number=len(stills) + 1,
                rgb=StampedImage(stamp, rgb),
                rgb_raw=None,
                depth=StampedImage(stamp, syn.render_depth(camera, pose, BOARD_CFG)),
                detection=detection,
                camera_info=factory_info(),
            )
        )
    assert len(stills) >= NodeConfig().limits.min_views
    return tuple(stills)


def make_cfg(tmp_path: Path, **limits: float) -> NodeConfig:
    return NodeConfig.model_validate({"state_dir": str(tmp_path / "state"), "limits": limits})


def make_session(
    store: SessionStore, count: int = VIEWS, info: CameraInfoData | None = None, depth_offset_mm: int = 0
) -> str:
    """An intrinsics session of the first count synthetic stills, the factory info and depth changed when given."""
    sid = store.create_session(NOW)
    for still in synthetic_stills()[:count]:
        depth = still.depth.image
        if depth_offset_mm:
            depth = np.where(depth > 0, depth.astype(np.int64) + depth_offset_mm, 0).astype(np.uint16)
        store.add_pose(
            sid,
            dataclasses.replace(
                still, depth=StampedImage(still.depth.stamp_ns, depth), camera_info=info or still.camera_info
            ),
        )
    return sid


def edit_pose(store: SessionStore, sid: str, pid: str, change) -> None:
    """Rewrite one stored pose.json through change(meta)."""
    path = store.pose_dir(sid, pid) / "pose.json"
    meta = json.loads(path.read_text())
    change(meta)
    path.write_text(json.dumps(meta))


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return make_cfg(tmp_path)


@pytest.fixture
def store(cfg: NodeConfig) -> SessionStore:
    return SessionStore(Path(cfg.state_dir), cfg)


def test_intrinsics_session_result(store: SessionStore, cfg: NodeConfig) -> None:
    """A check session computes a pass with the plane bias and writes no calibration file."""
    sid = make_session(store)
    rid = store.new_result_id(NOW)
    result = compute_result(store, sid, rid, cfg, NOW)
    out = store.result_dir(sid, rid)
    assert result["passed"] is True, result
    assert result["kind"] == "intrinsics"
    assert json.loads((out / "result.json").read_text()) == result
    assert {p.name for p in out.iterdir()} == RESULT_FILES
    assert [p.name for p in out.parent.iterdir()] == [rid]
    assert result["id"] == rid and result["session"] == sid and result["failed_limits"] == []
    assert result["pose_ids"] == store.pose_ids(sid) and result["views"] == len(store.pose_ids(sid))
    assert result["rms_px"] < cfg.limits.intrinsics_max_rms_px
    assert result["focal_diff_pct"] < cfg.limits.intrinsics_max_focal_diff_pct
    assert result["principal_diff_px"] < cfg.limits.intrinsics_max_principal_diff_px
    assert abs(result["plane_bias_median_mm"]) < 1.0 and 0.0 <= result["plane_bias_spread_mm"] < 2.0
    assert [p["id"] for p in result["poses"]] == store.pose_ids(sid)
    assert all(isinstance(p["error_px"], float) and isinstance(p["plane_bias_mm"], float) for p in result["poses"])
    assert result["limits"] == cfg.limits.model_dump()
    assert_matches_fixture({**result, "report": (out / "report.md").read_text()}, "result")
    report = (out / "report.md").read_text()
    assert sid in report and "Result: PASS" in report and "Board plane depth bias" in report


def test_focal_offset_fails_the_check(tmp_path: Path) -> None:
    cfg = make_cfg(tmp_path)
    store = SessionStore(Path(cfg.state_dir), cfg)
    k = factory_info().k.reshape(3, 3).copy()
    k[0, 0] *= FOCAL_ERROR
    k[1, 1] *= FOCAL_ERROR
    sid = make_session(store, info=factory_info(k))
    rid = store.new_result_id(NOW)
    result = compute_result(store, sid, rid, cfg, NOW)
    assert result["passed"] is False and "error" not in result
    assert any("limits.intrinsics_max_focal_diff_pct" in f for f in result["failed_limits"])
    assert result["focal_diff_pct"] == pytest.approx(100 * (1 - 1 / FOCAL_ERROR), abs=0.5)
    assert {p.name for p in store.result_dir(sid, rid).iterdir()} == RESULT_FILES
    assert "Result: FAIL" in (store.result_dir(sid, rid) / "report.md").read_text()


def test_plane_bias_over_the_limit_fails(tmp_path: Path) -> None:
    cfg = make_cfg(tmp_path)
    assert cfg.limits.plane_max_bias_mm == 10.0
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = make_session(store, depth_offset_mm=PLANE_OFFSET_MM)
    result = compute_result(store, sid, store.new_result_id(NOW), cfg, NOW)
    assert result["plane_bias_median_mm"] == pytest.approx(PLANE_OFFSET_MM, abs=1.0)
    assert result["passed"] is False
    assert [f for f in result["failed_limits"] if "limits.plane_max_bias_mm" in f]


def test_unmeasured_plane_bias_fails(store: SessionStore, cfg: NodeConfig) -> None:
    sid = store.create_session(NOW)
    for still in synthetic_stills():
        patch = np.zeros_like(still.depth.image)
        patch[:5, :5] = 500  # valid depth only in an image corner, off the board
        store.add_pose(sid, dataclasses.replace(still, depth=StampedImage(still.depth.stamp_ns, patch)))
    result = compute_result(store, sid, store.new_result_id(NOW), cfg, NOW)
    assert result["passed"] is False and "error" not in result
    assert result["plane_bias_median_mm"] is None and result["plane_bias_spread_mm"] is None
    assert any("limits.plane_max_bias_mm" in f and "not measured" in f for f in result["failed_limits"])


def test_missing_factory_camera_info_is_an_error(store: SessionStore, cfg: NodeConfig) -> None:
    sid = make_session(store)
    edit_pose(store, sid, store.pose_ids(sid)[0], lambda meta: meta.pop("camera_info"))
    rid = store.new_result_id(NOW)
    result = compute_result(store, sid, rid, cfg, NOW)
    assert result["passed"] is False and "camera_info" in result["error"]
    assert result["kind"] == "intrinsics"
    assert [p.name for p in store.result_dir(sid, rid).iterdir()] == ["result.json"]


def test_camera_info_changing_within_the_session_is_an_error(store: SessionStore, cfg: NodeConfig) -> None:
    sid = make_session(store)

    def shift(meta: dict) -> None:
        meta["camera_info"]["k"][0] += 5.0

    edit_pose(store, sid, store.pose_ids(sid)[1], shift)
    result = compute_result(store, sid, store.new_result_id(NOW), cfg, NOW)
    assert result["passed"] is False and "camera_info" in result["error"]


def test_too_few_poses_reports_error(store: SessionStore, cfg: NodeConfig) -> None:
    sid = make_session(store, 3)
    rid = store.new_result_id(NOW)
    result = compute_result(store, sid, rid, cfg, NOW)
    assert result["limits"] == cfg.limits.model_dump()
    out = store.result_dir(sid, rid)
    assert result["passed"] is False and result["error"]
    assert [p.name for p in out.iterdir()] == ["result.json"]
    assert json.loads((out / "result.json").read_text())["error"] == result["error"]


def test_head_arm_session_is_an_error(store: SessionStore, cfg: NodeConfig) -> None:
    sid = store.create_session(NOW, kind="head_arm")
    result = compute_result(store, sid, store.new_result_id(NOW), cfg, NOW)
    assert result["passed"] is False and "intrinsics" in result["error"]


def test_error_result_has_empty_pose_lists(store: SessionStore, cfg: NodeConfig) -> None:
    result = compute_result(store, make_session(store, 3), store.new_result_id(NOW), cfg, NOW)
    assert result["error"] and result["poses"] == [] and result["views"] is None
    assert result["rms_px"] is None and result["plane_bias_median_mm"] is None


def test_main_runs_one_computation(tmp_path: Path) -> None:
    cfg = make_cfg(tmp_path)
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = make_session(store)
    config_path = tmp_path / "node.yaml"
    config_path.write_text(yaml.safe_dump(cfg.model_dump(mode="json")))
    rid = store.new_result_id(NOW)
    assert main(["--config", str(config_path), "--session", sid, "--result", rid]) == 0
    assert json.loads((store.result_dir(sid, rid) / "result.json").read_text())["passed"] is True
