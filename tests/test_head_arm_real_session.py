"""The head-from-arm fit on the real session calib-20261023-074418 (tests/fixtures/head_arm/, detections and joints).

The live result r-20261023-075643 of that session failed at 12.2 px: the +-10 deg pan offsets misfit by 13 px. These
tests pin the diagnosis (the head pan axis sits about 38 mm behind and 11 mm right of the head URDF value relative to
the camera) and the evidence behind the head_arm fit limits in the node config: the pan-axis fit and a leave-poses-out
validation over five folds of the 29 arm poses.
"""

import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import BoardMountConfig, FloorFitConfig, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.floor_fit import FloorObservation, PlaneFit, fit_head_chain_floor
from ros2_common.head_rgbd_calibration.head_arm_fit import FLOOR_PARAMS, HeadArmFit, fit_head_from_arm, limit_checks
from ros2_common.head_rgbd_calibration.urdf_text import TOF_JOINT, Origin, rewrite_joint_origin
from ros2_common.head_rgbd_calibration.views import ArmBoardView

REPO_ROOT = Path(__file__).resolve().parent.parent
FIXTURE = REPO_ROOT / "tests" / "fixtures" / "head_arm" / "calib-20261023-074418.json"
FLOOR_FIXTURE = REPO_ROOT / "tests" / "fixtures" / "head_arm" / "floor-20261022-064337.json"
FLOOR_PAN_AXIS_TOLERANCE_DEG = 0.1
FLOOR_PAN_AXIS_TOLERANCE_M = 0.0005
# The floor fit and the head-from-arm fit disagree on the pan axis direction (mount roll and pitch, with the tilt zero
# keeping the camera pitch at pan 0): these floor cross-checks fail on this session.
FLOOR_DISAGREEMENT = ["floor_delta:mount_roll_deg", "floor_delta:mount_pitch_deg", "floor_delta:tilt_zero_deg"]
URDF_DIR = REPO_ROOT / "nodes" / "web_ui" / "urdf"
FOLDS = 5
# The live result r-20261023-075643 (fixed pan axis): rms 12.2 px, held-out 9.4 px.
LIVE_RMS_PX = 12.2
PAN_AXIS_MM = (-37.9, -11.2)
PAN_AXIS_TOLERANCE_MM = 3.0
ACCURACY_LIMITS = (
    "rms_px",
    "holdout_rms_px",
    "holdout_board_mm",
    "max_sigma_mm",
    "max_sigma_deg",
    "max_correlation",
    "start_delta:pan_axis_x_m",
    "start_delta:pan_axis_y_m",
)


def load_session() -> tuple[dict, list[ArmBoardView]]:
    """The fixture document and its views."""
    doc = json.loads(FIXTURE.read_text())
    views = [
        ArmBoardView(
            view_id=v["view_id"],
            arm_pose=v["arm_pose"],
            tier=v["tier"],
            joints=dict(v["joints"]),
            detection=Detection(ids=np.asarray(v["ids"], dtype=np.int32), points=np.asarray(v["points"], dtype=float)),
            head_offset=v["head_offset"],
        )
        for v in doc["views"]
    ]
    return doc, views


def applied_urdf(doc: dict) -> str:
    """nodes/web_ui/urdf/head.urdf with the session's applied head_chain.json joints."""
    text = (URDF_DIR / "head.urdf").read_text()
    for joint, origin in doc["head_chain"].items():
        text = rewrite_joint_origin(text, joint, Origin(tuple(origin["xyz"]), tuple(origin["rpy"])), "applied")
    return text


def floor_observations() -> tuple[dict, list[FloorObservation]]:
    """The floor sweep fixture document and its observations."""
    doc = json.loads(FLOOR_FIXTURE.read_text())
    observations = [
        FloorObservation(
            pose=o["pose"],
            pan_rad=o["pan_rad"],
            tilt_rad=o["tilt_rad"],
            plane=PlaneFit(np.asarray(o["normal"]), o["height_m"], o["inliers"], o["candidates"], o["rms_m"]),
        )
        for o in doc["observations"]
    ]
    return doc, observations


def node_fit_config(doc: dict, **overrides: object) -> HeadArmFitConfig:
    """The node fit config of the session (board mount start); limits and the rest at the defaults (= node config)."""
    mount = doc["board_mount"]
    board_mount = BoardMountConfig(xyz_init=tuple(mount["xyz_init"]), rpy_init=tuple(mount["rpy_init"]))
    return HeadArmFitConfig(board_mount=board_mount, **overrides)


def run_fit(doc: dict, views: list[ArmBoardView], cfg: HeadArmFitConfig) -> HeadArmFit:
    return fit_head_from_arm(
        views,
        np.asarray(doc["camera_matrix"], dtype=float).reshape(3, 3),
        np.asarray(doc["dist"], dtype=float),
        applied_urdf(doc),
        (URDF_DIR / "so101_arm.urdf").read_text(),
        cfg,
        doc["floor"],
    )


def failed_accuracy(fit: HeadArmFit, cfg: HeadArmFitConfig) -> list[str]:
    return [name for name, _, _, ok in limit_checks(fit, cfg.limits) if name in ACCURACY_LIMITS and not ok]


@pytest.fixture(scope="module")
def session() -> tuple[dict, list[ArmBoardView]]:
    return load_session()


@pytest.fixture(scope="module")
def pan_axis_fit(session: tuple[dict, list[ArmBoardView]]) -> HeadArmFit:
    doc, views = session
    return run_fit(doc, views, node_fit_config(doc))


def test_fixed_pan_axis_reproduces_live_failure(session: tuple[dict, list[ArmBoardView]]) -> None:
    doc, views = session
    cfg = node_fit_config(doc, fit_pan_axis=False)
    result = run_fit(doc, views, cfg)
    assert result.rms_px == pytest.approx(LIVE_RMS_PX, abs=0.1)
    assert min(result.offset_rms_px[1], result.offset_rms_px[2]) > result.offset_rms_px[0] + 2.0
    assert "rms_px" in failed_accuracy(result, cfg)


def test_pan_axis_fit_meets_accuracy_limits(pan_axis_fit: HeadArmFit, session: tuple[dict, list[ArmBoardView]]) -> None:
    doc, _ = session
    cfg = node_fit_config(doc)
    assert failed_accuracy(pan_axis_fit, cfg) == []
    assert pan_axis_fit.params["pan_axis_x_m"] * 1000.0 == pytest.approx(PAN_AXIS_MM[0], abs=PAN_AXIS_TOLERANCE_MM)
    assert pan_axis_fit.params["pan_axis_y_m"] * 1000.0 == pytest.approx(PAN_AXIS_MM[1], abs=PAN_AXIS_TOLERANCE_MM)
    # Every head offset fits alike: the pan offsets no longer carry a misfit.
    by_offset = pan_axis_fit.offset_rms_px
    assert max(by_offset.values()) < 1.2 * min(by_offset.values())
    assert max(by_offset.values()) < cfg.limits.rms_px


def test_real_session_outcome(pan_axis_fit: HeadArmFit) -> None:
    """With the evidence-based limits only the floor cross-check of the pan axis direction fails."""
    assert pan_axis_fit.failed_limits == FLOOR_DISAGREEMENT
    assert abs(pan_axis_fit.floor_delta["mount_pitch_deg"] + pan_axis_fit.floor_delta["tilt_zero_deg"]) < 0.1


def test_floor_values_do_not_depend_on_pan_axis(pan_axis_fit: HeadArmFit, session: tuple[dict, list]) -> None:
    """The floor cross-check stays a like-for-like comparison: refitting the floor sweep with the arm-fitted pan axis
    changes its values by less than 0.1 deg and 0.5 mm."""
    doc, _ = session
    floor_doc, observations = floor_observations()
    tof = floor_doc["tof_joint"]
    base = rewrite_joint_origin(applied_urdf(doc), TOF_JOINT, Origin(tuple(tof["xyz"]), tuple(tof["rpy"])), "tof")
    # The arm-fitted pan axis (head_pan origin, head_tilt moved back) with the applied (floor) head_tilt rotation.
    tilt = Origin(pan_axis_fit.origins["head_tilt"].xyz, tuple(doc["head_chain"]["head_tilt"]["rpy"]))
    moved = rewrite_joint_origin(base, "head_pan", pan_axis_fit.origins["head_pan"], "pan axis")
    moved = rewrite_joint_origin(moved, "head_tilt", tilt, "pan axis")
    cfg = FloorFitConfig()
    fixed = fit_head_chain_floor(observations, base, cfg)
    free = fit_head_chain_floor(observations, moved, cfg)
    assert fixed.passed and free.passed
    for name in FLOOR_PARAMS:
        tolerance = FLOOR_PAN_AXIS_TOLERANCE_M if name.endswith("_m") else FLOOR_PAN_AXIS_TOLERANCE_DEG
        assert fixed.params[name]["value"] == pytest.approx(doc["floor"][name], abs=tolerance), name
        assert free.params[name]["value"] == pytest.approx(fixed.params[name]["value"], abs=tolerance), name


def test_leave_poses_out_within_limits(session: tuple[dict, list[ArmBoardView]]) -> None:
    """Five folds: each holds out five of the 29 arm poses (every fifth from a different start); all meet the limits."""
    doc, views = session
    cfg = node_fit_config(doc)
    poses = sorted({v.arm_pose for v in views})
    every = cfg.limits.holdout_every
    assert every == FOLDS
    for fold in range(FOLDS):
        # Renumber the arm poses so that the fit's every-fifth hold-out picks fold `fold`.
        shift = (every - 1 - fold) % every
        renumbered = [replace(v, arm_pose=poses[(poses.index(v.arm_pose) + shift) % len(poses)]) for v in views]
        result = run_fit(doc, renumbered, cfg)
        assert failed_accuracy(result, cfg) == [], (fold, failed_accuracy(result, cfg))
        assert math.isfinite(result.holdout_board_mm)
