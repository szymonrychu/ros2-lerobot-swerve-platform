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
from ros2_common.head_rgbd_calibration.urdf_text import Origin, rewrite_joint_origin
from ros2_common.head_rgbd_calibration.views import ArmBoardView

REPO_ROOT = Path(__file__).resolve().parent.parent
FIXTURE = REPO_ROOT / "tests" / "fixtures" / "head_arm" / "calib-20261023-074418.json"
FLOOR_FIXTURE = REPO_ROOT / "tests" / "fixtures" / "head_arm" / "floor-20261022-064337.json"
FLOOR_PAN_AXIS_TOLERANCE_DEG = 0.1
FLOOR_PAN_AXIS_TOLERANCE_M = 0.0005
# Without the floor prior the floor fit and the head-from-arm fit disagree on the pan axis direction (mount roll and
# pitch, with the tilt zero keeping the camera pitch at pan 0): these floor cross-checks fail on this session.
PRIOR_RMS_PX = 2.93
CAMERA_Z_SIGMA_MM = 1.0
FLOOR_DISAGREEMENT = ["floor_delta:mount_roll_deg", "floor_delta:mount_pitch_deg", "floor_delta:tilt_zero_deg"]
URDF_DIR = REPO_ROOT / "nodes" / "web_ui" / "urdf"
# head.urdf as it was when the session was recorded (with the retired head camera and ToF frames, removed from
# nodes/web_ui/urdf/head.urdf by the D435 switchover).
RECORDED_URDF = REPO_ROOT / "tests" / "fixtures" / "head_arm" / "head_tof_rgb.urdf"
TOF_JOINT = "tof_joint"
# The session was recorded with the retired head RGB camera and the floor sweep with the retired ToF.
SESSION_CAMERA_FRAME = "head_camera_optical_frame"
SESSION_DEPTH_FRAME = "tof_optical_frame"
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
    """The recorded head URDF with the session's applied head_chain.json joints."""
    text = RECORDED_URDF.read_text()
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
    """The node fit config of the session (board mount start); limits and the rest at the defaults (= node config),
    except the camera mount: the evidence below was taken with the head camera mount fixed (the model of 2026-10-23,
    before `fit_camera_mount`)."""
    mount = doc["board_mount"]
    board_mount = BoardMountConfig(xyz_init=tuple(mount["xyz_init"]), rpy_init=tuple(mount["rpy_init"]))
    fields: dict[str, object] = {"fit_camera_mount": False, **overrides}
    return HeadArmFitConfig(board_mount=board_mount, **fields)


def run_fit(doc: dict, views: list[ArmBoardView], cfg: HeadArmFitConfig) -> HeadArmFit:
    return fit_head_from_arm(
        views,
        np.asarray(doc["camera_matrix"], dtype=float).reshape(3, 3),
        np.asarray(doc["dist"], dtype=float),
        applied_urdf(doc),
        (URDF_DIR / "so101_arm.urdf").read_text(),
        cfg,
        doc["floor"],
        camera_frame=SESSION_CAMERA_FRAME,
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
    # The pan offsets no longer carry the misfit (13 px with the fixed axis); the floor prior costs the +10 deg offset
    # about 0.5 px (3.33 px against 2.64 px at the aimed view).
    by_offset = pan_axis_fit.offset_rms_px
    assert max(by_offset.values()) < 1.3 * min(by_offset.values())
    assert max(by_offset.values()) < 3.5


def test_real_session_outcome(pan_axis_fit: HeadArmFit) -> None:
    """With the floor prior (weight 10, user decision 2026-10-23) the session passes every limit."""
    assert pan_axis_fit.failed_limits == []
    assert pan_axis_fit.passed
    assert pan_axis_fit.rms_px == pytest.approx(PRIOR_RMS_PX, abs=0.05)


def test_without_floor_prior_only_the_floor_cross_check_fails(session: tuple[dict, list[ArmBoardView]]) -> None:
    """Prior off (the D49 default before 2026-10-23): only the floor cross-check of the pan axis direction fails."""
    doc, views = session
    result = run_fit(doc, views, node_fit_config(doc, floor_prior_weight=0.0))
    assert result.failed_limits == FLOOR_DISAGREEMENT
    assert max(result.offset_rms_px.values()) < 1.2 * min(result.offset_rms_px.values())
    assert abs(result.floor_delta["mount_pitch_deg"] + result.floor_delta["tilt_zero_deg"]) < 0.1


def test_free_camera_mount_meets_every_limit(session: tuple[dict, list[ArmBoardView]]) -> None:
    """The camera mount free (x, z, roll, yaw of the camera joint, here the retired head_camera_joint): the session's
    29 arm poses keep every sigma inside the limits (camera z about 0.8 mm against 2 mm) and the fit passes."""
    doc, views = session
    cfg = node_fit_config(doc, fit_camera_mount=True)
    result = run_fit(doc, views, cfg)
    assert result.failed_limits == []
    assert result.sigma["camera_z_m"] * 1000.0 < CAMERA_Z_SIGMA_MM
    assert result.rms_px < PRIOR_RMS_PX


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
    fixed = fit_head_chain_floor(observations, base, cfg, sensor_frame=SESSION_DEPTH_FRAME)
    free = fit_head_chain_floor(observations, moved, cfg, sensor_frame=SESSION_DEPTH_FRAME)
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
