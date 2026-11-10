"""Tests of the ground board fit (shared/ros2_common/head_rgbd_calibration/ground_board_fit.py) on synthetic scenes:
the FULL board lying face up on the floor at up to three placements, seen through a truth head chain that differs
from the applied one (tests/ground_board_synthetic.py)."""

import dataclasses
import json
import math

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d
from ros2_common.head_rgbd_calibration.config import (
    FULL_BOARD,
    GroundBoardBounds,
    GroundBoardFitConfig,
    GroundBoardLimits,
)
from ros2_common.head_rgbd_calibration.ground_board_fit import (
    ACCURACY_LABELS,
    GAUGE_PARAMS,
    P95_1D_FACTOR,
    GroundBoardFit,
    assess,
    board_on_floor,
    fit_ground_board,
    free_head_names,
    ground_board_report,
    locate_board,
)
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    CAMERA_MOUNT_PARAMS,
    PAN_AXIS_PARAMS,
    camera_pose,
    head_values,
    param_scale,
)

from tests.ground_board_synthetic import (
    CAMERA_FRAME,
    DEPTH_FRAME,
    GROUND_CAMERA_MATRIX,
    GROUND_DIST,
    GROUND_PLACEMENTS,
    GROUND_TRUTH_DELTAS,
    ground_scene,
)

LENGTH_TOLERANCE_M = 0.001
ANGLE_TOLERANCE_DEG = 0.1
PLACEMENT_TOLERANCE_M = 0.002
PLACEMENT_TOLERANCE_DEG = 0.2
NOISE_PX = 0.3
REPORT_KEYS = {
    "plausible", "refusals", "warnings", "score", "params", "placements", "accuracy_examples", "comparison",
    "cross_check", "poses", "notes",
}  # fmt: skip
GAUGE_SHIFT = {"mount_x_m": 0.012, "mount_y_m": -0.009, "mount_yaw_deg": 1.5}


def config(**overrides: object) -> GroundBoardFitConfig:
    """The default fit config with overrides."""
    return GroundBoardFitConfig(**overrides)


def truth_values(scene: object) -> dict[str, float]:
    """Truth head parameter values (m, deg) of a scene."""
    chain = UrdfChain.from_text(scene.applied_urdf)
    truth = UrdfChain(
        [
            dataclasses.replace(j, xyz=scene.truth[j.name].xyz, rpy=scene.truth[j.name].rpy)
            if j.name in scene.truth
            else j
            for j in chain.joints.values()
        ]
    )
    return {name: value * param_scale(name) for name, value in head_values(truth).items()}


def run_fit(scene: object, cfg: GroundBoardFitConfig | None = None) -> GroundBoardFit:
    """Fit a scene with the synthetic intrinsics."""
    return fit_ground_board(
        scene.views,
        scene.planes,
        GROUND_CAMERA_MATRIX,
        GROUND_DIST,
        scene.applied_urdf,
        cfg or config(),
        CAMERA_FRAME,
        DEPTH_FRAME,
    )


@pytest.fixture(scope="module")
def scene() -> object:
    return ground_scene(noise_px=NOISE_PX)


@pytest.fixture(scope="module")
def fit(scene: object) -> GroundBoardFit:
    return run_fit(scene)


def test_board_on_floor_puts_the_pattern_face_up_at_the_board_thickness() -> None:
    pose = board_on_floor(FULL_BOARD, 0.5, -0.2, 0.4)
    corners = (pose[:3, :3] @ board_corners_3d(FULL_BOARD).T).T + pose[:3, 3]
    assert np.allclose(corners[:, 2], FULL_BOARD.thickness_mm / 1000.0)
    assert np.allclose(corners.mean(axis=0)[:2], [0.5, -0.2])
    assert np.allclose(pose[:3, 2], [0.0, 0.0, -1.0])  # board z (into the board) points down


def test_locate_board_on_the_truth_chain_recovers_the_placement(scene: object) -> None:
    view = scene.views[0]
    camera = camera_pose(
        UrdfChain.from_text(scene.applied_urdf),
        {"head_pan": view.pan_rad, "head_tilt": view.tilt_rad},
        scene.truth,
        CAMERA_FRAME,
    )
    x, y, yaw = locate_board(camera, view.detection, GROUND_CAMERA_MATRIX, GROUND_DIST, FULL_BOARD)
    tx, ty, tyaw = GROUND_PLACEMENTS[view.placement]
    assert math.hypot(x - tx, y - ty) < PLACEMENT_TOLERANCE_M
    assert abs(math.degrees(yaw - tyaw)) < 0.5


def test_locate_board_gives_none_when_the_rays_miss_the_floor(scene: object) -> None:
    view = scene.views[0]
    upward = np.eye(4)
    # identity rotation: the optical z axis points up, every ray climbs away from the floor
    upward[:3, 3] = [0.0, 0.0, 0.3]
    assert locate_board(upward, view.detection, GROUND_CAMERA_MATRIX, GROUND_DIST, FULL_BOARD) is None


def test_free_head_names_follow_the_switches() -> None:
    assert free_head_names(config()) == (
        "mount_z_m", "mount_roll_deg", "mount_pitch_deg", "tilt_zero_deg", "tilt_roll_deg",
        *CAMERA_MOUNT_PARAMS, *PAN_AXIS_PARAMS,
    )  # fmt: skip
    assert free_head_names(config(fit_camera_mount=False, fit_pan_axis=False)) == (
        "mount_z_m", "mount_roll_deg", "mount_pitch_deg", "tilt_zero_deg", "tilt_roll_deg",
    )  # fmt: skip
    assert not set(GAUGE_PARAMS) & set(free_head_names(config()))


def test_fit_recovers_the_truth_chain(scene: object, fit: GroundBoardFit) -> None:
    truth = truth_values(scene)
    for name in free_head_names(config()):
        tolerance = LENGTH_TOLERANCE_M if name.endswith("_m") else ANGLE_TOLERANCE_DEG
        assert abs(fit.params[name] - truth[name]) < tolerance, name
    assert fit.rms_px < 2.0 * NOISE_PX
    assert fit.holdout_rms_px < 2.0 * NOISE_PX


def test_fit_recovers_the_board_placements(fit: GroundBoardFit) -> None:
    for entry, (x, y, yaw) in zip(fit.placements, GROUND_PLACEMENTS, strict=True):
        assert math.hypot(entry["x_m"] - x, entry["y_m"] - y) < PLACEMENT_TOLERANCE_M
        assert abs(entry["yaw_deg"] - math.degrees(yaw)) < PLACEMENT_TOLERANCE_DEG


def test_fit_keeps_the_gauge_at_the_applied_values(scene: object, fit: GroundBoardFit) -> None:
    applied = {
        name: value * param_scale(name) for name, value in head_values(UrdfChain.from_text(scene.applied_urdf)).items()
    }
    for name in GAUGE_PARAMS:
        assert fit.params[name] == pytest.approx(applied[name], abs=1e-12)
        assert name not in fit.sigma


def test_gauge_shift_is_absorbed_by_the_placements() -> None:
    # The truth head moved in the floor plane and turned about vertical: the free board cannot tell, so every other
    # parameter still recovers, the residual stays at the noise level and the board placements take the shift.
    shifted = ground_scene(deltas={**GROUND_TRUTH_DELTAS, **GAUGE_SHIFT}, noise_px=NOISE_PX)
    result = run_fit(shifted)
    truth = truth_values(shifted)
    for name in free_head_names(config()):
        tolerance = LENGTH_TOLERANCE_M if name.endswith("_m") else ANGLE_TOLERANCE_DEG
        assert abs(result.params[name] - truth[name]) < tolerance, name
    assert result.rms_px < 2.0 * NOISE_PX
    turns = [
        entry["yaw_deg"] - math.degrees(yaw)
        for entry, (_, _, yaw) in zip(result.placements, GROUND_PLACEMENTS, strict=True)
    ]
    assert np.allclose(turns, -GAUGE_SHIFT["mount_yaw_deg"], atol=PLACEMENT_TOLERANCE_DEG)


def test_sigma_and_correlation_are_reported(fit: GroundBoardFit) -> None:
    for name in free_head_names(config()):
        assert 0.0 < fit.sigma[name] < (0.002 if name.endswith("_m") else 0.2), name
    assert abs(fit.max_correlation.value) < 0.99
    assert math.isfinite(fit.condition_number)


def test_holdout_takes_every_fifth_pose(fit: GroundBoardFit) -> None:
    held = [p for p in fit.poses if p["held_out"]]
    assert len(held) == fit.pose_count // 5
    assert fit.holdout_views == len(held)
    assert fit.views + fit.holdout_views == sum(p["views"] for p in fit.poses)


def test_applied_chain_scores_worse_than_the_fit(fit: GroundBoardFit) -> None:
    assert fit.current.rms_px > 3.0 * fit.rms_px


def test_depth_cross_check_agrees_with_the_fitted_chain(fit: GroundBoardFit) -> None:
    check = fit.cross_check
    assert check["planes"] == fit.pose_count
    assert check["fitted"]["height_rms_mm"] < 3.0
    assert check["fitted"]["normal_rms_deg"] < 0.3
    # the scene's height changes nearly cancel (mount z +5 mm, camera z -3 mm), its 0.4-0.7 deg turns do not
    assert check["applied"]["normal_rms_deg"] > 2.0 * check["fitted"]["normal_rms_deg"]


def test_accuracy_examples_are_plain_and_consistent(fit: GroundBoardFit) -> None:
    assert [example.label for example in fit.accuracy] == list(ACCURACY_LABELS)
    for example in fit.accuracy:
        assert 0.0 < example.sigma < math.inf
        assert example.p95 == pytest.approx(P95_1D_FACTOR * example.sigma)
        assert math.isfinite(example.shift)
    heights = {example.label: example for example in fit.accuracy if example.unit == "mm"}
    near, far = heights[ACCURACY_LABELS[0]], heights[ACCURACY_LABELS[1]]
    assert far.sigma > near.sigma  # an angle error grows with distance


def test_report_structure_and_json(fit: GroundBoardFit) -> None:
    report = ground_board_report(fit, config(), None)
    assert set(report) == REPORT_KEYS
    assert report["plausible"] is True
    assert report["refusals"] == []
    assert len(report["accuracy_examples"]) == 2 * len(ACCURACY_LABELS)
    names = [entry["name"] for entry in report["params"]]
    assert set(GAUGE_PARAMS) <= set(names)
    assert all(entry["sigma"] is None for entry in report["params"] if entry["name"] in GAUGE_PARAMS)
    json.dumps(report, allow_nan=False)


def test_assess_refuses_implausible_changes(fit: GroundBoardFit) -> None:
    params = {**fit.params}
    params["mount_z_m"] = fit.start_params["mount_z_m"] + 0.2
    params["tilt_zero_deg"] = fit.start_params["tilt_zero_deg"] - 15.0
    params["mount_roll_deg"] = fit.start_params["mount_roll_deg"] + 12.0
    params["camera_x_m"] = fit.start_params["camera_x_m"] + 0.05
    params["pan_axis_x_m"] = fit.start_params["pan_axis_x_m"] + 0.08
    implausible = dataclasses.replace(fit, params=params)
    refusals, _ = assess(implausible, config())
    names = {entry["name"] for entry in refusals}
    assert {"mount_move_mm", "tilt_change_deg", "mount_turn_deg", "camera_move_mm", "pan_axis_move_mm"} <= names
    for entry in refusals:
        assert set(entry) == {"name", "value", "limit", "unit", "message"}


def test_assess_refuses_non_finite_and_far_boards(fit: GroundBoardFit) -> None:
    far = [{**fit.placements[0], "x_m": 5.0}, *fit.placements[1:]]
    refusals, _ = assess(dataclasses.replace(fit, placements=far, rms_px=math.nan), config())
    names = {entry["name"] for entry in refusals}
    assert "non_finite" in names
    assert "board_distance_m:0" in names


def test_assess_warns_on_advisory_limits(fit: GroundBoardFit) -> None:
    strict = config(
        limits=GroundBoardLimits(rms_px=0.01, min_placements=4, min_views=1000, max_floor_height_rms_mm=0.01),
        bounds=GroundBoardBounds(),
    )
    _, warnings = assess(fit, strict)
    names = {entry["name"] for entry in warnings}
    assert {"rms_px", "min_placements", "min_views", "floor_height_rms_mm"} <= names


def test_single_placement_fit_warns_but_runs() -> None:
    single = ground_scene(placements=GROUND_PLACEMENTS[:1], noise_px=NOISE_PX)
    result = run_fit(single)
    _, warnings = assess(result, config())
    assert "min_placements" in {entry["name"] for entry in warnings}
    assert len(result.placements) == 1


def test_fit_without_depth_planes_warns(scene: object) -> None:
    result = fit_ground_board(
        scene.views[:40], [], GROUND_CAMERA_MATRIX, GROUND_DIST, scene.applied_urdf, config(), CAMERA_FRAME, DEPTH_FRAME
    )
    assert result.cross_check["planes"] == 0
    _, warnings = assess(result, config())
    assert "no_depth_planes" in {entry["name"] for entry in warnings}


def test_fit_needs_views(scene: object) -> None:
    with pytest.raises(ValueError, match="at least"):
        run_fit(dataclasses.replace(scene, views=scene.views[:2]))


def test_assess_refuses_unknown_accuracy(fit: GroundBoardFit) -> None:
    sigma = {**fit.sigma, "tilt_zero_deg": math.inf}
    refusals, _ = assess(dataclasses.replace(fit, sigma=sigma), config())
    assert "accuracy_unknown" in {entry["name"] for entry in refusals}
    assert "accuracy_unknown" not in {entry["name"] for entry in assess(fit, config())[0]}


def test_holdout_keeps_a_fit_pose_for_every_placement(scene: object) -> None:
    # placement 1 keeps only its fifth pose (a held-out index): it must stay in the fit, so the placement is observed
    poses = sorted({v.pose for v in scene.views})
    held_pose = poses[4]
    placement = next(v.placement for v in scene.views if v.pose == held_pose)
    views = [v for v in scene.views if v.placement != placement or v.pose == held_pose]
    result = run_fit(dataclasses.replace(scene, views=views))
    entry = next(p for p in result.poses if p["pose"] == held_pose)
    assert entry["held_out"] is False
    assert all(math.isfinite(result.sigma[name]) for name in result.free)
