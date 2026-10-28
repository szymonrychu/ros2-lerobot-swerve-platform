"""Tests for marker_detector.pose on synthetic scenes; ground truth is always the renderer's pose."""

import cv2
import numpy as np
import pytest
from ros2_common.marker_payload import decode_payload

import marker_detector.pose as pose_module
from marker_detector.config import PoseConfig
from marker_detector.decode import Candidate, decode_frame
from marker_detector.pose import (
    PoseResult,
    Solution,
    estimate_pose,
    finder_model_points,
    pose_confidence,
    predict_image_points,
    quad_is_valid,
    refine_corners,
    solve_robust,
    symbol_modules,
)
from marker_detector.synthetic import (
    STREAM_K,
    STREAM_SIZE,
    pose_facing,
    project_points,
    render_marker_scene,
    symbol_corners_m,
    symbol_matrix,
)

PAYLOAD = "P1/K:BASE/N:HOME/S:150"
SMALL_PAYLOAD = "P1/K:BASE/N:HOME/S:100"
SMALL_SIZE_MM = 100.0
SIZE_MM = 150.0
NO_DISTORTION = np.zeros(5)
SEEDS = range(5)
CORNER_NOISE_PX = 0.3
IMAGE_NOISE_GREY = 2.0
LOW_MODULE_CFG = PoseConfig(min_module_px=1.0)
PLANE_MAX_ANGLE_DEG = 20.0


def version_of(payload: str) -> int:
    return (symbol_matrix(payload).shape[0] - 17) // 4


def render(payload: str, size_mm: float, pose, seed: int = 0, noise_grey: float = IMAGE_NOISE_GREY) -> np.ndarray:
    return render_marker_scene([payload], [size_mm], [pose], STREAM_K, STREAM_SIZE, noise_px=noise_grey, seed=seed)


def truth_candidate(payload: str, size_mm: float, pose, corner_noise_px: float = 0.0, seed: int = 0) -> Candidate:
    """Candidate from the renderer's projected symbol corners plus Gaussian corner noise (stands in for the decoder)."""
    corners = project_points(symbol_corners_m(size_mm), pose, STREAM_K)
    corners = corners + np.random.default_rng(seed + 1000).normal(0.0, corner_noise_px, corners.shape)
    version = version_of(payload)
    modules = 17 + 4 * version
    edges = np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)
    return Candidate(decode_payload(payload), payload, corners, version, float(edges.mean() / modules))


def normal_of(rvec) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
    return rotation[:, 2]


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    cosine = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


class TruthPlaneChooser:
    """Test double with the ``choose`` contract of the depth ``plane_chooser``, using the renderer's true marker normal as
    the plane normal; returns the solution within PLANE_MAX_ANGLE_DEG of it when exactly one is, else None. Records each call."""

    def __init__(self, true_normal: np.ndarray) -> None:
        self.true_normal = true_normal
        self.calls: list[list[Solution]] = []

    def __call__(self, solutions: list[Solution]) -> Solution | None:
        self.calls.append(solutions)
        near = [s for s in solutions if angle_deg(s.normal, self.true_normal) <= PLANE_MAX_ANGLE_DEG]
        return near[0] if len(near) == 1 else None


# --- finder_model_points -------------------------------------------------------------------------------------------


@pytest.mark.parametrize("version", [1, 2, 5])
def test_finder_model_points_are_24_distinct_coplanar_points(version):
    points = finder_model_points(SIZE_MM, version)
    assert points.shape == (24, 3)
    assert np.all(points[:, 2] == 0.0)
    assert len({tuple(np.round(p, 9)) for p in points}) == 24


@pytest.mark.parametrize("version", [1, 2, 5])
def test_finder_model_outer_corners_at_half_size(version):
    half = SIZE_MM / 2000.0
    points = finder_model_points(SIZE_MM, version)
    module = SIZE_MM / 1000.0 / (17 + 4 * version)
    assert np.abs(points[:, :2]).max() == pytest.approx(half)
    # documented order: per finder (top-left, top-right, bottom-left) outer TL, TR, BR, BL then inner TL, TR, BR, BL
    np.testing.assert_allclose(points[0], [-half, half, 0.0])  # top-left finder, outer top-left = symbol top-left
    np.testing.assert_allclose(points[9], [half, half, 0.0])  # top-right finder, outer top-right = symbol top-right
    np.testing.assert_allclose(points[19], [-half, -half, 0.0])  # bottom-left finder, outer bottom-left
    np.testing.assert_allclose(points[2], [-half + 7 * module, half - 7 * module, 0.0])  # TL outer bottom-right
    np.testing.assert_allclose(points[4], [-half + 2 * module, half - 2 * module, 0.0])  # TL inner top-left
    np.testing.assert_allclose(points[6], [-half + 5 * module, half - 5 * module, 0.0])  # TL inner bottom-right


def test_finder_model_points_scale_with_size():
    np.testing.assert_allclose(finder_model_points(75.0, 2) * 2.0, finder_model_points(150.0, 2))


# --- predict_image_points / refine_corners -------------------------------------------------------------------------


@pytest.mark.parametrize("yaw", [0, 40])
def test_predicted_points_match_projected_finder_corners(yaw):
    pose = pose_facing(1.0, yaw_deg=yaw, roll_deg=30)
    version = version_of(PAYLOAD)
    corners = project_points(symbol_corners_m(SIZE_MM), pose, STREAM_K)
    expected = project_points(finder_model_points(SIZE_MM, version), pose, STREAM_K)
    predicted = predict_image_points(corners, version)
    assert predicted.shape == (24, 2)
    assert np.abs(predicted - expected).max() < 1e-3  # float32 homography


def test_refine_corners_pulls_noisy_points_onto_true_corners():
    pose = pose_facing(0.8, yaw_deg=20)
    version = version_of(PAYLOAD)
    grey = render(PAYLOAD, SIZE_MM, pose)
    expected = project_points(finder_model_points(SIZE_MM, version), pose, STREAM_K)
    noisy = expected + np.random.default_rng(3).normal(0.0, 0.7, expected.shape)
    module_px = truth_candidate(PAYLOAD, SIZE_MM, pose).module_px
    refined = refine_corners(grey, noisy, module_px)
    assert refined.shape == (24, 2)
    # The renderer samples about 0.4 px up-left of the pinhole projection (pixel-centre convention of its 3x
    # supersampling); a common shift is a translation, so it is removed before comparing corner scatter.
    offset = np.median(refined - expected, axis=0)
    assert np.abs(offset).max() < 0.6
    assert np.sqrt(np.mean(np.sum((refined - expected - offset) ** 2, axis=1))) < 0.25
    assert np.sqrt(np.mean(np.sum((noisy - expected) ** 2, axis=1))) > 0.6


# --- accuracy -------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("distance", [0.5, 1.0, 2.0])
@pytest.mark.parametrize("yaw", [0, 30, 50])
def test_accuracy_within_one_percent_and_two_degrees(distance, yaw):
    """150 mm marker, STREAM_K, 0.3 px corner noise (plus image noise sigma 2 grey levels): position error < 1 % of
    distance and normal error < 2 deg over 5 seeds.

    At 2 m the module is ~2.5 px (below the default 4 px gate, so min_module_px is lowered here) and the view is
    nearly weak-perspective: the planar flip reprojects within ~0.5 px, so the RMS ratio can fall just under 2 and
    the ``choose`` callback decides (design D10b), here TruthPlaneChooser with the true normal. Up to 1 m the RMS ratio or
    the frontal agreement alone must accept every pose, without consulting the chooser.
    """
    pose = pose_facing(distance, yaw_deg=yaw)
    true_normal = normal_of(pose[0])
    for seed in SEEDS:
        grey = render(PAYLOAD, SIZE_MM, pose, seed=seed)
        candidate = truth_candidate(PAYLOAD, SIZE_MM, pose, CORNER_NOISE_PX, seed)
        chooser = TruthPlaneChooser(true_normal)
        result = estimate_pose(grey, candidate, STREAM_K, NO_DISTORTION, LOW_MODULE_CFG, chooser)
        assert result.ok, (seed, result.reason, result.ratio, result.rms_px)
        if distance <= 1.0:
            assert chooser.calls == [], seed
        assert np.linalg.norm(result.tvec - pose[1]) < 0.01 * distance, (seed, result.tvec)
        assert angle_deg(normal_of(result.rvec), true_normal) < 2.0, seed
        assert 0.0 < result.confidence <= 1.0


def test_frontal_view_accepted_because_both_solutions_agree():
    """Frontal: the two IPPE solutions coincide (RMS ratio ~1), which is not an ambiguity."""
    pose = pose_facing(1.0)
    result = estimate_pose(
        render(PAYLOAD, SIZE_MM, pose), truth_candidate(PAYLOAD, SIZE_MM, pose), STREAM_K, None, PoseConfig()
    )
    assert result.ok, result.reason
    assert result.ratio < PoseConfig().ambiguity_ratio
    assert result.separation_deg < 1.0
    assert angle_deg(normal_of(result.rvec), normal_of(pose[0])) < 2.0


def test_accuracy_end_to_end_from_decoder():
    pose = pose_facing(1.0, yaw_deg=30, offset_m=(0.1, -0.05))
    grey = render(PAYLOAD, SIZE_MM, pose)
    candidates, _ = decode_frame(grey)
    result = estimate_pose(grey, candidates[0], STREAM_K, NO_DISTORTION, PoseConfig())
    assert result.ok, result.reason
    assert np.linalg.norm(result.tvec - pose[1]) < 0.01
    assert angle_deg(normal_of(result.rvec), normal_of(pose[0])) < 2.0
    assert result.inliers >= 12


# --- size scaling and roll ------------------------------------------------------------------------------------------


def test_declared_size_drives_scale():
    pose = pose_facing(1.0, yaw_deg=20)
    grey = render(PAYLOAD, SIZE_MM, pose)
    candidates, _ = decode_frame(grey)
    full = candidates[0]
    half_payload = full.payload.model_copy(update={"size_mm": 75.0})
    half = Candidate(half_payload, full.text, full.corners, full.version, full.module_px)
    result_full = estimate_pose(grey, full, STREAM_K, NO_DISTORTION, PoseConfig())
    result_half = estimate_pose(grey, half, STREAM_K, NO_DISTORTION, PoseConfig())
    assert result_full.ok and result_half.ok
    assert np.linalg.norm(result_half.tvec) == pytest.approx(np.linalg.norm(result_full.tvec) / 2.0, rel=1e-3)
    assert np.linalg.norm(result_full.tvec) == pytest.approx(np.linalg.norm(pose[1]), rel=0.01)
    assert angle_deg(normal_of(result_half.rvec), normal_of(result_full.rvec)) < 0.1


@pytest.mark.parametrize("roll", [0, 90, 180, 270])
def test_roll_recovered_without_quarter_turn_ambiguity(roll):
    pose = pose_facing(1.0, yaw_deg=25, roll_deg=roll)
    grey = render(PAYLOAD, SIZE_MM, pose)
    candidates, _ = decode_frame(grey)
    result = estimate_pose(grey, candidates[0], STREAM_K, NO_DISTORTION, PoseConfig())
    assert result.ok, result.reason
    estimated, _ = cv2.Rodrigues(result.rvec)
    true_rotation, _ = cv2.Rodrigues(pose[0])
    # the marker +x axis (in-plane rotation) and +z axis both match the renderer
    assert angle_deg(estimated[:, 0], true_rotation[:, 0]) < 2.0
    assert angle_deg(estimated[:, 2], true_rotation[:, 2]) < 2.0


# --- ambiguity ------------------------------------------------------------------------------------------------------


def ambiguous_scene(seed: int) -> tuple[np.ndarray, Candidate, tuple[np.ndarray, np.ndarray]]:
    """100 mm marker, nearly frontal (yaw 12 deg), 1.3 m away (~2.7 px/module), 1 px corner noise: the planar flip
    (about 25 deg away) reprojects almost as well as the true pose."""
    pose = pose_facing(1.3, yaw_deg=12.0)
    grey = render(SMALL_PAYLOAD, SMALL_SIZE_MM, pose, seed=seed)
    return grey, truth_candidate(SMALL_PAYLOAD, SMALL_SIZE_MM, pose, 1.0, seed), pose


@pytest.mark.parametrize("seed", SEEDS)
def test_ambiguous_pose_rejected_without_chooser(seed):
    grey, candidate, _ = ambiguous_scene(seed)
    result = estimate_pose(grey, candidate, STREAM_K, NO_DISTORTION, LOW_MODULE_CFG)
    assert not result.ok
    assert result.reason == "ambiguous"
    assert result.ratio < LOW_MODULE_CFG.ambiguity_ratio
    assert result.separation_deg > 10.0
    assert len(result.solutions) == 2
    assert result.rvec is None and result.tvec is None
    assert result.confidence == 0.0


@pytest.mark.parametrize("seed", SEEDS)
def test_ambiguous_pose_accepted_when_chooser_picks_true_solution(seed):
    grey, candidate, pose = ambiguous_scene(seed)
    true_normal = normal_of(pose[0])
    seen: list[list[Solution]] = []

    def choose(solutions: list[Solution]) -> Solution | None:
        seen.append(solutions)
        return min(solutions, key=lambda s: angle_deg(s.normal, true_normal))

    result = estimate_pose(grey, candidate, STREAM_K, NO_DISTORTION, LOW_MODULE_CFG, choose)
    assert result.ok, result.reason
    assert len(seen) == 1 and len(seen[0]) == 2
    truest = min(seen[0], key=lambda s: angle_deg(s.normal, true_normal))
    other = max(seen[0], key=lambda s: angle_deg(s.normal, true_normal))
    np.testing.assert_allclose(result.rvec, truest.rvec)
    np.testing.assert_allclose(result.tvec, truest.tvec)
    assert angle_deg(normal_of(result.rvec), true_normal) < angle_deg(other.normal, true_normal)
    assert result.confidence == pytest.approx(
        (1 - 0.5 * result.rms_px / LOW_MODULE_CFG.max_rms_px) * 0.5 * min(1.0, candidate.module_px / 2.0)
    )


def test_chooser_returning_none_keeps_ambiguous_rejection():
    grey, candidate, _ = ambiguous_scene(1)
    result = estimate_pose(grey, candidate, STREAM_K, NO_DISTORTION, LOW_MODULE_CFG, lambda solutions: None)
    assert not result.ok
    assert result.reason == "ambiguous"


def test_chooser_not_called_when_unambiguous():
    pose = pose_facing(0.6, yaw_deg=35)
    grey = render(PAYLOAD, SIZE_MM, pose)

    def choose(solutions: list[Solution]) -> Solution | None:
        raise AssertionError("chooser must not be consulted")

    result = estimate_pose(grey, truth_candidate(PAYLOAD, SIZE_MM, pose), STREAM_K, NO_DISTORTION, PoseConfig(), choose)
    assert result.ok
    assert result.ratio >= PoseConfig().ambiguity_ratio
    assert len(result.solutions) == 2
    assert result.solutions[0].rms_px <= result.solutions[1].rms_px
    assert result.rms_px == pytest.approx(result.solutions[0].rms_px)


# --- rejections -----------------------------------------------------------------------------------------------------


def test_small_module_rejected():
    pose = pose_facing(2.0)
    grey = render(PAYLOAD, SIZE_MM, pose)
    candidate = truth_candidate(PAYLOAD, SIZE_MM, pose)
    assert candidate.module_px < PoseConfig().min_module_px
    result = estimate_pose(grey, candidate, STREAM_K, NO_DISTORTION, PoseConfig())
    assert not result.ok
    assert result.reason == "module_too_small"
    assert result.rvec is None and result.tvec is None


def test_high_reprojection_error_rejected():
    """Corrupted corners: the decoder's quad is off by several modules from the printed symbol, so the refined finder
    corners land on unrelated structure and no planar pose explains them."""
    pose = pose_facing(0.8, yaw_deg=20)
    grey = render(PAYLOAD, SIZE_MM, pose)
    candidate = truth_candidate(PAYLOAD, SIZE_MM, pose)
    candidate.corners = candidate.corners + np.array([[0.0, 0.0], [9.0, -14.0], [-12.0, 15.0], [6.0, 3.0]])
    result = estimate_pose(grey, candidate, STREAM_K, NO_DISTORTION, PoseConfig())
    assert not result.ok
    assert result.reason == "rms"
    assert result.rms_px > PoseConfig().max_rms_px
    assert result.rvec is None


def test_marker_behind_camera_rejected(monkeypatch):
    """IPPE only returns poses in front of the camera, so the guard is exercised with a solver returning one behind."""
    pose = pose_facing(1.0, yaw_deg=20)
    behind = Solution(np.asarray(pose[0], float), np.array([0.0, 0.0, -1.0]), 0.1)
    flipped = Solution(np.array([0.3, 0.0, 0.0]), np.array([0.0, 0.0, -1.0]), 0.9)

    monkeypatch.setattr(pose_module, "refine_and_solve", lambda *_: ([behind, flipped], np.ones(24, dtype=bool)))
    candidate = truth_candidate(PAYLOAD, SIZE_MM, pose)
    result = estimate_pose(render(PAYLOAD, SIZE_MM, pose), candidate, STREAM_K, NO_DISTORTION, PoseConfig())
    assert not result.ok
    assert result.reason == "behind_camera"


@pytest.mark.parametrize("corners", [np.full((4, 2), np.nan), np.full((4, 2), 300.0)])
def test_pnp_failure_reported(corners):
    pose = pose_facing(1.0)
    candidate = truth_candidate(PAYLOAD, SIZE_MM, pose)
    candidate.corners = corners
    candidate.module_px = 10.0
    result = estimate_pose(render(PAYLOAD, SIZE_MM, pose), candidate, STREAM_K, NO_DISTORTION, PoseConfig())
    assert not result.ok
    assert result.reason == "pnp_failed"


def test_pose_result_rejection_shape():
    result = PoseResult(ok=False, reason="ambiguous")
    assert result.rvec is None and result.tvec is None
    assert result.confidence == 0.0
    assert result.solutions == []


@pytest.mark.parametrize(
    ("rms", "ratio", "separation", "module", "expected"),
    [
        (0.0, 8.0, 90.0, 8.0, 1.0),
        (1.0, 2.0, 90.0, 4.0, 0.125),
        (0.5, 1.0, 0.0, 8.0, 0.75),
        (0.5, 1.2, 30.0, 6.0, 0.75 * 0.5 * 0.75),
    ],
)
def test_pose_confidence_terms(rms, ratio, separation, module, expected):
    assert pose_confidence(rms, ratio, separation, module, PoseConfig()) == pytest.approx(expected)


# --- helpers --------------------------------------------------------------------------------------------------------


def test_symbol_modules():
    assert [symbol_modules(v) for v in (1, 2, 40)] == [21, 25, 177]


def test_quad_is_valid():
    square = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    assert quad_is_valid(square)
    assert not quad_is_valid(square[[0, 2, 1, 3]])  # bow tie
    assert not quad_is_valid(np.array([[0.0, 0.0], [10.0, 0.0], [2.0, 2.0], [0.0, 10.0]]))  # concave
    assert not quad_is_valid(np.full((4, 2), 5.0))
    assert not quad_is_valid(np.full((4, 2), np.nan))


def test_solve_robust_drops_corners_on_wrong_structure():
    pose = pose_facing(1.0, yaw_deg=30)
    model = finder_model_points(SIZE_MM, 2)
    image = project_points(model, pose, STREAM_K) + np.random.default_rng(0).normal(0.0, 0.05, (24, 2))
    image[[5, 17]] += [[3.0, -2.0], [-2.5, 2.5]]  # two corners snapped onto a neighbouring module
    solutions, keep = solve_robust(model, image, STREAM_K, None)
    assert keep.sum() == 22
    assert not keep[5] and not keep[17]
    assert solutions[0].rms_px < 0.1
    assert np.linalg.norm(solutions[0].tvec - pose[1]) < 0.002
