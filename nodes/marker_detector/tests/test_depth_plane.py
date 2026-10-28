"""Tests for marker_detector.depth_plane on synthetic D435 aligned depth planes; ground truth is always the renderer's marker
pose. The depth image is the /head_rgbd decimated D435 depth, aligned to the colour image in d435_color_optical_frame."""

import cv2
import numpy as np
import pytest
from ros2_common.camera_geometry import CameraIntrinsics
from ros2_common.marker_payload import decode_payload

from marker_detector import depth_plane
from marker_detector.config import DepthPlaneConfig, PoseConfig
from marker_detector.decode import Candidate
from marker_detector.depth_plane import (
    REASON_RESIDUAL,
    REASON_TOO_FEW_PX,
    Plane,
    fit_plane,
    fit_plane_detailed,
    nearest_depth,
    plane_chooser,
    scale_polygon,
)
from marker_detector.pose import Solution, estimate_pose
from marker_detector.synthetic import (
    STREAM_K,
    STREAM_SIZE,
    pose_facing,
    project_points,
    render_marker_scene,
    symbol_corners_m,
    symbol_matrix,
)

DEPTH_SIZE = (288, 162)
DEPTH_SCALE = DEPTH_SIZE[0] / STREAM_SIZE[0]
DEPTH_K = np.array(
    [
        [STREAM_K[0, 0] * DEPTH_SCALE, 0.0, (STREAM_K[0, 2] + 0.5) * DEPTH_SCALE - 0.5],
        [0.0, STREAM_K[1, 1] * DEPTH_SCALE, (STREAM_K[1, 2] + 0.5) * DEPTH_SCALE - 0.5],
        [0.0, 0.0, 1.0],
    ]
)
DEPTH_INTRINSICS = CameraIntrinsics.from_camera_info(
    DEPTH_SIZE[0], DEPTH_SIZE[1], DEPTH_K.ravel(), [0.0] * 5, "plumb_bob"
)
SIZE_MM = 150.0
PAYLOAD = "P1/K:BASE/N:HOME/S:100"
SMALL_SIZE_MM = 100.0
DEPTH_NOISE_MM = 2.0
SEEDS = range(5)
LOW_MODULE_CFG = PoseConfig(min_module_px=1.0)
CFG = DepthPlaneConfig()


def normal_of(rvec: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
    return rotation[:, 2]


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    cosine = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def depth_of_plane(pose: tuple[np.ndarray, np.ndarray], noise_mm: float = DEPTH_NOISE_MM, seed: int = 0) -> np.ndarray:
    """16UC1 mm depth image (288x162, d435_color_optical_frame) of the infinite plane through the marker, seen through DEPTH_K."""
    normal, point = normal_of(pose[0]), np.asarray(pose[1], dtype=float)
    u, v = np.meshgrid(np.arange(DEPTH_SIZE[0], dtype=float), np.arange(DEPTH_SIZE[1], dtype=float))
    rays = np.stack([(u - DEPTH_K[0, 2]) / DEPTH_K[0, 0], (v - DEPTH_K[1, 2]) / DEPTH_K[1, 1], np.ones_like(u)], -1)
    z_m = float(normal @ point) / (rays @ normal)
    depth = z_m * 1000.0 + np.random.default_rng(seed).normal(0.0, noise_mm, z_m.shape)
    return np.clip(np.round(depth), 0, 65535).astype(np.uint16)


def marker_polygon(pose: tuple[np.ndarray, np.ndarray], size_mm: float = SIZE_MM) -> np.ndarray:
    """(4, 2) symbol corners in the RGB stream image."""
    return project_points(symbol_corners_m(size_mm), pose, STREAM_K)


def polygon_mask(polygon_px: np.ndarray) -> np.ndarray:
    """Bool mask (162, 288) of the depth pixels inside the RGB polygon scaled to the depth image (OpenCV as the oracle)."""
    contour = scale_polygon(polygon_px, STREAM_SIZE, DEPTH_SIZE).astype(np.float32).reshape(-1, 1, 2)
    return np.array([[cv2.pointPolygonTest(contour, (u, v), False) >= 0 for u in range(288)] for v in range(162)])


# --- fit_plane ------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("seed", SEEDS)
def test_tilted_plane_normal_within_one_degree(seed):
    pose = pose_facing(1.0, yaw_deg=35.0, offset_m=(0.1, -0.05))
    plane = fit_plane(depth_of_plane(pose, seed=seed), DEPTH_INTRINSICS, marker_polygon(pose), STREAM_SIZE, CFG)
    assert plane is not None
    true_normal = normal_of(pose[0])
    assert angle_deg(plane.normal, true_normal) < 1.0
    assert np.linalg.norm(plane.normal) == pytest.approx(1.0)
    assert plane.normal @ np.asarray(pose[1]) < 0.0  # oriented towards the camera, like the marker +z
    assert plane.distance_mm == pytest.approx(-1000.0 * float(true_normal @ pose[1]), abs=5.0)
    assert plane.residual_mm < 1.5 * DEPTH_NOISE_MM
    assert plane.n_px >= CFG.min_valid_px


def test_tilted_plane_pitch_and_both_yaw_signs():
    for yaw, pitch in [(-35.0, 0.0), (0.0, 35.0), (20.0, -25.0)]:
        pose = pose_facing(0.8, yaw_deg=yaw, pitch_deg=pitch)
        plane = fit_plane(depth_of_plane(pose), DEPTH_INTRINSICS, marker_polygon(pose), STREAM_SIZE, CFG)
        assert plane is not None
        assert angle_deg(plane.normal, normal_of(pose[0])) < 1.0, (yaw, pitch)


def test_polygon_scaled_to_depth_image():
    polygon = np.array([[0.0, 0.0], [1151.0, 0.0], [1151.0, 647.0], [0.0, 647.0]])
    scaled = scale_polygon(polygon, STREAM_SIZE, DEPTH_SIZE)
    np.testing.assert_allclose(scaled, [[-0.375, -0.375], [287.375, -0.375], [287.375, 161.375], [-0.375, 161.375]])


def test_zeros_inside_polygon_ignored():
    pose = pose_facing(1.0, yaw_deg=35.0)
    polygon = marker_polygon(pose)
    depth = depth_of_plane(pose)
    inside = polygon_mask(polygon)
    full = fit_plane(depth, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG)
    holes = inside.copy()
    holes[:, ::2] = False  # zero every other column inside the polygon
    holed = depth.copy()
    holed[inside & ~holes] = 0
    plane = fit_plane(holed, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG)
    assert full is not None and plane is not None
    assert full.n_px == int(inside.sum())
    assert plane.n_px == int(holes.sum())
    assert angle_deg(plane.normal, normal_of(pose[0])) < 1.0
    assert plane.residual_mm < 1.5 * DEPTH_NOISE_MM  # a zero read as a 0 mm point would blow the residual


def test_too_few_valid_pixels_return_none():
    pose = pose_facing(1.0, yaw_deg=35.0)
    polygon = marker_polygon(pose)
    depth = depth_of_plane(pose)
    inside = np.argwhere(polygon_mask(polygon))
    sparse = np.zeros_like(depth)
    keep = inside[: CFG.min_valid_px - 1]
    sparse[keep[:, 0], keep[:, 1]] = depth[keep[:, 0], keep[:, 1]]
    assert fit_plane(sparse, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG) is None
    fit = fit_plane_detailed(sparse, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG)
    assert fit.plane is None and fit.reason == REASON_TOO_FEW_PX and fit.n_px == CFG.min_valid_px - 1
    exactly = sparse.copy()
    extra = inside[CFG.min_valid_px - 1]
    exactly[extra[0], extra[1]] = depth[extra[0], extra[1]]
    assert fit_plane(exactly, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG) is not None


def test_polygon_outside_depth_image_returns_none():
    polygon = np.array([[2000.0, 2000.0], [2100.0, 2000.0], [2100.0, 2100.0], [2000.0, 2100.0]])
    depth = np.full((DEPTH_SIZE[1], DEPTH_SIZE[0]), 1000, dtype=np.uint16)
    fit = fit_plane_detailed(depth, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG)
    assert fit.plane is None and fit.reason == REASON_TOO_FEW_PX and fit.n_px == 0


def test_step_inside_polygon_returns_none():
    pose = pose_facing(1.0, yaw_deg=35.0)
    polygon = marker_polygon(pose)
    depth = depth_of_plane(pose)
    centre_u = int(round(float(scale_polygon(polygon, STREAM_SIZE, DEPTH_SIZE)[:, 0].mean())))
    depth[:, centre_u:] += 150  # a 150 mm step through the middle of the marker
    assert fit_plane(depth, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG) is None
    fit = fit_plane_detailed(depth, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG)
    assert fit.reason == REASON_RESIDUAL
    assert fit.residual_mm > CFG.max_residual_mm


def test_uses_the_shared_deprojection(monkeypatch):
    """The plane is fitted to the points of ros2_common.camera_geometry.depth_to_points on the D435 intrinsics: the
    shared deprojection is called once with the polygon-masked depth, and a shift of its points moves the plane."""
    pose = pose_facing(1.0, yaw_deg=35.0, offset_m=(0.1, -0.05))
    polygon = marker_polygon(pose)
    depth = depth_of_plane(pose)
    calls = []
    shared = depth_plane.depth_to_points

    def spy(image, intrinsics, *args, **kwargs):
        calls.append((image.copy(), intrinsics))
        return shared(image, intrinsics, *args, **kwargs)

    monkeypatch.setattr(depth_plane, "depth_to_points", spy)
    plane = fit_plane(depth, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG)
    assert plane is not None and len(calls) == 1
    image, intrinsics = calls[0]
    assert intrinsics == DEPTH_INTRINSICS
    inside = polygon_mask(polygon)
    np.testing.assert_array_equal(image[inside], depth[inside])
    assert not image[~inside].any()
    assert plane.n_px == int(inside.sum())
    assert angle_deg(plane.normal, normal_of(pose[0])) < 1.0

    shift_m = 0.2
    monkeypatch.setattr(depth_plane, "depth_to_points", lambda *a, **k: shared(*a, **k) + [0.0, 0.0, shift_m])
    shifted = fit_plane(depth, DEPTH_INTRINSICS, polygon, STREAM_SIZE, CFG)
    assert shifted is not None
    expected_mm = plane.distance_mm - 1000.0 * shift_m * float(plane.normal[2])
    assert shifted.distance_mm == pytest.approx(expected_mm, abs=0.5)


# --- plane_chooser --------------------------------------------------------------------------------------------------


def ambiguous_solutions(seed: int) -> tuple[list[Solution], tuple[np.ndarray, np.ndarray], Candidate, np.ndarray]:
    """100 mm marker, yaw 12 deg, 1.3 m away (~2.7 px/module), 1 px corner noise: pose.py finds two IPPE solutions that
    reproject almost equally well and rejects the pose ``ambiguous``."""
    pose = pose_facing(1.3, yaw_deg=12.0)
    grey = render_marker_scene([PAYLOAD], [SMALL_SIZE_MM], [pose], STREAM_K, STREAM_SIZE, noise_px=2.0, seed=seed)
    corners = marker_polygon(pose, SMALL_SIZE_MM)
    corners = corners + np.random.default_rng(seed + 1000).normal(0.0, 1.0, corners.shape)
    modules = symbol_matrix(PAYLOAD).shape[0]
    module_px = float(np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1).mean() / modules)
    candidate = Candidate(decode_payload(PAYLOAD), PAYLOAD, corners, (modules - 17) // 4, module_px)
    result = estimate_pose(grey, candidate, STREAM_K, None, LOW_MODULE_CFG)
    assert result.reason == "ambiguous" and len(result.solutions) == 2
    return result.solutions, pose, candidate, grey


@pytest.mark.parametrize("seed", SEEDS)
def test_chooser_picks_true_solution_from_depth_plane(seed):
    solutions, pose, candidate, grey = ambiguous_solutions(seed)
    true_normal = normal_of(pose[0])
    plane = fit_plane(depth_of_plane(pose, seed=seed), DEPTH_INTRINSICS, candidate.corners, STREAM_SIZE, CFG)
    assert plane is not None
    chosen = plane_chooser(plane, CFG)(solutions)
    truest = min(solutions, key=lambda s: angle_deg(s.normal, true_normal))
    assert chosen is truest
    result = estimate_pose(grey, candidate, STREAM_K, None, LOW_MODULE_CFG, plane_chooser(plane, CFG))
    assert result.ok, result.reason
    np.testing.assert_allclose(result.rvec, truest.rvec)


def test_chooser_returns_none_for_plane_ninety_degrees_off():
    solutions, pose, _, _ = ambiguous_solutions(0)
    true_normal = normal_of(pose[0])
    rotation, _ = cv2.Rodrigues(np.array([0.0, np.pi / 2.0, 0.0]))
    off = Plane(normal=rotation @ true_normal, distance_mm=1300.0, residual_mm=1.0, n_px=100)
    assert angle_deg(off.normal, true_normal) == pytest.approx(90.0)
    assert plane_chooser(off, CFG)(solutions) is None


def test_chooser_returns_none_when_both_solutions_within_limit():
    solutions, pose, _, _ = ambiguous_solutions(0)
    plane = Plane(normal=normal_of(pose[0]), distance_mm=1300.0, residual_mm=1.0, n_px=100)
    assert plane_chooser(plane, DepthPlaneConfig(max_angle_deg=90.0))(solutions) is None


def test_chooser_handles_empty_and_single_lists():
    pose = pose_facing(1.0, yaw_deg=10.0)
    plane = Plane(normal=normal_of(pose[0]), distance_mm=1000.0, residual_mm=1.0, n_px=100)
    one = Solution(np.asarray(pose[0]), np.asarray(pose[1]), 0.1)
    flipped = Solution(
        cv2.Rodrigues(cv2.Rodrigues(pose[0])[0] @ np.diag([-1.0, 1.0, -1.0]))[0].reshape(3), pose[1], 0.1
    )
    choose = plane_chooser(plane, CFG)
    assert choose([]) is None
    assert choose([one]) is one
    assert choose([flipped]) is None


# --- nearest_depth --------------------------------------------------------------------------------------------------


def test_nearest_depth_picks_closest_within_limit():
    frames = [(1_000_000_000 + i * 100_000_000, np.full((2, 2), i, dtype=np.uint16)) for i in range(5)]
    stamp, image = nearest_depth(frames, 1_240_000_000, 0.15)
    assert stamp == 1_200_000_000 and image[0, 0] == 2
    stamp, _ = nearest_depth(frames, 1_260_000_000, 0.15)
    assert stamp == 1_300_000_000
    assert nearest_depth(frames, 900_000_000, 0.15)[0] == 1_000_000_000  # earlier than every frame, 0.1 s away


def test_nearest_depth_respects_max_dt():
    frames = [(1_000_000_000, np.zeros((2, 2), dtype=np.uint16)), (2_000_000_000, np.ones((2, 2), dtype=np.uint16))]
    assert nearest_depth(frames, 1_500_000_000, 0.15) is None
    assert nearest_depth(frames, 1_150_000_000, 0.15)[0] == 1_000_000_000  # exactly at the limit
    assert nearest_depth(frames, 1_150_000_001, 0.15) is None
    assert nearest_depth(frames, 2_200_000_000, 0.15) is None
    assert nearest_depth([], 1_000_000_000, 0.15) is None
