"""Unit tests for the disparity -> depth (mm) conversion and the StereoSGBM matcher on synthetic pairs."""

import cv2
import numpy as np
import pytest

from stereo_depth.config import DepthConfig
from stereo_depth.depth import compute_disparity, create_matcher, disparity_to_depth_mm
from tests.conftest import BASELINE_M, FX, HEIGHT, WIDTH, shifted_pair, textured_image

F = 400.0
T = 0.1  # Z = 40 / d metres; the restored conversion tests use these, the SGBM tests use conftest FX and BASELINE_M


def convert(values: list[list[float]], **kwargs: float) -> np.ndarray:
    params = {"f": F, "t": T, "min_disparity": 0.0, "min_depth_m": 0.2, "max_depth_m": 4.0}
    params.update(kwargs)
    return disparity_to_depth_mm(np.array(values, dtype=np.float32), **params)


def test_known_disparity_gives_exact_millimetres() -> None:
    out = convert([[40.0, 20.0, 10.0]])  # 1 m, 2 m, 4 m
    assert out.dtype == np.uint16
    assert out.tolist() == [[1000, 2000, 4000]]


def test_rounds_to_nearest_millimetre() -> None:
    assert convert([[30.0]]).tolist() == [[1333]]  # 1.3333 m


def test_non_positive_and_below_min_disparity_are_no_reading() -> None:
    out = convert([[0.0, -1.0, 5.0, 40.0]], min_disparity=8.0)
    assert out.tolist() == [[0, 0, 0, 1000]]


def test_nan_and_inf_are_no_reading() -> None:
    out = convert([[np.nan, np.inf, -np.inf, 40.0]])
    assert out.tolist() == [[0, 0, 0, 1000]]


def test_beyond_max_and_below_min_depth_are_no_reading() -> None:
    out = convert([[400.0, 200.0, 9.0, 10.0]])  # 0.1 m, 0.2 m, 4.44 m, 4.0 m
    assert out.tolist() == [[0, 200, 0, 4000]]


def test_values_are_clamped_to_uint16() -> None:
    out = convert([[0.5]], max_depth_m=1000.0)  # 80 m = 80000 mm
    assert out.tolist() == [[65535]]


def test_shape_is_preserved() -> None:
    assert convert([[40.0, 40.0], [0.0, 40.0], [40.0, 0.0]]).shape == (3, 2)


# StereoSGBM on synthetic rectified pairs (fx = 400 px, B = 0.08 m from conftest).


def sgbm_depth(left: np.ndarray, right: np.ndarray, **overrides: float) -> np.ndarray:
    """Run the configured matcher and the depth conversion on a rectified grey pair.

    Args:
        left (np.ndarray): Left grey image.
        right (np.ndarray): Right grey image.
        **overrides (float): DepthConfig overrides.

    Returns:
        np.ndarray: uint16 depth in mm.
    """
    config = DepthConfig(**overrides)
    matcher = create_matcher(config)
    disparity = compute_disparity(matcher, left, right)
    return disparity_to_depth_mm(disparity, FX, BASELINE_M, 0.0, config.min_depth_m, config.max_depth_m)


@pytest.mark.parametrize("disparity_px", [12, 20, 32])
def test_known_shift_gives_depth_within_two_percent(disparity_px: int) -> None:
    left, right = shifted_pair(disparity_px)
    depth = sgbm_depth(left, right)
    expected_mm = FX * BASELINE_M / disparity_px * 1000.0
    interior = depth[10:-10, 48 + 10 : -10]  # past the num_disparities border and the block margins
    valid = interior[interior > 0]
    assert valid.size > 0.9 * interior.size
    errors = np.abs(valid.astype(np.float64) - expected_mm) / expected_mm
    assert np.median(errors) < 0.02
    assert np.mean(errors < 0.02) > 0.95


def test_compute_disparity_is_in_pixels() -> None:
    left, right = shifted_pair(20)
    disparity = compute_disparity(create_matcher(DepthConfig()), left, right)
    assert disparity.dtype == np.float32 and disparity.shape == left.shape
    assert np.median(disparity[10:-10, 60:-10]) == pytest.approx(20.0, abs=0.25)


def test_left_border_without_a_match_is_zero() -> None:
    left, right = shifted_pair(20)
    depth = sgbm_depth(left, right)
    assert not depth[:, : DepthConfig().num_disparities - 1].any()


def test_textureless_pair_is_all_zero() -> None:
    flat = np.full((HEIGHT, WIDTH), 128, dtype=np.uint8)
    assert not sgbm_depth(flat, flat.copy()).any()


def test_unrelated_images_give_almost_no_depth() -> None:
    left = textured_image(1)
    right = textured_image(2)
    depth = sgbm_depth(left, right)
    assert np.count_nonzero(depth) < 0.05 * depth.size


def test_out_of_range_depth_is_zero() -> None:
    left, right = shifted_pair(20)  # 1.6 m
    assert not sgbm_depth(left, right, max_depth_m=1.0).any()
    assert not sgbm_depth(left, right, min_depth_m=2.0, max_depth_m=4.0).any()


def test_matcher_uses_the_config() -> None:
    config = DepthConfig(num_disparities=64, block_size=7, p1=100, p2=400, uniqueness_ratio=5)
    matcher = create_matcher(config)
    assert matcher.getNumDisparities() == 64
    assert matcher.getBlockSize() == 7
    assert (matcher.getP1(), matcher.getP2()) == (100, 400)
    assert matcher.getUniquenessRatio() == 5
    assert matcher.getMinDisparity() == 0
    assert matcher.getMode() == cv2.StereoSGBM_MODE_SGBM_3WAY
    assert (matcher.getSpeckleWindowSize(), matcher.getSpeckleRange()) == (100, 2)
    assert (matcher.getDisp12MaxDiff(), matcher.getPreFilterCap()) == (1, 31)
