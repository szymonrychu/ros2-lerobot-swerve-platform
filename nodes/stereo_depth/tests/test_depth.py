"""Unit tests for the pure numpy disparity -> depth (mm) conversion."""

import numpy as np

from stereo_depth.depth import disparity_to_depth_mm

F = 400.0
T = 0.1  # Z = 40 / d metres


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
