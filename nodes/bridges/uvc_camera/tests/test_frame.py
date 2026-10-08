"""Unit tests for the pure frame rotation helper."""

import numpy as np
import pytest
from frame import frame_due, rotate_frame

FRAME = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)  # height 2, width 3, bgr


def test_zero_returns_the_same_frame() -> None:
    """0 deg is a no-op (no copy)."""
    assert rotate_frame(FRAME, 0) is FRAME


def test_180_flips_both_axes_and_keeps_shape() -> None:
    """180 deg: pixel (r, c) moves to (h-1-r, w-1-c), shape unchanged."""
    out = rotate_frame(FRAME, 180)
    assert out.shape == FRAME.shape
    assert np.array_equal(out, FRAME[::-1, ::-1])
    assert np.array_equal(out[0, 0], FRAME[1, 2])


def test_90_is_clockwise_and_swaps_dimensions() -> None:
    """90 deg clockwise: top-left pixel ends up top-right, shape (w, h, 3)."""
    out = rotate_frame(FRAME, 90)
    assert out.shape == (3, 2, 3)
    assert np.array_equal(out[0, 1], FRAME[0, 0])
    assert np.array_equal(out[0, 0], FRAME[1, 0])


def test_270_is_counter_clockwise_and_swaps_dimensions() -> None:
    """270 deg clockwise: top-left pixel ends up bottom-left."""
    out = rotate_frame(FRAME, 270)
    assert out.shape == (3, 2, 3)
    assert np.array_equal(out[2, 0], FRAME[0, 0])


def test_output_is_contiguous_for_tobytes() -> None:
    """Rotated frames are C-contiguous so raw bytes match the reported step."""
    for deg in (90, 180, 270):
        assert rotate_frame(FRAME, deg).flags["C_CONTIGUOUS"]


def test_invalid_angle_raises() -> None:
    """Unsupported angle is rejected."""
    with pytest.raises(ValueError):
        rotate_frame(FRAME, 45)


def test_frame_due_without_cap_is_always_true() -> None:
    """No cap: every frame is published."""
    assert frame_due(None, 1.0, None)
    assert frame_due(1.0, 1.001, None)


def test_frame_due_first_frame_is_published() -> None:
    """Nothing published yet: the first frame goes out."""
    assert frame_due(None, 5.0, 10.0)


def test_frame_due_respects_the_period() -> None:
    """At 10 fps a frame 50 ms after the last publish is dropped, one 100 ms after is published."""
    assert not frame_due(1.0, 1.05, 10.0)
    assert frame_due(1.0, 1.1, 10.0)
