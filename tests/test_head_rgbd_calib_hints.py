"""Tests for ros2_common.head_rgbd_calibration.hints: capture-time corner-count hints."""

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import BoardConfig
from ros2_common.head_rgbd_calibration.extrinsics import MIN_COMMON_CORNERS
from ros2_common.head_rgbd_calibration.hints import FEW_CORNERS_FRACTION, capture_hints

BOARD = BoardConfig()
INNER = 20


def det(ids: list[int]) -> Detection:
    return Detection(ids=np.array(ids, dtype=np.int32), points=np.zeros((len(ids), 2), dtype=np.float32))


def full() -> Detection:
    return det(list(range(INNER)))


def test_fraction_is_half() -> None:
    assert FEW_CORNERS_FRACTION == 0.5


def test_full_board_pose_has_no_hints() -> None:
    assert capture_hints(full(), full(), BOARD) == []


def test_hints_at_threshold_edges() -> None:
    test_rgb_threshold_edge()
    test_tof_threshold_edge()
    test_common_corner_edge()


def test_rgb_threshold_edge() -> None:
    assert capture_hints(det(list(range(9))), full(), BOARD) == [
        "few corners: 9 RGB - move the board closer / fill the frame"
    ]
    assert capture_hints(det(list(range(10))), full(), BOARD) == []


def test_tof_threshold_edge() -> None:
    assert capture_hints(full(), det(list(range(9))), BOARD) == [
        "few corners: 9 ToF - move the board closer to the ToF camera / fill its frame"
    ]
    assert capture_hints(full(), det(list(range(10))), BOARD) == []


def test_common_corner_edge() -> None:
    rgb = det(list(range(10)))
    tof = det(list(range(10 - MIN_COMMON_CORNERS + 1, 20)))  # 5 shared ids
    assert capture_hints(rgb, tof, BOARD) == [
        "only 5 corners seen by both cameras - not used for the stereo fit; keep the board in both views"
    ]
    tof = det(list(range(10 - MIN_COMMON_CORNERS, 20)))  # 6 shared ids
    assert capture_hints(rgb, tof, BOARD) == []


def test_order_rgb_tof_common() -> None:
    hints = capture_hints(det([0, 1, 2, 3, 4, 5]), det([10, 11, 12, 13, 14, 15]), BOARD)
    assert [h.split(":")[0].split(" ")[0] for h in hints] == ["few", "few", "only"]
    assert "RGB" in hints[0] and "ToF" in hints[1] and "0 corners seen by both" in hints[2]


def test_threshold_follows_board_size() -> None:
    test_board_size_changes_threshold(5, 5, 16, 8)
    test_board_size_changes_threshold(8, 6, 35, 18)


@pytest.mark.parametrize(("sx", "sy", "inner", "edge"), [(5, 5, 16, 8), (8, 6, 35, 18)])
def test_board_size_changes_threshold(sx: int, sy: int, inner: int, edge: int) -> None:
    board = BoardConfig(squares_x=sx, squares_y=sy)
    ids = list(range(inner))
    assert capture_hints(det(ids[: edge - 1]), det(ids), board)[0].startswith(f"few corners: {edge - 1} RGB")
    assert capture_hints(det(ids[:edge]), det(ids), board) == []
