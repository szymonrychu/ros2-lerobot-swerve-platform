"""Detection margin of the ARM ChArUco board on D435 frames, and FULL and ARM boards in one image."""

import logging
from collections.abc import Callable

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.charuco import board_outline, detect, make_board
from ros2_common.head_rgbd_calibration.config import ARM_BOARD, FULL_BOARD, BoardConfig

from tests.rgbd_synthetic import (
    BACKGROUND_GREY,
    MAX_UINT8,
    Pose,
    SyntheticCamera,
    centred_pose,
    make_camera,
    render_grey,
    render_rgb,
)

LOGGER = logging.getLogger(__name__)

D435_SIZE = (640, 360)
D435_FX = 423.0
NEAR_M = 0.34
FAR_M = 0.6
MIN_CORNERS_NEAR = 12
WIDE_SIZE = (1280, 720)
WIDE_FX = 846.0
FULL_CENTRE_X_M = -0.15
ARM_CENTRE_X_M = 0.13
ALL_MARKER_IDS = list(range(30))


def composite_grey(camera: SyntheticCamera, scenes: list[tuple[BoardConfig, Pose]]) -> np.ndarray:
    """Render several boards into one uint8 grey image.

    Where the first board rendered the background value, the next board's pixel is used.

    Args:
        camera: Camera the boards are rendered through.
        scenes: (board definition, board pose) pairs, rendered in order.

    Returns:
        (height, width) uint8 grey image.
    """
    image: np.ndarray | None = None
    for board_cfg, pose in scenes:
        grey = np.rint(render_grey(camera, pose, board_cfg) * MAX_UINT8).astype(np.uint8)
        if image is None:
            image = grey
        else:
            image = np.where(image == BACKGROUND_GREY, grey, image)
    assert image is not None
    return image


def projected_outline(camera: SyntheticCamera, pose: Pose, board_cfg: BoardConfig) -> np.ndarray:
    """Board outline (outer squares, no margin) projected into the image.

    Args:
        camera: Camera the board is seen by.
        pose: Board pose (board frame to camera frame).
        board_cfg: Board definition.

    Returns:
        (4, 2) float32 pixel coordinates.
    """
    pixels, _ = cv2.projectPoints(board_outline(board_cfg, 0.0), pose.rvec, pose.t, camera.k, camera.d)
    return pixels.reshape(-1, 2).astype(np.float32)


def inside(points: np.ndarray, outline: np.ndarray) -> bool:
    """Whether every point lies inside or on the outline polygon.

    Args:
        points: (N, 2) pixel coordinates.
        outline: (M, 2) float32 polygon.

    Returns:
        True when all points are inside or on the polygon.
    """
    return all(cv2.pointPolygonTest(outline, (float(x), float(y)), False) >= 0 for x, y in points)


def test_arm_board_detected_at_0_34_m() -> None:
    camera = make_camera(D435_SIZE, D435_FX)
    pose = centred_pose(ARM_BOARD, np.eye(3), np.array([0.0, 0.0, NEAR_M]))
    image = render_rgb(camera, pose, ARM_BOARD)
    found = detect(image, make_board(ARM_BOARD))
    assert found is not None and len(found.ids) >= MIN_CORNERS_NEAR


def test_arm_board_detection_at_0_6_m_is_reported(record_property: Callable[[str, object], None]) -> None:
    camera = make_camera(D435_SIZE, D435_FX)
    pose = centred_pose(ARM_BOARD, np.eye(3), np.array([0.0, 0.0, FAR_M]))
    image = render_rgb(camera, pose, ARM_BOARD)
    found = detect(image, make_board(ARM_BOARD))
    count = 0 if found is None else len(found.ids)
    LOGGER.info("ARM board at %.2f m: %d corners", FAR_M, count)
    record_property("arm_corners_at_0_6_m", count)  # reported, not asserted (decision: 0.34 m is the working distance)


def test_both_boards_in_one_image_are_detected_separately() -> None:
    camera = make_camera(WIDE_SIZE, WIDE_FX)
    full_pose = centred_pose(FULL_BOARD, np.eye(3), np.array([FULL_CENTRE_X_M, 0.0, FAR_M]))
    arm_pose = centred_pose(ARM_BOARD, np.eye(3), np.array([ARM_CENTRE_X_M, 0.0, FAR_M]))
    grey = composite_grey(camera, [(FULL_BOARD, full_pose), (ARM_BOARD, arm_pose)])
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    _, marker_ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(grey)
    assert sorted(marker_ids.ravel().tolist()) == ALL_MARKER_IDS
    full, arm = detect(grey, make_board(FULL_BOARD)), detect(grey, make_board(ARM_BOARD))
    assert full is not None and arm is not None
    assert len(full.ids) >= MIN_CORNERS_NEAR and len(arm.ids) >= MIN_CORNERS_NEAR
    assert inside(full.points, projected_outline(camera, full_pose, FULL_BOARD))
    assert inside(arm.points, projected_outline(camera, arm_pose, ARM_BOARD))
