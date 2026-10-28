"""Tests for the pose thumbnails."""

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.charuco import Detection

from head_rgbd_calibration.images import depth_thumbnail, rgb_thumbnail
from tests.pose_factory import make_still

WIDTH_PX = 320
CORNER_GREEN = (0, 255, 0)


def decode(data: bytes) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    return image


def green_pixels(image: np.ndarray) -> int:
    return int(np.count_nonzero((image[..., 1] > 200) & (image[..., 0] < 80) & (image[..., 2] < 80)))


def test_rgb_thumbnail_is_jpeg_scaled_to_width() -> None:
    pose = make_still()
    data = rgb_thumbnail(pose.rgb.image, pose.detection, WIDTH_PX)
    assert data[:2] == b"\xff\xd8"
    thumb = decode(data)
    assert thumb.shape[1] == WIDTH_PX
    assert thumb.shape[0] == round(pose.rgb.image.shape[0] * WIDTH_PX / pose.rgb.image.shape[1])


def test_rgb_thumbnail_draws_corners() -> None:
    pose = make_still()
    plain = rgb_thumbnail(pose.rgb.image, Detection(pose.detection.ids[:0], pose.detection.points[:0]), WIDTH_PX)
    drawn = rgb_thumbnail(pose.rgb.image, pose.detection, WIDTH_PX)
    assert green_pixels(decode(drawn)) > green_pixels(decode(plain)) + 20


def test_rgb_thumbnail_does_not_modify_the_input() -> None:
    pose = make_still()
    before = pose.rgb.image.copy()
    rgb_thumbnail(pose.rgb.image, pose.detection, WIDTH_PX)
    assert np.array_equal(before, pose.rgb.image)


def test_rgb_thumbnail_never_enlarges() -> None:
    pose = make_still()
    thumb = decode(rgb_thumbnail(pose.rgb.image, pose.detection, 2000))
    assert thumb.shape[1] == pose.rgb.image.shape[1]


def colourful_pixels(image: np.ndarray) -> int:
    """Pixels whose channels differ by more than 40 levels (a colour map, not grey)."""
    spread = image.max(axis=2).astype(int) - image.min(axis=2).astype(int)
    return int(np.count_nonzero(spread > 40))


def test_depth_thumbnail() -> None:
    """The aligned depth is colourised, scaled like the colour thumbnail and carries the colour corners."""
    still = make_still()
    data = depth_thumbnail(still.depth.image, still.detection, WIDTH_PX)
    assert data is not None and data[:2] == b"\xff\xd8"
    thumb = decode(data)
    height, width = still.depth.image.shape
    assert thumb.shape[:2] == (round(height * WIDTH_PX / width), WIDTH_PX)
    assert colourful_pixels(thumb) > thumb.shape[0] * thumb.shape[1] // 10
    off_board = still.depth.image == 0
    small = cv2.resize(off_board.astype(np.uint8), (thumb.shape[1], thumb.shape[0]), interpolation=cv2.INTER_NEAREST)
    assert np.median(thumb[small > 0]) < 10  # no depth stays black
    plain = depth_thumbnail(still.depth.image, Detection(still.detection.ids[:0], still.detection.points[:0]), WIDTH_PX)
    assert plain is not None
    assert green_pixels(thumb) > green_pixels(decode(plain)) + 20


def test_depth_thumbnail_does_not_modify_the_input() -> None:
    still = make_still()
    before = still.depth.image.copy()
    depth_thumbnail(still.depth.image, still.detection, WIDTH_PX)
    assert np.array_equal(before, still.depth.image)


def test_depth_thumbnail_none_without_valid_depth() -> None:
    still = make_still()
    assert depth_thumbnail(np.zeros_like(still.depth.image), still.detection, WIDTH_PX) is None
