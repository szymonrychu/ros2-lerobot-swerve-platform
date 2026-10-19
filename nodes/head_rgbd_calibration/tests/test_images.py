"""Tests for the pose thumbnails."""

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.charuco import Detection

from head_rgbd_calibration.images import rgb_thumbnail, tof_thumbnail
from tests.pose_factory import SELECTION, make_pose

WIDTH_PX = 320
CORNER_GREEN = (0, 255, 0)


def decode(data: bytes) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    return image


def green_pixels(image: np.ndarray) -> int:
    return int(np.count_nonzero((image[..., 1] > 200) & (image[..., 0] < 80) & (image[..., 2] < 80)))


def test_rgb_thumbnail_is_jpeg_scaled_to_width() -> None:
    pose = make_pose()
    data = rgb_thumbnail(pose.rgb.image, pose.rgb_detection, WIDTH_PX)
    assert data[:2] == b"\xff\xd8"
    thumb = decode(data)
    assert thumb.shape[1] == WIDTH_PX
    assert thumb.shape[0] == round(pose.rgb.image.shape[0] * WIDTH_PX / pose.rgb.image.shape[1])


def test_rgb_thumbnail_draws_corners() -> None:
    pose = make_pose()
    plain = rgb_thumbnail(
        pose.rgb.image, Detection(pose.rgb_detection.ids[:0], pose.rgb_detection.points[:0]), WIDTH_PX
    )
    drawn = rgb_thumbnail(pose.rgb.image, pose.rgb_detection, WIDTH_PX)
    assert green_pixels(decode(drawn)) > green_pixels(decode(plain)) + 20


def test_rgb_thumbnail_does_not_modify_the_input() -> None:
    pose = make_pose()
    before = pose.rgb.image.copy()
    rgb_thumbnail(pose.rgb.image, pose.rgb_detection, WIDTH_PX)
    assert np.array_equal(before, pose.rgb.image)


def test_rgb_thumbnail_never_enlarges() -> None:
    pose = make_pose()
    thumb = decode(rgb_thumbnail(pose.rgb.image, pose.rgb_detection, 2000))
    assert thumb.shape[1] == pose.rgb.image.shape[1]


def test_tof_thumbnail_enlarged_twice_with_corners() -> None:
    pose = make_pose()
    data = tof_thumbnail(pose.amplitude.image, pose.tof_detection, SELECTION.amplitude_percentiles)
    assert data is not None
    thumb = decode(data)
    height, width = pose.amplitude.image.shape
    assert thumb.shape[:2] == (2 * height, 2 * width)
    assert green_pixels(thumb) > 20


def test_tof_thumbnail_none_for_empty_frame() -> None:
    pose = make_pose()
    flat = np.full(pose.amplitude.image.shape, 1000, dtype=np.uint16)
    assert tof_thumbnail(flat, pose.tof_detection, SELECTION.amplitude_percentiles) is None
