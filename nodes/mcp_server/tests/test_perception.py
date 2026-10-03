"""Tests for mcp_server.perception (scan sectors, map stats/crop, image encoding)."""

import math

import cv2
import numpy as np
import pytest

from mcp_server.perception import (
    SECTOR_NAMES,
    ImageEncodingError,
    encode_jpeg,
    fit_size,
    image_to_bgr,
    map_stats,
    recompress_jpeg,
    render_map_crop,
    summarize_scan,
)


def test_sector_names_cover_eight_directions() -> None:
    assert SECTOR_NAMES == ("front", "front_left", "left", "rear_left", "rear", "rear_right", "right", "front_right")


def test_summarize_scan_transforms_into_base_link() -> None:
    # Lidar mounted backwards (yaw pi) 0.1 m ahead of base_link: a return straight ahead of the lidar is behind the robot.
    ranges = [math.inf] * 360
    ranges[0] = 1.0
    sectors = summarize_scan(
        ranges,
        angle_min=0.0,
        angle_increment=2 * math.pi / 360,
        range_min=0.15,
        range_max=12.0,
        laser_x=0.1,
        laser_y=0.0,
        laser_yaw=math.pi,
    )
    by_name = {s.sector: s for s in sectors}
    assert by_name["rear"].nearest_m == pytest.approx(0.9)
    assert abs(abs(by_name["rear"].bearing_rad) - math.pi) < 1e-6
    assert by_name["front"].nearest_m is None
    assert [s.sector for s in sectors] == list(SECTOR_NAMES)


def test_summarize_scan_ignores_invalid_returns() -> None:
    ranges = [float("nan"), 0.05, 20.0, 2.0]
    sectors = summarize_scan(ranges, 0.0, 0.01, 0.15, 12.0, 0.0, 0.0, 0.0)
    front = next(s for s in sectors if s.sector == "front")
    assert front.nearest_m == pytest.approx(2.0)
    assert sum(s.nearest_m is not None for s in sectors) == 1


def test_summarize_scan_left_is_positive_y() -> None:
    sectors = summarize_scan([0.5], math.pi / 2, 0.01, 0.1, 10.0, 0.0, 0.0, 0.0)
    left = next(s for s in sectors if s.sector == "left")
    assert left.nearest_m == pytest.approx(0.5)


def test_map_stats() -> None:
    stats = map_stats([-1, 0, 100, 50, 0, -1], width=3, height=2, resolution=0.05)
    assert stats.width == 3 and stats.height == 2
    assert stats.known_cells == 4
    assert stats.occupied_cells == 1
    assert stats.free_cells == 2
    assert stats.width_m == pytest.approx(0.15)


def test_render_map_crop_png_with_bounded_size() -> None:
    w, h = 200, 100
    data = [0] * (w * h)
    data[50 * w + 100] = 100
    png = render_map_crop(data, w, h, 0.05, 0.0, 0.0, robot_x=5.0, robot_y=2.5, robot_yaw=0.0, radius_m=3.0, max_px=64)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
    assert max(img.shape[:2]) <= 64


def test_render_map_crop_robot_outside_map_still_renders() -> None:
    png = render_map_crop([0] * 100, 10, 10, 0.05, 0.0, 0.0, 50.0, 50.0, 0.0, 1.0, 128)
    assert png[:4] == b"\x89PNG"


def test_fit_size() -> None:
    assert fit_size(2000, 1000, 1024) == (1024, 512)
    assert fit_size(500, 300, 1024) == (500, 300)
    assert fit_size(300, 900, 300) == (100, 300)


def test_encode_jpeg_downscales() -> None:
    bgr = np.zeros((600, 1200, 3), np.uint8)
    jpg = encode_jpeg(bgr, max_px=300)
    assert jpg[:2] == b"\xff\xd8"
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (150, 300)


def test_image_to_bgr_rgb8_swaps_channels() -> None:
    data = bytes([255, 0, 0] * 4)  # red in rgb8
    bgr = image_to_bgr("rgb8", 2, 2, 6, data)
    assert bgr.shape == (2, 2, 3)
    assert tuple(bgr[0, 0]) == (0, 0, 255)


def test_image_to_bgr_handles_row_padding_and_mono() -> None:
    data = bytes([10, 20, 0, 0, 30, 40, 0, 0])  # 2x2 mono8 with step 4
    bgr = image_to_bgr("mono8", 2, 2, 4, data)
    assert bgr.shape == (2, 2, 3)
    assert bgr[1, 1, 0] == 40


def test_image_to_bgr_rejects_unknown_encoding() -> None:
    with pytest.raises(ImageEncodingError):
        image_to_bgr("16UC1", 1, 1, 2, b"\x00\x00")


def test_image_to_bgr_rejects_short_buffer() -> None:
    with pytest.raises(ImageEncodingError):
        image_to_bgr("rgb8", 2, 2, 6, b"\x00")


def test_recompress_jpeg_passthrough_and_downscale() -> None:
    jpg = encode_jpeg(np.zeros((100, 200, 3), np.uint8), max_px=1024)
    assert recompress_jpeg(jpg, 1024) == (jpg, 200, 100)
    small, w, h = recompress_jpeg(jpg, 50)
    assert (w, h) == (50, 25)
    assert small[:2] == b"\xff\xd8"


def test_recompress_jpeg_rejects_garbage() -> None:
    with pytest.raises(ImageEncodingError):
        recompress_jpeg(b"not a jpeg", 100)
