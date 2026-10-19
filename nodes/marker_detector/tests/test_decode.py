"""Tests for marker_detector.decode against synthetic scenes with independent projected ground truth."""

import numpy as np
import pytest
from ros2_common.marker_payload import decode_payload

from marker_detector.decode import decode_frame, find_undecoded, order_corners
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
PAYLOAD_2 = "P1/K:CHARGER/N:DOCK-2/S:100"


def scene(payloads, sizes, poses, **kwargs):
    return render_marker_scene(payloads, sizes, poses, STREAM_K, STREAM_SIZE, **kwargs)


def truth(size_mm, pose):
    return project_points(symbol_corners_m(size_mm), pose, STREAM_K)


def test_front_on_marker_decodes_with_corners_within_one_px():
    pose = pose_facing(1.0)
    candidates, rejections = decode_frame(scene([PAYLOAD], [150.0], [pose]))
    assert rejections == []
    assert len(candidates) == 1
    assert candidates[0].payload == decode_payload(PAYLOAD)
    assert candidates[0].text == PAYLOAD
    assert np.abs(candidates[0].corners - truth(150.0, pose)).max() < 1.0


@pytest.mark.parametrize("roll", [90, 180, 270])
def test_roll_keeps_corner_order_tied_to_symbol(roll):
    pose = pose_facing(1.0, roll_deg=roll)
    candidates, _ = decode_frame(scene([PAYLOAD], [150.0], [pose]))
    assert len(candidates) == 1
    assert np.abs(candidates[0].corners - truth(150.0, pose)).max() < 1.0


@pytest.mark.parametrize("roll", [0, 90, 180, 270])
@pytest.mark.parametrize("shift", [1, 2, 3])
def test_order_corners_restores_symbol_order_from_shifted_start(roll, shift):
    pose = pose_facing(1.0, roll_deg=roll)
    grey = scene([PAYLOAD], [150.0], [pose])
    candidates, _ = decode_frame(grey)
    expected = truth(150.0, pose)
    modules = 21 + 4 * (candidates[0].version - 1)
    shifted = np.roll(expected, -shift, axis=0)
    restored = order_corners(grey, shifted, modules)
    assert np.abs(restored - expected).max() < 1.0


def test_two_markers_give_two_candidates():
    poses = [pose_facing(1.0, offset_m=(-0.2, 0.0)), pose_facing(1.2, offset_m=(0.25, 0.0))]
    candidates, rejections = decode_frame(scene([PAYLOAD, PAYLOAD_2], [150.0, 100.0], poses))
    assert rejections == []
    assert sorted(c.text for c in candidates) == sorted([PAYLOAD, PAYLOAD_2])


def test_foreign_qr_texts_are_payload_rejections():
    poses = [pose_facing(1.0, offset_m=(-0.2, 0.0)), pose_facing(1.0, offset_m=(0.2, 0.0))]
    candidates, rejections = decode_frame(scene(["HELLO", "P2/K:BASE/N:X/S:100"], [150.0, 150.0], poses))
    assert candidates == []
    assert len(rejections) == 2
    assert {r.reason for r in rejections} == {"payload"}
    assert all(r.detail for r in rejections)


def test_module_px_matches_projection():
    pose = pose_facing(1.5, yaw_deg=30)
    candidates, _ = decode_frame(scene([PAYLOAD], [150.0], [pose]))
    corners = truth(150.0, pose)
    modules = symbol_matrix(PAYLOAD).shape[0]
    expected = np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]) / modules
    assert candidates[0].version == (modules - 17) // 4
    assert candidates[0].module_px == pytest.approx(expected, rel=0.05)


def test_sub_pixel_modules_not_decoded():
    pose = pose_facing(3.0)
    candidates, _ = decode_frame(scene([PAYLOAD], [60.0], [pose]))
    assert candidates == []


def test_noisy_marker_zxing_misses_is_located_for_zoom():
    """100 mm marker at 1.2 m (~2.5 px/module) with blur 0.8 px and noise sigma 40 grey levels: zxing fails, the
    Aruco quad finder still locates it, so it becomes a zoom target."""
    pose = pose_facing(1.2)
    grey = scene([PAYLOAD_2], [100.0], [pose], blur_px=0.8, noise_px=40)
    candidates, _ = decode_frame(grey)
    assert candidates == []
    quads = find_undecoded(grey, candidates)
    centre = truth(100.0, pose).mean(axis=0)
    assert any(np.linalg.norm(q.mean(axis=0) - centre) < 10 for q in quads)


def test_measured_decode_limit_documented():
    """Measured limit (noise free, version 3 symbol): zxing decodes down to about 1.4 px/module (100 mm at 2.5 m) and
    fails with blur sigma >= 0.8 px at 1.2 px/module; the Aruco finder is no more sensitive when blur is the cause."""
    sharp, _ = decode_frame(scene([PAYLOAD_2], [100.0], [pose_facing(2.5)]))
    assert len(sharp) == 1
    assert sharp[0].module_px == pytest.approx(1.4, abs=0.15)
    blurred, _ = decode_frame(scene([PAYLOAD_2], [100.0], [pose_facing(2.5)], blur_px=1.6))
    assert blurred == []


def test_find_undecoded_skips_decoded_markers():
    grey = scene([PAYLOAD], [150.0], [pose_facing(1.0)])
    candidates, _ = decode_frame(grey)
    assert find_undecoded(grey, candidates) == []
