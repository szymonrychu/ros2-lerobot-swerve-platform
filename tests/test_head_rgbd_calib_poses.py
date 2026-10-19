"""Tests for ros2_common.head_rgbd_calibration.poses: the live pose counter and the poses it captures."""

from functools import cache
from pathlib import Path

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.calibrate import detect_tof
from ros2_common.head_rgbd_calibration.charuco import detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig, SelectionConfig
from ros2_common.head_rgbd_calibration.frames import FrameSet, StampedImage, still_pairs
from ros2_common.head_rgbd_calibration.poses import AMPLITUDE, DEPTH, RGB, WAITING, PoseCounter, PoseEvent

from tests.rgbd_synthetic import (
    centred_pose,
    default_extrinsic,
    render_rgb,
    render_tof_amplitude,
    rgb_camera,
    rotation,
    tof_camera,
)

BOARD_CFG = BoardConfig()
T0_NS = 1_700_000_000 * 10**9
TOF_PERIOD_S = 0.1
RGB_EVERY = 5  # RGB at 2 Hz, ToF at 10 Hz
RGB_OFFSET_NS = 5_000_000
NEAR_M = 0.33
FAR_M = 1.2  # RGB still resolves the board, the 240x180 ToF does not
# (start s, end s, board distance or None for a moving board)
TIMELINE = [(0.0, 3.0, None), (3.0, 6.0, NEAR_M), (6.0, 7.0, None), (7.0, 10.0, FAR_M), (10.0, 11.0, None)]
TIMELINE += [(11.0, 14.0, NEAR_M), (14.0, 15.0, None)]
MOVE_STATUS = "moving"
CLOSER_STATUS = "move closer"
FIXTURES = Path(__file__).parent / "fixtures" / "head_rgbd"
DEPTH_SHAPE = (4, 4)


@cache
def still_view(distance_m: float) -> tuple[np.ndarray, np.ndarray]:
    rgb_pose = centred_pose(BOARD_CFG, rotation(8, -6, 3), np.array([0.0, 0.0, distance_m]))
    tof_pose = default_extrinsic().tof_pose(rgb_pose)
    return (
        render_rgb(rgb_camera(), rgb_pose, BOARD_CFG, seed=1),
        render_tof_amplitude(tof_camera(), tof_pose, BOARD_CFG, seed=1),
    )


@cache
def moving_view(step: int) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(step)
    rgb_pose = centred_pose(
        BOARD_CFG, rotation(*rng.uniform(-30, 30, 3)), np.array([*rng.uniform(-0.05, 0.05, 2), rng.uniform(0.3, 0.6)])
    )
    tof_pose = default_extrinsic().tof_pose(rgb_pose)
    return (
        render_rgb(rgb_camera(), rgb_pose, BOARD_CFG, seed=step),
        render_tof_amplitude(tof_camera(), tof_pose, BOARD_CFG, seed=step),
    )


def view_at(step: int) -> tuple[np.ndarray, np.ndarray]:
    t = step * TOF_PERIOD_S
    distance = next(d for a, b, d in TIMELINE if a <= t < b)
    return moving_view(step) if distance is None else still_view(distance)


def sequence() -> list[tuple[str, StampedImage]]:
    """(frame kind, frame) in arrival order: ToF every 0.1 s, RGB every 0.5 s just after its ToF frame."""
    out = []
    for step in range(int(TIMELINE[-1][1] / TOF_PERIOD_S)):
        rgb, amp = view_at(step)
        ns = T0_NS + int(step * TOF_PERIOD_S * 1e9)
        out.append((AMPLITUDE, StampedImage(ns, amp)))
        if step % RGB_EVERY == 0:
            out.append((RGB, StampedImage(ns + RGB_OFFSET_NS, rgb)))
    return out


def run_counter() -> tuple[PoseCounter, list[PoseEvent]]:
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig())
    events = [event for kind, frame in sequence() for event in counter.add(kind, frame)]
    return counter, events


def test_counter_captures_each_still_pose_once_when_both_cameras_see_the_board() -> None:
    counter, events = run_counter()
    assert counter.count == 2
    captured = [e.status for e in events if e.pose is not None]
    assert captured == ["pose 1 captured - move to the next pose", "pose 2 captured - move to the next pose"]
    assert [e.pose.number for e in events if e.pose is not None] == [1, 2]


def test_counter_reports_moving_and_too_far_for_the_tof() -> None:
    _, events = run_counter()
    statuses = [e.status for e in events]
    assert MOVE_STATUS in statuses
    assert any(CLOSER_STATUS in status for status in statuses)


def test_counter_agrees_with_the_offline_still_pairs() -> None:
    frames = FrameSet()
    for kind, frame in sequence():
        (frames.rgb if kind == RGB else frames.amplitude).append(frame)
    board = make_board(BOARD_CFG)
    sel = SelectionConfig()
    usable = [
        p for p in still_pairs(frames, sel) if detect(p.rgb, board) is not None and detect_tof(p.amplitude, board, sel)
    ]
    counter, _ = run_counter()
    assert counter.count == len(usable)


def test_counter_counts_a_still_board_under_mains_flicker() -> None:
    rgb = [cv2.imread(str(FIXTURES / f"rgb_flicker_{k}.png")) for k in range(2)]
    amplitude = cv2.imread(str(FIXTURES / "tof_amplitude_near_5.png"), cv2.IMREAD_GRAYSCALE).astype(np.uint16) * 16
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig())
    for step in range(60):
        ns = T0_NS + int(step * TOF_PERIOD_S * 1e9)
        counter.add(AMPLITUDE, StampedImage(ns, amplitude))
        if step % RGB_EVERY == 0:
            counter.add(RGB, StampedImage(ns + RGB_OFFSET_NS, rgb[step // RGB_EVERY % 2]))
    assert counter.count == 1


def depth_frame(ns: int) -> StampedImage:
    return StampedImage(ns, np.full(DEPTH_SHAPE, 500, dtype=np.uint16))


def run_with_depth(depth_offset_ns: int | None, **kwargs: object) -> list[PoseEvent]:
    """Run the sequence with a depth frame every 0.5 s next to an amplitude frame, offset by depth_offset_ns."""
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig(), **kwargs)
    events = []
    for kind, frame in sequence():
        if kind == AMPLITUDE and depth_offset_ns is not None and (frame.stamp_ns - T0_NS) % 500_000_000 == 0:
            events += counter.add(DEPTH, depth_frame(frame.stamp_ns + depth_offset_ns))
        events += counter.add(kind, frame, raw=b"jpeg" if kind == RGB else None)
    return events


def test_captured_pose_carries_frames_detections_raw_and_nearest_depth() -> None:
    poses = [e.pose for e in run_with_depth(2_000_000) if e.pose is not None]
    assert len(poses) == 2
    board = make_board(BOARD_CFG)
    sel = SelectionConfig()
    for pose in poses:
        assert pose.rgb_raw == b"jpeg"
        assert abs(pose.rgb.stamp_ns - pose.amplitude.stamp_ns) <= int(sel.max_pair_dt_s * 1e9)
        assert pose.depth is not None and pose.depth.stamp_ns - pose.amplitude.stamp_ns == 2_000_000
        assert np.array_equal(pose.rgb_detection.ids, detect(pose.rgb.image, board).ids)
        assert np.array_equal(pose.tof_detection.ids, detect_tof(pose.amplitude.image, board, sel).ids)


def test_captured_pose_has_no_depth_when_none_is_close_enough_and_never_a_substitute() -> None:
    far_ns = int((SelectionConfig().max_pair_dt_s + 0.01) * 1e9)
    assert all(e.pose.depth is None for e in run_with_depth(far_ns) if e.pose is not None)
    assert all(e.pose.depth is None for e in run_with_depth(None) if e.pose is not None)
    assert sum(e.pose is not None for e in run_with_depth(None)) == 2


def test_start_count_continues_the_numbering() -> None:
    events = run_with_depth(None, start_count=4)
    assert [e.pose.number for e in events if e.pose is not None] == [5, 6]
    assert [e.status for e in events if e.pose is not None][0] == "pose 5 captured - move to the next pose"


def test_status_is_waiting_until_an_rgb_and_an_amplitude_frame_arrived() -> None:
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig())
    assert counter.status == WAITING
    rgb, amp = still_view(NEAR_M)
    counter.add(RGB, StampedImage(T0_NS, rgb))
    assert counter.status == WAITING
    counter.add(AMPLITUDE, StampedImage(T0_NS, amp))
    assert counter.status == WAITING  # nothing judged yet
    for kind, frame in sequence():
        counter.add(kind, frame)
    assert counter.status != WAITING
    assert counter.status == [e.status for e in run_counter()[1]][-1]


def test_depth_history_is_pruned_like_the_amplitude() -> None:
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig())
    for kind, frame in sequence():
        counter.add(DEPTH, depth_frame(frame.stamp_ns))
        counter.add(kind, frame)
    assert counter.depth[-1].stamp_ns - counter.depth[0].stamp_ns <= int(6.0 * 1e9)


def test_unknown_frame_kind_is_rejected() -> None:
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig())
    with pytest.raises(ValueError):
        counter.add("thermal", depth_frame(T0_NS))
