"""Tests for scripts/head_rgbd_calibration/live.py: the live pose counter shown while a calibration bag records."""

import io
import struct
import subprocess
import sys
from functools import cache
from pathlib import Path

import cv2
import numpy as np
from rosbags.typesys import Stores, get_typestore

from scripts.head_rgbd_calibration.charuco import detect, make_board
from scripts.head_rgbd_calibration.config import BoardConfig, CalibrationConfig, SelectionConfig
from scripts.head_rgbd_calibration.frames import FrameSet, StampedImage, still_pairs
from scripts.head_rgbd_calibration.live import (
    AMPLITUDE_INDEX,
    BELL,
    FRAME_HEADER,
    RELAY_SCRIPT,
    RGB_INDEX,
    PoseCounter,
    monitor,
    read_stream,
    relay_command,
    start_live,
)
from scripts.head_rgbd_calibration.pipeline import detect_tof
from tests.rgbd_synthetic import (
    centred_pose,
    default_extrinsic,
    render_rgb,
    render_tof_amplitude,
    rgb_camera,
    rotation,
    tof_camera,
)

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)
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
# a relay run with rclpy and sensor_msgs stubbed: a native library writing a log line to fd 1 after the relay
# claimed its frame output (Fast DDS logs SHM port errors to stdout)
RELAY_STDOUT_PROBE = """
import os, sys, types
for name in ("rclpy", "rclpy.executors", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg"):
    sys.modules[name] = types.ModuleType(name)
sys.modules["rclpy.executors"].ExternalShutdownException = Exception
sys.modules["rclpy.qos"].qos_profile_sensor_data = None
sys.modules["sensor_msgs.msg"].CompressedImage = sys.modules["sensor_msgs.msg"].Image = object
from scripts.head_rgbd_calibration.live_relay import frame_output
out = frame_output()
os.write(1, b"[RTPS_TRANSPORT_SHM Error] Failed init_port\\n")
print("python print")
sys.stdout.flush()
out.write(b"FRAME")
out.flush()
"""


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


def sequence() -> list[tuple[int, StampedImage]]:
    """(topic index, frame) in arrival order: ToF every 0.1 s, RGB every 0.5 s just after its ToF frame."""
    out = []
    for step in range(int(TIMELINE[-1][1] / TOF_PERIOD_S)):
        rgb, amp = view_at(step)
        ns = T0_NS + int(step * TOF_PERIOD_S * 1e9)
        out.append((AMPLITUDE_INDEX, StampedImage(ns, amp)))
        if step % RGB_EVERY == 0:
            out.append((RGB_INDEX, StampedImage(ns + RGB_OFFSET_NS, rgb)))
    return out


def header(ns: int, frame: str):
    stamp = TYPESTORE.types["builtin_interfaces/msg/Time"](sec=ns // 10**9, nanosec=ns % 10**9)
    return TYPESTORE.types["std_msgs/msg/Header"](stamp=stamp, frame_id=frame)


def serialize(index: int, frame: StampedImage) -> bytes:
    if index == RGB_INDEX:
        ok, buf = cv2.imencode(".png", frame.image)
        assert ok
        msg = TYPESTORE.types["sensor_msgs/msg/CompressedImage"](
            header=header(frame.stamp_ns, "cam"), format="png", data=np.asarray(buf, dtype=np.uint8).reshape(-1)
        )
        data = TYPESTORE.serialize_cdr(msg, "sensor_msgs/msg/CompressedImage")
    else:
        arr = frame.image
        msg = TYPESTORE.types["sensor_msgs/msg/Image"](
            header=header(frame.stamp_ns, "tof"),
            height=arr.shape[0],
            width=arr.shape[1],
            encoding="mono16",
            is_bigendian=0,
            step=arr.shape[1] * 2,
            data=np.frombuffer(arr.astype("<u2").tobytes(), dtype=np.uint8),
        )
        data = TYPESTORE.serialize_cdr(msg, "sensor_msgs/msg/Image")
    return struct.pack(FRAME_HEADER, index, len(data)) + bytes(data)


def stream_bytes() -> bytes:
    return b"".join(serialize(index, frame) for index, frame in sequence())


def run_counter() -> tuple[PoseCounter, list[tuple[str, bool]]]:
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig())
    events = [event for index, frame in sequence() for event in counter.add(index, frame)]
    return counter, events


def test_read_stream_decodes_rgb_and_amplitude_and_ignores_a_truncated_tail() -> None:
    frames = sequence()[:7]
    data = b"".join(serialize(i, f) for i, f in frames)
    decoded = list(read_stream(io.BytesIO(data + data[:10])))
    assert [i for i, _ in decoded] == [i for i, _ in frames]
    for (_, got), (_, want) in zip(decoded, frames, strict=True):
        assert got.stamp_ns == want.stamp_ns
        assert got.image.shape == want.image.shape
        assert got.image.dtype == want.image.dtype
    assert np.array_equal(decoded[1][1].image, frames[1][1].image)


def test_counter_counts_each_still_pose_once_when_both_cameras_see_the_board() -> None:
    counter, events = run_counter()
    assert counter.count == 2
    captured = [status for status, is_new in events if is_new]
    assert captured == ["pose 1 captured - move to the next pose", "pose 2 captured - move to the next pose"]


def test_counter_reports_moving_and_too_far_for_the_tof() -> None:
    _, events = run_counter()
    statuses = [status for status, _ in events]
    assert MOVE_STATUS in statuses
    assert any(CLOSER_STATUS in status for status in statuses)


def test_counter_agrees_with_the_offline_still_pairs() -> None:
    frames = FrameSet()
    for index, frame in sequence():
        (frames.rgb if index == RGB_INDEX else frames.amplitude).append(frame)
    board = make_board(BOARD_CFG)
    sel = SelectionConfig()
    usable = [
        p for p in still_pairs(frames, sel) if detect(p.rgb, board) is not None and detect_tof(p.amplitude, board, sel)
    ]
    counter, _ = run_counter()
    assert counter.count == len(usable)


def test_monitor_prints_status_changes_and_rings_on_each_capture() -> None:
    out = io.StringIO()
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig())
    ticks = iter(range(10_000))
    monitor(io.BytesIO(stream_bytes()), counter, out, clock=lambda: float(next(ticks)))
    lines = out.getvalue().splitlines()
    assert sum(BELL in line for line in lines) == 2
    assert any("poses 1 | pose 1 captured" in line for line in lines)
    assert any("poses 2 | pose 2 captured" in line for line in lines)
    assert len(lines) == len(set(lines))  # repeated statuses are not reprinted
    assert all(line.startswith("[") for line in lines)


class FakeProcess:
    def __init__(self, stdout: bytes) -> None:
        self.stdin = io.BytesIO()
        self.stdin.close = lambda: None  # keep the written script readable
        self.stdout = io.BytesIO(stdout)
        self.terminated = False

    def terminate(self) -> None:
        self.terminated = True

    def wait(self, timeout: float | None = None) -> int:
        return 0


def test_start_live_runs_the_relay_and_stop_reports_the_count() -> None:
    cfg = CalibrationConfig()
    calls = []
    process = FakeProcess(stream_bytes())

    def popen(cmd: list[str], **kwargs: object) -> FakeProcess:
        calls.append((cmd, kwargs))
        return process

    out = io.StringIO()
    stop = start_live(cfg, popen=popen, out=out)
    stop()
    assert len(calls) == 1
    assert calls[0][0] == relay_command(cfg.capture)
    assert process.stdin.getvalue() == RELAY_SCRIPT.read_bytes()
    assert process.terminated
    assert "2 poses" in out.getvalue().splitlines()[-1]


def test_relay_command_runs_the_piped_script_bounded_by_the_capture() -> None:
    cfg = CalibrationConfig().capture
    cmd = relay_command(cfg)
    assert cmd[:2] == ["ssh", "szymonri@client.ros2.lan"]
    remote = cmd[-1]
    assert "source /opt/ros/jazzy/setup.bash" in remote
    assert "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4 &&" in remote
    assert f"timeout -s INT {cfg.duration_s + 10:g}" in remote
    assert "python3 -" in remote
    assert FRAME_HEADER in remote
    assert "/head_camera/mapping/image_raw/compressed" in remote
    assert "/tof/amplitude/image_raw" in remote


def test_relay_script_compiles_and_takes_header_and_topics_from_argv() -> None:
    source = RELAY_SCRIPT.read_text()
    compile(source, str(RELAY_SCRIPT), "exec")
    assert "sys.argv" in source
    assert "raw=True" in source


def test_live_is_on_by_default() -> None:
    assert CalibrationConfig().capture.live is True


def test_read_stream_skips_unknown_topic_index() -> None:
    frames = sequence()[:2]
    junk = struct.pack(FRAME_HEADER, 7, 3) + b"abc"
    data = junk + b"".join(serialize(i, f) for i, f in frames)
    assert [i for i, _ in read_stream(io.BytesIO(data))] == [i for i, _ in frames]


def test_counter_counts_a_still_board_under_mains_flicker() -> None:
    rgb = [cv2.imread(str(FIXTURES / f"rgb_flicker_{k}.png")) for k in range(2)]
    amplitude = cv2.imread(str(FIXTURES / "tof_amplitude_near_5.png"), cv2.IMREAD_GRAYSCALE).astype(np.uint16) * 16
    counter = PoseCounter(make_board(BOARD_CFG), SelectionConfig())
    for step in range(60):
        ns = T0_NS + int(step * TOF_PERIOD_S * 1e9)
        counter.add(AMPLITUDE_INDEX, StampedImage(ns, amplitude))
        if step % RGB_EVERY == 0:
            counter.add(RGB_INDEX, StampedImage(ns + RGB_OFFSET_NS, rgb[step // RGB_EVERY % 2]))
    assert counter.count == 1


def test_relay_keeps_native_stdout_writes_out_of_the_frame_stream() -> None:
    root = Path(__file__).resolve().parent.parent
    result = subprocess.run(
        [sys.executable, "-c", RELAY_STDOUT_PROBE], cwd=root, capture_output=True, timeout=30, check=True
    )
    assert result.stdout == b"FRAME"
    assert b"RTPS_TRANSPORT_SHM" in result.stderr
    assert b"python print" in result.stderr
