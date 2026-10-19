"""Tests for scripts/head_rgbd_calibration/live.py: relay stream, monitor and ssh start (counter: see poses tests)."""

import io
import struct
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.poses import AMPLITUDE, RGB, PoseCounter
from rosbags.typesys import Stores, get_typestore

from scripts.head_rgbd_calibration.charuco import make_board
from scripts.head_rgbd_calibration.config import BoardConfig, CalibrationConfig, SelectionConfig
from scripts.head_rgbd_calibration.frames import StampedImage
from scripts.head_rgbd_calibration.live import (
    AMPLITUDE_INDEX,
    BELL,
    FRAME_HEADER,
    RELAY_SCRIPT,
    RGB_INDEX,
    monitor,
    read_stream,
    relay_command,
    start_live,
)
from tests.test_head_rgbd_calib_poses import sequence as poses_sequence

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)
BOARD_CFG = BoardConfig()
KIND_INDEX = {RGB: RGB_INDEX, AMPLITUDE: AMPLITUDE_INDEX}
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


def sequence() -> list[tuple[int, StampedImage]]:
    """(topic index, frame) in arrival order, the scene of the pose counter tests."""
    return [(KIND_INDEX[kind], frame) for kind, frame in poses_sequence()]


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


def test_relay_keeps_native_stdout_writes_out_of_the_frame_stream() -> None:
    root = Path(__file__).resolve().parent.parent
    result = subprocess.run(
        [sys.executable, "-c", RELAY_STDOUT_PROBE], cwd=root, capture_output=True, timeout=30, check=True
    )
    assert result.stdout == b"FRAME"
    assert b"RTPS_TRANSPORT_SHM" in result.stderr
    assert b"python print" in result.stderr
