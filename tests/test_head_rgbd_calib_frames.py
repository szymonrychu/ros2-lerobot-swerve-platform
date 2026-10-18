"""Tests for scripts/head_rgbd_calibration: config, capture commands, bag reading and still-pair selection."""

import subprocess
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

from scripts.head_rgbd_calibration.config import (
    DEFAULT_CONFIG,
    MAPPING_IMAGE_TOPIC,
    CaptureConfig,
    SelectionConfig,
    load_calibration_config,
)
from scripts.head_rgbd_calibration.frames import (
    TopicNames,
    capture_bag,
    capture_commands,
    normalise_amplitude,
    read_frames,
    still_pairs,
)
from scripts.markers.config import load_generator_config

MARKERS_CONFIG = DEFAULT_CONFIG.parent / "markers_generate.yaml"
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)
TOF_SHAPE = (24, 32)
RGB_SHAPE = (48, 64)
TOPICS = TopicNames()


def test_defaults_load_from_shipped_yaml() -> None:
    cfg = load_calibration_config(DEFAULT_CONFIG)
    assert cfg.capture.host == "client.ros2.lan"
    assert cfg.capture.ssh_user == "szymonri"
    assert cfg.capture.duration_s == 120
    assert cfg.capture.remote_dir == "/tmp/head_rgbd_capture"
    assert cfg.capture.local_dir == ".logs/head_rgbd"
    assert cfg.capture.ros_setup == "/opt/ros/jazzy/setup.bash"
    assert MAPPING_IMAGE_TOPIC in cfg.capture.topics
    assert "/head_rgbd/rgb/camera_info" in cfg.capture.validation_topics
    assert set(cfg.capture.topics) < set(cfg.capture.validation_topics)
    assert cfg.selection.amplitude_percentiles == (1, 99)
    assert cfg.limits.min_views == 12
    assert cfg.outputs.urdf == "nodes/web_ui/urdf/head.urdf"
    assert cfg.validation.distances_m == [0.5, 1.0, 2.0, 3.0]


def test_unknown_key_rejected_naming_it(tmp_path: Path) -> None:
    data = yaml.safe_load(DEFAULT_CONFIG.read_text())
    data["selection"]["bogus_key"] = 1
    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValidationError, match="bogus_key"):
        load_calibration_config(path)


def test_topics_without_mapping_image_rejected() -> None:
    with pytest.raises(ValidationError, match="focus lock"):
        CaptureConfig(topics=["/tof/depth/image_raw"])
    with pytest.raises(ValidationError, match="validation_topics"):
        CaptureConfig(validation_topics=["/tof/depth/image_raw"])


def test_duration_range_enforced() -> None:
    with pytest.raises(ValidationError, match="duration_s"):
        CaptureConfig(duration_s=5)
    with pytest.raises(ValidationError, match="duration_s"):
        CaptureConfig(duration_s=601)


def test_board_matches_marker_generator_charuco() -> None:
    board = load_calibration_config(DEFAULT_CONFIG).board
    charuco = load_generator_config(MARKERS_CONFIG).charuco
    assert charuco is not None
    for key in ("squares_x", "squares_y", "square_mm", "marker_mm", "dictionary"):
        assert getattr(board, key) == getattr(charuco, key), key


def test_capture_commands_contents() -> None:
    cfg = load_calibration_config(DEFAULT_CONFIG).capture
    record, copy, remove = capture_commands(cfg, "session1")
    assert record[0] == "ssh"
    assert "szymonri@client.ros2.lan" in record
    remote = record[-1]
    assert "source /opt/ros/jazzy/setup.bash" in remote
    assert "ros2 bag record" in remote
    assert "-s sqlite3" in remote
    assert "--use-sim-time" not in remote
    assert "--max-bag-duration 120" in remote
    assert "-o /tmp/head_rgbd_capture/session1" in remote
    for topic in cfg.topics:
        assert topic in remote
    assert "/head_rgbd/depth/image_raw" not in remote
    assert copy[0] == "scp" and "-r" in copy
    assert copy[-2] == "szymonri@client.ros2.lan:/tmp/head_rgbd_capture/session1"
    assert copy[-1] == ".logs/head_rgbd/session1"
    assert remove[0] == "ssh" and "szymonri@client.ros2.lan" in remove
    assert remove[-1] == "rm -rf /tmp/head_rgbd_capture/session1"


def test_capture_commands_custom_topics() -> None:
    cfg = load_calibration_config(DEFAULT_CONFIG).capture
    record = capture_commands(cfg, "v", cfg.validation_topics)[0]
    assert "/head_rgbd/rgb/camera_info" in record[-1]


class FakeRunner:
    """Records commands; fails the n-th (1-based) when fail_at is set."""

    def __init__(self, fail_at: int | None = None) -> None:
        self.calls: list[list[str]] = []
        self.fail_at = fail_at

    def __call__(self, cmd: list[str], **_: object) -> subprocess.CompletedProcess:
        self.calls.append(cmd)
        code = 3 if self.fail_at == len(self.calls) else 0
        return subprocess.CompletedProcess(cmd, code)


def test_capture_bag_runs_steps_in_order(tmp_path: Path) -> None:
    cfg = CaptureConfig(local_dir=str(tmp_path / "out"))
    runner = FakeRunner()
    result = capture_bag(cfg, "s", run=runner)
    assert result == tmp_path / "out" / "s"
    assert [c[0] for c in runner.calls] == ["ssh", "scp", "ssh"]
    assert (tmp_path / "out").is_dir()


@pytest.mark.parametrize(("fail_at", "step"), [(1, "record"), (2, "copy"), (3, "remove")])
def test_capture_bag_names_failed_step(tmp_path: Path, fail_at: int, step: str) -> None:
    cfg = CaptureConfig(local_dir=str(tmp_path / "out"))
    runner = FakeRunner(fail_at=fail_at)
    with pytest.raises(RuntimeError, match=step):
        capture_bag(cfg, "s", run=runner)
    assert len(runner.calls) == fail_at


def test_normalise_amplitude_percentile_stretch() -> None:
    amp = np.tile(np.arange(100, dtype=np.uint16) * 10 + 100, (10, 1))
    out = normalise_amplitude(amp, (1, 99))
    assert out is not None and out.dtype == np.uint8
    low, high = np.percentile(amp, [1, 99])
    assert out[amp <= low].max() == 0
    assert (out[amp >= high] == 255).all()
    assert out.min() == 0 and out.max() == 255


def test_normalise_amplitude_empty_frame_is_none() -> None:
    assert normalise_amplitude(np.zeros((4, 4), dtype=np.uint16), (1, 99)) is None
    assert normalise_amplitude(np.full((4, 4), 7, dtype=np.uint16), (1, 99)) is None


def stamp(ns: int):
    return TYPESTORE.types["builtin_interfaces/msg/Time"](sec=ns // 10**9, nanosec=ns % 10**9)


def header(ns: int, frame: str):
    return TYPESTORE.types["std_msgs/msg/Header"](stamp=stamp(ns), frame_id=frame)


def image_msg(ns: int, arr: np.ndarray, encoding: str):
    return TYPESTORE.types["sensor_msgs/msg/Image"](
        header=header(ns, "tof"),
        height=arr.shape[0],
        width=arr.shape[1],
        encoding=encoding,
        is_bigendian=0,
        step=arr.shape[1] * 2,
        data=np.frombuffer(arr.astype("<u2").tobytes(), dtype=np.uint8),
    )


def compressed_msg(ns: int, bgr: np.ndarray):
    ok, buf = cv2.imencode(".png", bgr)
    assert ok
    return TYPESTORE.types["sensor_msgs/msg/CompressedImage"](
        header=header(ns, "cam"), format="png", data=np.asarray(buf, dtype=np.uint8).reshape(-1)
    )


def info_msg(ns: int, shape: tuple[int, int], fx: float):
    return TYPESTORE.types["sensor_msgs/msg/CameraInfo"](
        header=header(ns, "cam"),
        height=shape[0],
        width=shape[1],
        distortion_model="plumb_bob",
        d=np.zeros(5),
        k=np.array([fx, 0, shape[1] / 2, 0, fx, shape[0] / 2, 0, 0, 1], dtype=np.float64),
        r=np.eye(3).reshape(9),
        p=np.zeros(12),
        binning_x=0,
        binning_y=0,
        roi=TYPESTORE.types["sensor_msgs/msg/RegionOfInterest"](
            x_offset=0, y_offset=0, height=0, width=0, do_rectify=False
        ),
    )


def write_bag(path: Path, still_windows: list[tuple[float, float]], duration_s: float = 14.0) -> None:
    """ToF at 10 Hz, RGB at 2 Hz; scene is constant inside still windows and noise elsewhere."""
    rng = np.random.default_rng(0)
    base_rgb = rng.integers(0, 255, RGB_SHAPE + (3,), dtype=np.uint8)
    base_amp = rng.integers(500, 4000, TOF_SHAPE, dtype=np.uint16)
    base_depth = rng.integers(300, 3000, TOF_SHAPE, dtype=np.uint16)
    t0 = 1_700_000_000 * 10**9

    def in_still(t: float) -> bool:
        return any(a <= t <= b for a, b in still_windows)

    with Writer(path, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        conns = {
            name: writer.add_connection(topic, msg_type, typestore=TYPESTORE)
            for name, topic, msg_type in [
                ("rgb", TOPICS.rgb, "sensor_msgs/msg/CompressedImage"),
                ("amp", TOPICS.amplitude, "sensor_msgs/msg/Image"),
                ("depth", TOPICS.depth, "sensor_msgs/msg/Image"),
                ("tof_info", TOPICS.tof_info, "sensor_msgs/msg/CameraInfo"),
                ("rgb_info", TOPICS.rgb_info, "sensor_msgs/msg/CameraInfo"),
            ]
        }

        def put(name: str, ns: int, msg: object, msg_type: str) -> None:
            writer.write(conns[name], ns, TYPESTORE.serialize_cdr(msg, msg_type))

        for i in range(int(duration_s * 10)):
            t = i / 10
            ns = t0 + int(t * 1e9)
            still = in_still(t)
            amp = base_amp if still else rng.integers(500, 4000, TOF_SHAPE, dtype=np.uint16)
            put("amp", ns, image_msg(ns, amp, "mono16"), "sensor_msgs/msg/Image")
            put("depth", ns, image_msg(ns, base_depth, "16UC1"), "sensor_msgs/msg/Image")
            put("tof_info", ns, info_msg(ns, TOF_SHAPE, 100.0), "sensor_msgs/msg/CameraInfo")
            if i % 5 == 0:
                rns = ns + 5_000_000
                rgb = base_rgb if still else rng.integers(0, 255, RGB_SHAPE + (3,), dtype=np.uint8)
                put("rgb", rns, compressed_msg(rns, rgb), "sensor_msgs/msg/CompressedImage")
                put("rgb_info", rns, info_msg(rns, RGB_SHAPE, 200.0), "sensor_msgs/msg/CameraInfo")


def test_read_frames_decodes_everything(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    write_bag(bag, [(3.0, 5.0)], duration_s=3.0)
    frames = read_frames(bag, TOPICS)
    assert len(frames.rgb) == 6 and len(frames.amplitude) == 30 and len(frames.depth) == 30
    assert frames.rgb[0].image.shape == RGB_SHAPE + (3,) and frames.rgb[0].image.dtype == np.uint8
    assert frames.amplitude[0].image.dtype == np.uint16 and frames.amplitude[0].image.shape == TOF_SHAPE
    assert frames.depth[0].image.dtype == np.uint16
    assert frames.rgb[1].stamp_ns - frames.rgb[0].stamp_ns == 500_000_000
    assert frames.rgb_info is not None and frames.rgb_info.k[0] == 200.0 and frames.rgb_info.width == RGB_SHAPE[1]
    assert frames.tof_info is not None and frames.tof_info.k[0] == 100.0


def test_still_pairs_two_windows(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    write_bag(bag, [(4.0, 6.0), (9.0, 11.0)])
    frames = read_frames(bag, TOPICS)
    pairs = still_pairs(frames, SelectionConfig())
    assert len(pairs) == 2
    for pair in pairs:
        assert pair.dt_s <= 0.05
        assert abs(pair.rgb_stamp_ns - pair.tof_stamp_ns) <= 50_000_000
        assert pair.depth is not None and pair.depth.dtype == np.uint16
    t0 = frames.rgb[0].stamp_ns
    centres = [(p.rgb_stamp_ns - t0) / 1e9 for p in pairs]
    assert 4.0 < centres[0] < 6.0 and 9.0 < centres[1] < 11.0


def test_still_pairs_ignores_skip_start(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    write_bag(bag, [(0.0, 1.9), (6.0, 8.0)])
    frames = read_frames(bag, TOPICS)
    pairs = still_pairs(frames, SelectionConfig())
    assert len(pairs) == 1
    assert (pairs[0].rgb_stamp_ns - frames.rgb[0].stamp_ns) / 1e9 > 2.0


def test_still_pairs_none_when_always_moving(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    write_bag(bag, [])
    assert still_pairs(read_frames(bag, TOPICS), SelectionConfig()) == []
