"""Unit tests for the arducam_tof bridge with the Arducam SDK, rclpy and the ROS message packages faked.

The SDK (ArducamDepthCamera) has aarch64 Linux wheels only and rclpy is not installed on dev machines, so both are
replaced by small doubles registered in sys.modules before arducam_tof.bridge is imported.
"""

import json
import sys
import types
from collections.abc import Callable
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest
from prometheus_client import REGISTRY

from arducam_tof.config import TofConfig

INTRINSIC_FX = 21345  # SDK reports intrinsics x100
INTRINSIC_FY = 21290
INTRINSIC_CX = 11950
INTRINSIC_CY = 8975
WIDTH = 4
HEIGHT = 3
CLOCK_NS = 5_000_000_000


class FakeTime:
    """Stand-in rclpy Time: integer nanoseconds that convert to a comparable stamp."""

    def __init__(self, nanoseconds: int = 0, clock_type: Any = None) -> None:
        self.nanoseconds = nanoseconds
        self.clock_type = clock_type

    def to_msg(self) -> tuple[str, int]:
        return ("stamp", self.nanoseconds)


class NodeClock:
    """Stand-in node clock whose reading can be advanced."""

    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def now(self) -> FakeTime:
        return FakeTime(self.nanoseconds, "ros")


class Msg:
    """Stand-in ROS message: a plain object whose fields the bridge assigns."""


class Image(Msg):
    """Stand-in sensor_msgs/Image."""


class CameraInfo(Msg):
    """Stand-in sensor_msgs/CameraInfo."""


class Header(Msg):
    """Stand-in std_msgs/Header."""


class DepthData:
    """Stand-in SDK depth frame."""

    def __init__(
        self, depth: np.ndarray | None, confidence: np.ndarray | None, amplitude: np.ndarray | None = None
    ) -> None:
        """Create the frame.

        Args:
            depth (np.ndarray | None): Depth in millimetres (float32).
            confidence (np.ndarray | None): Confidence per pixel (float32).
            amplitude (np.ndarray | None): Amplitude per pixel (float32).
        """
        self.depth_data = depth
        self.confidence_data = confidence
        self.amplitude_data = amplitude


class OtherFrame:
    """A frame of a type the bridge did not request (not DepthData)."""


class ExternalShutdownException(Exception):
    """Stand-in rclpy.executors.ExternalShutdownException."""


class RCLError(Exception):
    """Stand-in rclpy._rclpy_pybind11.RCLError."""


class FakeCamera:
    """Arducam SDK camera double: return codes and frames are scripted per test."""

    def __init__(self) -> None:
        """Create a camera that opens, starts and serves no frames until frames are queued."""
        self.open_ret = 0
        self.start_ret = 0
        self.range_rets: dict[int, int] = {}
        self.controls: dict[str, int] = {
            "INTRINSIC_FX": INTRINSIC_FX,
            "INTRINSIC_FY": INTRINSIC_FY,
            "INTRINSIC_CX": INTRINSIC_CX,
            "INTRINSIC_CY": INTRINSIC_CY,
        }
        self.frames: list[Any] = []
        self.request_error: BaseException | None = None
        self.range_error: BaseException | None = None
        self.info_error: BaseException | None = None
        self.calls: list[tuple[Any, ...]] = []
        self.released: list[Any] = []

    def open(self, connection: str, index: int) -> int:
        self.calls.append(("open", connection, index))
        return self.open_ret

    def start(self, frame_type: str) -> int:
        self.calls.append(("start", frame_type))
        return self.start_ret

    def stop(self) -> int:
        self.calls.append(("stop",))
        return 0

    def close(self) -> int:
        self.calls.append(("close",))
        return 0

    def setControl(self, control: str, value: int) -> int:
        self.calls.append(("setControl", control, value))
        if control == "RANGE":
            if self.range_error is not None:
                raise self.range_error
            self.controls["RANGE"] = value
            return self.range_rets.get(value, 0)
        return 0

    def getControl(self, control: str) -> int:
        if control not in self.controls:
            raise RuntimeError(f"control {control} not supported")
        return self.controls[control]

    def getCameraInfo(self) -> types.SimpleNamespace:
        if self.info_error is not None:
            raise self.info_error
        return types.SimpleNamespace(width=WIDTH, height=HEIGHT, device_type="HQVGA")

    def requestFrame(self, timeout: int) -> Any:
        self.calls.append(("requestFrame", timeout))
        if self.request_error is not None:
            raise self.request_error
        return self.frames.pop(0) if self.frames else None

    def releaseFrame(self, frame: Any) -> int:
        self.released.append(frame)
        return 0

    def names(self) -> list[Any]:
        """Return the called method names in order.

        Returns:
            list[Any]: First element of each recorded call.
        """
        return [call[0] for call in self.calls]


def install_fakes() -> types.ModuleType:
    """Register the SDK, rclpy and message doubles in sys.modules.

    Returns:
        types.ModuleType: The fake ArducamDepthCamera module.
    """
    sdk = types.ModuleType("ArducamDepthCamera")
    sdk_attrs: dict[str, Any] = {
        "Connection": types.SimpleNamespace(CSI="CSI", USB="USB"),
        "FrameType": types.SimpleNamespace(DEPTH="DEPTH", RAW="RAW"),
        "Control": types.SimpleNamespace(
            RANGE="RANGE",
            INTRINSIC_FX="INTRINSIC_FX",
            INTRINSIC_FY="INTRINSIC_FY",
            INTRINSIC_CX="INTRINSIC_CX",
            INTRINSIC_CY="INTRINSIC_CY",
        ),
        "DepthData": DepthData,
        "ArducamCamera": FakeCamera,
    }
    for attr, value in sdk_attrs.items():
        setattr(sdk, attr, value)
    sys.modules["ArducamDepthCamera"] = sdk

    modules: dict[str, dict[str, Any]] = {
        "rclpy": {
            "init": MagicMock(),
            "ok": MagicMock(return_value=True),
            "spin": MagicMock(),
            "try_shutdown": MagicMock(),
        },
        "rclpy.node": {"Node": MagicMock()},
        "rclpy.executors": {"ExternalShutdownException": ExternalShutdownException},
        "rclpy._rclpy_pybind11": {"RCLError": RCLError},
        "rclpy.time": {"Time": FakeTime},
        "sensor_msgs": {},
        "sensor_msgs.msg": {"Image": Image, "CameraInfo": CameraInfo},
        "std_msgs": {},
        "std_msgs.msg": {"Header": Header},
    }
    for name, attrs in modules.items():
        module = types.ModuleType(name)
        for attr, value in attrs.items():
            setattr(module, attr, value)
        sys.modules.setdefault(name, module)
    return sdk


SDK = install_fakes()

from arducam_tof import bridge  # noqa: E402 - the fakes must be registered first


class Clock:
    """Monotonic clock double advanced by hand."""

    def __init__(self) -> None:
        """Start at t = 100 s."""
        self.now_s = 100.0

    def __call__(self) -> float:
        """Return the current time.

        Returns:
            float: Seconds.
        """
        return self.now_s


class Harness:
    """A TofBridge on a mocked node with one FakeCamera per open attempt."""

    def __init__(self, monkeypatch: pytest.MonkeyPatch, config: TofConfig) -> None:
        """Build the bridge.

        Args:
            monkeypatch (pytest.MonkeyPatch): Patches the SDK camera factory.
            config (TofConfig): Bridge config.
        """
        self.cameras: list[FakeCamera] = []
        self.prepare: Callable[[FakeCamera], None] = lambda _camera: None
        monkeypatch.setattr(SDK, "ArducamCamera", self.new_camera)
        self.publishers: dict[str, MagicMock] = {}
        self.node = MagicMock()
        self.node.create_publisher.side_effect = self.new_publisher
        self.node_clock = NodeClock(CLOCK_NS)
        self.node.get_clock.return_value = self.node_clock
        self.logger = self.node.get_logger.return_value
        self.clock = Clock()
        self.bridge = bridge.TofBridge(self.node, config, clock=self.clock)

    def new_camera(self) -> FakeCamera:
        camera = FakeCamera()
        self.prepare(camera)
        self.cameras.append(camera)
        return camera

    def new_publisher(self, _msg_type: type, topic: str, _qos: int) -> MagicMock:
        publisher = MagicMock(name=topic)
        self.publishers[topic] = publisher
        return publisher

    def published(self, topic: str) -> list[Any]:
        """Return the messages published on a topic.

        Args:
            topic (str): Topic name.

        Returns:
            list[Any]: Published messages in order.
        """
        return [call.args[0] for call in self.publishers[topic].publish.call_args_list]

    def total_published(self) -> int:
        """Count every publish on every topic.

        Returns:
            int: Number of publish calls.
        """
        return sum(publisher.publish.call_count for publisher in self.publishers.values())


def counter(name: str, labels: dict[str, str] | None = None) -> float:
    """Read a counter sample from the default registry (0 when absent).

    Args:
        name (str): Sample name, e.g. "tof_frames_published_total".
        labels (dict[str, str] | None): Sample labels.

    Returns:
        float: Current value.
    """
    return REGISTRY.get_sample_value(name, labels or {}) or 0.0


def good_frame() -> DepthData:
    """Build a 3x4 frame whose confidence is below 30 in one pixel.

    Returns:
        DepthData: The frame.
    """
    depth = np.arange(1, WIDTH * HEIGHT + 1, dtype=np.float32).reshape(HEIGHT, WIDTH) * 100.4
    confidence = np.full((HEIGHT, WIDTH), 80.0, dtype=np.float32)
    confidence[1, 2] = 29.9
    amplitude = np.full((HEIGHT, WIDTH), 300.6, dtype=np.float32)
    return DepthData(depth, confidence, amplitude)


@pytest.fixture
def harness(monkeypatch: pytest.MonkeyPatch) -> Harness:
    """Bridge on the default config with the flying pixel filter off (the fixture frames are steep ramps).

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches the SDK camera factory.

    Returns:
        Harness: The harness.
    """
    return Harness(monkeypatch, TofConfig(flying_pixel_filter=False))


def test_topics_and_encodings_are_the_shared_contract() -> None:
    assert bridge.DEPTH_TOPIC == "/tof/depth/image_raw"
    assert bridge.AMPLITUDE_TOPIC == "/tof/amplitude/image_raw"
    assert bridge.CAMERA_INFO_TOPIC == "/tof/camera_info"
    assert bridge.DEPTH_ENCODING == "16UC1"
    assert bridge.AMPLITUDE_ENCODING == "mono16"


def test_depth_below_confidence_threshold_is_zero_and_rest_is_rounded_millimetres() -> None:
    depth = np.array([[1000.4, 1000.6], [250.0, 3999.5]], dtype=np.float32)
    confidence = np.array([[50.0, 29.0], [30.0, 100.0]], dtype=np.float32)

    out = bridge.depth_to_mm(depth, confidence, 30.0)

    assert out.dtype == np.uint16
    assert out.tolist() == [[1000, 0], [250, 4000]]


def test_depth_that_is_not_a_finite_in_range_distance_is_zero() -> None:
    depth = np.array([[np.nan, np.inf, -5.0, 70000.0, 0.4, 65535.0]], dtype=np.float32)
    confidence = np.full(depth.shape, 255.0, dtype=np.float32)

    assert bridge.depth_to_mm(depth, confidence, 0.0).tolist() == [[0, 0, 0, 0, 0, 65535]]


def test_amplitude_is_rounded_and_clipped_to_mono16() -> None:
    amplitude = np.array([[0.4, 300.6, -3.0, 1e6, np.nan]], dtype=np.float32)

    out = bridge.amplitude_to_mono16(amplitude)

    assert out.dtype == np.uint16
    assert out.tolist() == [[0, 301, 0, 65535, 0]]


def test_publish_timer_runs_at_publish_rate(monkeypatch: pytest.MonkeyPatch) -> None:
    h = Harness(monkeypatch, TofConfig(publish_rate_hz=4.0))

    period_s = h.node.create_timer.call_args.args[0]
    assert period_s == pytest.approx(0.25)
    assert h.node.create_timer.call_args.args[1] == h.bridge.step


def test_good_frame_publishes_depth_amplitude_and_camera_info_with_one_header(harness: Harness) -> None:
    harness.prepare = lambda camera: camera.frames.append(good_frame())
    published_before = counter("tof_frames_published_total")

    harness.bridge.step()

    camera = harness.cameras[0]
    assert ("open", "CSI", 0) in camera.calls
    assert ("start", "DEPTH") in camera.calls
    assert ("setControl", "RANGE", 4000) in camera.calls
    assert camera.released == [camera.released[0]] and isinstance(camera.released[0], DepthData)

    [depth] = harness.published(bridge.DEPTH_TOPIC)
    [amplitude] = harness.published(bridge.AMPLITUDE_TOPIC)
    [info] = harness.published(bridge.CAMERA_INFO_TOPIC)
    assert depth.header is amplitude.header is info.header
    assert depth.header.stamp == ("stamp", CLOCK_NS)
    assert depth.header.frame_id == "tof_optical_frame"

    assert (depth.encoding, depth.height, depth.width, depth.step, depth.is_bigendian) == ("16UC1", 3, 4, 8, 0)
    pixels = np.frombuffer(bytes(depth.data), dtype="<u2").reshape(HEIGHT, WIDTH)
    radial = np.arange(1, 13, dtype=np.float32).reshape(HEIGHT, WIDTH) * 100.4
    fx, fy, cx, cy = (v / 100 for v in (INTRINSIC_FX, INTRINSIC_FY, INTRINSIC_CX, INTRINSIC_CY))
    rows, cols = np.mgrid[0:HEIGHT, 0:WIDTH]
    ray = np.sqrt(((cols - cx) / fx) ** 2 + ((rows - cy) / fy) ** 2 + 1)
    expected = np.rint(np.rint(radial) / ray).astype(np.uint16)
    expected[1, 2] = 0
    assert expected.max() < np.rint(radial).max()  # off-centre pixels really are scaled down
    assert pixels.tolist() == expected.tolist()

    assert (amplitude.encoding, amplitude.height, amplitude.width, amplitude.step) == ("mono16", 3, 4, 8)
    assert set(np.frombuffer(bytes(amplitude.data), dtype="<u2").tolist()) == {301}

    assert (info.width, info.height) == (WIDTH, HEIGHT)
    assert info.k == pytest.approx([213.45, 0.0, 119.5, 0.0, 212.9, 89.75, 0.0, 0.0, 1.0])
    assert info.p == pytest.approx([213.45, 0.0, 119.5, 0.0, 0.0, 212.9, 89.75, 0.0, 0.0, 0.0, 1.0, 0.0])
    assert info.r == pytest.approx([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert info.distortion_model == "plumb_bob"
    assert info.d == [0.0] * 5
    assert counter("tof_frames_published_total") == published_before + 1


def test_frame_id_and_near_range_come_from_config(monkeypatch: pytest.MonkeyPatch) -> None:
    h = Harness(monkeypatch, TofConfig(frame_id="head_tof", range_mode="near", camera_index=1))
    h.prepare = lambda camera: camera.frames.append(good_frame())

    h.bridge.step()

    assert ("open", "CSI", 1) in h.cameras[0].calls
    assert ("setControl", "RANGE", 2000) in h.cameras[0].calls
    assert h.published(bridge.DEPTH_TOPIC)[0].header.frame_id == "head_tof"


def test_open_uses_the_index_resolved_from_csi_device(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(bridge, "resolve_camera_index", lambda config: 10)
    h = Harness(monkeypatch, TofConfig(camera_index=8, csi_device="1f00128000.csi"))
    h.prepare = lambda camera: camera.frames.append(good_frame())

    h.bridge.step()

    assert ("open", "CSI", 10) in h.cameras[0].calls


def test_unresolvable_csi_device_is_an_open_failure_that_retries(monkeypatch: pytest.MonkeyPatch) -> None:
    def missing(_config: TofConfig) -> int:
        raise LookupError("no csi2_ch0 video node for 1f00128000.csi")

    monkeypatch.setattr(bridge, "resolve_camera_index", missing)
    h = Harness(monkeypatch, TofConfig(csi_device="1f00128000.csi"))

    h.bridge.step()

    assert h.bridge.camera is None
    assert "1f00128000.csi" in h.logger.error.call_args[0][0]


def test_range_in_metres_is_tried_when_the_sdk_rejects_millimetres(harness: Harness) -> None:
    def reject_mm(camera: FakeCamera) -> None:
        camera.range_rets[4000] = -1
        camera.frames.append(good_frame())

    harness.prepare = reject_mm

    harness.bridge.step()

    ranges = [call[2] for call in harness.cameras[0].calls if call[:2] == ("setControl", "RANGE")]
    assert ranges == [4000, 4]
    assert len(harness.published(bridge.DEPTH_TOPIC)) == 1


def test_amplitude_falls_back_to_confidence_when_the_sdk_has_no_amplitude(harness: Harness) -> None:
    frame = good_frame()
    frame.amplitude_data = None
    harness.prepare = lambda camera: camera.frames.append(frame)

    harness.bridge.step()

    [amplitude] = harness.published(bridge.AMPLITUDE_TOPIC)
    values = np.frombuffer(bytes(amplitude.data), dtype="<u2").reshape(HEIGHT, WIDTH)
    assert values[0, 0] == 80 and values[1, 2] == 30


def test_missing_frame_publishes_nothing_logs_and_counts(harness: Harness) -> None:
    before = counter("tof_frames_dropped_total", {"reason": "no_frame"})

    harness.bridge.step()

    assert harness.total_published() == 0
    assert harness.logger.warning.called
    assert counter("tof_frames_dropped_total", {"reason": "no_frame"}) == before + 1


def test_frame_request_error_publishes_nothing_logs_and_counts(harness: Harness) -> None:
    harness.prepare = lambda camera: setattr(camera, "request_error", RuntimeError("i2c timeout"))
    before = counter("tof_frames_dropped_total", {"reason": "error"})

    harness.bridge.step()

    assert harness.total_published() == 0
    assert "i2c timeout" in str(harness.logger.warning.call_args)
    assert counter("tof_frames_dropped_total", {"reason": "error"}) == before + 1


def test_frame_of_another_type_is_released_and_not_published(harness: Harness) -> None:
    other = OtherFrame()
    harness.prepare = lambda camera: camera.frames.append(other)
    before = counter("tof_frames_dropped_total", {"reason": "wrong_type"})

    harness.bridge.step()

    assert harness.total_published() == 0
    assert harness.cameras[0].released == [other]
    assert counter("tof_frames_dropped_total", {"reason": "wrong_type"}) == before + 1


@pytest.mark.parametrize(
    "frame",
    [
        DepthData(None, np.ones((HEIGHT, WIDTH), dtype=np.float32)),
        DepthData(np.ones((HEIGHT, WIDTH), dtype=np.float32), None),
        DepthData(np.ones((HEIGHT, WIDTH), dtype=np.float32), np.ones((HEIGHT, WIDTH + 1), dtype=np.float32)),
        DepthData(np.ones(WIDTH, dtype=np.float32), np.ones(WIDTH, dtype=np.float32)),
        DepthData(np.ones((0, 0), dtype=np.float32), np.ones((0, 0), dtype=np.float32)),
        DepthData(
            np.ones((HEIGHT, WIDTH), dtype=np.float32),
            np.ones((HEIGHT, WIDTH), dtype=np.float32),
            np.ones((1, 1), dtype=np.float32),
        ),
    ],
)
def test_malformed_frame_is_released_and_not_published(harness: Harness, frame: DepthData) -> None:
    harness.prepare = lambda camera: camera.frames.append(frame)
    before = counter("tof_frames_dropped_total", {"reason": "invalid"})

    harness.bridge.step()

    assert harness.total_published() == 0
    assert harness.cameras[0].released == [frame]
    assert counter("tof_frames_dropped_total", {"reason": "invalid"}) == before + 1


def test_camera_that_fails_to_open_logs_and_retries_without_publishing(harness: Harness) -> None:
    harness.prepare = lambda camera: setattr(camera, "open_ret", -1)
    before = counter("tof_camera_open_failures_total")

    harness.bridge.step()
    harness.bridge.step()  # still inside the retry delay: no second attempt

    assert len(harness.cameras) == 1
    assert harness.total_published() == 0
    assert harness.logger.error.called
    assert counter("tof_camera_open_failures_total") == before + 1

    harness.clock.now_s += bridge.OPEN_RETRY_S
    harness.prepare = lambda camera: camera.frames.append(good_frame())
    harness.bridge.step()

    assert len(harness.cameras) == 2
    assert len(harness.published(bridge.DEPTH_TOPIC)) == 1


def test_camera_that_fails_to_start_is_closed_and_retried(harness: Harness) -> None:
    harness.prepare = lambda camera: setattr(camera, "start_ret", -1)
    before = counter("tof_camera_open_failures_total")

    harness.bridge.step()

    assert harness.cameras[0].names()[-1] == "close"
    assert harness.total_published() == 0
    assert counter("tof_camera_open_failures_total") == before + 1

    harness.clock.now_s += bridge.OPEN_RETRY_S
    harness.bridge.step()
    assert len(harness.cameras) == 2


@pytest.mark.parametrize("fault", ["range_error", "info_error"])
def test_camera_that_fails_after_start_is_stopped_closed_and_retried(harness: Harness, fault: str) -> None:
    harness.prepare = lambda camera: setattr(camera, fault, RuntimeError("SDK error"))
    before = counter("tof_camera_open_failures_total")

    harness.bridge.step()

    assert harness.cameras[0].names()[-2:] == ["stop", "close"]
    assert harness.bridge.camera is None
    assert harness.total_published() == 0
    assert harness.logger.error.called
    assert counter("tof_camera_open_failures_total") == before + 1

    harness.clock.now_s += bridge.OPEN_RETRY_S
    harness.prepare = lambda camera: camera.frames.append(good_frame())
    harness.bridge.step()
    assert len(harness.cameras) == 2
    assert len(harness.published(bridge.DEPTH_TOPIC)) == 1


def test_camera_constructor_error_is_an_open_failure(monkeypatch: pytest.MonkeyPatch, harness: Harness) -> None:
    def broken() -> FakeCamera:
        raise RuntimeError("no /dev/video0")

    monkeypatch.setattr(SDK, "ArducamCamera", broken)

    harness.bridge.step()

    assert harness.total_published() == 0
    assert "no /dev/video0" in str(harness.logger.error.call_args)


def test_camera_info_is_skipped_when_the_sdk_has_no_intrinsics(harness: Harness) -> None:
    def no_intrinsics(camera: FakeCamera) -> None:
        camera.controls["INTRINSIC_FX"] = 0
        camera.frames.append(good_frame())

    harness.prepare = no_intrinsics

    harness.bridge.step()

    assert len(harness.published(bridge.DEPTH_TOPIC)) == 1
    assert len(harness.published(bridge.AMPLITUDE_TOPIC)) == 1
    assert harness.published(bridge.CAMERA_INFO_TOPIC) == []


def test_principal_point_defaults_to_the_image_centre_without_cx_cy(harness: Harness) -> None:
    def no_centre(camera: FakeCamera) -> None:
        del camera.controls["INTRINSIC_CX"]
        del camera.controls["INTRINSIC_CY"]
        camera.frames.append(good_frame())

    harness.prepare = no_centre

    harness.bridge.step()

    [info] = harness.published(bridge.CAMERA_INFO_TOPIC)
    assert (info.k[2], info.k[5]) == (WIDTH / 2, HEIGHT / 2)


def test_camera_without_frames_for_reopen_delay_is_closed_and_reopened(harness: Harness) -> None:
    before = counter("tof_camera_reopens_total")
    harness.bridge.step()
    harness.clock.now_s += bridge.REOPEN_AFTER_S - 0.1
    harness.bridge.step()
    assert len(harness.cameras) == 1

    harness.clock.now_s += 0.2
    harness.bridge.step()

    assert harness.cameras[0].names()[-2:] == ["stop", "close"]
    assert counter("tof_camera_reopens_total") == before + 1
    harness.bridge.step()
    assert len(harness.cameras) == 2
    assert harness.total_published() == 0


def test_close_stops_and_closes_an_open_camera(harness: Harness) -> None:
    harness.bridge.step()

    harness.bridge.close()
    harness.bridge.close()

    assert harness.cameras[0].names()[-2:] == ["stop", "close"]
    assert harness.cameras[0].names().count("close") == 1


@pytest.mark.parametrize("error", [KeyboardInterrupt(), ExternalShutdownException()])
def test_run_shuts_down_cleanly_on_stop(monkeypatch: pytest.MonkeyPatch, error: BaseException) -> None:
    rclpy = MagicMock()
    rclpy.spin.side_effect = error
    node_cls = MagicMock()
    start_metrics = MagicMock()
    monkeypatch.setattr(bridge, "rclpy", rclpy)
    monkeypatch.setattr(bridge, "Node", node_cls)
    monkeypatch.setattr(bridge, "start_metrics_server", start_metrics)
    monkeypatch.setattr(bridge, "resolve_metrics_port", lambda port: port)
    closed: list[bool] = []
    monkeypatch.setattr(bridge.TofBridge, "close", lambda _self: closed.append(True))

    bridge.run(TofConfig(metrics_port=19120))

    start_metrics.assert_called_once_with(19120, bridge.NODE_NAME)
    assert closed == [True]
    node_cls.return_value.destroy_node.assert_called_once()
    rclpy.try_shutdown.assert_called_once()


def test_main_returns_1_on_config_error(monkeypatch: pytest.MonkeyPatch, tmp_path: Any) -> None:
    from arducam_tof import __main__ as entry

    monkeypatch.setenv("ARDUCAM_TOF_CONFIG", str(tmp_path / "absent.yaml"))
    run = MagicMock()
    monkeypatch.setattr(entry, "run", run)

    assert entry.main() == 1
    run.assert_not_called()


def test_main_runs_the_bridge_with_the_loaded_config(monkeypatch: pytest.MonkeyPatch, tmp_path: Any) -> None:
    from arducam_tof import __main__ as entry

    path = tmp_path / "config.yaml"
    path.write_text("range_mode: near\n")
    monkeypatch.setenv("ARDUCAM_TOF_CONFIG", str(path))
    run = MagicMock()
    monkeypatch.setattr(entry, "run", run)

    assert entry.main() == 0
    assert run.call_args.args[0].range_mode == "near"


STAMP_LATENCY_MS = 40.0
CONVERSION_NS = 300_000_000


def advance_during_conversion(harness: Harness, monkeypatch: pytest.MonkeyPatch) -> None:
    """Make the depth conversion step move the node clock, as slow conversions would.

    Args:
        harness (Harness): The harness whose node clock advances.
        monkeypatch (pytest.MonkeyPatch): Patches bridge.depth_to_mm.
    """
    real = bridge.depth_to_mm

    def slow(*args: Any) -> np.ndarray:
        harness.node_clock.nanoseconds += CONVERSION_NS
        return real(*args)

    monkeypatch.setattr(bridge, "depth_to_mm", slow)


@pytest.mark.parametrize(
    ("latency_ms", "expected_ns"),
    [(STAMP_LATENCY_MS, CLOCK_NS - 40_000_000), (0.0, CLOCK_NS), (10_000.0, CLOCK_NS)],
)
def test_stamp_is_the_clock_reading_after_request_frame_minus_latency(
    monkeypatch: pytest.MonkeyPatch, latency_ms: float, expected_ns: int
) -> None:
    config = TofConfig.model_construct(**{**TofConfig().model_dump(), "stamp_latency_ms": latency_ms})
    harness = Harness(monkeypatch, config)
    harness.prepare = lambda camera: camera.frames.append(good_frame())
    advance_during_conversion(harness, monkeypatch)

    harness.bridge.step()

    [depth] = harness.published(bridge.DEPTH_TOPIC)
    [amplitude] = harness.published(bridge.AMPLITUDE_TOPIC)
    [info] = harness.published(bridge.CAMERA_INFO_TOPIC)
    assert depth.header is amplitude.header is info.header
    assert depth.header.stamp == ("stamp", expected_ns)


def test_clamped_stamp_is_logged_once(monkeypatch: pytest.MonkeyPatch) -> None:
    config = TofConfig.model_construct(**{**TofConfig().model_dump(), "stamp_latency_ms": 10_000.0})
    harness = Harness(monkeypatch, config)
    harness.prepare = lambda camera: camera.frames.extend([good_frame(), good_frame()])
    harness.bridge.step()
    harness.bridge.step()
    clamp_logs = [c for c in harness.logger.warning.call_args_list if "stamp_latency_ms" in c.args[0]]
    assert len(clamp_logs) == 1


def test_startup_log_names_stamp_latency(monkeypatch: pytest.MonkeyPatch) -> None:
    config = TofConfig.model_construct(**{**TofConfig().model_dump(), "stamp_latency_ms": 12.5})
    harness = Harness(monkeypatch, config)
    assert "stamp_latency_ms=12.5" in harness.logger.info.call_args_list[0].args[0]


FIXTURE = Path(__file__).parent / "data" / "floor_capture.json"
SDK_FRAME_W = 240
SDK_FRAME_H = 180


def write_calibration(tmp_path: Path, k: list[float], width: int = WIDTH, height: int = HEIGHT) -> Path:
    """Write a camera_calibration YAML (with distortion coefficients that must be ignored).

    Args:
        tmp_path (Path): Test directory.
        k (list[float]): fx, fy, cx, cy.
        width (int): image_width.
        height (int): image_height.

    Returns:
        Path: The written file.
    """
    fx, fy, cx, cy = k
    path = tmp_path / "tof.yaml"
    path.write_text(
        f"image_width: {width}\nimage_height: {height}\ncamera_name: tof\n"
        f"camera_matrix: {{rows: 3, cols: 3, data: [{fx}, 0.0, {cx}, 0.0, {fy}, {cy}, 0.0, 0.0, 1.0]}}\n"
        "distortion_model: plumb_bob\ndistortion_coefficients: {rows: 1, cols: 5, data: [0.5, 0.5, 0.1, 0.1, 0.0]}\n"
    )
    return path


def run_one_frame(monkeypatch: pytest.MonkeyPatch, config: TofConfig) -> Harness:
    """Run one bridge step with good_frame().

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches the SDK camera factory.
        config (TofConfig): Bridge config.

    Returns:
        Harness: The harness after the step.
    """
    h = Harness(monkeypatch, config)
    h.prepare = lambda camera: camera.frames.append(good_frame())
    h.bridge.step()
    return h


def test_radial_depth_becomes_z_depth() -> None:
    k = bridge.Intrinsics(207.787, 208.171, 114.743, 95.996)
    rows, cols = np.mgrid[0:SDK_FRAME_H, 0:SDK_FRAME_W]
    rx, ry = (cols - k.cx) / k.fx, (rows - k.cy) / k.fy
    z_true = 1000.0
    radial = np.rint(z_true * np.sqrt(rx**2 + ry**2 + 1)).astype(np.uint16)
    radial[0, 0] = 0
    out = bridge.radial_to_z(radial, k)
    assert out.dtype == np.uint16 and out.shape == radial.shape
    assert out[0, 0] == 0
    valid = radial > 0
    assert np.abs(out[valid].astype(float) - z_true).max() <= 1.0
    centre = np.full((3, 3), 1234, dtype=np.uint16)
    assert bridge.radial_to_z(centre, bridge.Intrinsics(100.0, 100.0, 1.0, 1.0))[1, 1] == 1234
    assert bridge.ray_norm(np.array([k.cx]), np.array([k.cy]), k)[0] == pytest.approx(1.0)


def test_radial_to_z_factor_is_cached_per_shape_and_intrinsics(monkeypatch: pytest.MonkeyPatch) -> None:
    calls = []
    real = bridge.ray_norm
    monkeypatch.setattr(bridge, "ray_norm", lambda u, v, i: calls.append(i) or real(u, v, i))
    bridge.ray_norm_image.cache_clear()
    depth = np.full((5, 6), 500, dtype=np.uint16)
    k = bridge.Intrinsics(50.0, 50.0, 3.0, 2.0)
    bridge.radial_to_z(depth, k)
    bridge.radial_to_z(depth, k)
    assert len(calls) == 1
    bridge.radial_to_z(depth, bridge.Intrinsics(60.0, 50.0, 3.0, 2.0))
    assert len(calls) == 2


def floor_residual_dome_cm(points_cam: np.ndarray, pose: dict[str, float]) -> float:
    """Fit a plane to the floor 0.3-0.9 m ahead in map and return median residual centre columns minus edges, in cm.

    Args:
        points_cam (np.ndarray): (n, 3) xyz in tof_optical_frame.
        pose (dict[str, float]): frame_pose of the frame in map (x, y, z, qx, qy, qz, qw).

    Returns:
        float: Dome height in centimetres.
    """
    sdk = np.array(CAPTURE["sdk_k"])
    x, y, z = points_cam.T
    u = sdk[0] * x / z + sdk[2]
    qx, qy, qz, qw = (pose[k] for k in ("qx", "qy", "qz", "qw"))
    rot = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )
    origin = np.array([pose["x"], pose["y"], pose["z"]])
    world = points_cam @ rot.T + origin
    forward = rot[:, 2].copy()
    forward[2] = 0
    forward /= np.linalg.norm(forward)
    ahead = (world[:, :2] - origin[:2]) @ forward[:2]
    sel = (ahead > 0.3) & (ahead < 0.9) & (world[:, 2] < 0.15)
    pts, cols = world[sel], u[sel]
    design = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    keep = np.ones(len(pts), bool)
    for _ in range(3):
        coef, *_ = np.linalg.lstsq(design[keep], pts[keep, 2], rcond=None)
        res = pts[:, 2] - design @ coef
        keep = np.abs(res) < 3 * max(np.std(res[keep]), 0.005)
    res = pts[:, 2] - design @ coef
    centre = np.abs(cols - 120) < 30
    edge = np.abs(cols - 120) > 70
    assert centre.sum() > 10 and edge.sum() > 10
    return float((np.median(res[centre]) - np.median(res[edge])) * 100)


CAPTURE = json.loads(FIXTURE.read_text())


def test_floor_capture_dome_is_flat_after_radial_to_z() -> None:
    pts = np.array(CAPTURE["points"], dtype=float).reshape(-1, 3)
    pose = CAPTURE["frame_pose"]
    assert floor_residual_dome_cm(pts, pose) > 2.0  # as captured: SDK radial value taken as z

    sdk = np.array(CAPTURE["sdk_k"])
    cal = bridge.Intrinsics(*CAPTURE["calibrated_k"])
    x, y, z = pts.T
    u = np.rint(sdk[0] * x / z + sdk[2]).astype(int)
    v = np.rint(sdk[1] * y / z + sdk[3]).astype(int)
    image = np.zeros((SDK_FRAME_H, SDK_FRAME_W), dtype=np.uint16)
    image[v, u] = np.rint(z * 1000).astype(np.uint16)
    zmm = bridge.radial_to_z(image, cal)[v, u].astype(float) / 1000
    xn, yn = (u - cal.cx) / cal.fx, (v - cal.cy) / cal.fy
    fixed = np.c_[xn * zmm, yn * zmm, zmm]
    assert floor_residual_dome_cm(fixed, pose) < 1.0


def test_camera_info_uses_the_calibration_file_k(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    k = [207.787, 208.171, 1.9, 1.4]
    h = run_one_frame(
        monkeypatch, TofConfig(flying_pixel_filter=False, tof_calibration_file=write_calibration(tmp_path, k))
    )
    [info] = h.published(bridge.CAMERA_INFO_TOPIC)
    assert info.k == pytest.approx([k[0], 0.0, k[2], 0.0, k[1], k[3], 0.0, 0.0, 1.0])
    assert info.p == pytest.approx([k[0], 0.0, k[2], 0.0, 0.0, k[1], k[3], 0.0, 0.0, 0.0, 1.0, 0.0])
    assert info.d == [0.0] * 5  # the file's distortion coefficients are not used
    [depth] = h.published(bridge.DEPTH_TOPIC)
    pixels = np.frombuffer(bytes(depth.data), dtype="<u2").reshape(HEIGHT, WIDTH)
    radial = np.rint(np.arange(1, 13, dtype=np.float32).reshape(HEIGHT, WIDTH) * 100.4)
    rows, cols = np.mgrid[0:HEIGHT, 0:WIDTH]
    ray = np.sqrt(((cols - k[2]) / k[0]) ** 2 + ((rows - k[3]) / k[1]) ** 2 + 1)
    expected = np.rint(radial / ray).astype(np.uint16)
    expected[1, 2] = 0
    assert pixels.tolist() == expected.tolist()


def test_missing_calibration_file_falls_back_to_sdk_k(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    h = run_one_frame(monkeypatch, TofConfig(flying_pixel_filter=False, tof_calibration_file=tmp_path / "absent.yaml"))
    [info] = h.published(bridge.CAMERA_INFO_TOPIC)
    assert info.k[0] == pytest.approx(INTRINSIC_FX / 100)
    assert info.k[2] == pytest.approx(INTRINSIC_CX / 100)
    h.logger.warning.assert_called()


def test_invalid_calibration_file_falls_back_to_sdk_k(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text("camera_matrix: {data: [1, 2]}\n")
    h = run_one_frame(monkeypatch, TofConfig(flying_pixel_filter=False, tof_calibration_file=path))
    [info] = h.published(bridge.CAMERA_INFO_TOPIC)
    assert info.k[0] == pytest.approx(INTRINSIC_FX / 100)
    h.logger.warning.assert_called()


def test_calibration_file_of_another_size_falls_back_to_sdk_k(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    path = write_calibration(tmp_path, [207.787, 208.171, 114.7, 96.0], width=240, height=180)
    h = run_one_frame(monkeypatch, TofConfig(flying_pixel_filter=False, tof_calibration_file=path))
    [info] = h.published(bridge.CAMERA_INFO_TOPIC)
    assert info.k[0] == pytest.approx(INTRINSIC_FX / 100)
    h.logger.error.assert_called()


def test_calibration_file_gives_k_without_sdk_focal_length(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    k = [207.787, 208.171, 1.9, 1.4]
    h = Harness(monkeypatch, TofConfig(tof_calibration_file=write_calibration(tmp_path, k)))

    def no_intrinsics(camera: FakeCamera) -> None:
        camera.controls["INTRINSIC_FX"] = 0
        camera.frames.append(good_frame())

    h.prepare = no_intrinsics
    h.bridge.step()
    [info] = h.published(bridge.CAMERA_INFO_TOPIC)
    assert info.k[0] == pytest.approx(k[0])


def test_depth_is_radial_false_publishes_depth_unchanged(monkeypatch: pytest.MonkeyPatch) -> None:
    h = run_one_frame(monkeypatch, TofConfig(flying_pixel_filter=False, depth_is_radial=False))
    [depth] = h.published(bridge.DEPTH_TOPIC)
    pixels = np.frombuffer(bytes(depth.data), dtype="<u2").reshape(HEIGHT, WIDTH)
    expected = np.rint(np.arange(1, 13, dtype=np.float32).reshape(HEIGHT, WIDTH) * 100.4).astype(np.uint16)
    expected[1, 2] = 0
    assert pixels.tolist() == expected.tolist()
    assert len(h.published(bridge.CAMERA_INFO_TOPIC)) == 1


def test_depth_is_unconverted_when_there_are_no_intrinsics(harness: Harness) -> None:
    def no_intrinsics(camera: FakeCamera) -> None:
        camera.controls["INTRINSIC_FX"] = 0
        camera.frames.append(good_frame())

    harness.prepare = no_intrinsics
    harness.bridge.step()
    [depth] = harness.published(bridge.DEPTH_TOPIC)
    pixels = np.frombuffer(bytes(depth.data), dtype="<u2").reshape(HEIGHT, WIDTH)
    assert pixels[0, 0] == 100 and pixels[2, 3] == 1205


CAMERA_HEIGHT_M = 0.45
CAMERA_PITCH_RAD = np.radians(30.0)
BOX_NEAR_M = 0.60
BOX_FAR_M = 0.78
BOX_HEIGHT_M = 0.05
SCENE_K = bridge.Intrinsics(207.787, 208.171, 114.743, 95.996)
FLOOR_NEAR_M = 0.3
FLOOR_FAR_M = 2.0
FILTER_DEFAULTS = TofConfig()


def render_scene(box: bool, noise_mm: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Render z-depth of a floor (optionally with a 5 cm box) seen from 0.45 m up, pitched 30 deg down.

    Args:
        box (bool): Add the box top (x 0.60-0.78 m ahead) and its front face.
        noise_mm (float): Standard deviation of the depth noise in millimetres.

    Returns:
        tuple[np.ndarray, np.ndarray]: uint16 depth (mm, 0 = no surface in 0.3-2.0 m) of 180x240 and the
        float forward distance of each pixel's hit point.
    """
    rows, _cols = np.mgrid[0:SDK_FRAME_H, 0:SDK_FRAME_W]
    yn = (rows - SCENE_K.cy) / SCENE_K.fy
    sin_a, cos_a = np.sin(CAMERA_PITCH_RAD), np.cos(CAMERA_PITCH_RAD)
    fwd_per_t = cos_a - yn * sin_a
    drop_per_t = sin_a + yn * cos_a  # downward metres per unit z-depth
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(drop_per_t > 0, CAMERA_HEIGHT_M / drop_per_t, np.inf)
        x = t * fwd_per_t
        if box:
            t_top = (CAMERA_HEIGHT_M - BOX_HEIGHT_M) / drop_per_t
            x_top = t_top * fwd_per_t
            on_top = (drop_per_t > 0) & (x_top >= BOX_NEAR_M) & (x_top <= BOX_FAR_M)
            t_face = BOX_NEAR_M / fwd_per_t
            on_face = (drop_per_t > 0) & (x_top < BOX_NEAR_M) & (x > BOX_NEAR_M)
            t = np.where(on_top, t_top, np.where(on_face, t_face, t))
            x = np.where(on_top, x_top, np.where(on_face, BOX_NEAR_M, x))
    in_range = np.isfinite(t) & (x >= FLOOR_NEAR_M) & (x <= FLOOR_FAR_M)
    depth = np.rint(np.where(in_range, t, 0.0) * 1000.0)
    if noise_mm:
        depth = depth + np.random.default_rng(0).normal(0.0, noise_mm, depth.shape) * in_range
    return np.where(in_range, np.maximum(depth, 1.0), 0.0).astype(np.uint16), np.where(in_range, x, 0.0)


def box_edge_rows(depth: np.ndarray, x: np.ndarray) -> tuple[int, np.ndarray]:
    """Find the box's far top edge row and the mask of box-top pixels.

    Args:
        depth (np.ndarray): Rendered depth.
        x (np.ndarray): Forward distance of each hit.

    Returns:
        tuple[int, np.ndarray]: Row of the farthest box-top pixels and the box-top mask.
    """
    top = (x >= BOX_NEAR_M) & (x <= BOX_FAR_M) & (depth > 0)
    return int(np.flatnonzero(top[:, int(SCENE_K.cx)]).min()), top


def test_flying_pixel_filter_keeps_tilted_floor() -> None:
    depth, _x = render_scene(box=False, noise_mm=5.0)
    valid = depth > 0
    assert valid.sum() > 5000
    out = bridge.flying_pixel_filter(
        depth, FILTER_DEFAULTS.flying_pixel_jump_ratio, FILTER_DEFAULTS.flying_pixel_min_jump_mm
    )
    assert out.dtype == np.uint16 and out.shape == depth.shape
    assert (out[valid] > 0).mean() >= 0.95
    assert (out[~valid] == 0).all()
    assert (out[out > 0] == depth[out > 0]).all()


def test_flying_pixel_filter_removes_mixed_pixels_behind_box() -> None:
    depth, x = render_scene(box=True)
    edge, top = box_edge_rows(depth, x)
    edge_depth = depth[edge].astype(float)
    behind = depth[edge - 3].astype(float)  # true floor depth behind the box, clear of the band
    mixed = depth.copy()
    band = np.zeros(depth.shape, dtype=bool)
    for row, weight in ((edge - 1, 0.4), (edge - 2, 0.6)):
        mixed[row] = np.rint(edge_depth * (1 - weight) + behind * weight)
        band[row] = depth[row] > 0
    assert band.sum() > 400
    out = bridge.flying_pixel_filter(
        mixed, FILTER_DEFAULTS.flying_pixel_jump_ratio, FILTER_DEFAULTS.flying_pixel_min_jump_mm
    )
    assert (out[band] == 0).mean() >= 0.95
    far_from_edge = np.zeros(depth.shape, dtype=bool)
    far_from_edge[: edge - 4] = True
    far_from_edge[edge + 3 :] = True
    keep = (mixed > 0) & ~band & far_from_edge
    assert (out[keep] > 0).mean() >= 0.95
    assert (out[keep & (out > 0)] == mixed[keep & (out > 0)]).all()


def test_flying_pixel_filter_keeps_invalid_pixels_invalid() -> None:
    depth = np.full((5, 5), 1000, dtype=np.uint16)
    depth[2, 2] = 0
    out = bridge.flying_pixel_filter(depth, 0.05, 30)
    assert out[2, 2] == 0
    assert (out[depth > 0] == 1000).all()
    lone = np.zeros((4, 4), dtype=np.uint16)
    lone[0, 0], lone[3, 3] = 500, 3000
    assert bridge.flying_pixel_filter(lone, 0.05, 30)[0, 0] == 500  # a far valid pixel is not a neighbour
    adjacent = np.zeros((4, 4), dtype=np.uint16)
    adjacent[1, 1], adjacent[1, 2] = 500, 3000
    assert not bridge.flying_pixel_filter(adjacent, 0.05, 30).any()


def test_published_depth_has_flying_pixels_removed(monkeypatch: pytest.MonkeyPatch) -> None:
    def frame_with_band() -> DepthData:
        depth = np.full((HEIGHT, WIDTH), 1000.0, dtype=np.float32)
        depth[1, 1] = 1400.0
        return DepthData(
            depth, np.full((HEIGHT, WIDTH), 80.0, dtype=np.float32), np.full((HEIGHT, WIDTH), 300.0, dtype=np.float32)
        )

    results = {}
    for enabled in (True, False):
        before = counter("tof_flying_pixels_removed_total")
        h = Harness(monkeypatch, TofConfig(depth_is_radial=False, flying_pixel_filter=enabled))
        h.prepare = lambda camera: camera.frames.append(frame_with_band())
        h.bridge.step()
        [msg] = h.published(bridge.DEPTH_TOPIC)
        pixels = np.frombuffer(bytes(msg.data), dtype="<u2").reshape(HEIGHT, WIDTH)
        results[enabled] = (pixels, counter("tof_flying_pixels_removed_total") - before)
    pixels, removed = results[True]
    assert removed == 9
    assert (pixels[:, :3] == 0).all() and (pixels[:, 3] == 1000).all()
    pixels, removed = results[False]
    assert removed == 0
    assert pixels[1, 1] == 1400 and pixels[0, 0] == 1000
