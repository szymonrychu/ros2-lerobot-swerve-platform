"""Unit tests for the arducam_tof bridge with the Arducam SDK, rclpy and the ROS message packages faked.

The SDK (ArducamDepthCamera) has aarch64 Linux wheels only and rclpy is not installed on dev machines, so both are
replaced by small doubles registered in sys.modules before arducam_tof.bridge is imported.
"""

import sys
import types
from collections.abc import Callable
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
STAMP = "stamp-sentinel"


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
            self.controls["RANGE"] = value
            return self.range_rets.get(value, 0)
        return 0

    def getControl(self, control: str) -> int:
        if control not in self.controls:
            raise RuntimeError(f"control {control} not supported")
        return self.controls[control]

    def getCameraInfo(self) -> types.SimpleNamespace:
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
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = STAMP
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
    """Bridge on the default config.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches the SDK camera factory.

    Returns:
        Harness: The harness.
    """
    return Harness(monkeypatch, TofConfig())


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
    assert depth.header.stamp == STAMP
    assert depth.header.frame_id == "tof_optical_frame"

    assert (depth.encoding, depth.height, depth.width, depth.step, depth.is_bigendian) == ("16UC1", 3, 4, 8, 0)
    pixels = np.frombuffer(bytes(depth.data), dtype="<u2").reshape(HEIGHT, WIDTH)
    expected = np.rint(np.arange(1, 13, dtype=np.float32).reshape(HEIGHT, WIDTH) * 100.4).astype(np.uint16)
    expected[1, 2] = 0
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
