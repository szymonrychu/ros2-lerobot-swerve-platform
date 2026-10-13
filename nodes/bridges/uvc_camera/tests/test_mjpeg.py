"""MJPEG pass-through: camera JPEG bytes are published as is, rotated losslessly, raw Image only with subscribers."""

import importlib
import sys
from types import ModuleType
from typing import Any
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
from prometheus_client import REGISTRY

import metrics
from config import DEFAULT_CAPTURE_FORMAT, get_capture_format
from frame import rotate_frame, rotate_jpeg

from .ros_stubs import FakeRclpy, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")
sys.modules.setdefault("uvc_camera.metrics", metrics)  # one set of Prometheus series, not two registrations

MJPG_FOURCC = cv2.VideoWriter_fourcc(*"MJPG")
YUYV_FOURCC = cv2.VideoWriter_fourcc(*"YUYV")
ENCODE_COUNT = "camera_encode_seconds_count"
ROTATIONS = (90, 180, 270)


def make_jpeg(width: int = 64, height: int = 48) -> bytes:
    """Encode a smooth gradient with a corner marker as JPEG.

    Args:
        width (int): Image width in pixels.
        height (int): Image height in pixels.

    Returns:
        bytes: Baseline JPEG (4:2:0 subsampling).
    """
    ys, xs = np.mgrid[0:height, 0:width]
    image = np.stack([xs * 255 // width, ys * 255 // height, (xs + ys) * 255 // (width + height)], axis=-1)
    image = image.astype(np.uint8)
    image[: height // 4, : width // 4] = (0, 0, 255)  # asymmetric marker: a wrong direction or flip shows up
    ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    assert ok
    return buf.tobytes()


def decode(jpeg: bytes) -> np.ndarray:
    """Decode JPEG bytes to a BGR image.

    Args:
        jpeg (bytes): JPEG data.

    Returns:
        np.ndarray: BGR image.
    """
    return cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)


class FakeMsg:
    """Plain attribute bag standing in for ROS messages."""


def test_capture_format_defaults_to_mjpeg(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("UVC_CAPTURE_FORMAT", raising=False)
    assert DEFAULT_CAPTURE_FORMAT == "mjpeg"
    assert get_capture_format() == "mjpeg"


def test_capture_format_raw_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("UVC_CAPTURE_FORMAT", " RAW ")
    assert get_capture_format() == "raw"


def test_capture_format_rejects_unknown(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("UVC_CAPTURE_FORMAT", "h264")
    with pytest.raises(ValueError, match="UVC_CAPTURE_FORMAT"):
        get_capture_format()


def mean_abs_diff(a: np.ndarray, b: np.ndarray) -> float:
    """Mean absolute pixel difference of two equally shaped images.

    Args:
        a (np.ndarray): First image.
        b (np.ndarray): Second image.

    Returns:
        float: Mean of |a - b|.
    """
    return float(np.abs(a.astype(np.int16) - b.astype(np.int16)).mean())


@pytest.mark.parametrize("rotate_deg", ROTATIONS)
@pytest.mark.parametrize("size", [(64, 48), (70, 50)])
def test_rotate_jpeg_matches_pixel_rotation(rotate_deg: int, size: tuple[int, int]) -> None:
    """The DCT-domain rotation decodes to the rotated pixels (chroma upsampling differs by rounding only).

    A non-MCU-aligned size is trimmed: the result is a corner crop of the pixel rotation.
    """
    source = make_jpeg(*size)
    rotated = decode(rotate_jpeg(source, rotate_deg))
    expected = rotate_frame(decode(source), rotate_deg)
    height, width = rotated.shape[:2]
    assert (width, height) == (expected.shape[1], expected.shape[0]) or size == (70, 50)
    corners = (
        expected[:height, :width],
        expected[:height, -width:],
        expected[-height:, :width],
        expected[-height:, -width:],
    )
    assert min(mean_abs_diff(rotated, crop) for crop in corners) < 2.0
    if size == (64, 48):
        assert mean_abs_diff(rotated, expected) < 2.0
        assert mean_abs_diff(rotated, rotate_frame(decode(source), 360 - rotate_deg if rotate_deg != 180 else 0)) > 10


def test_rotate_jpeg_zero_returns_input_bytes() -> None:
    source = make_jpeg()
    assert rotate_jpeg(source, 0) is source


def test_rotate_jpeg_rejects_bad_angle() -> None:
    with pytest.raises(ValueError):
        rotate_jpeg(make_jpeg(), 45)


class Harness:
    """A patched bridge module plus the messages its fake publishers received."""

    def __init__(self, module: ModuleType, cap: MagicMock, pubs: dict[str, MagicMock], fake: FakeRclpy) -> None:
        self.module = module
        self.cap = cap
        self.pubs = pubs
        self.fake = fake

    def published(self, topic: str) -> list[Any]:
        """Messages published on a topic.

        Args:
            topic (str): Topic name.

        Returns:
            list[Any]: Message objects in publish order.
        """
        return [call.args[0] for call in self.pubs[topic].publish.call_args_list]


@pytest.fixture
def make_harness(monkeypatch: pytest.MonkeyPatch) -> Any:
    """Build a harness factory: one frame read per run, publishers keyed by topic.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        Callable[..., Harness]: Factory taking the frame the camera returns, the readback fourcc and subscriber count.
    """

    def factory(frame: np.ndarray, fourcc: int = MJPG_FOURCC, raw_subscribers: int = 1) -> Harness:
        module = importlib.import_module("uvc_camera.bridge")
        cap = MagicMock()
        cap.read.return_value = (True, frame)
        cap.get.side_effect = lambda prop: float(fourcc) if prop == cv2.CAP_PROP_FOURCC else 0.0
        pubs = {"/image": MagicMock(), "/image/compressed": MagicMock()}
        pubs["/image"].get_subscription_count.return_value = raw_subscribers
        pubs["/image/compressed"].get_subscription_count.return_value = 1
        node = MagicMock()
        node.create_publisher.side_effect = lambda _msg, topic, _depth: pubs[topic]
        fake = FakeRclpy()
        monkeypatch.setattr(module, "open_capture", lambda _device: cap)
        monkeypatch.setattr(module, "start_metrics_server", MagicMock())
        monkeypatch.setattr(module, "Node", MagicMock(return_value=node))
        monkeypatch.setattr(module, "rclpy", fake)
        for name in ("Image", "CompressedImage", "Header"):
            monkeypatch.setattr(module, name, FakeMsg)
        return Harness(module, cap, pubs, fake)

    return factory


def run(harness: Harness, rotate_deg: int = 0, capture_format: str = "mjpeg") -> None:
    """Run one frame through the bridge.

    Args:
        harness (Harness): Patched bridge.
        rotate_deg (int): Rotation.
        capture_format (str): 'mjpeg' or 'raw'.
    """
    harness.module.run_bridge("/dev/video0", "/image", "camera", rotate_deg, None, capture_format)


def encode_count() -> float:
    """Read the camera_encode_seconds observation count.

    Returns:
        float: Number of observations so far.
    """
    return REGISTRY.get_sample_value(ENCODE_COUNT) or 0.0


def camera_frame(jpeg: bytes) -> np.ndarray:
    """Shape JPEG bytes the way OpenCV returns them with CONVERT_RGB off.

    Args:
        jpeg (bytes): JPEG data.

    Returns:
        np.ndarray: (1, N) uint8 array.
    """
    return np.frombuffer(jpeg, dtype=np.uint8).reshape(1, -1)


def test_device_opened_with_mjpg_and_no_rgb_conversion(make_harness: Any) -> None:
    harness = make_harness(camera_frame(make_jpeg()))
    run(harness)
    harness.cap.set.assert_any_call(cv2.CAP_PROP_FOURCC, MJPG_FOURCC)
    harness.cap.set.assert_any_call(cv2.CAP_PROP_CONVERT_RGB, 0)


def test_camera_jpeg_published_unchanged_without_reencode(make_harness: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    source = make_jpeg()
    harness = make_harness(camera_frame(source))
    encode = MagicMock(side_effect=AssertionError("cv2.imencode must not run on the MJPEG path"))
    monkeypatch.setattr(cv2, "imencode", encode)
    before = encode_count()

    run(harness)

    [msg] = harness.published("/image/compressed")
    assert bytes(msg.data) == source
    assert msg.format == "jpeg"
    assert encode_count() == before + 1  # camera_encode_seconds still observes one produced JPEG per frame


@pytest.mark.parametrize("rotate_deg", ROTATIONS)
def test_rotation_is_lossless_on_the_jpeg(make_harness: Any, monkeypatch: pytest.MonkeyPatch, rotate_deg: int) -> None:
    source = make_jpeg()
    harness = make_harness(camera_frame(source))
    monkeypatch.setattr(cv2, "imencode", MagicMock(side_effect=AssertionError("re-encode")))

    run(harness, rotate_deg)

    [msg] = harness.published("/image/compressed")
    assert bytes(msg.data) == rotate_jpeg(source, rotate_deg)
    assert mean_abs_diff(decode(bytes(msg.data)), rotate_frame(decode(source), rotate_deg)) < 2.0


def test_raw_image_decoded_and_published_with_subscribers(make_harness: Any) -> None:
    source = make_jpeg()
    harness = make_harness(camera_frame(source), raw_subscribers=1)
    run(harness, 180)
    [raw] = harness.published("/image")
    expected = decode(rotate_jpeg(source, 180))
    assert raw.encoding == "bgr8"
    assert (raw.height, raw.width) == expected.shape[:2]
    assert np.array_equal(np.frombuffer(raw.data, dtype=np.uint8).reshape(expected.shape), expected)


def test_raw_image_not_decoded_without_subscribers(make_harness: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    harness = make_harness(camera_frame(make_jpeg()), raw_subscribers=0)
    monkeypatch.setattr(cv2, "imdecode", MagicMock(side_effect=AssertionError("decode without subscribers")))
    run(harness)
    assert harness.published("/image") == []
    assert len(harness.published("/image/compressed")) == 1


def test_non_mjpeg_device_falls_back_to_raw_path_and_logs_once(make_harness: Any) -> None:
    bgr = np.full((4, 6, 3), 9, dtype=np.uint8)
    harness = make_harness(bgr, fourcc=YUYV_FOURCC)
    log = MagicMock()
    harness.module.Node.return_value.get_logger.return_value = log

    run(harness)

    harness.cap.set.assert_any_call(cv2.CAP_PROP_CONVERT_RGB, 1)
    [raw] = harness.published("/image")
    assert (raw.height, raw.width) == (4, 6)
    [compressed] = harness.published("/image/compressed")
    assert decode(bytes(compressed.data)).shape == (4, 6, 3)
    mjpeg_logs = [c for c in log.warning.call_args_list if "MJPEG" in str(c)]
    assert len(mjpeg_logs) == 1


def test_raw_capture_format_never_requests_mjpg(make_harness: Any) -> None:
    harness = make_harness(np.full((4, 6, 3), 9, dtype=np.uint8), fourcc=YUYV_FOURCC)
    run(harness, capture_format="raw")
    assert all(call.args[0] != cv2.CAP_PROP_FOURCC for call in harness.cap.set.call_args_list)
    assert len(harness.published("/image")) == 1


@pytest.mark.parametrize("rotate_deg", [0, 180])
@pytest.mark.parametrize("raw_subscribers", [0, 1])
def test_corrupt_mjpeg_frame_publishes_nothing(make_harness: Any, rotate_deg: int, raw_subscribers: int) -> None:
    truncated = make_jpeg()[:40]
    for bad in (b"not a jpeg", truncated):
        harness = make_harness(camera_frame(bad), raw_subscribers=raw_subscribers)
        run(harness, rotate_deg)
        assert harness.published("/image/compressed") == []
        assert harness.published("/image") == []
