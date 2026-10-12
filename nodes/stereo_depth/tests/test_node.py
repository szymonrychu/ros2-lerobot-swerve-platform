"""Unit tests for the stereo_depth node with rclpy, message_filters and the message packages faked (conftest).

A FakeSynchronizer stands in for message_filters.ApproximateTimeSynchronizer: the tests call the registered pair
callback directly with synthetic camera images, and check what is published, dropped and counted.
"""

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest
from prometheus_client import REGISTRY

from stereo_depth import __main__ as entry
from stereo_depth import node as node_module
from stereo_depth.calibration import LEFT_FILE, write_stereo_calibration
from stereo_depth.config import DepthConfig
from stereo_depth.messages import stamp_ns
from tests.conftest import (
    BASELINE_M,
    FX,
    HEIGHT,
    WIDTH,
    FakeSynchronizer,
    ideal_calibration,
    image_msg,
    shifted_pair,
)

DISPARITY_PX = 20
EXPECTED_MM = FX * BASELINE_M / DISPARITY_PX * 1000.0


def sample(name: str, labels: dict[str, str] | None = None) -> float:
    """Read a metric sample, 0 when it was never touched.

    Args:
        name (str): Sample name.
        labels (dict[str, str] | None): Label values.

    Returns:
        float: The value.
    """
    return REGISTRY.get_sample_value(name, labels or {}) or 0.0


def dropped(reason: str) -> float:
    """Read stereo_depth_frames_dropped_total for one reason.

    Args:
        reason (str): Reason label.

    Returns:
        float: The count.
    """
    return sample("stereo_depth_frames_dropped_total", {"reason": reason})


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
    """A StereoDepthNode on a mocked rclpy node, publishers recorded per topic."""

    def __init__(self, tmp_path: Path, calibrated: bool = True, **overrides: Any) -> None:
        """Build the node.

        Args:
            tmp_path (Path): Calibration directory.
            calibrated (bool): Write the ideal calibration before the node starts.
            **overrides (Any): DepthConfig overrides.
        """
        if calibrated:
            write_stereo_calibration(tmp_path, ideal_calibration())
        self.config = DepthConfig(calibration_dir=tmp_path, **overrides)
        self.publishers: dict[str, MagicMock] = {}
        self.node = MagicMock()
        self.node.create_publisher.side_effect = self.new_publisher
        self.logger = self.node.get_logger.return_value
        self.clock = Clock()
        self.stereo = node_module.StereoDepthNode(self.node, self.config, clock=self.clock)
        self.sync = FakeSynchronizer.last

    def new_publisher(self, _msg_type: type, topic: str, _qos: Any) -> MagicMock:
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
        return sum(p.publish.call_count for p in self.publishers.values())

    def pair(
        self,
        disparity_px: int = DISPARITY_PX,
        skew_ns: int = 0,
        right_shape: tuple[int, int] | None = None,
        encoding: str = "bgr8",
        sec: int = 10,
    ) -> None:
        """Feed one synced pair to the registered callback.

        Args:
            disparity_px (int): Constant disparity of the synthetic pair.
            skew_ns (int): Right stamp minus left stamp, nanoseconds.
            right_shape (tuple[int, int] | None): Crop the right image to this (rows, cols).
            encoding (str): Image encoding of both images.
            sec (int): Left stamp seconds.
        """
        left, right = shifted_pair(disparity_px)
        if right_shape is not None:
            right = right[: right_shape[0], : right_shape[1]]
        left_msg = image_msg(left, encoding, sec=sec, nanosec=1000)
        right_msg = image_msg(right, encoding, sec=sec, nanosec=1000 + skew_ns)
        assert self.sync is not None
        for callback in self.sync.callbacks:
            callback(left_msg, right_msg)


def test_synchronizer_pairs_the_two_image_topics_with_the_configured_slop(tmp_path: Path) -> None:
    h = Harness(tmp_path, sync_slop_s=0.004)
    assert h.sync is not None
    assert [s.topic for s in h.sync.fs] == ["/overview_camera/image_raw", "/stereo/right/image_raw"]
    assert all(s.node is h.node and s.qos_profile == "sensor-qos" for s in h.sync.fs)
    assert h.sync.slop == 0.004
    assert h.sync.callbacks == [h.stereo.on_pair]
    assert set(h.publishers) == {"/stereo/depth/image_raw", "/stereo/camera_info"}


def test_calibration_recheck_timer_uses_the_config(tmp_path: Path) -> None:
    h = Harness(tmp_path, calibration_check_s=3.0)
    h.node.create_timer.assert_called_once_with(3.0, h.stereo.check_calibration)


def test_valid_pair_publishes_depth_and_camera_info_under_the_left_header(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    published_before = sample("stereo_depth_frames_published_total")
    matcher_before = sample("stereo_depth_matcher_seconds_count")
    skew_before = sample("stereo_depth_pair_skew_seconds_count")
    h.pair()
    [depth] = h.published("/stereo/depth/image_raw")
    [info] = h.published("/stereo/camera_info")
    assert depth.header is info.header
    assert (depth.header.stamp.sec, depth.header.stamp.nanosec) == (10, 1000)
    assert depth.header.frame_id == "stereo_left_optical_frame"
    assert depth.encoding == "16UC1" and (depth.width, depth.height) == (WIDTH, HEIGHT)
    mm = np.frombuffer(depth.data, dtype="<u2").reshape(HEIGHT, WIDTH).astype(np.float64)
    interior = mm[10:-10, 60:-10]
    assert np.median(np.abs(interior[interior > 0] - EXPECTED_MM) / EXPECTED_MM) < 0.02
    assert not mm[:, :40].any()
    assert info.p == list(ideal_calibration().left.p)
    assert sample("stereo_depth_frames_published_total") == published_before + 1
    assert sample("stereo_depth_matcher_seconds_count") == matcher_before + 1
    assert sample("stereo_depth_pair_skew_seconds_count") == skew_before + 1
    assert sample("stereo_depth_calibration_loaded") == 1.0


def test_no_calibration_publishes_nothing_logs_and_counts(tmp_path: Path) -> None:
    h = Harness(tmp_path, calibrated=False)
    before = dropped("no_calibration")
    h.pair()
    assert h.total_published() == 0
    assert dropped("no_calibration") == before + 1
    assert sample("stereo_depth_calibration_loaded") == 0.0
    message = h.logger.warning.call_args.args[0]
    assert LEFT_FILE in message
    assert h.logger.warning.call_args.kwargs["throttle_duration_sec"] > 0


def test_calibration_written_later_is_picked_up_by_the_recheck(tmp_path: Path) -> None:
    h = Harness(tmp_path, calibrated=False)
    h.pair()
    assert h.total_published() == 0
    write_stereo_calibration(tmp_path, ideal_calibration())
    h.stereo.check_calibration()
    h.clock.now_s += 1.0
    h.pair()
    assert len(h.published("/stereo/depth/image_raw")) == 1


def test_changed_calibration_is_reloaded(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    calibration = ideal_calibration()
    wider = calibration.right.model_copy(update={"p": [FX, 0.0, 240.0, -FX * 0.1, 0.0, FX, 180.0, 0, 0, 0, 1.0, 0]})
    extrinsics = calibration.extrinsics.model_copy(update={"t": [-0.1, 0.0, 0.0]})
    write_stereo_calibration(tmp_path, calibration.model_copy(update={"right": wider, "extrinsics": extrinsics}))
    h.stereo.check_calibration()
    assert h.stereo.calibration is not None
    assert h.stereo.calibration.baseline_m == pytest.approx(0.1)


def test_invalid_calibration_publishes_nothing(tmp_path: Path) -> None:
    write_stereo_calibration(tmp_path, ideal_calibration())
    (tmp_path / LEFT_FILE).write_text("image_width: 0\n")
    h = Harness(tmp_path, calibrated=False)
    before = dropped("no_calibration")
    h.pair()
    assert h.total_published() == 0
    assert dropped("no_calibration") == before + 1


def test_calibration_that_becomes_invalid_stops_publishing(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    (tmp_path / LEFT_FILE).write_text("image_width: 0\n")
    h.stereo.check_calibration()
    h.pair()
    assert h.total_published() == 0


def test_skew_above_slop_publishes_nothing(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    before = dropped("skew")
    h.pair(skew_ns=3_000_000)
    assert h.total_published() == 0
    assert dropped("skew") == before + 1
    h.logger.warning.assert_called()


def test_skew_within_slop_publishes(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.pair(skew_ns=-1_500_000)
    assert len(h.published("/stereo/depth/image_raw")) == 1


def test_size_mismatch_between_the_images_publishes_nothing(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    before = dropped("size_mismatch")
    h.pair(right_shape=(HEIGHT - 2, WIDTH))
    assert h.total_published() == 0
    assert dropped("size_mismatch") == before + 1


def test_size_mismatch_with_the_calibration_publishes_nothing(tmp_path: Path) -> None:
    write_stereo_calibration(tmp_path, ideal_calibration(width=640, height=480))
    h = Harness(tmp_path, calibrated=False)
    before = dropped("size_mismatch")
    h.pair()
    assert h.total_published() == 0
    assert dropped("size_mismatch") == before + 1


def test_unsupported_encoding_publishes_nothing(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    left, right = shifted_pair(DISPARITY_PX)
    left_msg, right_msg = image_msg(left), image_msg(right)
    left_msg.encoding = "16UC1"
    before = dropped("bad_image")
    h.stereo.on_pair(left_msg, right_msg)
    assert h.total_published() == 0
    assert dropped("bad_image") == before + 1


def test_pair_without_any_valid_depth_publishes_nothing(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    flat = np.full((HEIGHT, WIDTH), 90, dtype=np.uint8)
    before = dropped("no_valid_pixels")
    h.stereo.on_pair(image_msg(flat), image_msg(flat))
    assert h.total_published() == 0
    assert dropped("no_valid_pixels") == before + 1


def test_rate_cap_holds(tmp_path: Path) -> None:
    h = Harness(tmp_path, max_rate_hz=5.0)
    before = dropped("rate_limit")
    h.pair()  # t = 100.000: published, next slot 100.2
    h.clock.now_s = 100.05
    h.pair()  # too early
    h.clock.now_s = 100.19
    h.pair()  # within the jitter allowance of the 100.2 slot: published, next slot 100.4
    h.clock.now_s = 100.3
    h.pair()  # too early
    h.clock.now_s = 100.4
    h.pair()  # published
    assert len(h.published("/stereo/depth/image_raw")) == 3
    assert dropped("rate_limit") == before + 2


def test_rate_cap_never_exceeds_max_rate_over_a_long_run(tmp_path: Path) -> None:
    h = Harness(tmp_path, max_rate_hz=5.0)
    left, right = shifted_pair(DISPARITY_PX)
    for frame in range(30):  # 3 s of a 10 fps camera
        h.clock.now_s = 100.0 + frame * 0.1
        h.stereo.on_pair(image_msg(left, sec=frame), image_msg(right, sec=frame))
    assert len(h.published("/stereo/depth/image_raw")) == 15


def test_disparity_is_published_only_when_enabled(tmp_path: Path) -> None:
    h = Harness(tmp_path, publish_disparity=True, disparity_topic="/d")
    h.pair()
    [disparity] = h.published("/d")
    [depth] = h.published("/stereo/depth/image_raw")
    assert disparity.header is depth.header
    assert disparity.image.encoding == "32FC1"
    values = np.frombuffer(disparity.image.data, dtype="<f4").reshape(HEIGHT, WIDTH)
    assert np.median(values[10:-10, 60:-10]) == pytest.approx(DISPARITY_PX, abs=0.25)


def test_matcher_threads_are_set(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[int] = []
    monkeypatch.setattr(node_module.cv2, "setNumThreads", calls.append)
    Harness(tmp_path, matcher_threads=3)
    assert calls == [3]


def test_run_starts_metrics_spins_and_shuts_down(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    started: list[tuple[Any, str]] = []
    monkeypatch.setattr(node_module, "start_metrics_server", lambda port, name: started.append((port, name)))
    rclpy = node_module.rclpy
    rclpy.spin.reset_mock()
    rclpy.try_shutdown.reset_mock()
    node_module.run(DepthConfig(calibration_dir=tmp_path))
    assert started == [(19113, "stereo_depth")]
    rclpy.spin.assert_called_once()
    rclpy.try_shutdown.assert_called_once()


def test_run_ignores_keyboard_interrupt(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(node_module, "start_metrics_server", lambda _port, _name: None)
    monkeypatch.setattr(node_module.rclpy, "spin", MagicMock(side_effect=KeyboardInterrupt))
    node_module.run(DepthConfig(calibration_dir=tmp_path))


def test_main_returns_1_on_config_error(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("STEREO_DEPTH_CONFIG", str(tmp_path / "missing.yaml"))
    assert entry.main() == 1


def test_main_runs_the_node(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("max_rate_hz: 2.0\n")
    monkeypatch.setenv("STEREO_DEPTH_CONFIG", str(path))
    ran: list[DepthConfig] = []
    monkeypatch.setattr(entry, "run", ran.append)
    assert entry.main() == 0
    assert ran[0].max_rate_hz == 2.0


def test_stamp_helper_matches_the_left_header(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.pair(sec=42)
    [depth] = h.published("/stereo/depth/image_raw")
    assert stamp_ns(depth.header.stamp) == 42_000_001_000
