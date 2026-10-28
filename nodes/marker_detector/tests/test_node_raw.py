"""Tests that the head image and camera_info are subscribed raw and deserialized only in the process timer."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from marker_detector.config import MarkerDetectorConfig
from tests.ros_stubs import install_ros_stubs

IMAGE_STAMP_SEC = 7
JPEG = b"jpeg-bytes"


def make_image(sec: int = IMAGE_STAMP_SEC, data: bytes = JPEG) -> SimpleNamespace:
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=5)), data=list(data))


def make_info() -> SimpleNamespace:
    return SimpleNamespace(
        k=list(np.eye(3).ravel()), d=[0.0] * 5, width=640, height=480, header=SimpleNamespace(frame_id="cam")
    )


@pytest.fixture
def rig(monkeypatch: pytest.MonkeyPatch):
    """(node wrapper, node mock, pipeline mock, decode calls list) with a counting deserializer."""
    install_ros_stubs()
    from marker_detector import node as node_module

    calls: list[object] = []

    def counting(raw: object, msg_type: type) -> object:
        calls.append(msg_type)
        return raw

    monkeypatch.setattr(node_module, "deserialize_message", counting)
    ros_node, pipeline = MagicMock(), MagicMock()
    pipeline.process_frame.return_value = SimpleNamespace(rejections={}, commands=[], report=None)
    wrapper = node_module.MarkerDetectorRos(MarkerDetectorConfig(), ros_node, pipeline)
    return wrapper, ros_node, pipeline, calls


def test_image_is_deserialized_only_in_the_timer(rig) -> None:
    wrapper, ros_node, pipeline, calls = rig
    raw_subs = {c.args[1]: c for c in ros_node.create_subscription.call_args_list if c.kwargs.get("raw")}
    cfg = wrapper.cfg.topics
    assert cfg.image in raw_subs and cfg.camera_info in raw_subs
    wrapper.on_image(make_image())
    assert calls == []
    pipeline.process_frame.assert_not_called()
    wrapper.on_timer()
    assert len(calls) == 1
    stamp, jpeg = pipeline.process_frame.call_args.args[:2]
    assert stamp == IMAGE_STAMP_SEC * 1_000_000_000 + 5
    assert jpeg == JPEG


def test_camera_info_is_deserialized_only_when_new(rig) -> None:
    wrapper, _, pipeline, calls = rig
    wrapper.on_camera_info(make_info())
    wrapper.on_image(make_image())
    assert calls == []
    wrapper.on_timer()
    assert len(calls) == 2
    pipeline.on_camera_info.assert_called_once()
    wrapper.on_image(make_image(sec=8))
    wrapper.on_timer()
    assert len(calls) == 3
    pipeline.on_camera_info.assert_called_once()
    wrapper.on_camera_info(make_info())
    wrapper.on_camera_info(make_info())
    wrapper.on_image(make_image(sec=9))
    wrapper.on_timer()
    assert len(calls) == 5
    assert pipeline.on_camera_info.call_count == 2


def test_timer_processes_the_newest_frame_once(rig) -> None:
    wrapper, _, pipeline, calls = rig
    wrapper.on_image(make_image(sec=1))
    wrapper.on_image(make_image(sec=2))
    wrapper.on_image(make_image(sec=3, data=b"newest"))
    wrapper.on_timer()
    wrapper.on_timer()
    assert len(calls) == 1
    pipeline.process_frame.assert_called_once()
    stamp, jpeg = pipeline.process_frame.call_args.args[:2]
    assert stamp == 3 * 1_000_000_000 + 5
    assert jpeg == b"newest"
