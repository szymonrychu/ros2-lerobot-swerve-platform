"""Tests that each camera's image and camera_info are subscribed raw, kept per camera and deserialized only in the timer."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from marker_detector.config import MarkerDetectorConfig
from marker_detector.pipeline import MarkerPipeline
from tests.ros_stubs import install_ros_stubs

IMAGE_STAMP_SEC = 7
JPEG = b"jpeg-bytes"
DEPTH_TOPIC = "/head_rgbd/depth/image_raw"
DEPTH_INFO_TOPIC = "/head_rgbd/depth/camera_info"


def camera(name: str, depth_plane: bool = False) -> dict:
    return {
        "name": name,
        "image": f"/{name}/image_raw/compressed",
        "camera_info": f"/{name}/camera_info",
        "depth_plane": depth_plane,
    }


def three_cameras(head_depth_plane: bool = True) -> MarkerDetectorConfig:
    return MarkerDetectorConfig.model_validate(
        {"cameras": [camera("head", head_depth_plane), camera("front"), camera("back")]}
    )


def mock_pipeline(cfg: MarkerDetectorConfig) -> MagicMock:
    pipeline = MagicMock()
    pipeline.process_frame.return_value = SimpleNamespace(rejections={}, commands=[], report=None)
    return pipeline


def make_image(sec: int = IMAGE_STAMP_SEC, data: bytes = JPEG) -> SimpleNamespace:
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=5)), data=list(data))


def make_info() -> SimpleNamespace:
    return SimpleNamespace(
        k=list(np.eye(3).ravel()), d=[0.0] * 5, width=640, height=480, header=SimpleNamespace(frame_id="cam")
    )


@pytest.fixture
def rig(monkeypatch: pytest.MonkeyPatch):
    """(node wrapper, node mock, head pipeline mock, decode calls list) with a counting deserializer."""
    install_ros_stubs()
    from marker_detector import node as node_module

    calls: list[object] = []

    def counting(raw: object, msg_type: type) -> object:
        calls.append(msg_type)
        return raw

    monkeypatch.setattr(node_module, "deserialize_message", counting)
    ros_node = MagicMock()
    wrapper = node_module.MarkerDetectorRos(three_cameras(), ros_node, mock_pipeline)
    return wrapper, ros_node, wrapper.cameras["head"].pipeline, calls


def test_image_is_deserialized_only_in_the_timer(rig) -> None:
    wrapper, ros_node, pipeline, calls = rig
    raw_subs = {c.args[1] for c in ros_node.create_subscription.call_args_list if c.kwargs.get("raw")}
    assert raw_subs == {topic for c in wrapper.cfg.cameras for topic in (c.image, c.camera_info)}
    wrapper.cameras["head"].on_image(make_image())
    assert calls == []
    pipeline.process_frame.assert_not_called()
    wrapper.on_timer()
    assert len(calls) == 1
    stamp, jpeg = pipeline.process_frame.call_args.args[:2]
    assert stamp == IMAGE_STAMP_SEC * 1_000_000_000 + 5
    assert jpeg == JPEG


def test_camera_info_is_deserialized_only_when_new(rig) -> None:
    wrapper, _, pipeline, calls = rig
    wrapper.cameras["head"].on_camera_info(make_info())
    wrapper.cameras["head"].on_image(make_image())
    assert calls == []
    wrapper.on_timer()
    assert len(calls) == 2
    pipeline.on_camera_info.assert_called_once()
    wrapper.cameras["head"].on_image(make_image(sec=8))
    wrapper.on_timer()
    assert len(calls) == 3
    pipeline.on_camera_info.assert_called_once()
    wrapper.cameras["head"].on_camera_info(make_info())
    wrapper.cameras["head"].on_camera_info(make_info())
    wrapper.cameras["head"].on_image(make_image(sec=9))
    wrapper.on_timer()
    assert len(calls) == 5
    assert pipeline.on_camera_info.call_count == 2


def test_timer_processes_the_newest_frame_once(rig) -> None:
    wrapper, _, pipeline, calls = rig
    wrapper.cameras["head"].on_image(make_image(sec=1))
    wrapper.cameras["head"].on_image(make_image(sec=2))
    wrapper.cameras["head"].on_image(make_image(sec=3, data=b"newest"))
    wrapper.on_timer()
    wrapper.on_timer()
    assert len(calls) == 1
    pipeline.process_frame.assert_called_once()
    stamp, jpeg = pipeline.process_frame.call_args.args[:2]
    assert stamp == 3 * 1_000_000_000 + 5
    assert jpeg == b"newest"


def test_each_camera_has_its_own_pipeline(rig, monkeypatch: pytest.MonkeyPatch) -> None:
    wrapper, ros_node, head, _ = rig
    from marker_detector import metrics
    from marker_detector import node as node_module

    front, back = wrapper.cameras["front"].pipeline, wrapper.cameras["back"].pipeline
    assert len({id(head), id(front), id(back)}) == 3
    front_frames = metrics.FRAMES_PROCESSED.labels("front")._value.get()
    wrapper.cameras["head"].on_image(make_image(sec=1, data=b"head"))
    wrapper.cameras["front"].on_image(make_image(sec=2, data=b"front"))
    wrapper.on_timer()
    assert head.process_frame.call_args.args[:2] == (1_000_000_005, b"head")
    assert front.process_frame.call_args.args[:2] == (2_000_000_005, b"front")
    back.process_frame.assert_not_called()
    assert metrics.FRAMES_PROCESSED.labels("front")._value.get() == front_frames + 1

    # Depth goes to the depth_plane camera only, and only it gets the depth subscriptions.
    subscribed = [c.args[1] for c in ros_node.create_subscription.call_args_list]
    assert subscribed.count(DEPTH_TOPIC) == 1 and subscribed.count(DEPTH_INFO_TOPIC) == 1
    wrapper.on_depth_info(
        SimpleNamespace(width=4, height=2, k=[2.0, 0, 2, 0, 2.0, 1, 0, 0, 1], d=[0.0] * 5, distortion_model="plumb_bob")
    )
    depth = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        encoding="16UC1",
        height=2,
        width=4,
        step=8,
        data=bytes(16),
    )
    wrapper.on_depth(depth)
    head.on_depth.assert_called_once()
    front.on_depth.assert_not_called()
    back.on_depth.assert_not_called()

    no_depth = MagicMock()
    node_module.MarkerDetectorRos(three_cameras(head_depth_plane=False), no_depth, mock_pipeline)
    subscribed = {c.args[1] for c in no_depth.create_subscription.call_args_list}
    assert DEPTH_TOPIC not in subscribed and DEPTH_INFO_TOPIC not in subscribed

    # A camera that never got camera_info publishes nothing (the real pipeline skips the frame).
    monkeypatch.setattr(node_module, "deserialize_message", lambda raw, msg_type: raw)
    real = MagicMock()
    wrapper = node_module.MarkerDetectorRos(three_cameras(), real, MarkerPipeline)
    wrapper.cameras["front"].on_image(make_image())
    wrapper.on_timer()
    real.create_publisher.return_value.publish.assert_not_called()
