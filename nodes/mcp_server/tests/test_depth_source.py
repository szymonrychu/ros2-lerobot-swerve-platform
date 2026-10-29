"""Aligned depth frames: Image decoding and pairing with the colour frame of the same frameset by header stamp."""

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from mcp_server.camera_scene import CameraInfoData
from mcp_server.config import CameraArmSettings
from mcp_server.depth_source import DepthSource, decode_depth_image
from mcp_server.models import RobotError
from mcp_server.spin import FrameCache

INFO = CameraInfoData(640, 360, (455.2, 0.0, 321.7, 0.0, 455.4, 180.9, 0.0, 0.0, 1.0), (0.0,) * 5, "plumb_bob", 5.0)
TIMEOUT_S = 0.15


def stamp(seconds: float) -> SimpleNamespace:
    return SimpleNamespace(sec=int(seconds), nanosec=round((seconds % 1) * 1e9))


def image_msg(seconds: float, depth_mm: int = 400, encoding: str = "16UC1", w: int = 8, h: int = 4) -> SimpleNamespace:
    data = np.full((h, w), depth_mm, dtype="<u2").tobytes()
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(seconds)),
        width=w,
        height=h,
        encoding=encoding,
        step=w * 2,
        is_bigendian=False,
        data=data,
    )


def colour_msg(seconds: float) -> SimpleNamespace:
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(seconds)))


class Rig:
    def __init__(self, info: CameraInfoData | None = INFO, age_s: float = 0.1, cfg: CameraArmSettings | None = None):
        self.depth, self.colour = FrameCache(), FrameCache()
        self.touched = 0
        self.age_s = age_s
        self.info = info
        self.source = DepthSource(
            depth_cache=self.depth,
            colour_cache=self.colour,
            read_info=lambda: self.info,
            touch=self.touch,
            ros_age=lambda _stamp: self.age_s,
            cfg=cfg or CameraArmSettings(),
            first_message_s=TIMEOUT_S,
            max_age_s=1.0,
            depth_topic="/d/depth",
            colour_topic="/d/colour",
        )

    def touch(self) -> None:
        self.touched += 1


def feed_after(cache: FrameCache, msg: SimpleNamespace, delay_s: float) -> threading.Thread:
    thread = threading.Thread(target=lambda: (time.sleep(delay_s), cache.update(msg)))
    thread.start()
    return thread


def test_decode_16uc1_is_millimetres_as_uint16() -> None:
    depth = decode_depth_image(image_msg(1.0, 437))
    assert depth.dtype == np.uint16 and depth.shape == (4, 8) and (depth == 437).all()


def test_decode_respects_the_row_step() -> None:
    msg = image_msg(1.0, 300, w=5, h=3)
    row = np.full((5,), 300, dtype="<u2").tobytes() + b"\xff\xff\xff\xff"  # 4 padding bytes per row
    msg.data, msg.step = row * 3, 14
    depth = decode_depth_image(msg)
    assert depth.shape == (3, 5) and (depth == 300).all()


def test_decode_32fc1_is_metres_as_float32() -> None:
    msg = image_msg(1.0, encoding="32FC1")
    msg.data = np.full((4, 8), 0.5, dtype="<f4").tobytes()
    msg.step = 8 * 4
    depth = decode_depth_image(msg)
    assert depth.dtype == np.float32 and depth[0, 0] == pytest.approx(0.5)


def test_decode_refuses_other_encodings_and_short_data() -> None:
    with pytest.raises(RobotError, match="encoding"):
        decode_depth_image(image_msg(1.0, encoding="rgb8"))
    short = image_msg(1.0)
    short.data = short.data[:-4]
    with pytest.raises(RobotError, match="size"):
        decode_depth_image(short)


def test_decode_big_endian() -> None:
    msg = image_msg(1.0)
    msg.data = np.full((4, 8), 400, dtype=">u2").tobytes()
    msg.is_bigendian = True
    assert (decode_depth_image(msg) == 400).all()


def test_pair_within_half_a_frame_is_returned_with_both_stamps() -> None:
    rig = Rig()
    feed_after(rig.colour, colour_msg(10.010), 0.0)
    feed_after(rig.depth, image_msg(10.0), 0.02)
    frame = rig.source.read()
    assert (frame.stamp_s, frame.color_stamp_s) == (10.0, 10.010)
    assert frame.depth.shape == (4, 8) and frame.info == INFO and frame.age_s == 0.1
    assert rig.touched == 1


def test_stamps_a_frame_apart_are_refused_naming_both() -> None:
    rig = Rig()
    rig.colour.update(colour_msg(10.100))
    feed_after(rig.depth, image_msg(10.0), 0.02)
    with pytest.raises(RobotError, match=r"differ by 0\.100 s .*0\.033") as err:
        rig.source.read()
    assert "same frameset" in str(err.value)


def test_a_colour_frame_that_arrives_late_still_pairs() -> None:
    rig = Rig()
    rig.colour.update(colour_msg(9.934))  # the previous frame
    feed_after(rig.depth, image_msg(10.0), 0.02)
    late = feed_after(rig.colour, colour_msg(10.002), 0.07)
    frame = rig.source.read()
    late.join()
    assert frame.color_stamp_s == 10.002


def test_no_depth_frame_is_refused() -> None:
    with pytest.raises(RobotError, match="no aligned depth frame on /d/depth"):
        Rig().source.read()


def test_no_colour_frame_is_refused() -> None:
    rig = Rig()
    feed_after(rig.depth, image_msg(10.0), 0.02)
    with pytest.raises(RobotError, match="no colour frame on /d/colour"):
        rig.source.read()


def test_old_frame_is_refused() -> None:
    rig = Rig(age_s=2.5)
    rig.colour.update(colour_msg(10.0))
    feed_after(rig.depth, image_msg(10.0), 0.02)
    with pytest.raises(RobotError, match="2.5 s old"):
        rig.source.read()


def test_zero_stamp_is_refused() -> None:
    rig = Rig()
    rig.colour.update(colour_msg(10.0))
    feed_after(rig.depth, image_msg(0.0), 0.02)
    with pytest.raises(RobotError, match="no capture timestamp"):
        rig.source.read()


def test_missing_camera_info_is_refused() -> None:
    rig = Rig(info=None)
    rig.colour.update(colour_msg(10.0))
    feed_after(rig.depth, image_msg(10.0), 0.02)
    with pytest.raises(RobotError, match="CameraInfo"):
        rig.source.read()


def test_a_frame_that_arrived_before_the_call_is_not_reused() -> None:
    rig = Rig()
    rig.colour.update(colour_msg(10.0))
    rig.depth.update(image_msg(10.0))  # arrived before read(): the call waits for a newer one
    with pytest.raises(RobotError, match="no aligned depth frame"):
        rig.source.read()
