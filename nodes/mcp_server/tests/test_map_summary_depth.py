"""get_map_summary adds `obstacles_3d` from the head D435 cloud (/head/d435/points_filtered), lazily subscribed.

Covers: the topic and staleness defaults; RosRobot.depth_points subscribes the D435 cloud topic on the first call and
returns the cloud in base_link with the TF at the cloud stamp, and None for no cloud / stale cloud / no TF; the summary
carries obstacle ahead and cliff left from a fake cloud, null with a note when the cloud is missing, and the tool
description names the head depth (D435).
"""

import struct
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from mcp_server.config import McpServerConfig
from mcp_server.models import DepthPoints
from mcp_server.staleness import Stamped

from .test_tools import TOKEN, FakeRobot, build_mcp_server, call, tool_descriptions

D435_CLOUD_TOPIC = "/head/d435/points_filtered"
D435_CLOUD_FRAME = "d435_depth_optical_frame"


def cloud_msg(points: list[tuple[float, float, float]], frame: str = D435_CLOUD_FRAME, sec: int = 5) -> SimpleNamespace:
    fields = [SimpleNamespace(name=n, offset=4 * i) for i, n in enumerate("xyz")]
    data = b"".join(struct.pack("<fff", *p) for p in points)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0), frame_id=frame),
        fields=fields,
        is_bigendian=False,
        point_step=12,
        width=len(points),
        height=1,
        data=data,
    )


def tf_msg(x: float, y: float, z: float, qz: float = 0.0, qw: float = 1.0) -> SimpleNamespace:
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z), rotation=SimpleNamespace(x=0.0, y=0.0, z=qz, w=qw)
        )
    )


class TfError(Exception):
    """Stands in for tf2_ros.TransformException (mocked away in these tests)."""


def ros_robot(ros_iface: ModuleType, msg: Any | None, tf: Any | None, subscribed: bool = True) -> Any:
    ros_iface.TransformException = TfError
    robot = ros_iface.RosRobot.__new__(ros_iface.RosRobot)
    robot.cfg = McpServerConfig()
    robot.node = MagicMock()
    robot.group = None
    robot.lock = __import__("threading").Lock()
    robot.latest = {} if msg is None else {"depth_points": Stamped(value=msg, stamp=time.monotonic())}
    robot.depth_subscribed = subscribed
    robot.cache = MagicMock()
    stamps: list[Any] = []

    def tf_lookup(frame: str, child: str, stamp: Any = None) -> Any:
        stamps.append((frame, child, stamp))
        if tf is None:
            raise TfError("no tf")
        return tf

    robot.tf_lookup = tf_lookup
    robot.tf_stamps = stamps
    return robot


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


def summary_of(robot: FakeRobot) -> dict[str, Any]:
    return call(build_mcp_server(robot, McpServerConfig(), TOKEN), "get_map_summary", {}).structured_content


def test_defaults() -> None:
    cfg = McpServerConfig()
    assert cfg.topics.depth_points == D435_CLOUD_TOPIC
    assert cfg.timeouts.depth_points_stale_s == 1.0
    assert "tof_points" not in type(cfg.topics).model_fields
    assert "tof_points_stale_s" not in type(cfg.timeouts).model_fields


def test_obstacles_3d_from_the_d435_cloud(ros_iface: ModuleType, robot: FakeRobot) -> None:
    ros = ros_robot(ros_iface, cloud_msg([(0.0, 0.0, 0.8)]), tf_msg(0.0, 0.0, 0.0), subscribed=False)
    cloud = ros.depth_points()
    ros.cache.assert_called_once()
    _, topic, key, _ = ros.cache.call_args.args
    assert (topic, key) == (D435_CLOUD_TOPIC, "depth_points")
    assert ros.tf_stamps[0][:2] == ("base_link", D435_CLOUD_FRAME)
    assert isinstance(cloud, DepthPoints) and cloud.frame == "base_link"
    assert ros.depth_points() is not None and ros.cache.call_count == 1  # subscribed once, latest message only
    robot.depth_cloud = DepthPoints(points=np.array([[0.8, 0.0, 0.3]]), age_s=0.1, frame="base_link", stamp_s=5.0)
    obs = summary_of(robot)["obstacles_3d"]
    assert {s["sector"]: s for s in obs["sectors"]}["front"]["nearest_m"] == 0.8


def test_stale_depth_cloud_is_not_summarised(ros_iface: ModuleType, robot: FakeRobot) -> None:
    ros = ros_robot(ros_iface, cloud_msg([(1.0, 0.0, 0.0)]), tf_msg(0, 0, 0))
    ros.latest["depth_points"] = Stamped(value=cloud_msg([(1.0, 0.0, 0.0)]), stamp=time.monotonic() - 5.0)
    assert ros.depth_points() is None
    ros.latest["depth_points"] = Stamped(value=cloud_msg([(2.0, 0.0, 0.0)], sec=9), stamp=time.monotonic())
    fresh = ros.depth_points()
    assert fresh is not None and fresh.stamp_s == 9.0 and np.allclose(fresh.points, [[2.0, 0.0, 0.0]])
    robot.depth_cloud = None  # what depth_points returns for a stale cloud
    summary = summary_of(robot)
    assert summary["obstacles_3d"] is None
    assert any("head depth (D435)" in n and D435_CLOUD_TOPIC in n for n in summary["notes"])


def test_depth_points_are_transformed_to_base_link(ros_iface: ModuleType) -> None:
    robot = ros_robot(ros_iface, cloud_msg([(1.0, 0.0, 0.0)]), tf_msg(0.5, 0.0, 0.3))
    cloud = robot.depth_points()
    assert cloud is not None
    assert cloud.frame == "base_link" and cloud.stamp_s == 5.0
    assert np.allclose(cloud.points, [[1.5, 0.0, 0.3]])
    assert robot.tf_stamps[0][:2] == ("base_link", D435_CLOUD_FRAME) and robot.tf_stamps[0][2] is not None


def test_depth_points_apply_the_rotation(ros_iface: ModuleType) -> None:
    half = float(np.sqrt(0.5))
    robot = ros_robot(ros_iface, cloud_msg([(1.0, 0.0, 0.0)]), tf_msg(0.0, 0.0, 0.0, qz=half, qw=half))
    assert np.allclose(robot.depth_points().points, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_depth_points_none_without_cloud_or_tf(ros_iface: ModuleType) -> None:
    assert ros_robot(ros_iface, None, tf_msg(0, 0, 0)).depth_points() is None
    assert ros_robot(ros_iface, cloud_msg([(1.0, 0.0, 0.0)]), None).depth_points() is None


def test_summary_has_obstacle_ahead_and_cliff_left(robot: FakeRobot) -> None:
    points = np.array([[0.8, 0.0, 0.3], [0.0, 0.7, -0.3]])
    robot.depth_cloud = DepthPoints(points=points, age_s=0.2, frame="base_link", stamp_s=5.0)
    obs = summary_of(robot)["obstacles_3d"]
    sectors = {s["sector"]: s for s in obs["sectors"]}
    assert sectors["front"]["nearest_m"] == 0.8 and sectors["front"]["max_z"] == 0.3
    assert sectors["left"]["cliff_m"] == 0.7 and sectors["left"]["nearest_m"] is None
    assert obs["frame"] == "base_link" and obs["cloud_age_s"] == 0.2 and obs["point_count"] == 2


def test_summary_null_with_reason_when_cloud_missing(robot: FakeRobot) -> None:
    robot.depth_cloud = None
    summary = summary_of(robot)
    assert summary["obstacles_3d"] is None
    assert any("head depth (D435)" in n for n in summary["notes"])
    assert not any("ToF" in n or "/tof/" in n for n in summary["notes"])


def test_description_names_the_head_depth_d435(robot: FakeRobot) -> None:
    doc = tool_descriptions(build_mcp_server(robot, McpServerConfig(), TOKEN))["get_map_summary"]
    assert "obstacles_3d" in doc and "head depth (D435)" in doc
    assert "ToF depth camera" not in doc
