"""Camera intrinsics from sensor_msgs/CameraInfo: the on-demand ros_iface subscription and the camera tools using it.

Covers: RosRobot subscribes a camera's camera_info_topic only when camera_info is first asked for (sensor QoS), waits
inside the first-message window, converts the latest message to CameraInfoData, answers None for a camera without a
camera_info_topic or without a message, and drops the cached message when the subscription is reaped; the camera tools
(pixel_to_ground, get_camera_image, the lookup view) build their intrinsics from the robot's latest CameraInfo and answer
"not calibrated" while there is none.
"""

import json
import math
from dataclasses import replace
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.camera_scene import CameraInfoData
from mcp_server.config import McpServerConfig
from mcp_server.tools import build_mcp_server

from .camera_helpers import HEAD_MOUNT, HEIGHT, WIDTH, front_ground_of_center
from .test_camera_tools import CameraRobot, text_json
from .test_on_demand_subscriptions import Rig
from .test_tools import TOKEN, call

INFO_TOPIC = "/head/d435/color/camera_info"
K = [455.2, 0.0, 321.7, 0.0, 455.4, 180.9, 0.0, 0.0, 1.0]
D = [0.01, -0.02, 0.0, 0.0, 0.003]
NOT_CALIBRATED = "camera head not calibrated: set intrinsics and mount in mcp_server config (see README calibration)"
# The synthetic head camera of camera_helpers (640x480, 90 deg hfov) as a CameraInfo: exact pinhole, no distortion.
SYNTHETIC_INFO = CameraInfoData(
    width=WIDTH,
    height=HEIGHT,
    k=(WIDTH / 2, 0.0, WIDTH / 2, 0.0, WIDTH / 2, HEIGHT / 2, 0.0, 0.0, 1.0),
    d=(),
    distortion_model="plumb_bob",
    stamp=40.0,
)


def info_config(**extra: Any) -> McpServerConfig:
    """Config whose head camera takes its intrinsics from INFO_TOPIC (mount of the synthetic head camera)."""
    return McpServerConfig.model_validate(
        {
            "cameras": {"head": {"intrinsics": {"camera_info_topic": INFO_TOPIC}, "mount": HEAD_MOUNT}},
            "arm": {"arm_base_height_m": 0.165, "base_in_base_link": None},
            **extra,
        }
    )


def info_msg(width: int = 640, height: int = 360) -> SimpleNamespace:
    """A sensor_msgs/CameraInfo-like message."""
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=7, nanosec=250_000_000)),
        width=width,
        height=height,
        k=list(K),
        d=list(D),
        distortion_model="plumb_bob",
    )


@pytest.fixture
def info_rig(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> Rig:
    return Rig(ros_iface, monkeypatch, info_config())


def deliver_on_sleep(rig: Rig, msg: SimpleNamespace) -> None:
    """Make the robot's first-message wait receive msg on the camera_info subscription."""

    def sleep(seconds: float) -> None:
        rig.clock[0] += seconds
        found = rig.latest(INFO_TOPIC)
        if found is not None:
            found[0](msg)

    rig.robot.demand_sleep = sleep


def test_camera_info_is_subscribed_on_demand_with_sensor_qos(info_rig: Rig, ros_iface: ModuleType) -> None:
    assert INFO_TOPIC not in info_rig.topics()  # idle: nothing subscribed
    deliver_on_sleep(info_rig, info_msg())
    info = info_rig.robot.camera_info("head")
    assert info == CameraInfoData(
        width=640, height=360, k=tuple(K), d=tuple(D), distortion_model="plumb_bob", stamp=7.25
    )
    assert info_rig.topics().count(INFO_TOPIC) == 1
    qos = [c.args[3] for c in info_rig.node.create_subscription.call_args_list if c.args[1] == INFO_TOPIC]
    assert qos == [ros_iface.SENSOR_QOS]
    info_rig.robot.camera_info("head")
    assert info_rig.topics().count(INFO_TOPIC) == 1  # one subscription, reused


def test_camera_info_keeps_the_latest_message(info_rig: Rig) -> None:
    deliver_on_sleep(info_rig, info_msg())
    assert info_rig.robot.camera_info("head").width == 640
    callback, _sub = info_rig.latest(INFO_TOPIC)
    callback(info_msg(1280, 720))
    latest = info_rig.robot.camera_info("head")
    assert (latest.width, latest.height) == (1280, 720)


def test_camera_info_none_without_topic_or_message(info_rig: Rig) -> None:
    assert info_rig.robot.camera_info("gripper") is None  # no camera_info_topic configured
    assert info_rig.robot.camera_info("rear") is None
    start = info_rig.clock[0]
    assert info_rig.robot.camera_info("head") is None  # subscribed, no publisher inside the first-message window
    assert INFO_TOPIC in info_rig.topics()
    assert info_rig.clock[0] - start == pytest.approx(info_rig.robot.cfg.timeouts.image_timeout_s, abs=0.2)


def test_camera_info_dropped_when_the_subscription_is_reaped(info_rig: Rig) -> None:
    deliver_on_sleep(info_rig, info_msg())
    assert info_rig.robot.camera_info("head") is not None
    _cb, sub = info_rig.latest(INFO_TOPIC)
    info_rig.robot.demand_sleep = lambda seconds: info_rig.clock.__setitem__(0, info_rig.clock[0] + seconds)
    info_rig.clock[0] += info_rig.robot.cfg.on_demand.linger_s + 1.0
    info_rig.robot.reap_on_demand()
    assert sub in info_rig.destroyed()
    assert info_rig.robot.camera_info("head") is None  # resubscribed, no message yet: no stale intrinsics
    assert info_rig.topics().count(INFO_TOPIC) == 2


def make(tmp_path: Path) -> tuple[Any, CameraRobot, McpServerConfig]:
    cfg = info_config(lookups={"store_dir": str(tmp_path / "lookups")})
    cfg.cameras.calibration_dir = tmp_path / "calib"
    robot = CameraRobot(tmp_path)
    robot.photo_store.root = tmp_path / "lookups"
    return build_mcp_server(robot, cfg, TOKEN), robot, cfg


def test_camera_tools_use_the_latest_camera_info(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path)
    robot.camera_infos["head"] = SYNTHETIC_INFO
    data = call(server, "pixel_to_ground", {"camera": "head", "u": WIDTH / 2, "v": HEIGHT / 2}).structured_content
    ex, _ = front_ground_of_center()
    assert data["ground_base_link"]["x"] == pytest.approx(ex, abs=1e-3)
    assert "approximate intrinsics" not in data["uncertainty_note"]
    assert "head" in robot.camera_info_calls
    meta = text_json(call(server, "get_camera_image", {"camera": "head"}))
    assert meta["calibrated_image"] == {"width": WIDTH, "height": HEIGHT}


def test_annotated_lookup_view_hfov_from_camera_info(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path)
    robot.camera_infos["head"] = SYNTHETIC_INFO
    call(server, "get_annotated_camera_image", {"camera": "head"})
    rows = robot.photo_store.root / "index.jsonl"
    assert rows.exists()
    row = json.loads(rows.read_text().splitlines()[0])
    assert row["view"]["hfov"] == pytest.approx(math.radians(90.0))


def test_camera_tools_not_calibrated_without_camera_info(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path)
    with pytest.raises(ToolError) as exc:
        call(server, "pixel_to_ground", {"camera": "head", "u": 1.0, "v": 1.0})
    assert NOT_CALIBRATED in str(exc.value)
    meta = text_json(call(server, "get_camera_image", {"camera": "head"}))
    assert meta["calibrated_image"] is None
    robot.camera_infos["head"] = replace(SYNTHETIC_INFO, k=(0.0,) * 9)  # a zero K: no factory calibration
    with pytest.raises(ToolError) as exc:
        call(server, "pixel_to_ground", {"camera": "head", "u": 1.0, "v": 1.0})
    assert NOT_CALIBRATED in str(exc.value)
