"""ros_iface.RosRobot on-demand camera, scan and /tf subscriptions without ROS (conftest ros_iface fixture mocks rclpy).

Covers: an idle node has no camera, scan or /tf subscription (only /tf_static); a camera tool call subscribes the
camera and returns a real JPEG frame delivered by a fake publisher; the subscription is destroyed after
on_demand.linger_s without use, kept while used, and created again on the next call; scan_points subscribes the scan,
waits at most timeouts.state_stale_s for a first message and drops its cached sample when reaped; a TF lookup subscribes
/tf, and reaping clears the TF buffer and re-inserts the static transforms; the motion-critical topics
(MOTION_CRITICAL_TOPICS plus the battery topic) are subscribed at start and never destroyed; the linger config field
and its client.yml value.
"""

import json
import threading
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
import yaml
from pydantic import ValidationError

from mcp_server.config import McpServerConfig, OnDemandSettings
from mcp_server.monitor import RobotMonitor

REPO_ROOT = Path(__file__).resolve().parents[3]
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
CAMERA_TOPICS = ("/camera_0/image_raw/compressed", "/front_camera/image_raw/compressed")
SCAN_TOPIC = "/scan_filtered"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
ON_DEMAND_TOPICS = (*CAMERA_TOPICS, SCAN_TOPIC, TF_TOPIC)
BATTERY = {"topic": "/battery/state"}
PUBLISH_WAIT_S = 2.0


class Rig:
    """A RosRobot on mocked ROS whose node hands out one distinct subscription mock per create_subscription call."""

    def __init__(
        self, module: ModuleType, monkeypatch: pytest.MonkeyPatch, config: McpServerConfig, battery_guard: Any = None
    ) -> None:
        self.node = MagicMock()
        self.subs: list[tuple[str, Any, Any]] = []  # (topic, callback, subscription)
        self.subs_lock = threading.Lock()

        def create_subscription(msg_type: type, topic: str, callback: Any, qos: Any, **kwargs: Any) -> Any:
            sub = MagicMock(name=f"sub {topic}")
            with self.subs_lock:
                self.subs.append((topic, callback, sub))
            return sub

        self.node.create_subscription.side_effect = create_subscription
        monkeypatch.setattr(module, "Node", lambda name: self.node)
        self.robot = module.RosRobot(config, battery_guard)
        self.robot.monitor = MagicMock()
        self.clock = [1000.0]
        self.robot.demand_clock = lambda: self.clock[0]

        def sleep(seconds: float) -> None:
            self.clock[0] += seconds

        self.robot.demand_sleep = sleep

    def topics(self) -> list[str]:
        """Topics of every create_subscription call so far."""
        with self.subs_lock:
            return [topic for topic, _cb, _sub in self.subs]

    def latest(self, topic: str) -> tuple[Any, Any] | None:
        """(callback, subscription) of the newest subscription on a topic, or None."""
        with self.subs_lock:
            found = [(cb, sub) for t, cb, sub in self.subs if t == topic]
        return found[-1] if found else None

    def destroyed(self) -> list[Any]:
        """Subscriptions passed to node.destroy_subscription."""
        return [c.args[0] for c in self.node.destroy_subscription.call_args_list]

    def destroyed_topics(self) -> set[str]:
        """Topics of the destroyed subscriptions."""
        gone = {id(s) for s in self.destroyed()}
        with self.subs_lock:
            return {t for t, _cb, sub in self.subs if id(sub) in gone}


@pytest.fixture
def rig(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> Rig:
    return Rig(ros_iface, monkeypatch, McpServerConfig())


def real_jpeg(width: int = 64, height: int = 48) -> bytes:
    """A real JPEG with a gradient (decodable by cv2)."""
    image = np.tile(np.linspace(0, 255, width, dtype=np.uint8), (height, 1))
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
    assert ok
    return buf.tobytes()


def fake_camera_publisher(rig: Rig, topic: str, jpeg: bytes, stop: threading.Event) -> threading.Thread:
    """Thread that publishes a CompressedImage-like message every 10 ms to the newest subscription on topic."""

    def run() -> None:
        deadline = time.monotonic() + PUBLISH_WAIT_S
        while not stop.is_set() and time.monotonic() < deadline:
            found = rig.latest(topic)
            if found is not None:
                stamp = SimpleNamespace(sec=5, nanosec=0)
                found[0](SimpleNamespace(header=SimpleNamespace(stamp=stamp), data=jpeg))
            time.sleep(0.01)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def camera_frame(rig: Rig, camera: str, topic: str) -> Any:
    """Run one camera_image call against a fake publisher of a real JPEG."""
    rig.robot.ros_age = lambda stamp: 0.05
    stop = threading.Event()
    publisher = fake_camera_publisher(rig, topic, real_jpeg(), stop)
    try:
        return rig.robot.camera_image(camera, 640)
    finally:
        stop.set()
        publisher.join(PUBLISH_WAIT_S)


def test_idle_node_has_no_camera_scan_or_tf_subscription(rig: Rig) -> None:
    topics = rig.topics()
    for topic in ON_DEMAND_TOPICS:
        assert topic not in topics, f"{topic} must not be subscribed while no tool needs it"
    assert TF_STATIC_TOPIC in topics  # static transforms stay subscribed (latched, ~0 msg/s)


def test_camera_tool_call_subscribes_and_returns_a_real_frame(rig: Rig) -> None:
    frame = camera_frame(rig, "gripper", CAMERA_TOPICS[0])
    assert rig.topics().count(CAMERA_TOPICS[0]) == 1
    assert CAMERA_TOPICS[1] not in rig.topics()  # only the camera that was asked for
    assert (frame.width, frame.height) == (64, 48)
    decoded = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded is not None and decoded.shape[:2] == (48, 64)
    assert frame.topic == CAMERA_TOPICS[0]
    assert frame.stamp_s == 5.0


def test_camera_subscription_destroyed_after_idle_linger_and_recreated(rig: Rig) -> None:
    camera_frame(rig, "front", CAMERA_TOPICS[1])
    _cb, sub = rig.latest(CAMERA_TOPICS[1])
    linger = rig.robot.cfg.on_demand.linger_s
    rig.clock[0] += linger - 1.0
    rig.robot.reap_on_demand()
    assert rig.destroyed() == []
    camera_frame(rig, "front", CAMERA_TOPICS[1])  # a use restarts the linger
    rig.clock[0] += linger - 1.0
    rig.robot.reap_on_demand()
    assert rig.destroyed() == []
    rig.clock[0] += 1.5
    rig.robot.reap_on_demand()
    assert rig.destroyed() == [sub]
    rig.robot.reap_on_demand()
    assert rig.destroyed() == [sub]  # destroyed once
    camera_frame(rig, "front", CAMERA_TOPICS[1])
    assert rig.topics().count(CAMERA_TOPICS[1]) == 2


def test_camera_call_without_publisher_fails_after_image_timeout(ros_iface: ModuleType, monkeypatch: Any) -> None:
    rig = Rig(ros_iface, monkeypatch, McpServerConfig(timeouts={"image_timeout_s": 0.05}))
    with pytest.raises(ros_iface.RobotError, match="no frame"):
        rig.robot.camera_image("gripper", 640)
    assert CAMERA_TOPICS[0] in rig.topics()


def scan_msg() -> SimpleNamespace:
    """A three-ray LaserScan-like message in the laser frame."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="laser"),
        ranges=[1.0, 2.0, float("inf")],
        angle_min=0.0,
        angle_increment=0.1,
        range_min=0.1,
        range_max=10.0,
    )


def identity_tf() -> SimpleNamespace:
    """A TransformStamped-like identity transform."""
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def test_scan_points_subscribes_waits_bounded_then_reads(rig: Rig) -> None:
    rig.robot.tf_buffer.lookup_transform.return_value = identity_tf()
    start = rig.clock[0]
    assert rig.robot.scan_points() is None  # no message yet
    assert rig.topics().count(SCAN_TOPIC) == 1
    waited = rig.clock[0] - start
    stale = rig.robot.cfg.timeouts.state_stale_s
    assert stale <= waited < stale + 0.2
    before = rig.clock[0]
    assert rig.robot.scan_points() is None
    assert rig.clock[0] == before  # past the first-message window: no second wait
    callback, _sub = rig.latest(SCAN_TOPIC)
    callback(scan_msg())
    points = rig.robot.scan_points()
    assert points is not None and len(points.points) == 2
    assert rig.topics().count(SCAN_TOPIC) == 1


def test_scan_reaped_drops_its_sample(rig: Rig) -> None:
    rig.robot.tf_buffer.lookup_transform.return_value = identity_tf()
    rig.robot.scan_points()
    callback, sub = rig.latest(SCAN_TOPIC)
    callback(scan_msg())
    assert "scan" in rig.robot.latest
    rig.clock[0] += rig.robot.cfg.on_demand.linger_s + 1.0
    rig.robot.reap_on_demand()
    assert rig.destroyed() == [sub]
    assert "scan" not in rig.robot.latest


def test_tf_lookup_subscribes_tf_and_reap_clears_dynamic_keeps_static(rig: Rig) -> None:
    static = SimpleNamespace(child_frame_id="laser")
    static_cb, _static_sub = rig.latest(TF_STATIC_TOPIC)
    static_cb(SimpleNamespace(transforms=[static]))
    rig.robot.tf_buffer.set_transform_static.assert_called_with(static, "default_authority")
    rig.robot.tf_buffer.lookup_transform.return_value = identity_tf()
    pose = rig.robot.lookup_pose("map", "base_link")
    assert pose is not None
    assert rig.topics().count(TF_TOPIC) == 1
    timeout_kw = rig.robot.tf_buffer.lookup_transform.call_args.kwargs["timeout"]
    assert timeout_kw is not None  # bounded wait
    tf_cb, tf_sub = rig.latest(TF_TOPIC)
    dynamic = SimpleNamespace(child_frame_id="base_link")
    tf_cb(SimpleNamespace(transforms=[dynamic]))
    rig.robot.tf_buffer.set_transform.assert_called_with(dynamic, "default_authority")
    rig.robot.lookup_pose("map", "base_link")
    assert rig.topics().count(TF_TOPIC) == 1
    rig.robot.tf_buffer.reset_mock()
    rig.clock[0] += rig.robot.cfg.on_demand.linger_s + 1.0
    rig.robot.reap_on_demand()
    assert rig.destroyed() == [tf_sub]
    rig.robot.tf_buffer.clear.assert_called_once()
    rig.robot.tf_buffer.set_transform_static.assert_called_with(static, "default_authority")
    assert TF_STATIC_TOPIC not in rig.destroyed_topics()


def test_first_tf_lookup_after_subscribing_waits_longer_than_tf_timeout(
    ros_iface: ModuleType, monkeypatch: Any
) -> None:
    durations: list[float] = []
    monkeypatch.setattr(ros_iface, "Duration", lambda seconds: durations.append(seconds) or seconds)
    rig = Rig(ros_iface, monkeypatch, McpServerConfig())
    rig.robot.tf_buffer.lookup_transform.return_value = identity_tf()
    rig.robot.lookup_pose("map", "base_link")
    t = rig.robot.cfg.timeouts
    assert durations[-1] == pytest.approx(max(t.tf_timeout_s, t.state_stale_s))
    rig.clock[0] += t.state_stale_s + 0.1
    rig.robot.lookup_pose("map", "base_link")
    assert durations[-1] == pytest.approx(t.tf_timeout_s)


def critical_topics(module: ModuleType, config: McpServerConfig) -> list[str]:
    """Every motion-critical topic name of the config (topic fields plus the battery topic); an idle node reads a
    dual-rate joint-state field from its throttled copy (test_joint_state_dual_rate.py)."""
    suffix = config.joint_state_rate.throttled_suffix
    topics = [
        getattr(config.topics, field) + (suffix if field in module.DUAL_RATE_TOPICS else "")
        for field in module.MOTION_CRITICAL_TOPICS
    ]
    if config.battery is not None:
        topics.append(config.battery.topic)
    return topics


def test_motion_critical_list_names_the_monitor_and_motion_feeds(ros_iface: ModuleType) -> None:
    expected = {
        "follower_joint_states",
        "follower_joint_currents",
        "active_source",
        "odom",
        "swerve_odom",
        "rf2o_twist",
        "imu",
        "cmd_vel",
        "servo_registers",
        "base_servo_registers",
        "tilt_guard_state",
        "collision_monitor_state",
        "leader_joint_updates",
    }
    assert expected <= set(ros_iface.MOTION_CRITICAL_TOPICS)
    assert set(ros_iface.MOTION_CRITICAL_TOPICS) <= set(type(McpServerConfig().topics).model_fields)


def test_motion_critical_topics_subscribed_at_start_and_never_destroyed(
    ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = McpServerConfig(battery=BATTERY)
    rig = Rig(ros_iface, monkeypatch, config, battery_guard=MagicMock())
    at_start = list(rig.topics())
    critical = critical_topics(ros_iface, config)
    for topic in critical:
        assert topic in at_start, f"{topic} must be subscribed at start"
    # Use every on-demand topic, then idle far past the linger and reap repeatedly.
    rig.robot.tf_buffer.lookup_transform.return_value = identity_tf()
    rig.robot.scan_points()
    rig.robot.lookup_pose("map", "base_link")
    camera_frame(rig, "gripper", CAMERA_TOPICS[0])
    camera_frame(rig, "front", CAMERA_TOPICS[1])
    for _ in range(5):
        rig.clock[0] += config.on_demand.linger_s
        rig.robot.reap_on_demand()
        rig.robot.reap_steer_states()
    assert rig.destroyed_topics() == set(ON_DEMAND_TOPICS)
    for topic in critical:
        assert rig.topics().count(topic) == 1, f"{topic} must be subscribed exactly once"


def test_base_servo_registers_dump_reaches_the_monitor(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    config = McpServerConfig()
    rig = Rig(ros_iface, monkeypatch, config)
    monitor = RobotMonitor(config.monitor, None)
    rig.robot.monitor = monitor
    found = rig.latest("/swerve_drive/servo_registers")
    assert found is not None, "base_servo_registers must be subscribed at start"
    callback, _sub = found
    callback(SimpleNamespace(data=json.dumps({"fl_drive": {"present_temperature": 70, "status": 0}})))
    assert "fl_drive" in monitor.servos
    assert monitor.level("overheat", "fl_drive") == "critical"


def test_reaper_timer_runs_well_inside_the_linger(rig: Rig) -> None:
    linger = rig.robot.cfg.on_demand.linger_s
    periods = [c.args[0] for c in rig.node.create_timer.call_args_list if c.args[1] == rig.robot.reap_on_demand]
    assert len(periods) == 1
    assert 0.0 < periods[0] <= linger / 2


def test_linger_config_default_and_validation() -> None:
    assert McpServerConfig().on_demand.linger_s == 30.0
    assert OnDemandSettings(linger_s=5.0).linger_s == 5.0
    with pytest.raises(ValidationError):
        OnDemandSettings(linger_s=0.0)


def test_client_yml_sets_the_linger() -> None:
    doc = yaml.safe_load(CLIENT_VARS.read_text())
    node = next(n for n in doc["ros2_nodes"] if n["name"] == "mcp_server")
    deployed = yaml.safe_load(node["config"])
    assert deployed["on_demand"]["linger_s"] == McpServerConfig.model_validate(deployed).on_demand.linger_s
    assert deployed["on_demand"]["linger_s"] > 0
