"""Tests for on-demand ROS subscriptions: a topic is subscribed only while a WebSocket client wants it."""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable, Iterator
from contextlib import ExitStack, contextmanager
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient
from ros2_common.maps import RESULT_TOPIC, STATUS_TOPIC

from web_ui.bridge import (
    DEMAND_LINGER_S,
    GPS_ANCHOR_TOPIC,
    MAP_GUARD_RESET_TOPIC,
    MAP_SUB_QOS,
    ROBOT_POSE_TOPIC,
    TF_LISTENER_JOIN_TIMEOUT_S,
    BridgeNode,
)
from web_ui.config import AppConfig, TabConfig
from web_ui.gps_anchor import CompassSettings
from web_ui.tilt_guard import TILT_GUARD_ERROR_SOURCE, TILT_GUARD_STATE_TOPIC

ON_DEMAND = {"/odom", "/camera_0/image_raw/compressed", "/plan", "/poi/list", "/client/gps/status", "/h/rgb"}
ALWAYS_ON_ROLES = {
    "/map": "map",
    "/battery": "battery",
    "/poi/result": "poi_result",
    "/grasp/result": "grasp_result",
    "/client/gps/fix": "gps",
}
ROLES = {
    "/plan": "path",
    "/poi/list": "poi_list",
    "/client/gps/status": "gps_status",
    "/h/rgb": "head_rgb",
    **ALWAYS_ON_ROLES,
}
ALWAYS_ON = {*ALWAYS_ON_ROLES, MAP_GUARD_RESET_TOPIC, TILT_GUARD_STATE_TOPIC, STATUS_TOPIC, RESULT_TOPIC}
WARNING = {"level": "warning", "roll_deg": 12.5, "pitch_deg": -3.0}


class FakeRos:
    """Records the subscriptions, timers and TF listeners the bridge creates and destroys."""

    def __init__(self) -> None:
        self.subs: list[Any] = []
        self.destroyed: list[Any] = []
        self.timers: list[Any] = []
        self.destroyed_timers: list[Any] = []
        self.listeners: list[Any] = []
        self.publishers: dict[str, Any] = {}

    def create_subscription(self, msg_cls: Any, topic: str, callback: Any, qos: Any) -> Any:
        sub = SimpleNamespace(msg_cls=msg_cls, topic=topic, callback=callback, qos=qos)
        self.subs.append(sub)
        return sub

    def create_timer(self, period: float, callback: Any) -> Any:
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def create_publisher(self, msg_cls: Any, topic: str, qos: Any) -> Any:
        pub = MagicMock()
        pub.get_subscription_count.return_value = 1
        self.publishers[topic] = pub
        return pub

    def transform_listener(self, *args: Any, **kwargs: Any) -> Any:
        listener = MagicMock()
        listener.call_args = (args, kwargs)
        self.listeners.append(listener)
        return listener

    def live(self) -> list[Any]:
        """Subscriptions created and not destroyed."""
        return [s for s in self.subs if not any(s is d for d in self.destroyed)]

    def live_topics(self) -> list[str]:
        """Topics of the live subscriptions (duplicates kept)."""
        return [s.topic for s in self.live()]

    def created_topics(self) -> list[str]:
        """Topics of every subscription ever created (duplicates kept)."""
        return [s.topic for s in self.subs]

    def callback(self, topic: str) -> Any:
        """Callback of the live subscription of topic."""
        return next(s.callback for s in self.live() if s.topic == topic)


@pytest.fixture
def ros() -> Iterator[FakeRos]:
    fake = FakeRos()
    with ExitStack() as stack:
        stack.enter_context(patch("web_ui.bridge.Node.__init__", return_value=None))
        for name, side_effect in (
            ("create_subscription", fake.create_subscription),
            ("destroy_subscription", fake.destroyed.append),
            ("create_timer", fake.create_timer),
            ("destroy_timer", fake.destroyed_timers.append),
            ("create_publisher", fake.create_publisher),
            ("get_logger", None),
            ("create_client", None),
            ("get_clock", None),
        ):
            stack.enter_context(patch.object(BridgeNode, name, create=True, side_effect=side_effect))
        stack.enter_context(patch("web_ui.bridge.Buffer"))
        stack.enter_context(patch("web_ui.bridge.TransformListener", side_effect=fake.transform_listener))
        yield fake


def demand(node: BridgeNode, wanted: Any, now: float | None = None) -> None:
    """Record the wanted topics and run the ROS-thread apply step (what the bridge timer does)."""
    node.update_demand(wanted)
    node.apply_demand(now)


def make_node(**kwargs: Any) -> BridgeNode:
    """BridgeNode over every role kind; extra kwargs override the defaults."""
    args: dict[str, Any] = {
        "topics": [*sorted(ON_DEMAND), *ALWAYS_ON_ROLES],
        "allowed_publish_topics": set(),
        "topic_roles": ROLES,
        "battery_guard": MagicMock(),
        "gps_anchor": MagicMock(),
        "robot_pose_frames": ("map", "base_link"),
        "poi_command_topic": "/poi/command",
        "grasp_command_topic": "/grasp/command",
    }
    args.update(kwargs)
    return BridgeNode(**args)


# --- subscription lifecycle --------------------------------------------------------------------------------------


def test_zero_clients_means_zero_on_demand_subscriptions(ros: FakeRos) -> None:
    node = make_node()
    assert set(ros.live_topics()) == ALWAYS_ON
    demand(node, set(), now=0.0)
    demand(node, set(), now=DEMAND_LINGER_S * 10)
    assert set(ros.live_topics()) == ALWAYS_ON
    assert [t for t in ros.timers if t.callback == node.update_robot_pose] == []
    assert node.always_on_topics() == ALWAYS_ON


def test_client_subscribe_creates_the_subscription_once(ros: FakeRos) -> None:
    node = make_node()
    demand(node, {"/odom"}, now=0.0)
    demand(node, {"/odom"}, now=1.0)
    demand(node, {"/odom", "/not/configured"}, now=2.0)
    assert ros.created_topics().count("/odom") == 1
    assert set(ros.live_topics()) == ALWAYS_ON | {"/odom"}


def test_last_client_leaving_destroys_after_linger(ros: FakeRos) -> None:
    node = make_node()
    demand(node, {"/odom"}, now=0.0)
    node._make_callback("/odom", lambda msg: {"v": msg}, None)(1)
    demand(node, set(), now=10.0)
    demand(node, set(), now=10.0 + DEMAND_LINGER_S - 0.1)
    assert "/odom" in ros.live_topics()
    demand(node, set(), now=10.0 + DEMAND_LINGER_S + 0.1)
    assert "/odom" not in ros.live_topics()
    assert [s.topic for s in ros.destroyed] == ["/odom"]
    assert node.latest_envelopes({"/odom"}) == []
    assert "/odom" not in node._topic_last_rx


def test_topic_wanted_again_within_linger_keeps_its_subscription(ros: FakeRos) -> None:
    node = make_node()
    demand(node, {"/odom"}, now=0.0)
    demand(node, set(), now=1.0)
    demand(node, {"/odom"}, now=1.0 + DEMAND_LINGER_S - 1.0)
    demand(node, {"/odom"}, now=1.0 + DEMAND_LINGER_S + 1.0)
    assert ros.created_topics().count("/odom") == 1
    assert ros.destroyed == []


def test_unwanted_again_after_rewant_restarts_the_linger(ros: FakeRos) -> None:
    node = make_node()
    demand(node, {"/odom"}, now=0.0)
    demand(node, set(), now=1.0)
    demand(node, {"/odom"}, now=2.0)
    demand(node, set(), now=3.0)
    demand(node, set(), now=1.0 + DEMAND_LINGER_S + 0.5)
    assert "/odom" in ros.live_topics()
    demand(node, set(), now=3.0 + DEMAND_LINGER_S + 0.1)
    assert "/odom" not in ros.live_topics()


def test_resubscribe_after_destroy_creates_a_new_subscription(ros: FakeRos) -> None:
    node = make_node()
    demand(node, {"/odom"}, now=0.0)
    demand(node, set(), now=1.0)
    demand(node, set(), now=2.0 + DEMAND_LINGER_S)
    demand(node, {"/odom"}, now=10.0)
    assert ros.created_topics().count("/odom") == 2
    assert ros.live_topics().count("/odom") == 1


def test_wanted_none_subscribes_every_configured_topic(ros: FakeRos) -> None:
    node = make_node()
    demand(node, None, now=0.0)
    assert set(ros.live_topics()) == ALWAYS_ON | ON_DEMAND


def test_always_on_subscriptions_are_never_destroyed(ros: FakeRos) -> None:
    node = make_node()
    always = ros.live()
    demand(node, None, now=0.0)
    demand(node, set(), now=1.0)
    demand(node, set(), now=100.0)
    demand(node, set(ALWAYS_ON), now=200.0)
    demand(node, set(), now=300.0)
    assert all(any(s is a for s in ros.live()) for a in always)
    assert not any(any(d is a for a in always) for d in ros.destroyed)
    assert set(ros.live_topics()) == ALWAYS_ON


def test_latched_topic_is_resubscribed_transient_local(ros: FakeRos) -> None:
    node = make_node()
    for now in (0.0, 1.0, 2.0 + DEMAND_LINGER_S):
        demand(node, {"/poi/list"} if now == 0.0 else set(), now=now)
    demand(node, {"/poi/list"}, now=20.0)
    created = [s for s in ros.subs if s.topic == "/poi/list"]
    assert len(created) == 2
    assert all(s.qos is MAP_SUB_QOS for s in created)


def test_gps_fix_is_on_demand_without_an_anchor_fit(ros: FakeRos) -> None:
    make_node(gps_anchor=None)
    assert "/client/gps/fix" not in ros.live_topics()


def test_battery_without_guard_is_not_subscribed(ros: FakeRos) -> None:
    node = make_node(battery_guard=None)
    demand(node, None, now=0.0)
    assert "/battery" not in ros.created_topics()


# --- no placeholder data ------------------------------------------------------------------------------------------


def test_topic_without_subscription_has_no_sample(ros: FakeRos) -> None:
    node = make_node()
    assert node.latest_envelopes({"/odom"}) == []
    demand(node, {"/odom"}, now=0.0)
    assert node.latest_envelopes({"/odom"}) == []
    assert node.flush_dirty({"/odom"}) == []


def test_inactive_on_demand_topics_are_not_reported_stale(ros: FakeRos) -> None:
    node = make_node()
    assert ON_DEMAND.isdisjoint(node._topic_last_rx)
    demand(node, {"/odom"}, now=0.0)
    assert "/odom" in node._topic_last_rx
    demand(node, set(), now=1.0)
    demand(node, set(), now=2.0 + DEMAND_LINGER_S)
    assert "/odom" not in node._topic_last_rx


def test_late_callback_after_destroy_does_not_resume_health_tracking(ros: FakeRos) -> None:
    node = make_node()
    demand(node, {"/odom"}, now=0.0)
    callback = ros.callback("/odom")
    demand(node, set(), now=1.0)
    demand(node, set(), now=2.0 + DEMAND_LINGER_S)
    callback(SimpleNamespace())
    assert "/odom" not in node._topic_last_rx


# --- synthetic topics: robot pose, GPS anchor compass, TF --------------------------------------------------------


def test_robot_pose_timer_runs_only_while_the_pose_is_wanted(ros: FakeRos) -> None:
    node = make_node()
    assert [t for t in ros.timers if t.callback == node.update_robot_pose] == []
    demand(node, {ROBOT_POSE_TOPIC}, now=0.0)
    pose_timers = [t for t in ros.timers if t.callback == node.update_robot_pose]
    assert len(pose_timers) == 1
    node._latest[ROBOT_POSE_TOPIC] = {"topic": ROBOT_POSE_TOPIC, "data": {"x": 1.0}}
    demand(node, set(), now=1.0)
    demand(node, set(), now=2.0 + DEMAND_LINGER_S)
    assert ros.destroyed_timers == pose_timers
    assert node.latest_envelopes({ROBOT_POSE_TOPIC}) == []


def test_compass_imu_is_subscribed_only_while_the_gps_anchor_is_wanted(ros: FakeRos) -> None:
    node = make_node(gps_compass=CompassSettings(imu_topic="/imu/data", calibration_topic="/imu/calibration"))
    assert {"/imu/data", "/imu/calibration"}.isdisjoint(ros.live_topics())
    demand(node, {GPS_ANCHOR_TOPIC}, now=0.0)
    assert {"/imu/data", "/imu/calibration"} <= set(ros.live_topics())
    node.on_anchor_imu(
        SimpleNamespace(
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=[0.01] + [0.0] * 8,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id="imu"),
        )
    )
    demand(node, set(), now=1.0)
    demand(node, set(), now=2.0 + DEMAND_LINGER_S)
    assert {"/imu/data", "/imu/calibration"}.isdisjoint(ros.live_topics())
    assert node._imu_heading is None


def test_tf_stays_on_while_the_gps_anchor_fit_needs_it(ros: FakeRos) -> None:
    node = make_node()
    assert len(ros.listeners) == 1
    demand(node, set(), now=0.0)
    demand(node, set(), now=100.0)
    ros.listeners[0].executor.shutdown.assert_not_called()


def test_tf_listener_follows_demand_without_an_anchor_fit(ros: FakeRos) -> None:
    node = make_node(gps_anchor=None)
    assert ros.listeners == []
    demand(node, {"/odom"}, now=0.0)
    assert ros.listeners == []
    demand(node, {"/odom", ROBOT_POSE_TOPIC}, now=1.0)
    assert len(ros.listeners) == 1
    demand(node, {"/odom", "/plan"}, now=2.0)
    demand(node, {"/odom", "/plan"}, now=3.0 + DEMAND_LINGER_S)
    ros.listeners[0].executor.shutdown.assert_not_called()  # /plan (map-frame role) still needs TF
    demand(node, {"/odom"}, now=10.0)
    demand(node, {"/odom"}, now=11.0 + DEMAND_LINGER_S)
    listener = ros.listeners[0]
    listener.executor.shutdown.assert_called_once()
    listener.dedicated_listener_thread.join.assert_called_once_with(timeout=TF_LISTENER_JOIN_TIMEOUT_S)
    listener.node.destroy_node.assert_called_once()
    listener.unregister.assert_not_called()  # would destroy subscriptions the dedicated thread may still spin
    node._tf_buffer.clear.assert_called()
    demand(node, {ROBOT_POSE_TOPIC}, now=30.0)
    assert len(ros.listeners) == 2
    assert ros.listeners[1] is not listener


def test_tf_listener_runs_on_its_own_node_and_thread(ros: FakeRos) -> None:
    node = make_node()
    args, kwargs = ros.listeners[0].call_args
    assert args == (node._tf_buffer, None)
    assert kwargs == {"spin_thread": True}


def test_tf_listener_stop_orders_shutdown_join_destroy_then_clear(ros: FakeRos) -> None:
    node = make_node(gps_anchor=None)
    demand(node, {ROBOT_POSE_TOPIC}, now=0.0)
    order = MagicMock()
    listener = ros.listeners[0]
    order.attach_mock(listener.executor.shutdown, "shutdown")
    order.attach_mock(listener.dedicated_listener_thread.join, "join")
    order.attach_mock(listener.node.destroy_node, "destroy")
    order.attach_mock(node._tf_buffer.clear, "clear")
    demand(node, set(), now=1.0)
    demand(node, set(), now=2.0 + DEMAND_LINGER_S)
    assert [c[0] for c in order.mock_calls] == ["shutdown", "join", "destroy", "clear"]


def test_tf_listener_join_timeout_is_logged_and_node_still_destroyed(ros: FakeRos) -> None:
    node = make_node(gps_anchor=None)
    demand(node, {ROBOT_POSE_TOPIC}, now=0.0)
    listener = ros.listeners[0]
    listener.dedicated_listener_thread.is_alive.return_value = True
    with patch("web_ui.bridge.log") as log:
        demand(node, set(), now=1.0)
        demand(node, set(), now=2.0 + DEMAND_LINGER_S)
    log.warning.assert_called()
    assert node._tf_listener is None


# --- server: the client topic sets drive the demand --------------------------------------------------------------


def wait_for(condition: Callable[[], bool], timeout_s: float = 2.0) -> None:
    """Poll until condition() holds (the server handles WebSocket messages on its own thread)."""
    deadline = time.monotonic() + timeout_s
    while not condition():
        assert time.monotonic() < deadline, "condition not met in time"
        time.sleep(0.01)


@contextmanager
def spinning(node: BridgeNode) -> Iterator[None]:
    """Run the bridge's apply step repeatedly on a thread, like the executor thread does with its timer."""
    stop = threading.Event()

    def spin() -> None:
        while not stop.wait(0.005):
            node.apply_demand()

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        yield
    finally:
        stop.set()
        thread.join()


def make_app(tmp_path: Path, urdf_dir: Path, node: BridgeNode) -> Any:
    from web_ui.server import build_app

    config = AppConfig(
        tabs=[
            TabConfig(
                id="map",
                type="map_nav",
                label="Map",
                goal_topic="/goal_pose",
                poi_list_topic="/poi/list",
                poi_command_topic="/poi/command",
                poi_result_topic="/poi/result",
                grasp_command_topic="/grasp/command",
                grasp_result_topic="/grasp/result",
            )
        ]
    )
    return build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=node)


def test_two_clients_share_one_subscription_destroyed_after_the_last_leaves(
    ros: FakeRos, tmp_path: Path, urdf_dir: Path
) -> None:
    node = make_node()
    client = TestClient(make_app(tmp_path, urdf_dir, node))
    with patch("web_ui.server.WS_RECONNECT_MIN_INTERVAL_S", 0.0), spinning(node):
        with client.websocket_connect("/ws") as first:
            first.send_json({"type": "subscribe", "topics": ["/odom"]})
            wait_for(lambda: "/odom" in ros.live_topics())
            with client.websocket_connect("/ws") as second:
                second.send_json({"type": "subscribe", "topics": ["/odom", "/plan"]})
                wait_for(lambda: "/plan" in ros.live_topics())
                assert ros.created_topics().count("/odom") == 1
            wait_for(lambda: node.lingering_topics() == {"/plan"})
            assert ros.live_topics().count("/odom") == 1
        wait_for(lambda: node.lingering_topics() == {"/odom", "/plan"})
        assert {"/odom", "/plan"} <= set(ros.live_topics())  # lingering, not destroyed yet
    demand(node, set(), now=time.monotonic() + DEMAND_LINGER_S + 1.0)
    assert set(ros.live_topics()) == ALWAYS_ON
    assert ros.created_topics().count("/odom") == 1


def test_demanded_topics_ignores_a_client_still_inside_its_subscribe_grace() -> None:
    from web_ui.server import UNSUBSCRIBED_GRACE_S, demanded_topics

    narrowed = SimpleNamespace(topics=frozenset({"/x"}))
    fresh = SimpleNamespace(topics=None)
    clients = {"a": narrowed, "b": fresh}
    connected_at = {"a": 0.0, "b": 10.0}
    assert demanded_topics(clients, connected_at, 10.0 + UNSUBSCRIBED_GRACE_S / 2) == {"/x"}
    assert demanded_topics(clients, connected_at, 10.0 + UNSUBSCRIBED_GRACE_S + 0.1) is None
    assert demanded_topics({}, {}, 0.0) == set()


def test_broadcaster_updates_demand_even_with_no_client() -> None:
    import asyncio

    from web_ui.server import _make_start_broadcaster

    bridge = MagicMock()

    async def main() -> None:
        start = _make_start_broadcaster(MagicMock(), {}, bridge, 0.01, MagicMock(), {})
        await start()
        await asyncio.sleep(0.1)
        for task in asyncio.all_tasks() - {asyncio.current_task()}:
            task.cancel()

    asyncio.run(main())
    assert bridge.update_demand.call_args.args[0] == set()
    bridge.flush_dirty.assert_not_called()


# --- REST routes and safety state keep working with zero clients -------------------------------------------------


def answer_on_publish(ros: FakeRos, node: BridgeNode, command_topic: str, result_topic: str, extra: dict) -> None:
    """Make the command publisher answer every request on result_topic through its always-on subscription."""

    def publish(msg: Any) -> None:
        request = json.loads(msg.data)
        ros.callback(result_topic)(SimpleNamespace(data=json.dumps({"request_id": request["request_id"], **extra})))

    ros.publishers[command_topic].publish.side_effect = publish


def test_rest_grasp_stop_and_poi_resolve_with_zero_clients(ros: FakeRos, tmp_path: Path, urdf_dir: Path) -> None:
    node = make_node()
    answer_on_publish(ros, node, "/grasp/command", "/grasp/result", {"action": "stop", "ok": True})
    answer_on_publish(ros, node, "/poi/command", "/poi/result", {"ok": True, "message": "added"})
    client = TestClient(make_app(tmp_path, urdf_dir, node))
    assert client.post("/api/grasp/stop?tab=map").json()["ok"] is True
    response = client.post("/api/poi?tab=map", json={"op": "add", "poi": {"name": "a"}})
    assert response.json()["message"] == "added"
    assert set(ros.live_topics()) == ALWAYS_ON


def test_tilt_state_and_battery_are_fed_with_zero_clients(ros: FakeRos, tmp_path: Path, urdf_dir: Path) -> None:
    guard = MagicMock()
    guard.cells = 3
    guard.update.return_value = True
    guard.state.return_value = {"cutoff": False}
    node = make_node(battery_guard=guard)
    ros.callback(TILT_GUARD_STATE_TOPIC)(SimpleNamespace(data=json.dumps(WARNING)))
    assert node.tilt_state() == WARNING
    with patch("web_ui.bridge.serialize_battery", return_value={"voltage": 11.1}):
        ros.callback("/battery")(SimpleNamespace())
    guard.update.assert_called_once_with(11.1)
    client = TestClient(make_app(tmp_path, urdf_dir, node))
    with patch("web_ui.server.WS_RECONNECT_MIN_INTERVAL_S", 0.0), client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "subscribe", "topics": []})
        ws.send_json({"type": "publish", "topic": "/goal_pose", "msg_type": "geometry_msgs/PoseStamped", "data": {}})
        frames = [ws.receive_json() for _ in range(3)]
    assert any(f.get("type") == "error" and f.get("source") == TILT_GUARD_ERROR_SOURCE for f in frames)
