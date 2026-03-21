"""Unit tests for master2master proxy (RelayNode) — rclpy mocked at module level."""

from __future__ import annotations

import sys
import types
from typing import Any
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Build minimal rclpy mock before importing proxy so the module-level
# attributes (RELAY_QOS, RELAY_MESSAGE_TYPES) are resolved against mocks.
# ---------------------------------------------------------------------------


def _make_rclpy_mock() -> types.ModuleType:
    """Return a minimal rclpy mock that satisfies proxy.py imports."""
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.shutdown = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.ok = MagicMock(return_value=False)  # type: ignore[attr-defined]

    # rclpy.node
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = MagicMock(name="Node")  # type: ignore[attr-defined]
    rclpy_mod.node = node_mod  # type: ignore[attr-defined]

    # rclpy.executors
    executors_mod = types.ModuleType("rclpy.executors")
    executors_mod.MultiThreadedExecutor = MagicMock(name="MultiThreadedExecutor")  # type: ignore[attr-defined]
    rclpy_mod.executors = executors_mod  # type: ignore[attr-defined]

    # rclpy.qos
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = MagicMock(name="QoSProfile")  # type: ignore[attr-defined]
    qos_mod.ReliabilityPolicy = MagicMock(name="ReliabilityPolicy")  # type: ignore[attr-defined]
    qos_mod.HistoryPolicy = MagicMock(name="HistoryPolicy")  # type: ignore[attr-defined]
    rclpy_mod.qos = qos_mod  # type: ignore[attr-defined]

    return rclpy_mod


def _make_sensor_msgs_mock() -> types.ModuleType:
    """Return a minimal sensor_msgs mock."""
    sensor_msgs = types.ModuleType("sensor_msgs")
    msg_mod = types.ModuleType("sensor_msgs.msg")
    msg_mod.JointState = MagicMock(name="JointState")  # type: ignore[attr-defined]
    sensor_msgs.msg = msg_mod  # type: ignore[attr-defined]
    return sensor_msgs


def _make_std_msgs_mock() -> types.ModuleType:
    """Return a minimal std_msgs mock."""
    std_msgs = types.ModuleType("std_msgs")
    msg_mod = types.ModuleType("std_msgs.msg")
    msg_mod.String = MagicMock(name="String")  # type: ignore[attr-defined]
    std_msgs.msg = msg_mod  # type: ignore[attr-defined]
    return std_msgs


# Register mocks in sys.modules before any proxy import happens.
_rclpy_mock = _make_rclpy_mock()
_sensor_msgs_mock = _make_sensor_msgs_mock()
_std_msgs_mock = _make_std_msgs_mock()

sys.modules.setdefault("rclpy", _rclpy_mock)
sys.modules.setdefault("rclpy.node", _rclpy_mock.node)
sys.modules.setdefault("rclpy.executors", _rclpy_mock.executors)
sys.modules.setdefault("rclpy.qos", _rclpy_mock.qos)
sys.modules.setdefault("sensor_msgs", _sensor_msgs_mock)
sys.modules.setdefault("sensor_msgs.msg", _sensor_msgs_mock.msg)
sys.modules.setdefault("std_msgs", _std_msgs_mock)
sys.modules.setdefault("std_msgs.msg", _std_msgs_mock.msg)

# Now it is safe to import proxy.
from master2master.config import TopicRule  # noqa: E402
from master2master.proxy import get_message_class, get_supported_message_types, run_all_relays  # noqa: E402

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _single_rule() -> TopicRule:
    """Return a single relay rule for /src -> /dst."""
    return TopicRule(source="/src", dest="/dst", direction="in", msg_type="string")


def _two_rules() -> list[TopicRule]:
    """Return two relay rules with different msg_types."""
    return [
        TopicRule(source="/a", dest="/b", direction="in", msg_type="string"),
        TopicRule(source="/c", dest="/d", direction="out", msg_type="jointstate"),
    ]


class _TrackingNodeBase:
    """Real Python base class that records create_publisher/create_subscription calls.

    Used as a drop-in replacement for rclpy.node.Node so that RelayNode
    (defined inside run_all_relays) can inherit from it without any MagicMock
    subclassing complications.  All relevant methods return MagicMock objects.
    """

    _instances: list["_TrackingNodeBase"] = []

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self.publishers: list[MagicMock] = []
        self.subscriptions: list[MagicMock] = []
        self._callbacks: list[Any] = []
        self._logger = MagicMock(name="Logger")
        _TrackingNodeBase._instances.append(self)

    def create_publisher(self, msg_cls: type, topic: str, qos: Any) -> MagicMock:
        """Record publisher creation and return a MagicMock publisher."""
        pub = MagicMock(name=f"Publisher({topic})")
        self.publishers.append(pub)
        return pub

    def create_subscription(self, msg_cls: type, topic: str, cb: Any, qos: Any) -> MagicMock:
        """Record subscription creation and capture the callback."""
        self._callbacks.append(cb)
        sub = MagicMock(name=f"Subscription({topic})")
        self.subscriptions.append(sub)
        return sub

    def get_logger(self) -> MagicMock:
        """Return a mock logger."""
        return self._logger

    def destroy_node(self) -> None:
        """No-op destroy."""


def _run_with_tracking_node(rules: list[TopicRule], **kwargs: Any) -> "_TrackingNodeBase":
    """Run run_all_relays with a _TrackingNodeBase as the Node mock.

    Returns the RelayNode instance created inside run_all_relays.
    """
    _TrackingNodeBase._instances.clear()

    with (
        patch("master2master.proxy.rclpy") as mock_rclpy,
        patch("master2master.proxy.MultiThreadedExecutor"),
        patch("master2master.proxy.Node", _TrackingNodeBase),
    ):
        mock_rclpy.ok.return_value = False
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()

        run_all_relays(rules, **kwargs)

    assert _TrackingNodeBase._instances, "No RelayNode was instantiated"
    return _TrackingNodeBase._instances[0]


# ---------------------------------------------------------------------------
# get_message_class
# ---------------------------------------------------------------------------


def test_get_message_class_string() -> None:
    """get_message_class returns the String mock for 'string'."""
    cls = get_message_class("string")
    assert cls is _std_msgs_mock.msg.String


def test_get_message_class_jointstate() -> None:
    """get_message_class returns the JointState mock for 'jointstate'."""
    cls = get_message_class("jointstate")
    assert cls is _sensor_msgs_mock.msg.JointState


def test_get_message_class_case_insensitive() -> None:
    """get_message_class is case-insensitive."""
    assert get_message_class("STRING") is _std_msgs_mock.msg.String
    assert get_message_class("JointState") is _sensor_msgs_mock.msg.JointState


def test_get_message_class_unknown_raises() -> None:
    """get_message_class raises KeyError for an unknown type."""
    import pytest

    with pytest.raises(KeyError):
        get_message_class("image")


# ---------------------------------------------------------------------------
# get_supported_message_types
# ---------------------------------------------------------------------------


def test_get_supported_message_types_contains_expected() -> None:
    """get_supported_message_types returns a tuple with 'string' and 'jointstate'."""
    types_tuple = get_supported_message_types()
    assert "string" in types_tuple
    assert "jointstate" in types_tuple


# ---------------------------------------------------------------------------
# run_all_relays — construction: subs and pubs created per rule
# ---------------------------------------------------------------------------


def test_run_all_relays_creates_one_pub_and_sub_per_rule() -> None:
    """run_all_relays creates exactly one publisher and subscriber for a single rule."""
    node = _run_with_tracking_node([_single_rule()])
    assert len(node.publishers) == 1
    assert len(node.subscriptions) == 1


def test_run_all_relays_creates_pub_and_sub_for_each_rule() -> None:
    """run_all_relays creates one publisher and subscriber per relay rule."""
    node = _run_with_tracking_node(_two_rules())
    assert len(node.publishers) == 2
    assert len(node.subscriptions) == 2


def test_run_all_relays_relay_callback_publishes_message() -> None:
    """The relay callback forwarded to the subscription calls publisher.publish(msg)."""
    node = _run_with_tracking_node([_single_rule()])

    assert len(node._callbacks) == 1, "Expected exactly one subscription callback"
    assert len(node.publishers) == 1, "Expected exactly one publisher"

    fake_msg = MagicMock(name="FakeMessage")
    node._callbacks[0](fake_msg)
    node.publishers[0].publish.assert_called_once_with(fake_msg)


def test_run_all_relays_calls_rclpy_init_and_shutdown() -> None:
    """run_all_relays calls rclpy.init() and rclpy.shutdown() around the spin loop."""
    with (
        patch("master2master.proxy.rclpy") as mock_rclpy,
        patch("master2master.proxy.MultiThreadedExecutor"),
        patch("master2master.proxy.Node", _TrackingNodeBase),
    ):
        mock_rclpy.ok.return_value = False
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()

        run_all_relays([_single_rule()])

        mock_rclpy.init.assert_called_once()
        mock_rclpy.shutdown.assert_called_once()


def test_run_all_relays_shutdown_called_even_on_exception() -> None:
    """rclpy.shutdown() is called in the finally block even if the node raises."""
    import pytest

    class _FailingNode(_TrackingNodeBase):
        """Node whose create_publisher always raises."""

        def create_publisher(self, msg_cls: type, topic: str, qos: Any) -> MagicMock:
            raise RuntimeError("boom")

    with (
        patch("master2master.proxy.rclpy") as mock_rclpy,
        patch("master2master.proxy.MultiThreadedExecutor"),
        patch("master2master.proxy.Node", _FailingNode),
    ):
        mock_rclpy.ok.return_value = False
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()

        with pytest.raises(RuntimeError, match="boom"):
            run_all_relays([_single_rule()])

        mock_rclpy.shutdown.assert_called_once()


def test_run_all_relays_rejects_relay_loop() -> None:
    """run_all_relays raises ValueError when rules form a relay loop."""
    import pytest

    rules = [
        TopicRule(source="/a", dest="/b"),
        TopicRule(source="/b", dest="/c"),
    ]
    with pytest.raises(ValueError, match="Relay loop guard"):
        run_all_relays(rules)


def test_run_all_relays_rejects_unknown_msg_type() -> None:
    """run_all_relays raises KeyError for unsupported msg_type in a rule."""
    import pytest

    rule = TopicRule(source="/a", dest="/b", msg_type="string")
    # Force an unsupported type post-construction (bypass Pydantic validation).
    object.__setattr__(rule, "msg_type", "image")

    with pytest.raises(KeyError):
        run_all_relays([rule])


def test_run_all_relays_shutdown_callback_exits_loop() -> None:
    """run_all_relays respects the shutdown_callback to stop spinning."""
    _TrackingNodeBase._instances.clear()
    call_count: list[int] = [0]

    with (
        patch("master2master.proxy.rclpy") as mock_rclpy,
        patch("master2master.proxy.MultiThreadedExecutor") as mock_executor_cls,
        patch("master2master.proxy.Node", _TrackingNodeBase),
    ):
        mock_rclpy.ok.return_value = True  # would spin forever without the callback
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()

        mock_executor = MagicMock()
        mock_executor_cls.return_value = mock_executor

        def shutdown_cb() -> bool:
            call_count[0] += 1
            return True  # request stop immediately

        run_all_relays([_single_rule()], shutdown_callback=shutdown_cb)

    assert call_count[0] >= 1
    mock_rclpy.shutdown.assert_called_once()


def test_run_all_relays_node_destroyed_after_run() -> None:
    """run_all_relays calls destroy_node() on the node after the spin loop exits."""
    destroyed: list[Any] = []

    class _DestroyTracker(_TrackingNodeBase):
        """Node that records its own destruction."""

        def destroy_node(self) -> None:
            destroyed.append(self)

    _TrackingNodeBase._instances.clear()

    with (
        patch("master2master.proxy.rclpy") as mock_rclpy,
        patch("master2master.proxy.MultiThreadedExecutor"),
        patch("master2master.proxy.Node", _DestroyTracker),
    ):
        mock_rclpy.ok.return_value = False
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()

        run_all_relays([_single_rule()])

    assert len(destroyed) == 1, "destroy_node() should be called exactly once"


def test_run_all_relays_two_rules_each_callback_publishes_to_correct_pub() -> None:
    """Each relay rule's callback publishes to its own publisher, not the other."""
    node = _run_with_tracking_node(_two_rules())

    assert len(node._callbacks) == 2
    assert len(node.publishers) == 2

    msg_a = MagicMock(name="MsgA")
    msg_c = MagicMock(name="MsgC")

    node._callbacks[0](msg_a)
    node._callbacks[1](msg_c)

    node.publishers[0].publish.assert_called_once_with(msg_a)
    node.publishers[1].publish.assert_called_once_with(msg_c)
    # Cross-check: publisher[0] never received msg_c.
    for c in node.publishers[0].publish.call_args_list:
        assert c.args[0] is not msg_c
