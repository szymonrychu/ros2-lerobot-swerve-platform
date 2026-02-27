"""Single-process topic relay: one node, one executor, multiple subscribe->publish rules."""

import signal
from typing import Any, Callable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .config import SUPPORTED_MSG_TYPES, TopicRule

RELAY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

RELAY_MESSAGE_TYPES: dict[str, type] = {
    "string": String,
    "jointstate": JointState,
}


def get_message_class(msg_type: str) -> type:
    """Return ROS2 message class for given msg_type string.

    Args:
        msg_type: One of 'string' or 'jointstate'.

    Returns:
        type: The ROS2 message class (String or JointState).

    Raises:
        KeyError: If msg_type is not supported.
    """
    key = (msg_type or "string").lower().strip()
    if key not in RELAY_MESSAGE_TYPES:
        raise KeyError(f"Unsupported msg_type {msg_type!r}; supported: {list(RELAY_MESSAGE_TYPES)}")
    return RELAY_MESSAGE_TYPES[key]


def run_all_relays(
    rules: list[TopicRule],
    shutdown_callback: Callable[[], bool] | None = None,
) -> None:
    """Run all relays in a single node with MultiThreadedExecutor until shutdown.

    Args:
        rules: List of TopicRule (source, dest, direction, msg_type).
        shutdown_callback: Optional callable; when it returns True, spinning stops.
        If None, runs until process receives SIGINT/SIGTERM (rclpy shutdown).
    """
    for r in rules:
        if (r.msg_type or "string").lower().strip() not in RELAY_MESSAGE_TYPES:
            raise KeyError(
                "Unsupported msg_type %r for rule %s -> %s; available: %s"
                % (r.msg_type, r.source, r.dest, list(RELAY_MESSAGE_TYPES))
            )

    class RelayNode(Node):
        """Single node that runs all topic relays (one sub+pub per rule)."""

        def __init__(self) -> None:
            super().__init__("master2master")

        def add_relay(self, rule: TopicRule) -> None:
            """Add one relay: subscribe to rule.source, publish to rule.dest with rule.msg_type."""
            msg_class = get_message_class(rule.msg_type)
            pub = self.create_publisher(msg_class, rule.dest, RELAY_QOS)

            def callback(msg: Any) -> None:
                pub.publish(msg)

            self.create_subscription(msg_class, rule.source, callback, RELAY_QOS)
            self.get_logger().info(
                "Relay: %s -> %s [%s]" % (rule.source, rule.dest, rule.msg_type),
                throttle_duration_sec=10.0,
            )

    rclpy.init()
    node: RelayNode | None = None
    shutdown_flag: list[bool] = [False]

    def set_shutdown() -> None:
        shutdown_flag[0] = True

    try:
        node = RelayNode()
        for rule in rules:
            node.add_relay(rule)

        def should_stop() -> bool:
            if shutdown_callback is not None and shutdown_callback():
                return True
            return shutdown_flag[0]

        try:
            signal.signal(signal.SIGINT, lambda *_: set_shutdown())
            signal.signal(signal.SIGTERM, lambda *_: set_shutdown())
        except (ValueError, AttributeError):
            pass

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        while rclpy.ok() and not should_stop():
            executor.spin_once(timeout_sec=0.5)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def get_supported_message_types() -> tuple[str, ...]:
    """Return supported msg_type strings (e.g. for validation).

    Returns:
        tuple[str, ...]: e.g. ('string', 'jointstate').
    """
    return SUPPORTED_MSG_TYPES
