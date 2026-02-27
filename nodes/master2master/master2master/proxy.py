"""Single-process topic relay: one node, one executor, multiple subscribe->publish rules."""

from typing import Any, Callable

from .config import SUPPORTED_MSG_TYPES, TopicRule

# Lazy-loaded ROS2 message classes (avoids requiring rclpy for lint/import outside ROS env).
_RELAY_MESSAGE_TYPES: dict[str, type] = {}


def _register_message_types() -> None:
    """Lazy-load ROS2 message classes and register them for relay dispatch."""
    if _RELAY_MESSAGE_TYPES:
        return
    try:
        from std_msgs.msg import String

        _RELAY_MESSAGE_TYPES["string"] = String
    except ImportError:
        pass
    try:
        from sensor_msgs.msg import JointState

        _RELAY_MESSAGE_TYPES["jointstate"] = JointState
    except ImportError:
        pass


def _get_message_class(msg_type: str) -> type:
    """Return ROS2 message class for given msg_type string. Raises KeyError if unsupported."""
    _register_message_types()
    key = (msg_type or "string").lower().strip()
    if key not in _RELAY_MESSAGE_TYPES:
        raise KeyError(f"Unsupported msg_type {msg_type!r}; supported: {list(_RELAY_MESSAGE_TYPES)}")
    return _RELAY_MESSAGE_TYPES[key]


def run_all_relays(rules: list[TopicRule], shutdown_callback: Callable[[], bool] | None = None) -> None:
    """Run all relays in a single node with MultiThreadedExecutor until shutdown.

    If shutdown_callback is provided, it is checked periodically; when it returns True, spinning stops.
    Otherwise, runs until process receives SIGINT/SIGTERM (rclpy shutdown).
    """
    import signal

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    _register_message_types()
    for r in rules:
        if (r.msg_type or "string").lower().strip() not in _RELAY_MESSAGE_TYPES:
            raise KeyError(
                "Unsupported msg_type %r for rule %s -> %s; available: %s"
                % (r.msg_type, r.source, r.dest, list(_RELAY_MESSAGE_TYPES))
            )

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    class RelayNode(Node):
        """Single node that runs all topic relays (one sub+pub per rule)."""

        def __init__(self) -> None:
            super().__init__("master2master")

        def add_relay(self, rule: TopicRule) -> None:
            """Add one relay: subscribe to rule.source, publish to rule.dest with rule.msg_type."""
            msg_class = _get_message_class(rule.msg_type)
            pub = self.create_publisher(msg_class, rule.dest, qos)

            def callback(msg: Any) -> None:
                pub.publish(msg)

            self.create_subscription(msg_class, rule.source, callback, qos)
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
    """Return supported msg_type strings (e.g. for validation)."""
    return SUPPORTED_MSG_TYPES
