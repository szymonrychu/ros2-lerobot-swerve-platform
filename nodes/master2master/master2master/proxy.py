"""Per-topic relay: one thread per topic, subscribe source -> publish dest."""

import threading
from typing import Any, Callable

from .config import TopicRule


def _relay_string(rule: TopicRule, node: Any, qos: Any, shutdown: Callable[[], bool]) -> None:
    import rclpy
    from std_msgs.msg import String

    pub = node.create_publisher(String, rule.dest, qos)

    def callback(msg: String) -> None:
        pub.publish(msg)

    node.create_subscription(String, rule.source, callback, qos)
    while rclpy.ok() and not shutdown():
        rclpy.spin_once(node, timeout_sec=0.5)


def _relay_joint_state(rule: TopicRule, node: Any, qos: Any, shutdown: Callable[[], bool]) -> None:
    import rclpy
    from sensor_msgs.msg import JointState

    pub = node.create_publisher(JointState, rule.dest, qos)

    def callback(msg: JointState) -> None:
        pub.publish(msg)

    node.create_subscription(JointState, rule.source, callback, qos)
    while rclpy.ok() and not shutdown():
        rclpy.spin_once(node, timeout_sec=0.5)


def run_relay_thread(rule: TopicRule, shutdown: Callable[[], bool]) -> None:
    """Run a single topic relay in a thread (rclpy node in thread).

    Message type from rule.msg_type: "string" (std_msgs/String) or "jointstate" (sensor_msgs/JointState).
    When shutdown() returns True, the thread exits.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    except ImportError:
        while not shutdown():
            threading.Event().wait(0.5)
        return

    rclpy.init()
    node: Any = None
    try:
        node = Node("master2master_relay_" + rule.source.replace("/", "_").strip("_"))
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        if (rule.msg_type or "string").lower() == "jointstate":
            _relay_joint_state(rule, node, qos, shutdown)
        else:
            _relay_string(rule, node, qos, shutdown)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def run_all_relays(rules: list[TopicRule]) -> None:
    """Start one thread per rule and block until shutdown (e.g. SIGINT)."""
    shutdown_flag = threading.Event()

    def should_stop() -> bool:
        return shutdown_flag.is_set()

    threads = []
    for rule in rules:
        t = threading.Thread(target=run_relay_thread, args=(rule, should_stop))
        t.daemon = True
        t.start()
        threads.append(t)

    try:
        import signal

        signal.signal(signal.SIGINT, lambda *_: shutdown_flag.set())
        signal.signal(signal.SIGTERM, lambda *_: shutdown_flag.set())
    except (ValueError, AttributeError):
        pass  # not main thread or Windows

    for t in threads:
        t.join(timeout=1.0)
