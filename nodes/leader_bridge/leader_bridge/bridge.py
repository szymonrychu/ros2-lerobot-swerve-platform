"""Relay the leader arm joint states into filter_node's leader input, unchanged, dropping stale or malformed ones.

One subscription (config.source_topic) and one publisher (config.dest_topic), nothing else. A valid message is
published as the very object received, so names, positions, velocities, efforts and the header stamp are untouched.
A stale or malformed message is dropped and logged (throttled per reason); nothing is published in its place.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .config import BridgeConfig, QosSettings

NODE_NAME = "leader_bridge"
NS_PER_S = 1_000_000_000
RELIABILITY = {"reliable": ReliabilityPolicy.RELIABLE, "best_effort": ReliabilityPolicy.BEST_EFFORT}


def qos_profile(settings: QosSettings) -> QoSProfile:
    """Build a KEEP_LAST QoS profile from config.

    Args:
        settings (QosSettings): Reliability and depth.

    Returns:
        QoSProfile: The rclpy profile.
    """
    return QoSProfile(
        reliability=RELIABILITY[settings.reliability], history=HistoryPolicy.KEEP_LAST, depth=settings.depth
    )


def stamp_ns(msg: Any) -> int:
    """Return the header stamp of a JointState in nanoseconds.

    Args:
        msg (JointState): The message.

    Returns:
        int: header.stamp as nanoseconds.
    """
    return int(msg.header.stamp.sec) * NS_PER_S + int(msg.header.stamp.nanosec)


def drop_reason(msg: Any, now_ns: int, last_stamp_ns: int | None, config: BridgeConfig) -> str | None:
    """Decide whether a leader JointState must be dropped.

    Args:
        msg (JointState): Received message.
        now_ns (int): Local clock now, nanoseconds.
        last_stamp_ns (int | None): Stamp of the last forwarded message, None before the first.
        config (BridgeConfig): Staleness limits.

    Returns:
        str | None: Why it is dropped, or None when it is valid and must be forwarded.
    """
    names = list(msg.name)
    count = len(names)
    if count == 0:
        return "malformed: no joints"
    if any(not name for name in names):
        return "malformed: empty joint name"
    if len(set(names)) != count:
        return "malformed: duplicate joint names"
    if len(msg.position) != count:
        return f"malformed: {len(msg.position)} position values for {count} joints"
    for field in ("velocity", "effort"):
        values = getattr(msg, field)
        if len(values) not in (0, count):
            return f"malformed: {len(values)} {field} values for {count} joints"
    if not np.all(np.isfinite(np.asarray(msg.position, dtype=float))):
        return "malformed: non-finite position"
    stamp = stamp_ns(msg)
    if stamp == 0:
        return "malformed: unstamped header"
    age_s = (now_ns - stamp) / NS_PER_S
    if config.max_age_s > 0.0 and age_s > config.max_age_s:
        return f"stale: stamp {age_s:.3f} s old (max_age_s {config.max_age_s})"
    if -age_s > config.max_future_s:
        return f"future stamp: {-age_s:.3f} s ahead (max_future_s {config.max_future_s}); check host clock sync"
    if last_stamp_ns is not None and stamp <= last_stamp_ns:
        return "out of order: stamp not newer than the last forwarded message"
    return None


class LeaderBridge:
    """Forward source_topic to dest_topic on an existing node.

    Attributes:
        forwarded (int): Messages published.
        dropped (int): Messages dropped (stale or malformed).
    """

    def __init__(self, node: Node, config: BridgeConfig) -> None:
        """Create the one subscription and the one publisher.

        Args:
            node (Node): Node that owns the endpoints.
            config (BridgeConfig): Topics, QoS and drop rules.
        """
        self.node = node
        self.config = config
        self.forwarded = 0
        self.dropped = 0
        self.last_stamp_ns: int | None = None
        # reason kind -> (last warning time ns, drops since that warning)
        self.drop_log: dict[str, tuple[int, int]] = {}
        self.publisher = node.create_publisher(JointState, config.dest_topic, qos_profile(config.publish_qos))
        node.create_subscription(JointState, config.source_topic, self.on_message, qos_profile(config.subscribe_qos))
        node.get_logger().info(
            f"leader_bridge: {config.source_topic} -> {config.dest_topic} "
            f"(max_age_s {config.max_age_s}, max_future_s {config.max_future_s})"
        )

    def on_message(self, msg: JointState) -> None:
        """Publish msg unchanged when valid; otherwise drop and log it.

        Args:
            msg (JointState): Leader joint state.
        """
        now_ns = self.node.get_clock().now().nanoseconds
        reason = drop_reason(msg, now_ns, self.last_stamp_ns, self.config)
        if reason is None:
            self.last_stamp_ns = stamp_ns(msg)
            self.publisher.publish(msg)
            self.forwarded += 1
            return
        self.dropped += 1
        self.log_drop(reason, now_ns)

    def log_drop(self, reason: str, now_ns: int) -> None:
        """Warn about a drop, at most once per drop_log_period_s for each kind of reason.

        Args:
            reason (str): drop_reason result; the text before the first colon is its kind.
            now_ns (int): Local clock now, nanoseconds.
        """
        kind = reason.split(":", 1)[0]
        last_ns, count = self.drop_log.get(kind, (None, 0))
        count += 1
        period_ns = int(self.config.drop_log_period_s * NS_PER_S)
        if last_ns is not None and now_ns - last_ns < period_ns:
            self.drop_log[kind] = (last_ns, count)
            return
        self.drop_log[kind] = (now_ns, 0)
        self.node.get_logger().warning(
            f"dropped leader joint state ({reason}); {count} dropped since the last warning of this kind, "
            f"{self.forwarded} forwarded in total"
        )


def run(config: BridgeConfig) -> None:
    """Run the bridge node until shutdown (SIGINT/SIGTERM).

    Args:
        config (BridgeConfig): Validated settings.
    """
    rclpy.init()
    node = Node(NODE_NAME)
    LeaderBridge(node, config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
