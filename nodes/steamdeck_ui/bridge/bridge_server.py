"""ROS2 bridge: rclpy subscriber node + WebSocket server for Electron renderer."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import signal
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
import websockets
import websockets.server
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, Imu, JointState, LaserScan, NavSatFix

from .config import AppConfig, load_config
from .msg_serializer import msg_to_dict

log = logging.getLogger(__name__)

MSG_TYPE_MAP: dict[str, type] = {
    "sensor_msgs/Imu": Imu,
    "sensor_msgs/JointState": JointState,
    "sensor_msgs/NavSatFix": NavSatFix,
    "sensor_msgs/LaserScan": LaserScan,
    "sensor_msgs/Image": Image,
    "sensor_msgs/CompressedImage": CompressedImage,
    "nav_msgs/OccupancyGrid": OccupancyGrid,
    "nav_msgs/Odometry": Odometry,
    "geometry_msgs/PoseStamped": PoseStamped,
}

TOPIC_TYPE_HINTS: dict[str, type] = {
    "/controller/imu/data": Imu,
    "/controller/follower/joint_states": JointState,
    "/controller/swerve_drive/joint_states": JointState,
    "/controller/gps/fix": NavSatFix,
    "/controller/scan": LaserScan,
    "/controller/local_costmap": OccupancyGrid,
    "/controller/odom": Odometry,
    "/controller/camera_0/image_compressed": CompressedImage,
    "/controller/goal_pose": PoseStamped,
}

BROADCAST_INTERVAL_S = 0.05  # 20 Hz
HEARTBEAT_INTERVAL_S = 5.0

# Subscribe BEST_EFFORT for sensor topics so the bridge can receive from both
# RELIABLE and BEST_EFFORT publishers (master2master uses BEST_EFFORT subs).
SENSOR_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Message types that publish high-rate sensor data — subscribe BEST_EFFORT.
SENSOR_TYPES: frozenset[type] = frozenset(
    {Imu, JointState, NavSatFix, LaserScan, OccupancyGrid, Odometry, Image, CompressedImage}
)


class BridgeNode(Node):
    """ROS2 node that subscribes to configured topics and stores the latest value per topic."""

    def __init__(self, topics: list[str]) -> None:
        """Initialise BridgeNode with topic list.

        Args:
            topics: List of ROS2 topic strings to subscribe to.
        """
        super().__init__("steamdeck_ui_bridge")
        self._latest: dict[str, dict[str, Any]] = {}
        self._dirty: set[str] = set()
        self._lock = threading.Lock()
        self.publishers_: dict[str, Any] = {}
        self._topic_last_rx: dict[str, float] = {t: time.monotonic() for t in topics if TOPIC_TYPE_HINTS.get(t)}
        for topic in topics:
            msg_cls = TOPIC_TYPE_HINTS.get(topic)
            if msg_cls is None:
                self.get_logger().warning(f"Unknown msg type for topic {topic} — skipping")
                continue
            qos = SENSOR_SUB_QOS if msg_cls in SENSOR_TYPES else 10
            self.create_subscription(msg_cls, topic, self._make_callback(topic), qos)
            self.get_logger().info(f"Subscribed: {topic}")
        self.create_timer(10.0, self._check_topic_health)

    def _make_callback(self, topic: str) -> Any:
        """Return a ROS2 subscription callback that stores the latest serialized message.

        ROS2 callbacks run in a worker thread. Writing to the latest-value dict is
        protected by a threading.Lock — no asyncio involvement needed.

        Args:
            topic: Topic name for the closure.

        Returns:
            Callable: rclpy subscription callback.
        """

        def callback(msg: Any) -> None:
            try:
                data = msg_to_dict(msg)
                envelope = {"type": "topic_data", "topic": topic, "data": data}
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warning(f"Serialize error on {topic}: {exc}")
                return
            log.debug("RX %s (%d bytes)", topic, len(str(data)))
            with self._lock:
                self._latest[topic] = envelope
                self._dirty.add(topic)
            self._topic_last_rx[topic] = time.monotonic()

        return callback

    def _check_topic_health(self) -> None:
        """Log a warning for any subscribed topic that has not received data for >10s."""
        now = time.monotonic()
        for topic, last_rx in self._topic_last_rx.items():
            if now - last_rx > 10.0:
                self.get_logger().warning(f"No data on {topic} for {now - last_rx:.0f}s")

    def flush_dirty(self) -> list[dict[str, Any]]:
        """Return all envelopes that have been updated since the last flush and clear the dirty set.

        Returns:
            list[dict[str, Any]]: List of envelopes for topics updated since last call.
        """
        with self._lock:
            envelopes = [self._latest[t] for t in self._dirty if t in self._latest]
            self._dirty.clear()
        return envelopes

    def create_publisher_for(self, topic: str, msg_type_str: str) -> None:
        """Create a ROS2 publisher for outbound commands from the Electron renderer.

        Args:
            topic: ROS2 topic name to publish to.
            msg_type_str: ROS2 message type string (e.g. 'geometry_msgs/PoseStamped').
        """
        if topic in self.publishers_:
            return
        msg_cls = MSG_TYPE_MAP.get(msg_type_str)
        if msg_cls is None:
            self.get_logger().warning(f"Unknown msg type {msg_type_str} for publish topic {topic}")
            return
        self.publishers_[topic] = (self.create_publisher(msg_cls, topic, 10), msg_cls)

    def publish_dict(self, topic: str, data: dict[str, Any]) -> None:
        """Publish a message to a ROS2 topic from a JSON dict.

        Args:
            topic: ROS2 topic name.
            data: JSON-deserialized message data dict.
        """
        if topic not in self.publishers_:
            self.get_logger().warning(f"No publisher for topic {topic}")
            return
        pub, msg_cls = self.publishers_[topic]
        try:
            msg = _dict_to_msg(msg_cls(), data)
            pub.publish(msg)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f"Publish error on {topic}: {exc}")


def _dict_to_msg(msg: Any, data: dict[str, Any]) -> Any:
    """Recursively populate a ROS2 message from a dict.

    Args:
        msg: ROS2 message instance to populate.
        data: Dict with field values.

    Returns:
        The populated message object.
    """
    for key, value in data.items():
        if not hasattr(msg, key):
            continue
        attr = getattr(msg, key)
        if isinstance(value, dict) and hasattr(attr, "__slots__"):
            _dict_to_msg(attr, value)
        elif isinstance(value, list):
            setattr(msg, key, value)
        else:
            try:
                # Coerce int→float to satisfy ROS2 C extension float64 field assertions
                if isinstance(value, int) and not isinstance(value, bool):
                    value = float(value)
                setattr(msg, key, value)
            except Exception:  # pylint: disable=broad-except
                pass
    return msg


async def run_bridge(config: AppConfig) -> None:
    """Run the bridge: start ROS2 node + WebSocket server concurrently.

    Uses a latest-value-per-topic store instead of a FIFO queue so that
    heavy topics (e.g. camera frames) cannot starve lightweight sensor topics.
    A periodic broadcaster (20 Hz) flushes all dirty topics to all connected
    clients in a single serialization pass.

    Args:
        config: Validated AppConfig with bridge settings and topic lists.
    """
    topics = config.all_subscribed_topics()
    publish_topics = config.publish_topics()

    rclpy.init()
    node = BridgeNode(topics)

    for topic in publish_topics:
        for tab in config.tabs:
            if tab.goal_topic == topic:
                node.create_publisher_for(topic, "geometry_msgs/PoseStamped")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def spin_ros() -> None:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

    stop_event = asyncio.Event()

    def shutdown_handler(*_: Any) -> None:
        stop_event.set()

    loop = asyncio.get_event_loop()
    loop.add_signal_handler(signal.SIGINT, shutdown_handler)
    loop.add_signal_handler(signal.SIGTERM, shutdown_handler)

    ros_task = loop.run_in_executor(None, spin_ros)

    clients: set[websockets.server.WebSocketServerProtocol] = set()

    async def broadcaster() -> None:
        _heartbeat_counter = 0
        while True:
            try:
                await asyncio.sleep(BROADCAST_INTERVAL_S)
                _heartbeat_counter += 1
                if not clients:
                    continue
                dead: set[Any] = set()
                if _heartbeat_counter % 100 == 0:
                    hb_frame = json.dumps({"type": "heartbeat", "ts": int(asyncio.get_event_loop().time() * 1000)})
                    for ws in list(clients):
                        try:
                            await ws.send(hb_frame)
                        except Exception as exc:  # pylint: disable=broad-except
                            log.warning("broadcaster heartbeat error (%s): %s", type(exc).__name__, exc)
                            dead.add(ws)
                    clients.difference_update(dead)
                    dead = set()
                envelopes = node.flush_dirty()
                if not envelopes:
                    continue
                frames = [json.dumps(e) for e in envelopes]
                log.debug("TX %d topics to %d clients", len(frames), len(clients))
                for ws in list(clients):
                    for frame in frames:
                        try:
                            await ws.send(frame)
                        except Exception as exc:  # pylint: disable=broad-except
                            log.warning("broadcaster send error (%s): %s", type(exc).__name__, exc)
                            dead.add(ws)
                            break
                clients.difference_update(dead)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # pylint: disable=broad-except
                log.error("broadcaster loop error: %s", exc, exc_info=True)

    async def handler(ws: websockets.server.WebSocketServerProtocol) -> None:
        log.info("Client connected: %s", ws.remote_address)
        clients.add(ws)
        try:
            async for raw in ws:
                try:
                    log.debug("WS recv: %s", raw[:200])
                    msg = json.loads(raw)
                    if msg.get("type") == "publish":
                        node.create_publisher_for(msg["topic"], msg.get("msg_type", ""))
                        node.publish_dict(msg["topic"], msg.get("data", {}))
                except json.JSONDecodeError:
                    pass
        except websockets.ConnectionClosed:
            pass
        finally:
            clients.discard(ws)
            log.info("Client disconnected")

    log.info("Bridge WebSocket server starting on %s:%d", config.bridge.host, config.bridge.port)
    broadcast_task = asyncio.ensure_future(broadcaster())
    async with websockets.serve(handler, config.bridge.host, config.bridge.port, ping_interval=5, ping_timeout=10):
        await stop_event.wait()

    log.info("Shutting down bridge")
    broadcast_task.cancel()
    ros_task.cancel()
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    """Entry point for the bridge server."""
    parser = argparse.ArgumentParser(description="SteamDeck UI ROS2 bridge")
    parser.add_argument("--config", type=Path, default=None, help="Path to config YAML")
    parser.add_argument("--log-level", default="INFO", help="Logging level")
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # Set DDS environment from config if not already set
    config = load_config(args.config)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    os.environ.setdefault("ROS_STATIC_PEERS", config.bridge.ros_static_peers)
    os.environ.setdefault("ROS_DOMAIN_ID", config.bridge.ros_domain_id)

    asyncio.run(run_bridge(config))


if __name__ == "__main__":
    main()
