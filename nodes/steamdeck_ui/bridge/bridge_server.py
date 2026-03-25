"""ROS2 bridge: rclpy subscriber node + WebSocket server for Electron renderer."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import signal
from pathlib import Path
from typing import Any

import rclpy
import websockets
import websockets.server
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
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


class BridgeNode(Node):
    """ROS2 node that subscribes to configured topics and fans out to WebSocket clients."""

    def __init__(
        self,
        topics: list[str],
        queue: asyncio.Queue[dict[str, Any]],
        loop: asyncio.AbstractEventLoop,
    ) -> None:
        """Initialise BridgeNode with topic list and asyncio queue.

        Args:
            topics: List of ROS2 topic strings to subscribe to.
            queue: asyncio queue for passing serialized messages to WebSocket server.
            loop: The running asyncio event loop (needed for thread-safe queue writes).
        """
        super().__init__("steamdeck_ui_bridge")
        self.queue = queue
        self.loop = loop
        self.publishers_: dict[str, Any] = {}
        for topic in topics:
            msg_cls = TOPIC_TYPE_HINTS.get(topic)
            if msg_cls is None:
                self.get_logger().warning(f"Unknown msg type for topic {topic} — skipping")
                continue
            self.create_subscription(msg_cls, topic, self._make_callback(topic), 10)
            self.get_logger().info(f"Subscribed: {topic}")

    def _make_callback(self, topic: str) -> Any:
        """Return a ROS2 subscription callback that serializes and enqueues messages.

        ROS2 callbacks run in a worker thread; asyncio Queue.put_nowait must be
        called via loop.call_soon_threadsafe so the event loop wakes up waiters.

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

            def _put() -> None:
                try:
                    self.queue.put_nowait(envelope)
                except asyncio.QueueFull:
                    pass  # drop oldest-strategy: silently drop when no consumer

            self.loop.call_soon_threadsafe(_put)

        return callback

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


async def ws_handler(
    websocket: websockets.server.WebSocketServerProtocol,
    queue: asyncio.Queue[dict[str, Any]],
    node: BridgeNode,
) -> None:
    """Handle a single WebSocket client connection.

    Listens for messages from Electron (subscribe/publish commands) and
    sends topic data from the queue.

    Args:
        websocket: Connected WebSocket client.
        queue: Shared asyncio queue with serialized ROS2 messages.
        node: BridgeNode for publishing commands back to ROS2.
    """
    log.info("Client connected: %s", websocket.remote_address)

    async def sender() -> None:
        while True:
            msg = await queue.get()
            try:
                await websocket.send(json.dumps(msg))
            except websockets.ConnectionClosed:
                break

    async def receiver() -> None:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
                if msg.get("type") == "publish":
                    node.create_publisher_for(msg["topic"], msg.get("msg_type", ""))
                    node.publish_dict(msg["topic"], msg.get("data", {}))
            except json.JSONDecodeError:
                pass

    try:
        await asyncio.gather(sender(), receiver())
    except websockets.ConnectionClosed:
        pass
    log.info("Client disconnected")


async def run_bridge(config: AppConfig) -> None:
    """Run the bridge: start ROS2 node + WebSocket server concurrently.

    Args:
        config: Validated AppConfig with bridge settings and topic lists.
    """
    topics = config.all_subscribed_topics()
    publish_topics = config.publish_topics()
    queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=200)

    rclpy.init()
    loop = asyncio.get_event_loop()
    node = BridgeNode(topics, queue, loop)

    for topic in publish_topics:
        # We need to know the msg_type; use hints
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

    loop.add_signal_handler(signal.SIGINT, shutdown_handler)
    loop.add_signal_handler(signal.SIGTERM, shutdown_handler)

    ros_task = loop.run_in_executor(None, spin_ros)

    async def handler(ws: websockets.server.WebSocketServerProtocol) -> None:
        # Each client gets its own per-client queue fan-out
        client_queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=100)

        async def forwarder() -> None:
            while True:
                msg = await queue.get()
                await client_queue.put(msg)

        fwd = asyncio.ensure_future(forwarder())
        try:
            await ws_handler(ws, client_queue, node)
        finally:
            fwd.cancel()

    log.info("Bridge WebSocket server starting on %s:%d", config.bridge.host, config.bridge.port)
    async with websockets.serve(handler, config.bridge.host, config.bridge.port):
        await stop_event.wait()

    log.info("Shutting down bridge")
    ros_task.cancel()
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    """Entry point for the bridge server."""
    parser = argparse.ArgumentParser(description="SteamDeck UI ROS2 bridge")
    parser.add_argument("--config", type=Path, default=None, help="Path to config YAML")
    parser.add_argument("--log-level", default="INFO", help="Logging level")
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    # Set DDS environment from config if not already set
    config = load_config(args.config)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    os.environ.setdefault("ROS_STATIC_PEERS", config.bridge.ros_static_peers)
    os.environ.setdefault("ROS_DOMAIN_ID", config.bridge.ros_domain_id)

    asyncio.run(run_bridge(config))


if __name__ == "__main__":
    main()
