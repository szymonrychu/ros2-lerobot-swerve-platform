"""Entry point: rclpy node (logging) plus the Claude agent HTTP/WebSocket API on uvicorn, bound to loopback."""

import os
import sys
import threading
from collections.abc import Callable
from pathlib import Path

import rclpy
import uvicorn
from pydantic import ValidationError
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .api import create_app
from .config import config_path_from_env, load_config
from .events import EventLog
from .runner import REMOVED_ENV_KEYS, AgentRunner

NODE_NAME = "claude_agent"
SESSION_LOG_RELPATH = Path("session") / "events.jsonl"
ROBOT_EVENTS_QOS_DEPTH = 50


def make_robot_event_callback(runner: AgentRunner) -> Callable[[String], None]:
    """Build the /robot_events subscription callback (runs on the rclpy executor thread).

    Args:
        runner (AgentRunner): Receives the raw JSON through its thread-safe post_robot_event.

    Returns:
        Callable[[String], None]: Callback taking the std_msgs/String message.
    """

    def on_robot_event(msg: String) -> None:
        runner.post_robot_event(msg.data)

    return on_robot_event


def main() -> int:
    """Load the config, start the ROS2 node and serve the API until uvicorn exits.

    Returns:
        int: 0 on clean shutdown, 1 when the config file is invalid or unreadable.
    """
    try:
        config = load_config(config_path_from_env(os.environ))
    except (ValidationError, OSError, ValueError) as exc:
        print(f"claude_agent: invalid configuration: {exc}", file=sys.stderr)
        return 1
    for key in REMOVED_ENV_KEYS:
        os.environ.pop(key, None)
    rclpy.init()
    node = Node(NODE_NAME)
    logger = node.get_logger()
    executor = SingleThreadedExecutor()
    try:
        events = EventLog(
            config.history_size,
            path=Path(config.state_dir) / SESSION_LOG_RELPATH,
            max_bytes=config.session_log_max_bytes,
        )
        runner = AgentRunner(config, events, logger=logger)
        # Best effort + volatile matches publishers of either reliability; only live events matter (no replay).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=ROBOT_EVENTS_QOS_DEPTH,
        )
        node.create_subscription(String, config.robot_events_topic, make_robot_event_callback(runner), qos)
        executor.add_node(node)
        threading.Thread(target=executor.spin, name="rclpy-executor", daemon=True).start()
        app = create_app(runner, events, config, on_shutdown=runner.close, on_startup=runner.bind_loop)
        logger.info(f"serving the agent API on http://{config.http_host}:{config.http_port} (model {config.model})")
        uvicorn.run(app, host=config.http_host, port=config.http_port, log_level="info")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
