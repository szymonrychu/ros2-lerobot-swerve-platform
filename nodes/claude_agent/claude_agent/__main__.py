"""Entry point: rclpy node (logging) plus the Claude agent HTTP/WebSocket API on uvicorn, bound to loopback."""

import os
import sys

import rclpy
import uvicorn
from pydantic import ValidationError
from rclpy.node import Node

from .api import create_app
from .config import config_path_from_env, load_config
from .events import EventLog
from .runner import REMOVED_ENV_KEYS, AgentRunner

NODE_NAME = "claude_agent"


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
    try:
        events = EventLog(config.history_size)
        runner = AgentRunner(config, events, logger=logger)
        app = create_app(runner, events, config, on_shutdown=runner.close)
        logger.info(f"serving the agent API on http://{config.http_host}:{config.http_port} (model {config.model})")
        uvicorn.run(app, host=config.http_host, port=config.http_port, log_level="info")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
