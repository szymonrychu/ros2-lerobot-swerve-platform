"""Entry point: rclpy node spun in a background executor thread + the MCP Streamable HTTP app on uvicorn."""

import logging
import sys
import threading

import rclpy
import uvicorn
from pydantic import ValidationError
from rclpy.executors import MultiThreadedExecutor
from ros2_common.battery import BatteryGuard

from .config import MissingTokenError, config_path_from_env, load_config, token_from_env
from .monitor import RobotMonitor
from .ros_iface import RosRobot, init_ros
from .tools import build_app, build_mcp_server

EXECUTOR_THREADS = 4
LOGGER = logging.getLogger("mcp_server")


def main() -> int:
    """Load config and token, start the ROS node and serve MCP until uvicorn exits.

    Returns:
        int: 0 on clean shutdown, 1 on configuration errors (missing token, bad config).
    """
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        token = token_from_env()
    except MissingTokenError as exc:
        LOGGER.error("%s", exc)
        return 1
    path = config_path_from_env()
    try:
        config = load_config(path)
    except (OSError, ValidationError, ValueError) as exc:
        LOGGER.error("invalid or missing config %s: %s", path, exc)
        return 1
    init_ros()
    guard = BatteryGuard.from_config(config.battery) if config.battery is not None else None
    monitor = RobotMonitor(config.monitor, guard, autonomy_source=config.arm.autonomy_source_name)
    robot = RosRobot(config, guard, monitor)
    executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
    executor.add_node(robot.node)
    spinner = threading.Thread(target=executor.spin, name="ros-executor", daemon=True)
    spinner.start()
    app = build_app(build_mcp_server(robot, config, token, guard, monitor), config)
    if config.battery is not None:
        LOGGER.info("battery cut-off gate on %s (%d cells)", config.battery.topic, config.battery.cells)
    LOGGER.info("serving MCP on http://%s:%d%s", config.server.host, config.server.port, config.server.path)
    try:
        uvicorn.run(app, host=config.server.host, port=config.server.port, log_level="info")
    finally:
        robot.shutdown()
        executor.shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
