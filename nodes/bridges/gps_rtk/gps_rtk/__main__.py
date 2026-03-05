"""Entry point for GPS RTK node: load config, run base or rover."""

import sys

import rclpy

from .base_node import GpsRtkBaseNode
from .config import load_config_from_env
from .rover_node import GpsRtkRoverNode


def main() -> int:
    """Load config and run base or rover node.

    Returns:
        0 on success, 1 on config error.
    """
    config = load_config_from_env()
    if config is None:
        print(
            "GPS RTK config not found or invalid. Set GPS_RTK_CONFIG to a YAML path "
            "or deploy to /etc/ros2/gps_rtk/config.yaml",
            file=sys.stderr,
        )
        return 1
    if config.mode == "rover" and not config.rtcm_server_host:
        print("Rover mode requires rtcm_server_host in config.", file=sys.stderr)
        return 1
    rclpy.init()
    if config.mode == "base":
        node = GpsRtkBaseNode(config)
    else:
        node = GpsRtkRoverNode(config)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
