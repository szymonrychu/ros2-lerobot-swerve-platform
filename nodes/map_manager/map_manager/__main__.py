"""Entry point of map_manager."""

import logging
import os
import sys

import yaml
from pydantic import ValidationError
from ros2_metrics import resolve_metrics_port, start_metrics_server

from map_manager.config import load_config_from_env
from map_manager.node import NODE_NAME, run_node


def main() -> int:
    """Load the config, start the metrics server and run the node.

    Returns:
        int: 0 on clean shutdown; ``os.EX_CONFIG`` (78, not restarted by systemd) when the config is missing,
        unreadable or invalid.
    """
    logging.basicConfig(level=logging.INFO)
    try:
        config = load_config_from_env()
        port = resolve_metrics_port(config.metrics_port)
    except (OSError, ValueError, yaml.YAMLError, ValidationError) as exc:
        print(f"map_manager config error: {exc}", file=sys.stderr)
        return os.EX_CONFIG
    start_metrics_server(port, NODE_NAME)
    run_node(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
