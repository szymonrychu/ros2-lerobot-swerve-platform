"""Entry point for the POI store node."""

import logging
import sys

from .config import load_config_from_env
from .node import run_node


def main() -> int:
    """Load config and run the node.

    Returns:
        int: 0 on success; 1 on config error.
    """
    logging.basicConfig(level=logging.INFO)
    config = load_config_from_env()
    if config is None:
        print(
            "poi_store config not found. Set POI_STORE_CONFIG or deploy to /etc/ros2/poi_store/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_node(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
