"""Entry point of the map guard."""

import sys

from .config import load_config_from_env
from .node import run_node


def main() -> int:
    """Load the config and run the node.

    Returns:
        int: 0 on success; 1 when the config is missing.
    """
    config = load_config_from_env()
    if config is None:
        print(
            "map_guard config not found. Set MAP_GUARD_CONFIG or deploy to /etc/ros2/map_guard/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_node(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
