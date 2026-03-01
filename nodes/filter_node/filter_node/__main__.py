"""Entry point for filter node: load config and run filter in a single process."""

import sys

from .config import load_config_from_env
from .node import run_filter_node


def main() -> int:
    """Load config and run filter node.

    Returns:
        int: 0 on success; 1 on config error.
    """
    config = load_config_from_env()
    if config is None:
        print(
            "Filter node config not found or invalid. Set FILTER_NODE_CONFIG to a YAML path "
            "or deploy to /etc/ros2/filter_node/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_filter_node(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
