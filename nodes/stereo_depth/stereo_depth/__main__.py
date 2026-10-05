"""Entry point for the stereo_depth node."""

import sys

from .config import load_config_from_env
from .node import run_node


def main() -> int:
    """Load config and run the node.

    Returns:
        int: 0 on success; 1 on config error.
    """
    config = load_config_from_env()
    if config is None:
        print(
            "stereo_depth config not found. Set STEREO_DEPTH_CONFIG or deploy to " "/etc/ros2/stereo_depth/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_node(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
