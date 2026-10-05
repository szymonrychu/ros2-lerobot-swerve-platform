"""Entry point for the rf2o odometry relay."""

import sys

from .config import load_config_from_env
from .node import run_relay


def main() -> int:
    """Load config and run the relay.

    Returns:
        int: 0 on success; 1 on config error.
    """
    config = load_config_from_env()
    if config is None:
        print(
            "rf2o odometry relay config not found. Set RF2O_ODOM_RELAY_CONFIG or deploy to "
            "/etc/ros2/rf2o_odom_relay/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_relay(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
