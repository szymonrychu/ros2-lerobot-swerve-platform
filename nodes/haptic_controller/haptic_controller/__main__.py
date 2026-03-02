"""Entry point for haptic controller node."""

import sys

from .config import load_config_from_env
from .node import run_haptic_node


def main() -> int:
    """Load config and run haptic node. Returns 1 on config error."""
    config = load_config_from_env()
    if config is None:
        print(
            "Haptic controller config not found or invalid. Set HAPTIC_CONTROLLER_CONFIG to a YAML path "
            "or deploy to /etc/ros2/haptic_controller/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_haptic_node(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
