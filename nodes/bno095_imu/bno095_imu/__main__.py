"""Entry point for BNO095 IMU node: load config and run publish loop."""

import sys

from .config import load_config_from_env
from .node import run_imu_node


def main() -> int:
    """Load config and run IMU node.

    Returns:
        int: 0 on success; 1 on config error.
    """
    config = load_config_from_env()
    if config is None:
        print(
            "BNO095 IMU config not found or invalid. Set BNO095_IMU_CONFIG to a YAML path "
            "or deploy to /etc/ros2/bno095_imu/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_imu_node(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
