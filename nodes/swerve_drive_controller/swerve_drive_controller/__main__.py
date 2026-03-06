"""Entry point for swerve drive controller: load config and run node."""

import sys

from .config import load_config_from_env
from .node import run_swerve_controller


def main() -> int:
    """Load config and run swerve controller.

    Returns:
        int: 0 on success; 1 on config error.
    """
    config = load_config_from_env()
    if config is None:
        print(
            "Swerve drive controller config not found or invalid. Set SWERVE_DRIVE_CONTROLLER_CONFIG "
            "to a YAML path or deploy to /etc/ros2/swerve_drive_controller/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_swerve_controller(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
