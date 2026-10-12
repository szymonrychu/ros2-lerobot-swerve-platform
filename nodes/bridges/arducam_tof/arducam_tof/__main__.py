"""Entry point for arducam_tof (python3 -m arducam_tof): load the YAML config and run the bridge."""

import sys

from .bridge import run
from .config import ConfigError, config_path_from_env, load_config


def main() -> int:
    """Load the config named by ARDUCAM_TOF_CONFIG (default /etc/ros2/arducam_tof/config.yaml) and run.

    Returns:
        int: 0 after a clean shutdown, 1 on a config error.
    """
    try:
        config = load_config(config_path_from_env())
    except ConfigError as e:
        print(f"arducam_tof config error: {e}", file=sys.stderr)
        return 1
    run(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
