"""Entry point for leader_bridge: load the YAML config and run the bridge."""

import os
import sys
from pathlib import Path

from .bridge import run
from .config import CONFIG_ENV_VAR, DEFAULT_CONFIG_PATH, ConfigError, load_config


def main() -> int:
    """Load the config named by LEADER_BRIDGE_CONFIG (default /etc/ros2/leader_bridge/config.yaml) and run.

    Returns:
        int: 0 after a clean shutdown, 1 on a config error.
    """
    path = Path(os.environ.get(CONFIG_ENV_VAR, str(DEFAULT_CONFIG_PATH)))
    try:
        config = load_config(path)
    except ConfigError as e:
        print(f"leader_bridge config error: {e}", file=sys.stderr)
        return 1
    run(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
