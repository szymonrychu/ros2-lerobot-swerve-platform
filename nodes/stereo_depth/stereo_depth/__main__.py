"""Entry point for stereo_depth (python3 -m stereo_depth): load the YAML config and run the node."""

import sys

from .config import ConfigError, config_path_from_env, load_config
from .node import run


def main() -> int:
    """Load the config named by STEREO_DEPTH_CONFIG (default /etc/ros2/stereo_depth/config.yaml) and run.

    Returns:
        int: 0 after a clean shutdown, 1 on a config error.
    """
    try:
        config = load_config(config_path_from_env())
    except ConfigError as e:
        print(f"stereo_depth config error: {e}", file=sys.stderr)
        return 1
    run(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
