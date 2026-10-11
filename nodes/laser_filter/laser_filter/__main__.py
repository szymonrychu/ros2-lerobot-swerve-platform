"""Entry point: python -m laser_filter (config from $LASER_FILTER_CONFIG)."""

import sys

from .config import config_path_from_env, load_config
from .node import run


def main() -> int:
    """Load the config and run the node.

    Returns:
        int: Exit code.
    """
    run(load_config(config_path_from_env()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
