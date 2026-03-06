"""Entry point for static TF publisher."""

import sys

from .config import load_config_from_env
from .node import run_static_tf_publisher


def main() -> int:
    config = load_config_from_env()
    if config is None:
        print(
            "Static TF publisher config not found or empty. Set STATIC_TF_PUBLISHER_CONFIG "
            "or deploy to /etc/ros2/static_tf_publisher/config.yaml",
            file=sys.stderr,
        )
        return 1
    run_static_tf_publisher(config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
