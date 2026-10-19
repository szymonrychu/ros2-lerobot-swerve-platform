"""Entry point: load the config file and run the marker detector node."""

import logging
import sys

from marker_detector.config import config_path, load_config
from marker_detector.node import run_node


def main() -> int:
    """Load config and run the node.

    Returns:
        0 on success, 1 when the config file is missing.
    """
    logging.basicConfig(level=logging.INFO)
    path = config_path()
    if not path.is_file():
        print(f"marker_detector config not found at {path}. Set MARKER_DETECTOR_CONFIG.", file=sys.stderr)
        return 1
    run_node(load_config(path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
