"""Entry point for master2master node: load config and run relays in a single process."""

import os
import sys
from pathlib import Path

from .config import ConfigError, load_config
from .proxy import run_all_relays


def main() -> int:
    """Load config and run relay node. Exits 0 if no rules; 1 on config error."""
    config_path = os.environ.get("MASTER2MASTER_CONFIG", "/etc/ros2/master2master/config.yaml")
    path = Path(config_path)
    try:
        rules = load_config(path)
    except ConfigError as e:
        print(f"Config error: {e}", file=sys.stderr)
        return 1
    if not rules:
        print("No topic rules in config; exiting.", file=sys.stderr)
        return 0
    run_all_relays(rules)
    return 0


if __name__ == "__main__":
    sys.exit(main())
