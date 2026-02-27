"""Entry point for master2master node: load config and run relays in a single process."""

import os
import sys
from pathlib import Path

from .config import DEFAULT_CONFIG_PATH, ConfigError, load_config
from .proxy import run_all_relays


def main() -> int:
    """Load config and run relay node.

    Returns:
        int: 0 on success or when no rules; 1 on config error.
    """
    config_path = os.environ.get("MASTER2MASTER_CONFIG", str(DEFAULT_CONFIG_PATH))
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
