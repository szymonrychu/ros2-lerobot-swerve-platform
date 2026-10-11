"""Generate (or check) the committed arm reach map nodes/mcp_server/mcp_server/data/reach_map.npz.

Thin wrapper: the map is computed by mcp_server's own arm and body model (ikpy, numpy), which only the mcp_server uv
environment has, so this runs `python -m mcp_server.reach_map build|check` there through `uv run --project`. Both use the
mcp_server config of ansible/group_vars/client.yml. See nodes/mcp_server/README.md ("Reach map").

Usage:
    uv run python scripts/gen_reach_map.py          # rebuild the artifact (about 1.5 min on the M4)
    uv run python scripts/gen_reach_map.py --check  # exit 1 when the artifact is missing or stale
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MCP_SERVER_DIR = REPO_ROOT / "nodes" / "mcp_server"
CLIENT_YML = Path("ansible/group_vars/client.yml")
ARTIFACT = Path("nodes/mcp_server/mcp_server/data/reach_map.npz")


def reach_map_command(check: bool) -> list[str]:
    """The mcp_server reach_map command line.

    Args:
        check (bool): Check the artifact instead of building it.

    Returns:
        list[str]: Command, run from the repository root.
    """
    base = ["uv", "run", "--project", str(MCP_SERVER_DIR), "python", "-m", "mcp_server.reach_map"]
    if check:
        return [*base, "check", "--client-yml", str(CLIENT_YML), "--file", str(ARTIFACT)]
    return [*base, "build", "--client-yml", str(CLIENT_YML), "--out", str(ARTIFACT)]


def main(argv: list[str]) -> int:
    """Build or check the reach map.

    Args:
        argv (list[str]): Command line arguments.

    Returns:
        int: Exit code of the mcp_server command.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="exit 1 when the artifact is missing or stale")
    args = parser.parse_args(argv)
    env = {k: v for k, v in os.environ.items() if k != "VIRTUAL_ENV"}
    return subprocess.run(reach_map_command(args.check), cwd=REPO_ROOT, env=env, check=False).returncode


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
