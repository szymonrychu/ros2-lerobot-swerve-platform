"""Repo-wide guard: the pan/tilt head is live, so no placeholder wording and no base_link-pinned front camera remain."""

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
THIS_FILE = "tests/test_head_live_refs.py"
ALLOWED_PREFIXES = ("docs/plans/",)
PLACEHOLDER_PATTERN = "no head mount " + "configured|disabled pan" + "/tilt|future head " + "mount"
PINNED_PATTERN = 'FRONT_CAMERA_PARENT *= *"base' + '_link"|front camera works in base' + "_link"
MCP_SERVER_DIR = "nodes/mcp_server"


def grep_tracked(pattern: str, pathspec: str = ".") -> list[str]:
    """Return `git grep` hits for a pattern in tracked files, minus docs/plans and this test.

    Args:
        pattern (str): Extended regex, case-insensitive.
        pathspec (str): Git pathspec to restrict the search.

    Returns:
        list[str]: Hit lines (`path:line:text`, truncated).
    """
    result = subprocess.run(
        ["git", "grep", "-n", "-I", "-i", "-E", pattern, "--", pathspec],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    hits = []
    for line in result.stdout.splitlines():
        path = line.split(":", 1)[0]
        if path == THIS_FILE or path.startswith(ALLOWED_PREFIXES):
            continue
        hits.append(line[:200])
    return hits


def test_no_head_placeholder_text() -> None:
    """No tracked file outside docs/plans words the head as unconfigured, disabled or a future mount."""
    assert grep_tracked(PLACEHOLDER_PATTERN) == []


def test_front_camera_not_pinned_to_base_link() -> None:
    """mcp_server neither pins the front camera parent to base_link nor claims the front camera works in base_link."""
    assert grep_tracked(PINNED_PATTERN, MCP_SERVER_DIR) == []
