"""Repo-wide guard: the retired front camera left no live reference, and no doc states the old preview size."""

import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
LIVE_REF_PATTERN = "/front_camera|front_camera_topic|topics\\.front_camera|pi_cameras_config|front_camera_optical_frame"
ALLOWED_PREFIXES = ("docs/plans/",)
ALLOWED_FILES = frozenset(
    {
        "tests/test_pi_cameras_retired.py",
        "tests/test_head_camera_live_refs.py",
        # Pins the MEMORY.md decision lines as regexes.
        "tests/test_" + "j" + "etson_retired.py",
        # Absence assertions that the retired names are gone.
        "tests/test_mcp_server_config.py",
        "nodes/web_ui/frontend/src/hooks/subscription.test.ts",
        # Retired pi_cameras entry and type, and the decision record.
        "ansible/group_vars/client.yml",
        "MEMORY.md",
    }
)
OLD_SIZE_PATTERN = re.compile(r"640\s*[x×]\s*480|\b10\s*(?:fps|Hz)\b", re.IGNORECASE)
CAMERA_WORD_PATTERN = re.compile(r"\b(?:front|head)\b", re.IGNORECASE)
WINDOW_CHARS = 60
CALIBRATED_PATTERN = re.compile(r"calibrat|intrinsic", re.IGNORECASE)
DOC_DIRS = (
    REPO_ROOT / "nodes" / "mcp_server" / "mcp_server",
    REPO_ROOT / "nodes" / "claude_agent" / "claude_agent",
)
DOC_FILES = (
    REPO_ROOT / "nodes" / "mcp_server" / "README.md",
    REPO_ROOT / "nodes" / "claude_agent" / "README.md",
    REPO_ROOT / "nodes" / "web_ui" / "README.md",
)


def find_live_refs() -> list[str]:
    """Return the tracked-file hits of the retired front camera names outside the allowed records.

    Returns:
        list[str]: `git grep -n` lines that are not in docs/plans or a retirement record.
    """
    result = subprocess.run(
        ["git", "grep", "-n", "-I", "-E", LIVE_REF_PATTERN],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    hits = []
    for line in result.stdout.splitlines():
        path = line.split(":", 1)[0]
        if path in ALLOWED_FILES or path.startswith(ALLOWED_PREFIXES):
            continue
        hits.append(line[:200])
    return hits


def find_old_preview_sizes(text: str) -> list[str]:
    """Find a front or head camera word within WINDOW_CHARS of 640x480 or 10 fps.

    Args:
        text (str): File content.

    Returns:
        list[str]: The offending snippets.
    """
    snippets = []
    for size in OLD_SIZE_PATTERN.finditer(text):
        start = max(0, size.start() - WINDOW_CHARS)
        window = text[start : size.end() + WINDOW_CHARS]
        # The calibrated size (640x480) is the pixel-tool frame, not the preview size.
        if CAMERA_WORD_PATTERN.search(window) and not CALIBRATED_PATTERN.search(window):
            snippets.append(window.replace("\n", " "))
    return snippets


def test_no_live_front_camera_reference() -> None:
    """No tracked file outside docs/plans and the retirement records names the retired front camera."""
    assert find_live_refs() == []


def test_no_640x480_head_or_front_preview_size() -> None:
    """Tool descriptions, the agent prompt/config and the web_ui README state no 640x480 or 10 fps head preview."""
    files = [*DOC_FILES]
    for directory in DOC_DIRS:
        files.extend(sorted(directory.glob("*.py")))
    offenders = {}
    for path in files:
        snippets = find_old_preview_sizes(path.read_text())
        if snippets:
            offenders[str(path.relative_to(REPO_ROOT))] = snippets
    assert offenders == {}
