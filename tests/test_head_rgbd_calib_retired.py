"""Retirement pins: the ssh-streamed live pose counter and the ToF+CM3 offline calibration CLI are gone.

The web UI Calibrate page replaces both.
"""

import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
THIS_FILE = ":!tests/test_head_rgbd_calib_retired.py"
REMOVED_PATHS = (
    "scripts/head_rgbd_calibration/live.py",
    "scripts/head_rgbd_calibration/live_relay.py",
    "tests/test_head_rgbd_calib_live.py",
)
RETIRED_NAMES = ("live_relay", "start_live", "capture.live")
OFFLINE_CLI_PATHS = (
    "scripts/head_rgbd_calibrate.py",
    "scripts/head_rgbd_calibrate.yaml",
    "scripts/head_rgbd_calibration",
)
OFFLINE_CLI_NAME = "head_rgbd_calibrate.py"


def git(*args: str) -> subprocess.CompletedProcess[str]:
    """Run git in the repo root and return the finished process.

    Args:
        *args (str): Arguments after `git`.

    Returns:
        subprocess.CompletedProcess[str]: The finished process with captured text output.
    """
    return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True, check=False)


def test_live_counter_files_untracked() -> None:
    assert git("ls-files", "--", *REMOVED_PATHS).stdout.strip() == ""


def test_no_live_counter_references() -> None:
    for name in RETIRED_NAMES:
        result = git("grep", "-n", "-F", name, "--", "scripts", "tests", THIS_FILE)
        assert result.stdout == "", name


def test_offline_cli_is_gone() -> None:
    assert git("ls-files", "--", *OFFLINE_CLI_PATHS).stdout.strip() == ""
    result = git("grep", "-n", "-F", OFFLINE_CLI_NAME, "--", ".", ":!docs", THIS_FILE)
    assert result.stdout == ""
