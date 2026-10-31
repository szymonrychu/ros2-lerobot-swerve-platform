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


CODE_PATHS = ("shared/*.py", "nodes/*.py")
TOF_FIT_NAMES = ("detect_tof", "calibrate_tof", "normalise_amplitude")
POSES_MODULE = "shared/ros2_common/head_rgbd_calibration/poses.py"
EXTRINSICS_IMPORT = r"head_rgbd_calibration(\.extrinsics( |$)| import .*extrinsics)"
EXTRINSICS_IMPORTERS_ALLOWED = (
    "shared/ros2_common/head_rgbd_calibration/extrinsics.py",
    "shared/ros2_common/head_rgbd_calibration/urdf_handoff.py",
)


def test_tof_fits_are_gone() -> None:
    """The ToF fits of the head RGB-D calibration are retired: no definition, no live pose counter module, and no
    code under shared/ or nodes/ outside extrinsics.py and urdf_handoff.py imports the extrinsics module."""
    defined = [
        line
        for name in TOF_FIT_NAMES
        for line in git("grep", "-n", "-E", rf"^[[:space:]]*def {name}\(", "--", *CODE_PATHS).stdout.splitlines()
    ]
    assert defined == []
    assert git("ls-files", "--", POSES_MODULE).stdout.strip() == ""
    importers = {
        line.split(":", 1)[0]
        for line in git("grep", "-n", "-E", EXTRINSICS_IMPORT, "--", *CODE_PATHS).stdout.splitlines()
    }
    assert importers <= set(EXTRINSICS_IMPORTERS_ALLOWED), sorted(importers)
