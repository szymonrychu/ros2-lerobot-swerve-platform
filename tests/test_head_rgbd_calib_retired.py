"""Retirement pins: the ssh-streamed live pose counter and the ToF+CM3 offline calibration CLI are gone.

The web UI Calibrate page replaces both.
"""

import subprocess
from pathlib import Path

import pytest
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.config import LimitsConfig, SelectionConfig

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


STEREO_MODULES = (
    "shared/ros2_common/head_rgbd_calibration/extrinsics.py",
    "shared/ros2_common/head_rgbd_calibration/urdf_handoff.py",
)
STEREO_MODULE_IMPORT = r"head_rgbd_calibration(\.(extrinsics|urdf_handoff)( |$)| import .*(extrinsics|urdf_handoff))"
RETIRED_SELECTION_KEYS = {
    "still_tof_mad": 0.01,
    "amplitude_percentiles": (1, 99),
    "tof_upscale": 2,
    "max_pair_dt_s": 0.05,
}
RETIRED_LIMIT_KEYS = {
    "tof_max_rms_px": 0.3,
    "stereo_max_rms_px": 1.0,
    "tof_zero_distortion_rms_px": 0.3,
    "rgb_max_rms_px": 0.5,
}


@pytest.mark.parametrize(
    ("model", "key", "value"),
    [
        *((SelectionConfig, key, value) for key, value in RETIRED_SELECTION_KEYS.items()),
        *((LimitsConfig, key, value) for key, value in RETIRED_LIMIT_KEYS.items()),
    ],
)
def test_tof_stereo_modules_are_gone(model: type, key: str, value: object) -> None:
    """The ToF-to-RGB extrinsics and the tof_joint URDF hand-off modules are untracked, no tracked file imports them,
    and the selection and limit keys only they used are rejected."""
    assert git("ls-files", "--", *STEREO_MODULES).stdout.strip() == ""
    assert git("grep", "-n", "-E", STEREO_MODULE_IMPORT, "--", "*.py").stdout == ""
    with pytest.raises(ValidationError, match=key):
        model.model_validate({key: value})
