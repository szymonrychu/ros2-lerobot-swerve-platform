"""Retirement pin: the ssh-streamed live pose counter is gone; the web UI Calibrate page replaces it."""

import subprocess
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from scripts.head_rgbd_calibration.config import DEFAULT_CONFIG, CalibrationConfig, load_calibration_config

REPO = Path(__file__).resolve().parents[1]
THIS_FILE = ":!tests/test_head_rgbd_calib_retired.py"
REMOVED_PATHS = (
    "scripts/head_rgbd_calibration/live.py",
    "scripts/head_rgbd_calibration/live_relay.py",
    "tests/test_head_rgbd_calib_live.py",
)
RETIRED_NAMES = ("live_relay", "start_live", "capture.live")


def git(*args: str) -> subprocess.CompletedProcess[str]:
    """Run git in the repo root and return the finished process."""
    return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True, check=False)


def test_removed_files_are_not_tracked() -> None:
    assert git("ls-files", "--", *REMOVED_PATHS).stdout.strip() == ""


@pytest.mark.parametrize("name", RETIRED_NAMES)
def test_no_tracked_reference_to_the_live_counter(name: str) -> None:
    result = git("grep", "-n", "-F", name, "--", "scripts", "tests", THIS_FILE)
    assert result.stdout == ""


def test_capture_config_has_no_live_field() -> None:
    assert "live" not in CalibrationConfig().capture.model_fields_set
    assert not hasattr(CalibrationConfig().capture, "live")


def test_config_still_setting_live_fails_naming_it(tmp_path: Path) -> None:
    data = yaml.safe_load(DEFAULT_CONFIG.read_text())
    data["capture"]["live"] = True
    config = tmp_path / "calibrate.yaml"
    config.write_text(yaml.safe_dump(data))
    with pytest.raises(ValidationError, match="live"):
        load_calibration_config(config)
