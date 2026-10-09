"""Unit tests for BNO055 calibration profile persistence (pure helpers and fake-chip restore/save flows)."""

import json
from pathlib import Path
from unittest.mock import patch

import pytest

from bno055_imu.calibration import (
    CONFIG_MODE_VALUE,
    CalibrationProfile,
    apply_profile,
    load_profile,
    profile_from_dict,
    profile_to_dict,
    read_profile,
    save_if_changed,
    should_save,
    write_profile,
)

NDOF = 0x0C
PROFILE = CalibrationProfile(
    accel_offset=(10, -20, 30),
    gyro_offset=(1, 2, -3),
    mag_offset=(-100, 200, -300),
    accel_radius=1000,
    mag_radius=480,
)
FULL = (0, 3, 3, 3)


class FakeBno:
    """Fake chip: offset registers accept access only in CONFIG mode, like the real BNO055."""

    def __init__(self, mode: int = NDOF, profile: CalibrationProfile | None = None) -> None:
        self.mode = mode
        self.mode_history: list[int] = [mode]
        self.regs: dict[str, object] = {}
        if profile is not None:
            self._store(profile)

    def _store(self, p: CalibrationProfile) -> None:
        self.regs = {
            "offsets_accelerometer": p.accel_offset,
            "offsets_gyroscope": p.gyro_offset,
            "offsets_magnetometer": p.mag_offset,
            "radius_accelerometer": p.accel_radius,
            "radius_magnetometer": p.mag_radius,
        }

    def __setattr__(self, name: str, value: object) -> None:
        if name == "mode":
            object.__setattr__(self, name, value)
            if hasattr(self, "mode_history"):
                self.mode_history.append(value)  # type: ignore[arg-type]
            return
        if name in ("regs", "mode_history"):
            object.__setattr__(self, name, value)
            return
        assert self.mode == CONFIG_MODE_VALUE, f"{name} written outside CONFIG mode"
        self.regs[name] = value

    def __getattr__(self, name: str) -> object:
        regs = object.__getattribute__(self, "regs")
        if name in regs:
            assert self.mode == CONFIG_MODE_VALUE, f"{name} read outside CONFIG mode"
            return regs[name]
        raise AttributeError(name)


def test_dict_roundtrip() -> None:
    """profile_to_dict then profile_from_dict returns the same profile."""
    assert profile_from_dict(profile_to_dict(PROFILE)) == PROFILE


@pytest.mark.parametrize(
    "mutate",
    [
        lambda d: d.pop("mag_offset"),
        lambda d: d.update(accel_offset=[1, 2]),
        lambda d: d.update(gyro_offset=[1, 2, 40000]),
        lambda d: d.update(mag_offset=[1.5, 2, 3]),
        lambda d: d.update(mag_offset=[True, 2, 3]),
        lambda d: d.update(accel_radius=0),
        lambda d: d.update(mag_radius=-5),
        lambda d: d.update(accel_radius=100000),
        lambda d: d.update(accel_radius="1000"),
    ],
)
def test_profile_from_dict_rejects_invalid(mutate) -> None:  # noqa: ANN001
    """Missing keys, wrong lengths, non-int, out-of-range values and implausible radii raise ValueError."""
    data = profile_to_dict(PROFILE)
    mutate(data)
    with pytest.raises(ValueError):
        profile_from_dict(data)


def test_profile_from_dict_rejects_non_dict() -> None:
    """A JSON list is not a profile."""
    with pytest.raises(ValueError):
        profile_from_dict([1, 2, 3])  # type: ignore[arg-type]


def test_should_save_requires_all_three_calibrated() -> None:
    """gyro, accel and mag must be 3; sys is ignored."""
    assert should_save((0, 3, 3, 3), 0.0, 100.0, 60.0, None, None)
    assert should_save((3, 3, 3, 3), 0.0, 100.0, 60.0, None, None)
    assert not should_save((0, 3, 3, 2), 0.0, 100.0, 60.0, None, None)
    assert not should_save((0, 2, 3, 3), 0.0, 100.0, 60.0, None, None)
    assert not should_save((0, 3, 2, 3), 0.0, 100.0, 60.0, None, None)


def test_should_save_invalid_status() -> None:
    """None or short status never saves."""
    assert not should_save(None, 0.0, 100.0, 60.0, None, None)
    assert not should_save((0, 3, 3), 0.0, 100.0, 60.0, None, None)
    assert not should_save((0, None, 3, 3), 0.0, 100.0, 60.0, None, None)


def test_should_save_respects_interval() -> None:
    """Not before the interval has elapsed."""
    assert not should_save(FULL, 50.0, 100.0, 60.0, None, None)
    assert should_save(FULL, 40.0, 100.0, 60.0, None, None)


def test_should_save_only_when_profile_differs() -> None:
    """An identical profile skips; unknown current profile (not yet read) or a different one saves."""
    assert not should_save(FULL, 0.0, 100.0, 60.0, PROFILE, PROFILE)
    other = CalibrationProfile((0, 0, 0), (0, 0, 0), (0, 0, 0), 1000, 480)
    assert should_save(FULL, 0.0, 100.0, 60.0, other, PROFILE)
    assert should_save(FULL, 0.0, 100.0, 60.0, PROFILE, None)


def test_write_profile_atomic_creates_parent(tmp_path: Path) -> None:
    """Parent dir is created, content parses back, no temp file remains."""
    path = tmp_path / "a" / "b" / "calibration.json"
    write_profile(path, PROFILE)
    assert load_profile(path) == PROFILE
    assert [p.name for p in path.parent.iterdir()] == ["calibration.json"]


def test_write_profile_uses_os_replace(tmp_path: Path) -> None:
    """The final file appears via os.replace of a temp file in the same directory."""
    path = tmp_path / "calibration.json"
    with patch("bno055_imu.calibration.os.replace") as repl:
        write_profile(path, PROFILE)
    src, dst = repl.call_args.args
    assert Path(dst) == path
    assert Path(src).parent == path.parent
    assert not path.exists()


def test_write_profile_failure_keeps_old_file(tmp_path: Path) -> None:
    """If the replace fails the old file is untouched and the temp file is cleaned up."""
    path = tmp_path / "calibration.json"
    write_profile(path, PROFILE)
    newer = CalibrationProfile((9, 9, 9), (9, 9, 9), (9, 9, 9), 1000, 480)
    with patch("bno055_imu.calibration.os.replace", side_effect=OSError("boom")), pytest.raises(OSError):
        write_profile(path, newer)
    assert load_profile(path) == PROFILE
    assert [p.name for p in tmp_path.iterdir()] == ["calibration.json"]


def test_load_profile_missing_returns_none(tmp_path: Path) -> None:
    """A missing file is not an error."""
    assert load_profile(tmp_path / "nope.json") is None


@pytest.mark.parametrize("content", ["", "not json", "[1,2]", '{"accel_offset": [1,2,3]}'])
def test_load_profile_corrupt_raises(tmp_path: Path, content: str) -> None:
    """Corrupt or incomplete content raises ValueError so the caller can warn and continue."""
    path = tmp_path / "c.json"
    path.write_text(content)
    with pytest.raises(ValueError):
        load_profile(path)


def test_apply_profile_writes_in_config_mode() -> None:
    """All five properties are written while the chip is in CONFIG mode (fake asserts it)."""
    bno = FakeBno(mode=NDOF)
    with patch("bno055_imu.calibration.time.sleep"):
        apply_profile(bno, PROFILE)
    assert bno.regs["offsets_magnetometer"] == PROFILE.mag_offset
    assert bno.regs["radius_accelerometer"] == PROFILE.accel_radius
    assert len(bno.regs) == 5
    assert bno.mode == CONFIG_MODE_VALUE  # caller switches to the operation mode afterwards


def test_read_profile_switches_to_config_and_back() -> None:
    """Offsets are read in CONFIG mode and the operation mode is restored afterwards."""
    bno = FakeBno(mode=NDOF, profile=PROFILE)
    with patch("bno055_imu.calibration.time.sleep"):
        assert read_profile(bno, NDOF) == PROFILE
    assert bno.mode == NDOF
    assert CONFIG_MODE_VALUE in bno.mode_history


def test_read_profile_restores_mode_on_error() -> None:
    """A failing read still switches back to the operation mode."""
    bno = FakeBno(mode=NDOF)  # no registers -> AttributeError on read
    with patch("bno055_imu.calibration.time.sleep"), pytest.raises(AttributeError):
        read_profile(bno, NDOF)
    assert bno.mode == NDOF


def test_save_if_changed_writes_new_profile(tmp_path: Path) -> None:
    """A differing profile is written and returned as the new saved profile."""
    path = tmp_path / "cal.json"
    bno = FakeBno(profile=PROFILE)
    with patch("bno055_imu.calibration.time.sleep"):
        saved = save_if_changed(bno, path, NDOF, None)
    assert saved == PROFILE
    assert load_profile(path) == PROFILE


def test_save_if_changed_skips_identical(tmp_path: Path) -> None:
    """An identical profile is not rewritten."""
    path = tmp_path / "cal.json"
    bno = FakeBno(profile=PROFILE)
    with patch("bno055_imu.calibration.time.sleep"), patch("bno055_imu.calibration.write_profile") as w:
        saved = save_if_changed(bno, path, NDOF, PROFILE)
    assert saved == PROFILE
    w.assert_not_called()


def test_save_if_changed_ignores_implausible_chip_profile(tmp_path: Path) -> None:
    """A radius of 0 read back from the chip is not persisted."""
    path = tmp_path / "cal.json"
    bad = CalibrationProfile((0, 0, 0), (0, 0, 0), (0, 0, 0), 0, 0)
    bno = FakeBno(profile=bad)
    with patch("bno055_imu.calibration.time.sleep"):
        saved = save_if_changed(bno, path, NDOF, None)
    assert saved is None
    assert not path.exists()


def test_profile_json_file_format(tmp_path: Path) -> None:
    """The file is plain JSON with the documented keys."""
    path = tmp_path / "cal.json"
    write_profile(path, PROFILE)
    assert set(json.loads(path.read_text())) == {
        "accel_offset",
        "gyro_offset",
        "mag_offset",
        "accel_radius",
        "mag_radius",
    }
