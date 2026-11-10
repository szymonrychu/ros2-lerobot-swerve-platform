"""The accepted arm calibration at runtime, shared by its consumers (mcp_server, filter_node's guard, web_ui).

head_rgbd_calibration's arm to camera accept writes applied/arm_calibration.json and then the `arm_calibration`
record of applied/applied.json (arm_calibration_file). A consumer keys its reload on the identity of applied.json
(`applied_marker_identity`: mtime_ns, size, inode; the inode catches the atomic replace even when two writes share one
coarse mtime tick) and re-reads only when it changed, without a thread of its own.

The accepted calibration overrides exactly `ARM_SETTINGS_FIELDS` (joint offsets, arm base pose, mount height = base z)
of the consumer's configured arm model (a pydantic model with those fields, e.g. arm_collision ArmModelSettings or
mcp_server ArmSettings); the configured values are the baseline, used while nothing is accepted or after the record is
removed. The tool point is not fitted and stays configured.

`AppliedArmCalibration` holds that state (baseline, applied_dir, the applied.json identity last seen, the accepted
calibration in use and the settings in use). When and how a change is swapped into a running model, and how it is
logged, is the consumer's: mcp_server waits for the motion lock, filter_node swaps between two guard checks.

This module imports no OpenCV (an import-closure test pins it).
"""

from pathlib import Path

from pydantic import BaseModel

from .arm_calibration_file import ARM_SETTINGS_FIELDS, ArmCalibrationFile, read_arm_calibration
from .urdf_text import APPLIED_MARKER_FILE

MarkerIdentity = tuple[int, int, int]  # applied.json (mtime_ns, size, inode)


def applied_marker_identity(applied_dir: Path | None) -> MarkerIdentity | None:
    """Identity of applied.json's current version, the one reload key of the applied directory.

    Args:
        applied_dir (Path | None): Applied calibration directory; None disables the override.

    Returns:
        MarkerIdentity | None: (mtime_ns, size, inode), None without applied_dir or applied.json.
    """
    if applied_dir is None:
        return None
    try:
        st = (applied_dir / APPLIED_MARKER_FILE).stat()
    except OSError:
        return None
    return (st.st_mtime_ns, st.st_size, st.st_ino)


def calibrated_settings[SettingsT: BaseModel](baseline: SettingsT, calibration: ArmCalibrationFile | None) -> SettingsT:
    """The configured arm settings with the accepted calibration's fields, validated as the baseline's model.

    Args:
        baseline (SettingsT): The configured (client.yml) arm settings.
        calibration (ArmCalibrationFile | None): Accepted calibration; None returns a copy of the baseline.

    Returns:
        SettingsT: New settings of the baseline's type.

    Raises:
        ValidationError: The merged block is not valid for the baseline's model.
    """
    if calibration is None:
        return baseline.model_copy(deep=True)
    return type(baseline).model_validate({**baseline.model_dump(), **calibration.arm_settings_update()})


def assign_calibration(target: BaseModel, source: BaseModel) -> None:
    """Copy the calibrated fields (ARM_SETTINGS_FIELDS) into a settings object in place, so every holder sees them.

    Args:
        target (BaseModel): Settings object to change.
        source (BaseModel): Validated settings to take the fields from.
    """
    for name in ARM_SETTINGS_FIELDS:
        value = getattr(source, name)
        setattr(target, name, value.model_copy() if isinstance(value, BaseModel) else value)


class AppliedArmCalibration[SettingsT: BaseModel]:
    """The accepted arm calibration over a configured baseline, re-read when applied.json changes.

    Attributes:
        baseline: Snapshot of the configured arm settings.
        applied_dir: Applied calibration directory; None disables the override (the baseline stays in use).
        stamp: applied.json identity of the last `changed` call (None before it, or without the file).
        loaded: The accepted calibration in use, None for the configured values.
        current: The arm settings in use.
    """

    def __init__(self, baseline: SettingsT, applied_dir: Path | None) -> None:
        """Snapshot the configured arm settings before any override.

        Args:
            baseline (SettingsT): The configured arm settings.
            applied_dir (Path | None): Applied calibration directory; None disables the override.
        """
        self.baseline = baseline.model_copy(deep=True)
        self.applied_dir = applied_dir
        self.stamp: MarkerIdentity | None = None
        self.loaded: ArmCalibrationFile | None = None
        self.current: SettingsT = baseline.model_copy(deep=True)

    def marker_stamp(self) -> MarkerIdentity | None:
        """Identity of applied.json's current version (see applied_marker_identity).

        Returns:
            MarkerIdentity | None: (mtime_ns, size, inode), None without applied_dir or applied.json.
        """
        return applied_marker_identity(self.applied_dir)

    def changed(self) -> bool:
        """Whether applied.json changed since the last call (the first call compares against no file); remembers it.

        Returns:
            bool: True when the identity differs from the one last seen.
        """
        stamp = self.marker_stamp()
        if stamp == self.stamp:
            return False
        self.stamp = stamp
        return True

    def read(self) -> tuple[SettingsT, ArmCalibrationFile | None]:
        """Read the accepted calibration and merge it over the baseline.

        Returns:
            tuple[SettingsT, ArmCalibrationFile | None]: (merged settings, calibration or None for the configured
                values; the baseline is returned for None as well as without applied_dir).

        Raises:
            ArmCalibrationError: applied.json or the arm_calibration.json it names is unreadable or invalid.
            ValidationError: The merged settings are not valid for the baseline's model.
        """
        calibration = None if self.applied_dir is None else read_arm_calibration(self.applied_dir)
        return calibrated_settings(self.baseline, calibration), calibration

    def same(self, calibration: ArmCalibrationFile | None) -> bool:
        """Whether a read calibration is the one in use.

        Args:
            calibration (ArmCalibrationFile | None): Calibration just read.

        Returns:
            bool: True for the same accepted file content or both the configured values.
        """
        return calibration == self.loaded

    def use(self, settings: SettingsT, calibration: ArmCalibrationFile | None) -> SettingsT:
        """Record the settings and calibration now in use.

        Args:
            settings (SettingsT): Settings now in use.
            calibration (ArmCalibrationFile | None): Accepted calibration now in use, None for the configured values.

        Returns:
            SettingsT: The settings in use before.
        """
        old = self.current
        self.current, self.loaded = settings, calibration
        return old
