"""The accepted arm calibration at runtime, shared by its consumers (mcp_server, filter_node's guard, web_ui).

head_rgbd_calibration's arm to camera accept writes applied/arm_calibration.json and then the `arm_calibration`
record of applied/applied.json (arm_calibration_file). A consumer keys its reload on the identity of applied.json
(`applied_marker_identity`: mtime_ns, size, inode; the inode catches the atomic replace even when two writes share one
coarse mtime tick) and re-reads only when it changed, without a thread of its own.

The accepted calibration overrides exactly `ARM_SETTINGS_FIELDS` (joint offsets, arm base pose, mount height = base z)
of the consumer's configured arm model (a pydantic model with those fields, e.g. arm_collision ArmModelSettings,
mcp_server ArmSettings, or `ArmCalibrationSettings` for a consumer that holds only those values, web_ui); the configured
values are the baseline, used while nothing is accepted or after the record is removed. The tool point is not fitted and
stays configured.

`AppliedArmCalibration` holds that state (baseline, applied_dir, the applied.json identity last seen, the accepted
calibration in use and the settings in use). When and how a change is swapped into a running model, and how it is
logged, is the consumer's: mcp_server waits for the motion lock, filter_node swaps between two guard checks.

This module imports no OpenCV (an import-closure test pins it).
"""

from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .arm_calibration_file import (
    ARM_SETTINGS_FIELDS,
    ArmCalibrationBase,
    ArmCalibrationFile,
    check_arm_joint_offsets,
    read_arm_calibration,
)
from .urdf_text import APPLIED_MARKER_FILE

MarkerIdentity = tuple[int, int, int]  # applied.json (mtime_ns, size, inode)
# Mount z and arm_base_height_m are one height: they may differ by this much (m), as in arm_collision ArmModelSettings.
MOUNT_HEIGHT_TOLERANCE_M = 1e-6


class ArmCalibrationSettings(BaseModel):
    """The calibrated arm values (ARM_SETTINGS_FIELDS) alone, the baseline of a consumer without an arm model (web_ui).

    Validated as arm_collision ArmModelSettings validates them: offsets for exactly the five arm joints, a mount height
    in (0, 1] m and a mount z equal to it.

    Attributes:
        joint_offsets_rad: Zero offset per arm joint (rad, urdf = measured + offset).
        arm_base_height_m: Arm mount height above the floor (m).
        base_in_base_link: Arm base pose in base_link (m, rad).
    """

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    joint_offsets_rad: dict[str, float]
    arm_base_height_m: float = Field(gt=0.0, le=1.0)
    base_in_base_link: ArmCalibrationBase

    @field_validator("joint_offsets_rad")
    @classmethod
    def five_finite_offsets(cls, value: dict[str, float]) -> dict[str, float]:
        """Require exactly the arm joints, each finite (check_arm_joint_offsets).

        Args:
            value (dict[str, float]): Joint name -> offset (rad).

        Returns:
            dict[str, float]: The offsets.
        """
        return check_arm_joint_offsets(value)

    @model_validator(mode="after")
    def mount_z_is_the_mount_height(self) -> "ArmCalibrationSettings":
        """Require base_in_base_link.z to equal arm_base_height_m.

        Returns:
            ArmCalibrationSettings: The validated settings.
        """
        if abs(self.base_in_base_link.z - self.arm_base_height_m) > MOUNT_HEIGHT_TOLERANCE_M:
            z, height = self.base_in_base_link.z, self.arm_base_height_m
            raise ValueError(f"base_in_base_link.z ({z}) must equal arm_base_height_m ({height})")
        return self


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
