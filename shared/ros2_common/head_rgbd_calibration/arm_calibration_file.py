"""applied/arm_calibration.json: the accepted arm to camera calibration (head_rgbd_calibration writes it, mcp_server
reads it).

The head_rgbd_calibration accept writes `arm_calibration.json` into the applied directory and then the
`arm_calibration` record ({session, result, accepted_at, score}) of `applied.json`, the commit point of every apply. The
file is in effect only while that record names its session and result. mcp_server overrides exactly
`ARM_SETTINGS_FIELDS` of its `arm:` block with it (`ArmCalibrationFile.arm_settings_update`), the keys the reviewed
client.yml snippet (`arm_camera_fit.calibration_snippet_yaml`) sets; the rest of the body (score, report, tool point,
snippet) is for people and is ignored here.

Arm zero: offsets fitted from stills hold only for the servo zero in effect when the stills were taken. The follower
bridge (feetech_servos) writes the stale marker after a re-zero and `zero_changed.json` next to it whenever it changes a
servo zero (neutral save, zero restore). `zero_change_refusal` tells whether the zero may have changed at or after the
capture; the head_rgbd_calibration accept refuses then, and mcp_server only clears joint_offsets from the marker when it
did not.
"""

import json
import math
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any

from pydantic import AwareDatetime, BaseModel, ConfigDict, Field, ValidationError, field_validator

from .config import ARM_JOINT_NAMES
from .urdf_text import APPLIED_MARKER_FILE

ARM_CALIBRATION_FILE = "arm_calibration.json"
ARM_CALIBRATION_KEY = "arm_calibration"  # the applied.json record of the accepted arm calibration
# mcp_server `arm:` keys the accepted calibration overrides (arm_base_height_m is the arm base z: mcp_server requires
# base_in_base_link.z == arm_base_height_m).
ARM_SETTINGS_FIELDS = ("joint_offsets_rad", "arm_base_height_m", "base_in_base_link")
# The follower bridge's record of its last servo zero change, next to its stale marker (feetech_servos neutral.py).
ZERO_RECORD_FILE = "zero_changed.json"
STALE_OFFSETS_ITEM = "joint_offsets"
# The bridge writes whole seconds: a zero change in the capture's second counts as after it (fail closed).
ZERO_TIME_RESOLUTION = timedelta(seconds=1)


class ArmCalibrationError(ValueError):
    """The applied.json record names an arm calibration that cannot be read or does not validate."""


class ArmCalibrationBase(BaseModel):
    """Fitted arm base pose in base_link (m, rad), all four keys required."""

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    x: float
    y: float
    z: float
    yaw: float


class ArmCalibrationFile(BaseModel):
    """The fields of arm_calibration.json mcp_server applies, with the identity of the accepted fit.

    Attributes:
        session: head_arm session id.
        result: arm to camera result id.
        accepted_at: Accept time.
        captured_from: Capture time of the session's earliest still: the calibration holds for the arm zero in effect
            from then on (an arm re-zero stale marker saved before it is cleared for joint_offsets).
        joint_offsets_rad: Zero offset per arm joint (rad, urdf = measured + offset), exactly the five arm joints.
        base_in_base_link: Arm base pose in base_link.
    """

    model_config = ConfigDict(extra="ignore", allow_inf_nan=False)

    session: str = Field(min_length=1)
    result: str = Field(min_length=1)
    accepted_at: AwareDatetime
    captured_from: AwareDatetime
    joint_offsets_rad: dict[str, float]
    base_in_base_link: ArmCalibrationBase

    @field_validator("joint_offsets_rad")
    @classmethod
    def five_finite_offsets(cls, value: dict[str, float]) -> dict[str, float]:
        """Require exactly the arm joints, each finite.

        Args:
            value (dict[str, float]): Joint name -> offset (rad).

        Returns:
            dict[str, float]: The offsets.
        """
        if set(value) != set(ARM_JOINT_NAMES):
            raise ValueError(f"joint_offsets_rad must name exactly {list(ARM_JOINT_NAMES)}, got {sorted(value)}")
        bad = [name for name, offset in value.items() if not math.isfinite(offset)]
        if bad:
            raise ValueError(f"joint_offsets_rad must be finite: {bad}")
        return value

    def arm_settings_update(self) -> dict[str, Any]:
        """The mcp_server `arm:` values this calibration sets, keyed by ARM_SETTINGS_FIELDS in that order.

        Returns:
            dict[str, Any]: {joint_offsets_rad: {joint: rad}, arm_base_height_m: base z, base_in_base_link: {x, y, z,
                yaw}}.
        """
        base = self.base_in_base_link.model_dump()
        return {
            "joint_offsets_rad": {name: self.joint_offsets_rad[name] for name in ARM_JOINT_NAMES},
            "arm_base_height_m": base["z"],
            "base_in_base_link": base,
        }


def read_arm_calibration(applied_dir: Path) -> ArmCalibrationFile | None:
    """The accepted arm calibration in effect: the arm_calibration.json the applied.json record names.

    Args:
        applied_dir (Path): Applied calibration directory.

    Returns:
        ArmCalibrationFile | None: None when nothing is accepted (no applied.json, or no arm_calibration record in it;
            a file without its record is not in effect).

    Raises:
        ArmCalibrationError: applied.json is unreadable, or the record names a file that is missing, unreadable,
            invalid or of another session or result.
    """
    marker_path = applied_dir / APPLIED_MARKER_FILE
    try:
        marker = json.loads(marker_path.read_text())
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as exc:
        raise ArmCalibrationError(f"{marker_path} unreadable: {exc}") from exc
    if not isinstance(marker, dict):
        raise ArmCalibrationError(f"{marker_path} is not a JSON object")
    record = marker.get(ARM_CALIBRATION_KEY)
    if record is None:
        return None
    if not isinstance(record, dict):
        raise ArmCalibrationError(f"{marker_path} {ARM_CALIBRATION_KEY} record is not an object")
    path = applied_dir / ARM_CALIBRATION_FILE
    try:
        calibration = ArmCalibrationFile.model_validate(json.loads(path.read_text()))
    except (OSError, ValueError, ValidationError) as exc:
        raise ArmCalibrationError(f"{path} unreadable or invalid: {exc}") from exc
    named = (record.get("session"), record.get("result"))
    if named != (calibration.session, calibration.result):
        raise ArmCalibrationError(
            f"{path} holds {calibration.session}/{calibration.result} but {marker_path} names {named[0]}/{named[1]}"
        )
    return calibration


def zero_record_path(marker: Path) -> Path:
    """The bridge's zero change record next to its stale marker.

    Args:
        marker (Path): Stale marker path.

    Returns:
        Path: `zero_changed.json` in the marker's directory.
    """
    return marker.with_name(ZERO_RECORD_FILE)


def aware_time(value: object) -> datetime | None:
    """A timezone-aware ISO 8601 time.

    Args:
        value (object): JSON value.

    Returns:
        datetime | None: The time; None when it is not a string, not ISO 8601 or has no timezone.
    """
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value)
    except ValueError:
        return None
    return parsed if parsed.tzinfo is not None else None


def zero_change_refusal(marker: Path, since: datetime) -> str | None:
    """Why offsets measured from `since` on may not hold in the current arm zero (None when they do).

    Args:
        marker (Path): The follower bridge's stale marker ({version, group, saved_at, joints, stale}); its directory
            holds the zero change record ({version, group, changed_at}).
        since (datetime): Capture time of the first still (timezone aware).

    Returns:
        str | None: The reason when the marker or the record is unreadable, the marker lists joint_offsets without a
        timezone-aware saved_at or for a re-zero at or after `since`, or the record names a zero change at or after
        `since` (both within ZERO_TIME_RESOLUTION); None otherwise (no marker and no record is no zero change).
    """
    record_path = zero_record_path(marker)
    for path in (marker, record_path):
        try:
            data = json.loads(path.read_text())
        except FileNotFoundError:
            continue
        except (OSError, ValueError) as exc:
            return f"{path} is unreadable ({exc}): the arm zero of the capture is unknown"
        if not isinstance(data, dict):
            return f"{path} is unreadable (not a JSON object): the arm zero of the capture is unknown"
        if path == marker:
            stale = data.get("stale")
            if not isinstance(stale, list):
                return f"{path} is unreadable (no stale list): the arm zero of the capture is unknown"
            if STALE_OFFSETS_ITEM not in stale:
                continue
            saved = aware_time(data.get("saved_at"))
            if saved is None:
                return f"{path} lists joint_offsets with saved_at {data.get('saved_at')!r}: the arm zero is unknown"
            if saved + ZERO_TIME_RESOLUTION > since:
                return (
                    f"{path} lists joint_offsets for an arm re-zero at {data['saved_at']}, at or after the first still "
                    f"at {since.isoformat()}: the offsets were measured in the old zero"
                )
        else:
            changed = aware_time(data.get("changed_at"))
            if changed is None:
                return f"{path} is unreadable (changed_at {data.get('changed_at')!r}): the arm zero is unknown"
            if changed + ZERO_TIME_RESOLUTION > since:
                return (
                    f"{path}: the arm zero changed at {data['changed_at']}, at or after the first still at "
                    f"{since.isoformat()}: the offsets were measured in another zero"
                )
    return None
