"""applied/arm_calibration.json: the accepted arm to camera calibration (head_rgbd_calibration writes it, mcp_server
reads it).

The head_rgbd_calibration accept writes `arm_calibration.json` into the applied directory and then the
`arm_calibration` record ({session, result, accepted_at, score}) of `applied.json`, the commit point of every apply. The
file is in effect only while that record names its session and result. mcp_server overrides exactly
`ARM_SETTINGS_FIELDS` of its `arm:` block with it (`ArmCalibrationFile.arm_settings_update`), the keys the reviewed
client.yml snippet (`arm_camera_fit.calibration_snippet_yaml`) sets; the rest of the body (score, report, tool point,
snippet) is for people and is ignored here.
"""

import json
import math
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
