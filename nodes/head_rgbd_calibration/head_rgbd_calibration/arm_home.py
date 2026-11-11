"""The stored arm home: mcp_server's home file (`{joints: {name: rad}}`, written by Set home), read at each press.

The node reads the file mcp_server's `arm.home_file` names (config `arm_home_file`) instead of keeping its own copy of
the home, so a re-stored home is the board Home target at the next press. Only the five arm joints are returned; the
gripper entry is dropped (the gripper keeps its own angle while the board is held). Every `joints` entry, the gripper
and any other joint included, must be a finite number (no booleans, strings or NaN/inf), or the whole file is refused.
"""

import math
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, ValidationError, field_validator
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES

NO_JOINTS = "no `joints` mapping of joint name -> rad"
NO_JOINTS_ERRORS = ("missing", "dict_type")
MESSAGE_NO_HOME = (
    "no stored arm home at {path}: store the arm home (Set home: Calibrate > Servo calibration, Advanced / repair, or "
    "the Arm tab) first"
)


class ArmHomeError(RuntimeError):
    """The stored arm home is missing, unreadable or invalid; the message names the file."""


class ArmHomeFile(BaseModel):
    """The home file: joint name -> finite angle (rad); every arm joint is present, other entries are ignored."""

    model_config = ConfigDict(extra="ignore")

    joints: dict[str, float]

    @field_validator("joints", mode="before")
    @classmethod
    def check_numbers(cls, joints: object) -> object:
        """Refuse booleans and strings before pydantic coerces them to floats.

        Args:
            joints (object): The raw `joints` value.

        Returns:
            object: `joints`, unchanged.

        Raises:
            ValueError: An entry (any joint, the gripper included) is a boolean or not an int or float.
        """
        if isinstance(joints, dict):
            for name, value in joints.items():
                if isinstance(value, bool) or not isinstance(value, int | float):
                    raise ValueError(f"joint {name}: {value!r} is not a number")
        return joints

    @field_validator("joints")
    @classmethod
    def check_arm_joints(cls, joints: dict[str, float]) -> dict[str, float]:
        """Require every arm joint at a finite angle.

        Args:
            joints (dict[str, float]): Joint name -> angle (rad).

        Returns:
            dict[str, float]: `joints`, unchanged.

        Raises:
            ValueError: An arm joint is missing, or an entry (any joint, the gripper included) is NaN or infinite; the
                message lists every problem.
        """
        errors = [f"missing joint {name}" for name in ARM_JOINT_NAMES if name not in joints]
        errors += [
            f"joint {name}: {value!r} is not finite" for name, value in joints.items() if not math.isfinite(value)
        ]
        if errors:
            raise ValueError("; ".join(errors))
        return joints


def load_arm_home(path: Path) -> dict[str, float]:
    """Read the stored arm home.

    Args:
        path (Path): mcp_server's home file (YAML `{joints: {name: rad}}`).

    Returns:
        dict[str, float]: The five arm joints (ARM_JOINT_NAMES order) -> angle (rad); the gripper entry is dropped.

    Raises:
        ArmHomeError: The file is missing, unreadable or not YAML, has no `joints` mapping, lacks an arm joint, or holds a
            non-number or non-finite angle; the message names the file.
    """
    if not path.is_file():
        raise ArmHomeError(MESSAGE_NO_HOME.format(path=path))
    try:
        text = path.read_text()
    except OSError as exc:
        raise ArmHomeError(f"cannot read the arm home {path}: {exc}") from exc
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ArmHomeError(f"cannot parse {path}: {exc}") from exc
    if not isinstance(doc, dict):
        raise ArmHomeError(f"invalid arm home {path}: {NO_JOINTS}")
    try:
        joints = ArmHomeFile.model_validate(doc).joints
    except ValidationError as exc:
        details = "; ".join(
            NO_JOINTS if error["type"] in NO_JOINTS_ERRORS else str(error["msg"]).removeprefix("Value error, ")
            for error in exc.errors()
        )
        raise ArmHomeError(f"invalid arm home {path}: {details}") from exc
    return {name: joints[name] for name in ARM_JOINT_NAMES}
