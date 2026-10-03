"""Arm home pose stored as YAML ({joints: {name: rad}}), written atomically."""

import math
import os
import tempfile
from pathlib import Path

import yaml


class HomeStoreError(RuntimeError):
    """Raised for an unreadable home file or an invalid pose."""


def validate_pose(joints: object) -> dict[str, float]:
    """Check a pose mapping of joint name -> finite float.

    Args:
        joints (object): Candidate pose.

    Returns:
        dict[str, float]: The validated pose.
    """
    if not isinstance(joints, dict) or not joints:
        raise HomeStoreError("home pose must be a non-empty mapping of joint name -> radians")
    out: dict[str, float] = {}
    for name, value in joints.items():
        if not isinstance(name, str) or isinstance(value, bool) or not isinstance(value, int | float):
            raise HomeStoreError(f"invalid home pose entry {name!r}: {value!r}")
        if not math.isfinite(value):
            raise HomeStoreError(f"home pose value for {name} is not finite")
        out[name] = float(value)
    return out


def load_home(path: Path) -> dict[str, float] | None:
    """Load the stored home pose.

    Args:
        path (Path): Home YAML file.

    Returns:
        dict[str, float] | None: Joint name -> rad, or None when no pose was stored yet.
    """
    if not path.is_file():
        return None
    try:
        doc = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as exc:
        raise HomeStoreError(f"cannot parse {path}: {exc}") from exc
    if not isinstance(doc, dict):
        raise HomeStoreError(f"{path} must contain a mapping")
    return validate_pose(doc.get("joints"))


def save_home(path: Path, joints: dict[str, float]) -> None:
    """Store the home pose atomically (temp file + rename), creating parent directories.

    Args:
        path (Path): Home YAML file.
        joints (dict[str, float]): Joint name -> rad.
    """
    pose = validate_pose(joints)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".home-", suffix=".yaml")
    try:
        with os.fdopen(fd, "w") as fh:
            yaml.safe_dump({"joints": pose}, fh, sort_keys=False)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
