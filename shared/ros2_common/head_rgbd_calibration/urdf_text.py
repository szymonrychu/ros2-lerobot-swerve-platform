"""Rewrite joint origins of a head URDF and apply the calibrated head chain overrides.

Standard library only: the head_state_publisher launch loads this file by path.
"""

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

TOF_JOINT = "tof_joint"
NOTE_MARKER = f"{TOF_JOINT} calibrated"
NUMBER_FORMAT = "{:.9f}"
HEAD_CHAIN_FILE = "head_chain.json"
TOF_JOINT_FILE = "tof_joint.json"
APPLIED_MARKER_FILE = "applied.json"
LEGACY_CHAIN_JOINT = "head_camera_joint"
HEAD_CHAIN_JOINTS = ("head_mount_joint", "head_pan", "head_tilt", "d435_joint")


@dataclass(frozen=True)
class Origin:
    """A URDF joint origin: xyz in metres, rpy in radians (fixed-axis)."""

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def format_vec(values: tuple[float, float, float]) -> str:
    """Space separated numbers for an origin attribute."""
    return " ".join(NUMBER_FORMAT.format(v + 0.0) for v in values)


def rewrite_joint_origin(urdf_text: str, joint: str, origin: Origin, note: str) -> str:
    """Replace a joint's origin xyz/rpy and the calibration comment above it; all other bytes stay.

    Args:
        urdf_text: URDF XML text.
        joint: Joint name; the comment written is ``<!-- <joint> calibrated: note -->``.
        origin: New joint origin.
        note: Calibration note (date, fit result), written as a comment above the joint.

    Returns:
        The new URDF text.

    Raises:
        ValueError: The joint or its origin is missing.
    """
    marker = NOTE_MARKER if joint == TOF_JOINT else f"{joint} calibrated"
    joint_re = re.compile(
        rf"(?P<indent>[ \t]*)(?P<comment><!--\s*{re.escape(marker)}[^>]*?-->[ \t]*\n[ \t]*)?"
        rf'(?P<joint><joint\s+name="{re.escape(joint)}".*?</joint>)',
        re.DOTALL,
    )
    match = joint_re.search(urdf_text)
    if match is None:
        raise ValueError(f"URDF has no joint {joint}")
    body = match.group("joint")
    origin_match = re.search(r"<origin\b[^>]*?/?>", body)
    if origin_match is None:
        raise ValueError(f"{joint} has no <origin>")
    tag = origin_match.group(0)
    for attr, values in (("xyz", origin.xyz), ("rpy", origin.rpy)):
        attr_re = re.compile(rf'{attr}="[^"]*"')
        value = f'{attr}="{format_vec(values)}"'
        tag = attr_re.sub(value, tag, count=1) if attr_re.search(tag) else tag.replace("<origin", f"<origin {value}", 1)
    body = body[: origin_match.start()] + tag + body[origin_match.end() :]
    indent = match.group("indent")
    comment = f"{indent}<!-- {marker}: {note} -->\n"
    return urdf_text[: match.start()] + comment + indent + body + urdf_text[match.end() :]


def rewrite_tof_joint(urdf_text: str, origin: Origin, note: str) -> str:
    """Replace the tof_joint origin and its calibration comment (see rewrite_joint_origin)."""
    return rewrite_joint_origin(urdf_text, TOF_JOINT, origin, note)


def parse_vec3(value: object, where: str) -> tuple[float, float, float]:
    """Validate a list of three finite numbers (bool refused).

    Args:
        value: Parsed JSON value.
        where: Key path used in the error message.

    Returns:
        The three numbers as floats.

    Raises:
        ValueError: Not a 3-list of finite non-bool numbers.
    """
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{where} must be a list of 3 numbers")
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise ValueError(f"{where} must hold finite numbers")
    return (float(value[0]), float(value[1]), float(value[2]))


def parse_head_chain(data: object) -> dict[str, Origin]:
    """Parse the head_chain.json content.

    Args:
        data: Parsed JSON, ``{"joints": {name: {"xyz": [3], "rpy": [3]}}, "note": str, ...}``.

    Returns:
        Origin per joint name, only names in HEAD_CHAIN_JOINTS.

    Raises:
        ValueError: Wrong shape, unknown joint, no joint, or bad numbers; the message names the key.
    """
    if not isinstance(data, dict):
        raise ValueError("top level must be an object")
    joints = data.get("joints")
    if not isinstance(joints, dict) or not joints:
        raise ValueError("joints must be a non-empty object")
    origins: dict[str, Origin] = {}
    for name, entry in joints.items():
        if name == LEGACY_CHAIN_JOINT:
            raise ValueError(
                f"joints.{name}: this fit predates the D435 (the joint is gone); re-run head-from-arm calibration"
            )
        if name not in HEAD_CHAIN_JOINTS:
            raise ValueError(f"joints.{name}: unknown joint")
        if not isinstance(entry, dict):
            raise ValueError(f"joints.{name} must be an object")
        xyz = parse_vec3(entry.get("xyz"), f"joints.{name}.xyz")
        rpy = parse_vec3(entry.get("rpy"), f"joints.{name}.rpy")
        origins[name] = Origin(xyz, rpy)
    return origins


@dataclass(frozen=True)
class ChainResidual:
    """How well the chain agrees with the trusted arm, as measured by the fit that wrote it.

    Attributes:
        kind: The fit that measured it ("head_arm": board on the gripper moved by the arm, seen by the D435).
        holdout_board_mm: RMS over the held-out views of the board centre distance between arm forward kinematics and
            the camera through the fitted chain (mm): the camera-to-arm frame accuracy.
        holdout_rms_px: Corner reprojection RMS over the held-out views (px).
        rms_px: Corner reprojection RMS over the fit views (px).
    """

    kind: str
    holdout_board_mm: float
    holdout_rms_px: float
    rms_px: float


def parse_residual(data: object) -> ChainResidual | None:
    """Parse the ``residual`` entry of a head_chain.json document.

    Args:
        data: Parsed head_chain.json.

    Returns:
        The residual, or None when the entry is absent or unusable (not an object, a missing, non-numeric, bool,
        non-finite or negative value, or an empty kind).
    """
    entry = data.get("residual") if isinstance(data, dict) else None
    if not isinstance(entry, dict):
        return None
    kind = entry.get("kind")
    values = [entry.get(key) for key in ("holdout_board_mm", "holdout_rms_px", "rms_px")]
    if not isinstance(kind, str) or not kind:
        return None
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            return None
    return ChainResidual(kind, float(values[0]), float(values[1]), float(values[2]))


def head_chain_json(
    origins: dict[str, Origin], note: str, session: str, result: str, residual: ChainResidual | None = None
) -> str:
    """Serialise head chain origins as head_chain.json text (keys sorted, 9 decimals).

    Args:
        origins: Origin per joint name.
        note: Calibration note.
        session: Calibration session id.
        result: Short result summary.
        residual: The arm-frame residual of the fit (written as ``residual``); omitted when None.

    Returns:
        JSON text ending in a newline.
    """

    def vec(values: tuple[float, float, float]) -> str:
        return "[" + ", ".join(NUMBER_FORMAT.format(v + 0.0) for v in values) + "]"

    entries = ",\n".join(
        f'    {json.dumps(name)}: {{"rpy": {vec(o.rpy)}, "xyz": {vec(o.xyz)}}}' for name, o in sorted(origins.items())
    )
    residual_text = ""
    if residual is not None:
        body = {
            "holdout_board_mm": residual.holdout_board_mm,
            "holdout_rms_px": residual.holdout_rms_px,
            "kind": residual.kind,
            "rms_px": residual.rms_px,
        }
        residual_text = f'  "residual": {json.dumps(body)},\n'
    return (
        f'{{\n  "joints": {{\n{entries}\n  }},\n  "note": {json.dumps(note)},\n{residual_text}'
        f'  "result": {json.dumps(result)},\n  "session": {json.dumps(session)}\n}}\n'
    )


def apply_overrides(urdf_text: str, applied_dir: Path) -> tuple[str, list[str]]:
    """Apply head_chain.json (each joint) and then tof_joint.json to a URDF.

    A missing file is skipped silently; an unreadable or invalid file is skipped whole with one warning.

    Args:
        urdf_text: Repo URDF text.
        applied_dir: Directory holding the override files.

    Returns:
        The new URDF text and the warning strings.
    """
    warnings: list[str] = []
    for filename in (HEAD_CHAIN_FILE, TOF_JOINT_FILE):
        path = Path(applied_dir) / filename
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
            if not isinstance(data, dict):
                raise ValueError("top level must be an object")
            note = data.get("note", "")
            if not isinstance(note, str):
                raise ValueError("note must be a string")
            if filename == HEAD_CHAIN_FILE:
                origins = parse_head_chain(data)
            else:
                origins = {TOF_JOINT: Origin(parse_vec3(data.get("xyz"), "xyz"), parse_vec3(data.get("rpy"), "rpy"))}
            text = urdf_text
            for name, origin in origins.items():
                text = rewrite_joint_origin(text, name, origin, note)
        except (OSError, ValueError) as error:
            warnings.append(f"{path}: {error}")
            continue
        urdf_text = text
    return urdf_text, warnings
