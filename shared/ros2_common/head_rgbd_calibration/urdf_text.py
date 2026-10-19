"""Rewrite the tof_joint origin of a head URDF.

Standard library only: the head_state_publisher launch loads this file by path.
"""

import re
from dataclasses import dataclass

TOF_JOINT = "tof_joint"
NOTE_MARKER = "tof_joint calibrated"
NUMBER_FORMAT = "{:.9f}"


@dataclass(frozen=True)
class Origin:
    """A URDF joint origin: xyz in metres, rpy in radians (fixed-axis)."""

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def format_vec(values: tuple[float, float, float]) -> str:
    """Space separated numbers for an origin attribute."""
    return " ".join(NUMBER_FORMAT.format(v + 0.0) for v in values)


def rewrite_tof_joint(urdf_text: str, origin: Origin, note: str) -> str:
    """Replace the tof_joint origin xyz/rpy and the calibration comment above it; all other bytes stay.

    Args:
        urdf_text: URDF XML text.
        origin: New tof_joint origin.
        note: Calibration note (date and stereo RMS), written as a comment above the joint.

    Returns:
        The new URDF text.

    Raises:
        ValueError: tof_joint or its origin is missing.
    """
    joint_re = re.compile(
        rf"(?P<indent>[ \t]*)(?P<comment><!--\s*{re.escape(NOTE_MARKER)}[^>]*?-->[ \t]*\n[ \t]*)?"
        rf'(?P<joint><joint\s+name="{TOF_JOINT}".*?</joint>)',
        re.DOTALL,
    )
    match = joint_re.search(urdf_text)
    if match is None:
        raise ValueError(f"URDF has no joint {TOF_JOINT}")
    joint = match.group("joint")
    origin_re = re.compile(r"<origin\b[^>]*?/?>")
    origin_match = origin_re.search(joint)
    if origin_match is None:
        raise ValueError(f"{TOF_JOINT} has no <origin>")
    tag = origin_match.group(0)
    for attr, values in (("xyz", origin.xyz), ("rpy", origin.rpy)):
        attr_re = re.compile(rf'{attr}="[^"]*"')
        value = f'{attr}="{format_vec(values)}"'
        tag = attr_re.sub(value, tag, count=1) if attr_re.search(tag) else tag.replace("<origin", f"<origin {value}", 1)
    joint = joint[: origin_match.start()] + tag + joint[origin_match.end() :]
    indent = match.group("indent")
    comment = f"{indent}<!-- {NOTE_MARKER}: {note} -->\n"
    return urdf_text[: match.start()] + comment + indent + joint + urdf_text[match.end() :]
