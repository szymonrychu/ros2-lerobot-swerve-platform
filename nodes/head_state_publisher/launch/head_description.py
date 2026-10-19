"""Build the head robot_description, applying the calibrated tof_joint override when present.

Standard library only: the launch runs on the system Python. Loaded by path from the launch file.
"""

import importlib.util
import json
import logging
from pathlib import Path

LOGGER = logging.getLogger("head_description")
URDF_TEXT_MODULE = "head_rgbd_urdf_text"
VECTOR_LENGTH = 3


def is_vector(value: object) -> bool:
    """True for a list of three real numbers (bool excluded)."""
    return (
        isinstance(value, list)
        and len(value) == VECTOR_LENGTH
        and all(isinstance(v, int | float) and not isinstance(v, bool) for v in value)
    )


def robot_description(urdf_path: Path, override_path: Path, urdf_text_path: Path) -> str:
    """Read the head URDF and apply the tof_joint override when one is stored.

    Args:
        urdf_path: Repo head URDF.
        override_path: JSON {xyz: [3 numbers], rpy: [3 numbers], note: str}; may be absent.
        urdf_text_path: Path of urdf_text.py (rewrite_tof_joint, Origin), loaded by file path.

    Returns:
        The URDF text; unchanged when the override is missing or unusable (a warning names the file and reason).
    """
    urdf = urdf_path.read_text()
    if not override_path.exists():
        return urdf
    try:
        data = json.loads(override_path.read_text())
        if not isinstance(data, dict):
            raise ValueError("expected a JSON object")
        if not is_vector(data.get("xyz")) or not is_vector(data.get("rpy")) or not isinstance(data.get("note"), str):
            raise ValueError("expected xyz and rpy as 3 numbers each and note as text")
        spec = importlib.util.spec_from_file_location(URDF_TEXT_MODULE, urdf_text_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        origin = module.Origin(tuple(float(v) for v in data["xyz"]), tuple(float(v) for v in data["rpy"]))
        return module.rewrite_tof_joint(urdf, origin, data["note"])
    except (OSError, ValueError, ImportError, AttributeError) as error:
        LOGGER.warning("ignoring tof_joint override %s: %s", override_path, error)
        return urdf
