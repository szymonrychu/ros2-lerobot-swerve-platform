"""Build the head robot_description, applying the calibrated head_chain.json and tof_joint.json overrides.

Standard library only: the launch runs on the system Python. Loaded by path from the launch file.
"""

import importlib.util
import logging
from pathlib import Path

LOGGER = logging.getLogger("head_description")
URDF_TEXT_MODULE = "head_rgbd_urdf_text"


def robot_description(urdf_path: Path, applied_dir: Path, urdf_text_path: Path) -> str:
    """Read the head URDF and apply the stored head_chain.json and tof_joint.json overrides.

    Args:
        urdf_path: Repo head URDF.
        applied_dir: Directory of applied overrides; either file may be absent.
        urdf_text_path: Path of urdf_text.py (apply_overrides), loaded by file path.

    Returns:
        The URDF text; unchanged when urdf_text cannot be loaded. Each unusable override file is skipped with a warning.
    """
    urdf = urdf_path.read_text()
    try:
        spec = importlib.util.spec_from_file_location(URDF_TEXT_MODULE, urdf_text_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except (OSError, ImportError, AttributeError, SyntaxError) as error:
        LOGGER.warning("ignoring head overrides, cannot load %s: %s", urdf_text_path, error)
        return urdf
    text, warnings = module.apply_overrides(urdf, applied_dir)
    for warning in warnings:
        LOGGER.warning("ignoring head override %s", warning)
    return text
