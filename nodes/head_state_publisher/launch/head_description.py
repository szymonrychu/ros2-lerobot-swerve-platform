"""Build the head robot_description, applying the calibrated head_chain.json override.

Standard library only: the launch runs on the system Python. Loaded by path from the launch file.
"""

import importlib.util
import logging
from pathlib import Path

LOGGER = logging.getLogger("head_description")
URDF_TEXT_MODULE = "head_rgbd_urdf_text"


def robot_description(urdf_path: Path, applied_dir: Path, urdf_text_path: Path) -> str:
    """Read the head URDF and apply the stored head_chain.json override.

    Args:
        urdf_path: Repo head URDF.
        applied_dir: Directory of applied overrides; head_chain.json may be absent.
        urdf_text_path: Path of urdf_text.py (apply_overrides), loaded by file path.

    Returns:
        The URDF text; unchanged when urdf_text cannot be loaded. An unusable override file is skipped with a warning.
    """
    # head_chain.json overrides head_mount_joint, head_pan, head_tilt and d435_joint (the D435 mount, design D4); a
    # tof_joint.json left there by the retired head ToF is ignored.
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
