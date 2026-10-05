"""Pure-Python helpers of the stereo_camera launch file (no ROS imports, unit-tested from the repo root).

Settings merge, camera selection by libcamera ID or index, calibration gating of the rectify / disparity stages and
the parameter dictionaries of camera_ros and stereo_image_proc.
"""

import math
from pathlib import Path
from typing import Any

import yaml

LAUNCH_KEYS = ("left_camera_id", "right_camera_id", "publish_points")
AE_EXPOSURE_MODES = {"normal": 0, "short": 1, "long": 2}
SYNC_MODES = {"off": 0, "server": 1, "client": 2}
SIDE_INDEX = {"left": 0, "right": 1}
DISPARITY_DOUBLE_KEYS = ("P1", "P2", "uniqueness_ratio", "approximate_sync_tolerance_seconds")
DISPARITY_INT_KEYS = (
    "stereo_algorithm",
    "sgbm_mode",
    "correlation_window_size",
    "disparity_range",
    "min_disparity",
    "speckle_size",
    "speckle_range",
    "disp12_max_diff",
    "prefilter_cap",
)
PROJECTION_SIZE = 12
FOCAL_INDEXES = (0, 5)


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Merge two nested dicts without modifying either.

    Args:
        base: Defaults.
        override: Values that win; nested dicts are merged key by key.

    Returns:
        dict[str, Any]: A new merged dict.
    """
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_settings(defaults_path: Path, override_path: Path) -> dict[str, Any]:
    """Repo defaults with the deployed override applied on top.

    Args:
        defaults_path: config/params.yaml.
        override_path: Deployed /etc/ros2/stereo_camera/config.yaml; skipped when missing or empty. The flat keys
            left_camera_id, right_camera_id and publish_points are moved into the `launch` section.

    Returns:
        dict[str, Any]: Merged settings.
    """
    settings = yaml.safe_load(defaults_path.read_text())
    if not override_path.is_file():
        return settings
    override = yaml.safe_load(override_path.read_text()) or {}
    flat = {key: override.pop(key) for key in LAUNCH_KEYS if key in override}
    return deep_merge(settings, deep_merge(override, {"launch": flat} if flat else {}))


def camera_selector(camera_id: str, index: int) -> tuple[str | int, str | None]:
    """Value of camera_ros' `camera` parameter for one camera.

    Args:
        camera_id: libcamera camera ID string (`cam -l`); empty when not known yet.
        index: Camera index used while the ID is unknown (0 left, 1 right).

    Returns:
        tuple[str | int, str | None]: The ID, or the index with a warning text.
    """
    camera_id = str(camera_id or "").strip()
    if camera_id:
        return camera_id, None
    warning = (
        f"No camera ID configured: selecting camera by index {index}. The index order is not stable across boots, "
        "read the libcamera IDs with `cam -l` and set left_camera_id / right_camera_id."
    )
    return index, warning


def projection_is_valid(projection: Any) -> bool:
    """True for a 3x4 projection matrix P that is finite and has non-zero focal lengths.

    Args:
        projection: The `projection_matrix.data` list of a camera_info YAML, or None.

    Returns:
        bool: Whether the matrix is a real calibration result.
    """
    if not isinstance(projection, list) or len(projection) != PROJECTION_SIZE:
        return False
    try:
        values = [float(v) for v in projection]
    except (TypeError, ValueError):
        return False
    return all(math.isfinite(v) for v in values) and all(values[i] != 0.0 for i in FOCAL_INDEXES)


def calibration_problem(path: Path) -> str | None:
    """Why one camera_info file cannot drive rectification, or None when it can.

    Args:
        path: Calibration YAML (as written by camera_calibration).

    Returns:
        str | None: Problem description, or None.
    """
    if not path.is_file():
        return f"{path.name} is missing"
    try:
        doc = yaml.safe_load(path.read_text())
    except yaml.YAMLError as err:
        return f"{path.name} is not valid YAML ({err.__class__.__name__})"
    projection = (doc.get("projection_matrix") or {}).get("data") if isinstance(doc, dict) else None
    if not projection_is_valid(projection):
        return f"{path.name} has no non-zero projection matrix P"
    return None


def calibration_ready(left: Path, right: Path) -> tuple[bool, list[str]]:
    """Whether both calibration files allow the rectify and disparity stages.

    Args:
        left: Left camera_info YAML.
        right: Right camera_info YAML.

    Returns:
        tuple[bool, list[str]]: Ready flag and one reason per unusable file.
    """
    reasons = [problem for problem in (calibration_problem(left), calibration_problem(right)) if problem]
    return not reasons, reasons


def camera_info_url(calibration_dir: Path, side: str) -> str:
    """camera_info_url of one side.

    Args:
        calibration_dir: Directory holding left.yaml / right.yaml.
        side: "left" or "right".

    Returns:
        str: file:// URL.
    """
    return f"file://{calibration_dir / (side + '.yaml')}"


def enum_value(table: dict[str, int], name: str, what: str) -> int:
    """Look up a libcamera enum value by its lower-case name.

    Args:
        table: Name to value mapping.
        name: Configured name.
        what: Setting name for the error message.

    Returns:
        int: The enum value.

    Raises:
        ValueError: When the name is not in the table.
    """
    try:
        return table[str(name).lower()]
    except KeyError:
        raise ValueError(f"{what}: unknown value {name!r}, expected one of {sorted(table)}") from None


def camera_parameters(settings: dict[str, Any], side: str, camera: str | int, info_url: str) -> dict[str, Any]:
    """Parameters of one camera_ros CameraNode.

    Args:
        settings: Merged settings.
        side: "left" or "right".
        camera: libcamera ID or index from camera_selector.
        info_url: camera_info_url of this side.

    Returns:
        dict[str, Any]: Parameter dict (node params plus libcamera control params).

    Raises:
        ValueError: On an unknown AeExposureMode or sync_mode name.
    """
    common = dict(settings["camera"])
    common["AeExposureMode"] = enum_value(AE_EXPOSURE_MODES, common["AeExposureMode"], "AeExposureMode")
    side_settings = settings[side]
    common["SyncMode"] = enum_value(SYNC_MODES, side_settings["sync_mode"], "sync_mode")
    common.update(camera=camera, frame_id=side_settings["frame_id"], camera_info_url=info_url)
    return common


def disparity_parameters(settings: dict[str, Any]) -> dict[str, Any]:
    """Parameters of stereo_image_proc DisparityNode with the types the node declares.

    Args:
        settings: Merged settings.

    Returns:
        dict[str, Any]: Parameter dict.
    """
    params: dict[str, Any] = dict(settings["disparity"])
    for key in DISPARITY_DOUBLE_KEYS:
        params[key] = float(params[key])
    for key in DISPARITY_INT_KEYS:
        params[key] = int(params[key])
    params["approximate_sync"] = bool(params["approximate_sync"])
    return params


def stage_plan(calibrated: bool, publish_points: bool) -> list[str]:
    """Pipeline stages to launch.

    Args:
        calibrated: Both calibration files are usable.
        publish_points: The point cloud flag.

    Returns:
        list[str]: Stage names in start order; the cloud needs the calibration (and the mount TF) as well.
    """
    stages = ["camera_left", "camera_right"]
    if calibrated:
        stages += ["rectify_left", "rectify_right", "disparity"]
        if publish_points:
            stages.append("points")
    return stages
