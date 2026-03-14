"""Configuration for static TF publisher (frame offsets from base_link)."""

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/static_tf_publisher/config.yaml")
ENV_CONFIG_PATH_KEY = "STATIC_TF_PUBLISHER_CONFIG"


@dataclass
class FrameTransform:
    """Single static transform: parent -> child with translation and yaw.

    Attributes:
        parent_frame: Parent frame ID (e.g. base_link).
        child_frame: Child frame ID (e.g. imu_link).
        x: Translation x, meters.
        y: Translation y, meters.
        z: Translation z, meters (default 0).
        yaw: Rotation about z, radians (default 0).
    """

    parent_frame: str
    child_frame: str
    x: float
    y: float
    z: float
    yaw: float


def load_config(path: Path | None = None) -> list[FrameTransform] | None:
    """Load static TF config from YAML.

    Expected format: frames: [{ parent: base_link, child: imu_link, x: 0, y: 0, z: 0, yaw: 0 }, ...]
    Or shorthand: imu_link: [x, y, yaw], rplidar_frame: [x, y, yaw] with default parent base_link.

    Returns:
        list[FrameTransform] | None: Parsed transforms, or None if file missing/invalid.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if data is None or not isinstance(data, dict):
        return None

    out: list[FrameTransform] = []
    default_parent = (data.get("parent_frame") or "base_link").strip()

    # Explicit list
    frames = data.get("frames")
    if isinstance(frames, list):
        for item in frames:
            if not isinstance(item, dict):
                continue
            parent = (item.get("parent") or item.get("parent_frame") or default_parent).strip()
            child = (item.get("child") or item.get("child_frame") or "").strip()
            if not child:
                continue
            x = float(item.get("x", 0.0))
            y = float(item.get("y", 0.0))
            z = float(item.get("z", 0.0))
            yaw = float(item.get("yaw", 0.0))
            out.append(
                FrameTransform(
                    parent_frame=parent,
                    child_frame=child,
                    x=x,
                    y=y,
                    z=z,
                    yaw=yaw,
                )
            )

    # Shorthand: frame_id: [x, y, yaw]
    for key, value in data.items():
        if key in ("frames", "parent_frame"):
            continue
        if not isinstance(value, list) or len(value) < 3:
            continue
        try:
            x, y, yaw = float(value[0]), float(value[1]), float(value[2])
            z = float(value[3]) if len(value) > 3 else 0.0
        except (TypeError, ValueError, IndexError):
            continue
        child = str(key).strip()
        if child:
            out.append(
                FrameTransform(
                    parent_frame=default_parent,
                    child_frame=child,
                    x=x,
                    y=y,
                    z=z,
                    yaw=yaw,
                )
            )
    return out if out else None


def load_config_from_env() -> list[FrameTransform] | None:
    """Load config from path in STATIC_TF_PUBLISHER_CONFIG env, or default path."""
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
