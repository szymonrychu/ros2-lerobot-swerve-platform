#!/usr/bin/env python3
"""Solve a camera mount pose from marker samples and print it as YAML.

Usage: solve_camera_mount.py SAMPLES_JSON [--camera NAME]

SAMPLES_JSON format::

    {
      "camera": "gripper",
      "intrinsics": {"width": 640, "height": 480, "fx": ..., "fy": ..., "cx": ..., "cy": ..., "distortion": []}
                    or "path/to/camera_calibration.yaml",
      "initial": {"parent_frame": "base_link", "x": 0, "y": 0, "z": 0.5, "roll": 0, "pitch": 0.5, "yaw": 0},
      "samples": [{"T_frame_parent": [[4x4]], "pixel": [u, v], "ground": [x, y, z]}]
    }
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

SHARED_DIR = Path(__file__).resolve().parent.parent / "shared"
sys.path.insert(0, str(SHARED_DIR))

from ros2_common.camera_geometry import CameraIntrinsics, MountPose, solve_mount_pose  # noqa: E402

PRECISION = 6


def load_problem(
    path: Path, camera: str | None
) -> tuple[str, CameraIntrinsics, MountPose, list[tuple[np.ndarray, tuple[float, float], tuple[float, float, float]]]]:
    """Parse a samples JSON file.

    Args:
        path: Path to the samples JSON.
        camera: Expected camera name, or None to accept the file's.

    Returns:
        tuple: (camera name, intrinsics, initial mount, samples for solve_mount_pose).

    Raises:
        ValueError: If ``camera`` differs from the camera named in the file.
    """
    data = json.loads(path.read_text())
    name = data["camera"]
    if camera is not None and camera != name:
        raise ValueError(f"file is for camera {name!r}, not {camera!r}")
    raw_intr = data["intrinsics"]
    if isinstance(raw_intr, str):
        intr = CameraIntrinsics.from_calibration_yaml((path.parent / raw_intr).resolve())
    else:
        intr = CameraIntrinsics(**raw_intr)
    initial = MountPose(**data["initial"])
    samples = [
        (np.array(s["T_frame_parent"], dtype=float), (s["pixel"][0], s["pixel"][1]), tuple(s["ground"]))
        for s in data["samples"]
    ]
    return name, intr, initial, samples


def format_result(name: str, mount: MountPose, rms_px: float) -> str:
    """Render the solved mount as YAML ready to paste into ansible config.

    Args:
        name: Camera name.
        mount: Solved mount pose.
        rms_px: RMS reprojection error in pixels.

    Returns:
        str: YAML text with a trailing comment carrying the RMS.
    """
    body = {k: round(v, PRECISION) if isinstance(v, float) else v for k, v in mount.model_dump().items()}
    return yaml.safe_dump({name: {"mount": body}}, sort_keys=False) + f"# rms_px: {rms_px:.3f}\n"


def main() -> None:
    """Entry point: load samples, solve, print YAML."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("samples", type=Path, help="samples JSON file")
    parser.add_argument("--camera", default=None, help="camera name to check against the file")
    args = parser.parse_args()
    name, intr, initial, samples = load_problem(args.samples, args.camera)
    mount, rms = solve_mount_pose(samples, intr, initial)
    sys.stdout.write(format_result(name, mount, rms))


if __name__ == "__main__":
    main()
