"""Calibration samples stored on disk (one JSON file per camera) and the mount solver wiring."""

import json
import math
import os
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from ros2_common.camera_geometry import MIN_SOLVER_SAMPLES, CameraIntrinsics, MountPose, solve_mount_pose

Sample = tuple[np.ndarray, tuple[float, float], tuple[float, float, float]]
YAML_DECIMALS = 6
MOUNT_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


class CalibrationError(ValueError):
    """Raised for unusable calibration samples, files or solver input."""


class SampleStore:
    """Calibration samples of each camera in ``<directory>/<camera>.json`` (the directory is created by Ansible)."""

    def __init__(self, directory: Path) -> None:
        """Remember the sample directory.

        Args:
            directory (Path): Calibration directory (e.g. /var/lib/ros2/camera_calibration).
        """
        self.directory = directory

    def path(self, camera: str) -> Path:
        """Sample file of a camera.

        Args:
            camera (str): Camera name.

        Returns:
            Path: JSON file path.
        """
        return self.directory / f"{camera}.json"

    def read(self, camera: str) -> dict[str, Any]:
        """Parsed sample file, or an empty record when none exists.

        Args:
            camera (str): Camera name.

        Returns:
            dict[str, Any]: {camera, parent_frame, samples}.

        Raises:
            CalibrationError: When the file is unreadable or corrupt.
        """
        path = self.path(camera)
        if not path.exists():
            return {"camera": camera, "parent_frame": None, "samples": []}
        try:
            data = json.loads(path.read_text())
            if not isinstance(data, dict) or not isinstance(data.get("samples"), list):
                raise ValueError("missing samples list")
            return data
        except (OSError, ValueError) as exc:
            raise CalibrationError(f"calibration file {path} is corrupt or unreadable: {exc}") from exc

    def write(self, camera: str, data: dict[str, Any]) -> None:
        """Atomically write the sample file.

        Args:
            camera (str): Camera name.
            data (dict[str, Any]): Record to store.

        Raises:
            CalibrationError: When the directory or file cannot be written.
        """
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=self.directory, prefix=f".{camera}.", suffix=".tmp")
            with os.fdopen(fd, "w") as handle:
                json.dump(data, handle, indent=1)
            os.replace(tmp, self.path(camera))
        except OSError as exc:
            raise CalibrationError(f"cannot write calibration samples to {self.path(camera)}: {exc}") from exc

    def add(
        self,
        camera: str,
        parent_frame: str,
        t_frame_parent: np.ndarray,
        pixel: tuple[float, float],
        ground: tuple[float, float, float],
    ) -> int:
        """Append one sample.

        Args:
            camera (str): Camera name.
            parent_frame (str): Mount parent frame (URDF link or base_link) the pose refers to.
            t_frame_parent (np.ndarray): 4x4 pose of the parent link in the ground frame at capture time.
            pixel (tuple[float, float]): Marker pixel (u, v).
            ground (tuple[float, float, float]): Marker position (x, y, z) in the camera's reference frame.

        Returns:
            int: Number of samples now stored.

        Raises:
            CalibrationError: For non-finite values, a changed parent frame or an unwritable store.
        """
        values = [*pixel, *ground, *np.asarray(t_frame_parent, dtype=float).ravel()]
        if not all(math.isfinite(v) for v in values):
            raise CalibrationError("sample values must be finite numbers")
        data = self.read(camera)
        stored_parent = data.get("parent_frame")
        if stored_parent not in (None, parent_frame) and data["samples"]:
            raise CalibrationError(
                f"stored samples use parent frame {stored_parent!r}, not {parent_frame!r}; "
                "clear_calibration_samples first"
            )
        samples = list(data["samples"])
        samples.append(
            {
                "t_frame_parent": np.asarray(t_frame_parent, dtype=float).tolist(),
                "pixel": [float(pixel[0]), float(pixel[1])],
                "ground": [float(g) for g in ground],
                "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            }
        )
        self.write(camera, {"camera": camera, "parent_frame": parent_frame, "samples": samples})
        return len(samples)

    def load(self, camera: str) -> tuple[str | None, list[Sample]]:
        """Stored samples in solver form.

        Args:
            camera (str): Camera name.

        Returns:
            tuple[str | None, list[Sample]]: (parent frame or None, [(T_frame_parent, (u, v), (x, y, z))]).
        """
        data = self.read(camera)
        try:
            samples: list[Sample] = [
                (np.array(s["t_frame_parent"], dtype=float), (s["pixel"][0], s["pixel"][1]), tuple(s["ground"]))
                for s in data["samples"]
            ]
        except (KeyError, TypeError, IndexError) as exc:
            raise CalibrationError(f"calibration file {self.path(camera)} is corrupt: {exc}") from exc
        return data.get("parent_frame"), samples

    def count(self, camera: str) -> int:
        """Number of stored samples.

        Args:
            camera (str): Camera name.

        Returns:
            int: Sample count.
        """
        return len(self.read(camera)["samples"])

    def clear(self, camera: str) -> int:
        """Delete the stored samples.

        Args:
            camera (str): Camera name.

        Returns:
            int: Number of samples removed.

        Raises:
            CalibrationError: When the file cannot be deleted.
        """
        removed = self.count(camera)
        try:
            self.path(camera).unlink(missing_ok=True)
        except OSError as exc:
            raise CalibrationError(f"cannot delete {self.path(camera)}: {exc}") from exc
        return removed


def solve_samples(samples: list[Sample], intr: CameraIntrinsics, initial: MountPose) -> tuple[MountPose, float]:
    """Fit the camera mount to the stored samples.

    Args:
        samples (list[Sample]): Stored samples.
        intr (CameraIntrinsics): Configured intrinsics.
        initial (MountPose): Starting mount (its parent_frame is kept).

    Returns:
        tuple[MountPose, float]: (solved mount, RMS reprojection error in pixels).

    Raises:
        CalibrationError: With fewer than MIN_SOLVER_SAMPLES samples.
    """
    if len(samples) < MIN_SOLVER_SAMPLES:
        raise CalibrationError(f"need at least {MIN_SOLVER_SAMPLES} samples, have {len(samples)}")
    return solve_mount_pose(samples, intr, initial)


def mount_yaml(camera: str, mount: MountPose) -> str:
    """YAML snippet to paste under the mcp_server config of ansible/group_vars/client.yml.

    Args:
        camera (str): Camera name.
        mount (MountPose): Solved mount.

    Returns:
        str: YAML with a top-level ``cameras:`` key, floats rounded to 1e-6.
    """
    body: dict[str, Any] = {"parent_frame": mount.parent_frame}
    body.update({k: round(float(getattr(mount, k)), YAML_DECIMALS) for k in MOUNT_FIELDS})
    return yaml.safe_dump({"cameras": {camera: {"mount": body}}}, sort_keys=False)
