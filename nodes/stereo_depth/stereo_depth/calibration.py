"""Stereo calibration files: load, validate and write them, and build the rectification maps.

This is the stereo-calibration-files contract, shared by the depth node (reader) and the calibration tool (writer).
Three files live in one directory (default /var/lib/ros2/camera_calibration, host state that survives redeploys):

- stereo_left.yaml, stereo_right.yaml: ROS camera_info YAML, the format camera_calibration's cameracalibrator and
  camera_info_manager use (image_width, image_height, camera_name, camera_matrix, distortion_model,
  distortion_coefficients, rectification_matrix, projection_matrix; each matrix as {rows, cols, data}). P is the
  rectified projection; the right P[3] is -fx * B with B the baseline in metres.
- stereo.yaml: the stereo extrinsics, image_width, image_height, R (3x3), T (3x1, metres), Q (4x4) and rms (px).

The camera_info files are what camera_ros reads through camera_info_url, so the launch gate (P with 12 elements and
non-zero focal terms) accepts every file this module writes.
"""

import math
import os
import tempfile
from pathlib import Path
from typing import Any, Literal, NamedTuple

import cv2
import numpy as np
import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator, model_validator

LEFT_FILE = "stereo_left.yaml"
RIGHT_FILE = "stereo_right.yaml"
STEREO_FILE = "stereo.yaml"
CALIBRATION_FILES = (LEFT_FILE, RIGHT_FILE, STEREO_FILE)
LEFT_CAMERA_NAME = "stereo_left"
RIGHT_CAMERA_NAME = "stereo_right"
DISTORTION_LENGTHS = {"plumb_bob": 5, "rational_polynomial": 8}
RECTIFIED_REL_TOL = 1e-6
LEFT_TX_ABS_TOL = 1e-6
BASELINE_REL_TOL = 0.02
FILE_MODE = 0o644


class CalibrationError(Exception):
    """Raised when a calibration file is missing, unreadable or invalid."""


def check_finite(values: list[float], name: str) -> list[float]:
    """Reject NaN and infinite values.

    Args:
        values (list[float]): Matrix data.
        name (str): Matrix name for the message.

    Returns:
        list[float]: The values.
    """
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} values must be finite")
    return values


class CameraCalibration(BaseModel):
    """One camera of the pair, as in a ROS camera_info YAML.

    Attributes:
        camera_name (str): Camera name.
        width (int): Image width in pixels (YAML image_width).
        height (int): Image height in pixels (YAML image_height).
        distortion_model (str): plumb_bob (5 coefficients) or rational_polynomial (8).
        d (list[float]): Distortion coefficients.
        k (list[float]): 3x3 intrinsic matrix, row-major.
        r (list[float]): 3x3 rectification rotation, row-major.
        p (list[float]): 3x4 rectified projection, row-major.
    """

    model_config = ConfigDict(frozen=True, validate_by_name=True, validate_by_alias=True)

    camera_name: str = ""
    width: int = Field(gt=0, alias="image_width")
    height: int = Field(gt=0, alias="image_height")
    distortion_model: Literal["plumb_bob", "rational_polynomial"] = "plumb_bob"
    d: list[float]
    k: list[float] = Field(min_length=9, max_length=9)
    r: list[float] = Field(min_length=9, max_length=9)
    p: list[float] = Field(min_length=12, max_length=12)

    @field_validator("d", "k", "r", "p")
    @classmethod
    def check_values(_cls, values: list[float], info: Any) -> list[float]:
        """Reject non-finite matrix values.

        Args:
            values (list[float]): Matrix data.
            info (Any): pydantic ValidationInfo (field name).

        Returns:
            list[float]: The values.
        """
        return check_finite(values, info.field_name.upper())

    @model_validator(mode="after")
    def check_camera(self) -> "CameraCalibration":
        """Require positive focal lengths in K and P and the coefficient count of the distortion model.

        Returns:
            CameraCalibration: The validated camera.
        """
        if self.k[0] <= 0.0 or self.k[4] <= 0.0:
            raise ValueError("focal lengths in K must be positive")
        if self.p[0] <= 0.0 or self.p[5] <= 0.0:
            raise ValueError("focal lengths in P must be positive")
        expected = DISTORTION_LENGTHS[self.distortion_model]
        if len(self.d) != expected:
            raise ValueError(f"distortion_model {self.distortion_model} needs {expected} coefficients")
        return self


class StereoExtrinsics(BaseModel):
    """The pair's stereo.yaml.

    Attributes:
        width (int): Image width in pixels (YAML image_width).
        height (int): Image height in pixels (YAML image_height).
        r (list[float]): 3x3 rotation of the right camera relative to the left, row-major.
        t (list[float]): Translation of the right camera relative to the left, metres.
        q (list[float]): 4x4 disparity-to-depth matrix from stereoRectify, row-major.
        rms (float): Stereo reprojection error, pixels.
    """

    model_config = ConfigDict(frozen=True, validate_by_name=True, validate_by_alias=True)

    width: int = Field(gt=0, alias="image_width")
    height: int = Field(gt=0, alias="image_height")
    r: list[float] = Field(min_length=9, max_length=9)
    t: list[float] = Field(min_length=3, max_length=3)
    q: list[float] = Field(min_length=16, max_length=16)
    rms: float = Field(ge=0.0, allow_inf_nan=False)

    @field_validator("r", "t", "q")
    @classmethod
    def check_values(_cls, values: list[float], info: Any) -> list[float]:
        """Reject non-finite matrix values.

        Args:
            values (list[float]): Matrix data.
            info (Any): pydantic ValidationInfo (field name).

        Returns:
            list[float]: The values.
        """
        return check_finite(values, info.field_name.upper())


class StereoCalibration(BaseModel):
    """The validated pair: both cameras and the extrinsics, consistent with each other.

    Attributes:
        left (CameraCalibration): Left camera (the rectified reference, P[3] = 0).
        right (CameraCalibration): Right camera (P[3] = -fx * B).
        extrinsics (StereoExtrinsics): R, T, Q and rms.
    """

    model_config = ConfigDict(frozen=True)

    left: CameraCalibration
    right: CameraCalibration
    extrinsics: StereoExtrinsics

    @model_validator(mode="after")
    def check_pair(self) -> "StereoCalibration":
        """Require one image size, a rectified pair sharing fx, fy, cx and cy, and a baseline that matches |T|.

        Returns:
            StereoCalibration: The validated pair.
        """
        sizes = {
            "left": (self.left.width, self.left.height),
            "right": (self.right.width, self.right.height),
            "stereo": (self.extrinsics.width, self.extrinsics.height),
        }
        if len(set(sizes.values())) != 1:
            described = ", ".join(f"{name} {w}x{h}" for name, (w, h) in sizes.items())
            raise ValueError(f"image size mismatch: {described}")
        for index in (0, 2, 5, 6):
            if not math.isclose(self.left.p[index], self.right.p[index], rel_tol=RECTIFIED_REL_TOL):
                raise ValueError(f"rectified left and right P must share fx, fy, cx and cy (P[{index}] differs)")
        if abs(self.left.p[3]) > LEFT_TX_ABS_TOL:
            raise ValueError("left P[3] must be 0: the left camera is the rectified reference")
        if self.right.p[3] >= 0.0:
            raise ValueError("right P[3] must be -fx * B < 0 (a positive baseline, right camera on the right)")
        t_norm = math.dist(self.extrinsics.t, (0.0, 0.0, 0.0))
        if not math.isclose(self.baseline_m, t_norm, rel_tol=BASELINE_REL_TOL):
            raise ValueError(f"baseline from right P ({self.baseline_m:.4f} m) does not match |T| ({t_norm:.4f} m)")
        return self

    @property
    def fx(self) -> float:
        """Rectified focal length, pixels.

        Returns:
            float: P[0] of the left camera.
        """
        return self.left.p[0]

    @property
    def baseline_m(self) -> float:
        """Baseline from the rectified right projection, metres.

        Returns:
            float: -P_right[3] / P_right[0].
        """
        return -self.right.p[3] / self.right.p[0]

    @property
    def width(self) -> int:
        """Image width of the pair, pixels.

        Returns:
            int: Width.
        """
        return self.left.width

    @property
    def height(self) -> int:
        """Image height of the pair, pixels.

        Returns:
            int: Height.
        """
        return self.left.height


class CalibrationLoad(NamedTuple):
    """Result of load_stereo_calibration.

    Attributes:
        calibration (StereoCalibration | None): The pair, or None when missing or invalid.
        reason (str): Why calibration is None; empty when it loaded.
    """

    calibration: StereoCalibration | None
    reason: str


class RectificationMaps(NamedTuple):
    """Fixed-point (CV_16SC2) remap tables of both cameras.

    Attributes:
        left (tuple[np.ndarray, np.ndarray]): (map1 int16 HxWx2, map2 uint16 HxW) of the left camera.
        right (tuple[np.ndarray, np.ndarray]): The same for the right camera.
    """

    left: tuple[np.ndarray, np.ndarray]
    right: tuple[np.ndarray, np.ndarray]


def validation_reason(error: ValidationError) -> str:
    """Condense a pydantic error to one line.

    Args:
        error (ValidationError): The error.

    Returns:
        str: "loc: message" entries joined by "; ".
    """
    parts = []
    for item in error.errors():
        loc = ".".join(str(part) for part in item["loc"])
        parts.append(f"{loc}: {item['msg']}" if loc else str(item["msg"]))
    return "; ".join(parts)


def matrix_yaml(values: list[float], rows: int, cols: int) -> dict[str, Any]:
    """Build a {rows, cols, data} matrix entry.

    Args:
        values (list[float]): Row-major data.
        rows (int): Rows.
        cols (int): Columns.

    Returns:
        dict[str, Any]: The YAML mapping.
    """
    return {"rows": rows, "cols": cols, "data": [float(v) for v in values]}


def matrix_data(data: dict[str, Any], key: str, rows: int | None, cols: int | None) -> list[float]:
    """Read a {rows, cols, data} matrix entry and check its shape.

    Args:
        data (dict[str, Any]): Parsed YAML mapping.
        key (str): Matrix key.
        rows (int | None): Required rows, or None for any.
        cols (int | None): Required columns, or None for any.

    Returns:
        list[float]: Row-major data.

    Raises:
        CalibrationError: If the entry is missing or malformed or its shape is wrong.
    """
    entry = data.get(key)
    if not isinstance(entry, dict) or not isinstance(entry.get("data"), list):
        raise CalibrationError(f"{key} must be a mapping with rows, cols and data")
    entry_rows, entry_cols, values = entry.get("rows"), entry.get("cols"), entry["data"]
    if not isinstance(entry_rows, int) or not isinstance(entry_cols, int) or len(values) != entry_rows * entry_cols:
        raise CalibrationError(f"{key}: data must hold rows x cols values")
    if (rows is not None and entry_rows != rows) or (cols is not None and entry_cols != cols):
        raise CalibrationError(f"{key} must be {rows or entry_rows}x{cols or entry_cols}")
    return values


def read_yaml_mapping(path: Path) -> dict[str, Any]:
    """Read a YAML file whose top level is a mapping.

    Args:
        path (Path): File.

    Returns:
        dict[str, Any]: The mapping.

    Raises:
        CalibrationError: If the file is missing, not YAML or not a mapping.
    """
    if not path.is_file():
        raise CalibrationError(f"{path} is missing")
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as e:
        raise CalibrationError(f"{path}: unreadable: {e}") from e
    if not isinstance(data, dict):
        raise CalibrationError(f"{path}: expected a mapping at the top level")
    return data


def load_camera_info_yaml(path: Path) -> CameraCalibration:
    """Load and validate one ROS camera_info YAML.

    Args:
        path (Path): stereo_left.yaml or stereo_right.yaml (any camera_info YAML).

    Returns:
        CameraCalibration: The camera.

    Raises:
        CalibrationError: If the file is missing or invalid; the message names the file and the reason.
    """
    data = read_yaml_mapping(path)
    try:
        return CameraCalibration.model_validate(
            {
                "camera_name": str(data.get("camera_name") or ""),
                "image_width": data.get("image_width"),
                "image_height": data.get("image_height"),
                "distortion_model": data.get("distortion_model"),
                "d": matrix_data(data, "distortion_coefficients", 1, None),
                "k": matrix_data(data, "camera_matrix", 3, 3),
                "r": matrix_data(data, "rectification_matrix", 3, 3),
                "p": matrix_data(data, "projection_matrix", 3, 4),
            }
        )
    except CalibrationError as e:
        raise CalibrationError(f"{path}: {e}") from e
    except ValidationError as e:
        raise CalibrationError(f"{path}: {validation_reason(e)}") from e


def load_stereo_extrinsics_yaml(path: Path) -> StereoExtrinsics:
    """Load and validate stereo.yaml.

    Args:
        path (Path): stereo.yaml.

    Returns:
        StereoExtrinsics: The extrinsics.

    Raises:
        CalibrationError: If the file is missing or invalid; the message names the file and the reason.
    """
    data = read_yaml_mapping(path)
    try:
        return StereoExtrinsics.model_validate(
            {
                "image_width": data.get("image_width"),
                "image_height": data.get("image_height"),
                "r": matrix_data(data, "R", 3, 3),
                "t": matrix_data(data, "T", 3, 1),
                "q": matrix_data(data, "Q", 4, 4),
                "rms": data.get("rms"),
            }
        )
    except CalibrationError as e:
        raise CalibrationError(f"{path}: {e}") from e
    except ValidationError as e:
        raise CalibrationError(f"{path}: {validation_reason(e)}") from e


def load_stereo_calibration(directory: Path) -> CalibrationLoad:
    """Load and validate the three calibration files of the pair.

    Args:
        directory (Path): Calibration directory.

    Returns:
        CalibrationLoad: The pair and an empty reason, or None and why (missing file, invalid file, inconsistent pair).
    """
    try:
        left = load_camera_info_yaml(directory / LEFT_FILE)
        right = load_camera_info_yaml(directory / RIGHT_FILE)
        extrinsics = load_stereo_extrinsics_yaml(directory / STEREO_FILE)
    except CalibrationError as e:
        return CalibrationLoad(None, str(e))
    try:
        return CalibrationLoad(StereoCalibration(left=left, right=right, extrinsics=extrinsics), "")
    except ValidationError as e:
        return CalibrationLoad(None, f"{directory}: inconsistent stereo calibration: {validation_reason(e)}")


def camera_info_yaml(camera: CameraCalibration) -> dict[str, Any]:
    """Build the ROS camera_info YAML mapping of one camera.

    Args:
        camera (CameraCalibration): The camera.

    Returns:
        dict[str, Any]: The mapping, in the cameracalibrator key order.
    """
    return {
        "image_width": camera.width,
        "image_height": camera.height,
        "camera_name": camera.camera_name,
        "camera_matrix": matrix_yaml(camera.k, 3, 3),
        "distortion_model": camera.distortion_model,
        "distortion_coefficients": matrix_yaml(camera.d, 1, len(camera.d)),
        "rectification_matrix": matrix_yaml(camera.r, 3, 3),
        "projection_matrix": matrix_yaml(camera.p, 3, 4),
    }


def stereo_yaml(extrinsics: StereoExtrinsics) -> dict[str, Any]:
    """Build the stereo.yaml mapping.

    Args:
        extrinsics (StereoExtrinsics): The extrinsics.

    Returns:
        dict[str, Any]: The mapping.
    """
    return {
        "image_width": extrinsics.width,
        "image_height": extrinsics.height,
        "R": matrix_yaml(extrinsics.r, 3, 3),
        "T": matrix_yaml(extrinsics.t, 3, 1),
        "Q": matrix_yaml(extrinsics.q, 4, 4),
        "rms": float(extrinsics.rms),
    }


def write_yaml_atomic(path: Path, data: dict[str, Any]) -> None:
    """Write YAML through a temporary file and a rename, so a reader never sees a half-written file.

    Args:
        path (Path): Destination.
        data (dict[str, Any]): Mapping to write.
    """
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as handle:
            yaml.safe_dump(data, handle, sort_keys=False)
        os.chmod(tmp, FILE_MODE)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def write_stereo_calibration(directory: Path, calibration: StereoCalibration) -> list[Path]:
    """Write stereo_left.yaml, stereo_right.yaml and stereo.yaml (creating the directory).

    Args:
        directory (Path): Calibration directory.
        calibration (StereoCalibration): The validated pair.

    Returns:
        list[Path]: The written files.
    """
    directory.mkdir(parents=True, exist_ok=True)
    contents = {
        LEFT_FILE: camera_info_yaml(calibration.left),
        RIGHT_FILE: camera_info_yaml(calibration.right),
        STEREO_FILE: stereo_yaml(calibration.extrinsics),
    }
    paths = []
    for name, data in contents.items():
        path = directory / name
        write_yaml_atomic(path, data)
        paths.append(path)
    return paths


def flat(matrix: Any) -> list[float]:
    """Flatten an array-like to a list of Python floats.

    Args:
        matrix (Any): numpy array or nested sequence.

    Returns:
        list[float]: Row-major values.
    """
    return [float(v) for v in np.asarray(matrix, dtype=np.float64).ravel()]


def stereo_calibration_from_opencv(
    width: int,
    height: int,
    k1: Any,
    d1: Any,
    k2: Any,
    d2: Any,
    r: Any,
    t: Any,
    r1: Any,
    r2: Any,
    p1: Any,
    p2: Any,
    q: Any,
    rms: float,
) -> StereoCalibration:
    """Build the pair from cv2.stereoCalibrate and cv2.stereoRectify outputs.

    Args:
        width (int): Image width.
        height (int): Image height.
        k1 (Any): Left 3x3 intrinsics.
        d1 (Any): Left distortion (5 or 8 coefficients).
        k2 (Any): Right 3x3 intrinsics.
        d2 (Any): Right distortion (5 or 8 coefficients).
        r (Any): 3x3 rotation between the cameras.
        t (Any): Translation between the cameras, metres.
        r1 (Any): Left rectification rotation.
        r2 (Any): Right rectification rotation.
        p1 (Any): Left 3x4 rectified projection.
        p2 (Any): Right 3x4 rectified projection.
        q (Any): 4x4 disparity-to-depth matrix.
        rms (float): Stereo reprojection error, pixels.

    Returns:
        StereoCalibration: The validated pair.

    Raises:
        pydantic.ValidationError: If the result is not a valid rectified pair.
    """
    models = {len_: name for name, len_ in DISTORTION_LENGTHS.items()}

    def camera(name: str, k: Any, d: Any, rect: Any, proj: Any) -> CameraCalibration:
        coefficients = flat(d)
        return CameraCalibration(
            camera_name=name,
            width=width,
            height=height,
            distortion_model=models.get(len(coefficients), "unsupported"),
            d=coefficients,
            k=flat(k),
            r=flat(rect),
            p=flat(proj),
        )

    return StereoCalibration(
        left=camera(LEFT_CAMERA_NAME, k1, d1, r1, p1),
        right=camera(RIGHT_CAMERA_NAME, k2, d2, r2, p2),
        extrinsics=StereoExtrinsics(width=width, height=height, r=flat(r), t=flat(t), q=flat(q), rms=float(rms)),
    )


def rectification_maps(calibration: StereoCalibration) -> RectificationMaps:
    """Build both cameras' undistort-rectify maps once (fixed point, CV_16SC2, the fastest cv2.remap input).

    Args:
        calibration (StereoCalibration): The validated pair.

    Returns:
        RectificationMaps: The maps for the calibrated image size.
    """
    size = (calibration.width, calibration.height)

    def maps(camera: CameraCalibration) -> tuple[np.ndarray, np.ndarray]:
        k = np.array(camera.k, dtype=np.float64).reshape(3, 3)
        r = np.array(camera.r, dtype=np.float64).reshape(3, 3)
        p = np.array(camera.p, dtype=np.float64).reshape(3, 4)
        d = np.array(camera.d, dtype=np.float64)
        map1, map2 = cv2.initUndistortRectifyMap(k, d, r, p[:, :3], size, cv2.CV_16SC2)
        return map1, map2

    return RectificationMaps(left=maps(calibration.left), right=maps(calibration.right))


def rectify(maps: RectificationMaps, left: np.ndarray, right: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rectify a raw pair with the prebuilt maps.

    Args:
        maps (RectificationMaps): From rectification_maps.
        left (np.ndarray): Left image of the calibrated size.
        right (np.ndarray): Right image of the calibrated size.

    Returns:
        tuple[np.ndarray, np.ndarray]: (left, right) rectified images.
    """
    rect_left = cv2.remap(left, maps.left[0], maps.left[1], cv2.INTER_LINEAR)
    rect_right = cv2.remap(right, maps.right[0], maps.right[1], cv2.INTER_LINEAR)
    return rect_left, rect_right


def calibration_signature(directory: Path) -> tuple[tuple[int, int, int] | None, ...]:
    """Identify the current calibration files cheaply (inode, mtime, size), to detect a new calibration.

    Args:
        directory (Path): Calibration directory.

    Returns:
        tuple[tuple[int, int, int] | None, ...]: One entry per file in CALIBRATION_FILES; None when it is missing.
    """
    entries: list[tuple[int, int, int] | None] = []
    for name in CALIBRATION_FILES:
        try:
            stat = (directory / name).stat()
        except OSError:
            entries.append(None)
            continue
        entries.append((stat.st_ino, stat.st_mtime_ns, stat.st_size))
    return tuple(entries)
