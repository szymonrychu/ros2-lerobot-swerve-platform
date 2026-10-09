"""GPS anchor auto-fit: place the SLAM map frame on the globe from paired GPS fixes and map-frame robot positions.

Each sample pairs a GPS fix (latitude, longitude) with the robot position in the map frame at the same time.
Fixes are projected to local East/North metres around the first fix (equirectangular tangent-plane
approximation, accurate to millimetres over the few hundred metres a robot drives). A 2D rigid transform
(rotation + translation, no scale) from map to ENU is fitted by least squares (Kabsch/Umeyama):

    enu = R(heading_rad) @ map_xy + t

The anchor reported to the browser is the latitude/longitude of the map origin (t converted back to WGS84)
plus heading_rad, the angle of the map +x axis measured counter-clockwise from East. Pure numpy, no ROS.

While parked (no drive fit yet) compass_anchor builds the same anchor from one fix plus the BNO055 compass heading
(NDOF mode: orientation referenced to magnetic north).
"""

from __future__ import annotations

import math
import threading
from collections import deque
from typing import Any

import numpy as np
from pydantic import BaseModel

# WGS84 equatorial radius (metres), used for the local tangent-plane projection.
EARTH_RADIUS_M = 6378137.0
# Defaults for the acceptance gates; the map_nav tab config overrides them.
DEFAULT_MIN_POINTS = 10
DEFAULT_MIN_SPREAD_M = 5.0
DEFAULT_MAX_RESIDUAL_M = 1.5
# A sample is kept only once the robot moved this far (map frame) from the last kept sample,
# so a parked robot does not flood the buffer with identical points.
DEFAULT_MIN_SAMPLE_SPACING_M = 0.2
# BNO055 NDOF orientation is the chip's world frame: x magnetic north, y west, z up (identity when level and facing
# north, yaw counter-clockwise). ENU yaw = this offset + chip yaw - declination (east declination is positive).
NWU_TO_ENU_YAW_RAD = math.pi / 2.0
# An IMU orientation is used for a fix only when its stamp is within this many seconds of the fix stamp.
COMPASS_IMU_MAX_AGE_S = 1.0
# /imu/calibration older than this (monotonic seconds) is treated as missing (the node publishes it at 1 Hz).
CALIBRATION_MAX_AGE_S = 5.0
# BNO055 calibration gates (0-3 scale): magnetometer and overall system level required to trust the heading.
MIN_MAG_CALIBRATION = 2
# A republished compass anchor must differ from the cached one by more than this (position metres / heading degrees).
COMPASS_MIN_CHANGE_M = 0.05
COMPASS_MIN_CHANGE_DEG = 0.2
# Oldest samples are dropped beyond this many (bounds memory and refit cost).
DEFAULT_MAX_SAMPLES = 500


class CompassSettings(BaseModel):
    """Settings of the compass (BNO055) GPS anchor.

    Attributes:
        imu_topic (str): sensor_msgs/Imu topic with the absolute (NDOF) orientation.
        calibration_topic (str | None): std_msgs/String JSON {sys, gyro, accel, mag}; when set the heading is
            trusted only while the calibration gate passes.
        declination_deg (float): Magnetic declination in degrees, east positive (true = magnetic + declination).
        imu_yaw_offset_deg (float): Mounting yaw of the IMU in base_link (degrees), used only without a TF.
    """

    imu_topic: str
    calibration_topic: str | None = None
    declination_deg: float = 0.0
    imu_yaw_offset_deg: float = 0.0


def normalize_angle(angle: float) -> float:
    """Wrap an angle into (-pi, pi].

    Args:
        angle (float): Angle in radians.

    Returns:
        float: Equivalent angle in (-pi, pi].
    """
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, ...]:
    """Hamilton product a * b of two (x, y, z, w) quaternions.

    Args:
        a (tuple[float, float, float, float]): Left quaternion (x, y, z, w).
        b (tuple[float, float, float, float]): Right quaternion (x, y, z, w).

    Returns:
        tuple[float, ...]: Product (x, y, z, w).
    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def valid_imu_orientation(quat_xyzw: tuple[float, float, float, float], covariance: list[float]) -> bool:
    """Return True if an Imu orientation carries data (known covariance, finite non-zero quaternion).

    Args:
        quat_xyzw (tuple[float, float, float, float]): Orientation (x, y, z, w).
        covariance (list[float]): orientation_covariance; element 0 equal to -1 means "no orientation".

    Returns:
        bool: Whether the orientation may be used.
    """
    if len(covariance) < 1 or covariance[0] == -1.0:
        return False
    if not all(math.isfinite(v) for v in quat_xyzw):
        return False
    return sum(v * v for v in quat_xyzw) > 0.0


def calibration_trusted(status: dict[str, Any] | None, min_mag: int = MIN_MAG_CALIBRATION) -> bool:
    """Return True if the BNO055 calibration status is good enough to trust the compass heading.

    Only the magnetometer is gated: the system status stays 0 in NDOF on the robot even with mag fully calibrated.

    Args:
        status (dict[str, Any] | None): {"sys", "gyro", "accel", "mag"} (each 0-3) or None when unknown.
        min_mag (int): Minimum magnetometer calibration.

    Returns:
        bool: True when the magnetometer gate passes; False for missing or incomplete status.
    """
    if not status:
        return False
    mag = status.get("mag")
    return isinstance(mag, int) and mag >= min_mag


def enu_yaw_from_imu(
    imu_xyzw: tuple[float, float, float, float],
    mount_xyzw: tuple[float, float, float, float] | None,
    yaw_offset_rad: float,
    declination_deg: float,
) -> float:
    """Convert a BNO055 NDOF orientation into the ENU yaw of base_link in true-north terms.

    The orientation is the IMU frame in the chip world frame (x magnetic north, y west, z up). The IMU mounting
    removes the base_link <- imu_link rotation; without it a plain yaw offset is used.

    Args:
        imu_xyzw (tuple[float, float, float, float]): Imu.orientation (x, y, z, w).
        mount_xyzw (tuple[float, float, float, float] | None): Rotation of imu_link in base_link (TF base_link ->
            imu_link), or None when unavailable.
        yaw_offset_rad (float): Mounting yaw of the IMU in base_link (radians), used when mount_xyzw is None.
        declination_deg (float): Magnetic declination in degrees, east positive.

    Returns:
        float: Yaw of base_link counter-clockwise from true East, radians in (-pi, pi].
    """
    if mount_xyzw is None:
        half = yaw_offset_rad / 2.0
        mount_xyzw = (0.0, 0.0, math.sin(half), math.cos(half))
    mount_inv = (-mount_xyzw[0], -mount_xyzw[1], -mount_xyzw[2], mount_xyzw[3])
    x, y, z, w = quat_multiply(imu_xyzw, mount_inv)
    chip_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return normalize_angle(NWU_TO_ENU_YAW_RAD + chip_yaw - math.radians(declination_deg))


def compass_anchor(
    lat: float, lon: float, map_x: float, map_y: float, map_yaw_rad: float, enu_yaw_rad: float
) -> dict[str, Any]:
    """Build a GPS anchor from one fix and the robot's compass heading (same semantics as the drive fit).

    heading_rad is the rotation map -> ENU, enu_yaw - map_yaw. The map origin lies at t = -R(heading) @ [x, y]
    relative to the fix, so the robot's map pose maps back onto the fix.

    Args:
        lat (float): Fix latitude in degrees.
        lon (float): Fix longitude in degrees.
        map_x (float): Robot x in the map frame (metres).
        map_y (float): Robot y in the map frame (metres).
        map_yaw_rad (float): Robot yaw in the map frame (radians).
        enu_yaw_rad (float): Robot yaw counter-clockwise from true East (radians), e.g. from enu_yaw_from_imu.

    Returns:
        dict[str, Any]: {lat, lon, heading_rad, residual_m (None), n_points (1), source ("compass")}.
    """
    heading = normalize_angle(enu_yaw_rad - map_yaw_rad)
    c, s = math.cos(heading), math.sin(heading)
    east = -(c * map_x - s * map_y)
    north = -(s * map_x + c * map_y)
    anchor_lat, anchor_lon = enu_to_latlon(east, north, lat, lon)
    return {
        "lat": anchor_lat,
        "lon": anchor_lon,
        "heading_rad": heading,
        "residual_m": None,
        "n_points": 1,
        "source": "compass",
    }


def latlon_to_enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Project a WGS84 position to local East/North metres around a reference.

    Args:
        lat (float): Latitude in degrees.
        lon (float): Longitude in degrees.
        ref_lat (float): Reference latitude in degrees (tangent point).
        ref_lon (float): Reference longitude in degrees (tangent point).

    Returns:
        tuple[float, float]: (east_m, north_m).
    """
    east = EARTH_RADIUS_M * math.cos(math.radians(ref_lat)) * math.radians(lon - ref_lon)
    north = EARTH_RADIUS_M * math.radians(lat - ref_lat)
    return east, north


def enu_to_latlon(east: float, north: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Inverse of latlon_to_enu.

    Args:
        east (float): East offset in metres.
        north (float): North offset in metres.
        ref_lat (float): Reference latitude in degrees.
        ref_lon (float): Reference longitude in degrees.

    Returns:
        tuple[float, float]: (latitude, longitude) in degrees.
    """
    lat = ref_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = ref_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat))))
    return lat, lon


def fit_rigid_2d(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, float]:
    """Least-squares 2D rotation + translation mapping src onto dst (Kabsch without scale).

    Args:
        src (np.ndarray): Source points, shape (N, 2).
        dst (np.ndarray): Destination points, shape (N, 2), paired with src.

    Returns:
        tuple[float, np.ndarray, float]: (theta, t, residual_rms) with dst ~= R(theta) @ src + t,
            theta in radians (-pi, pi], t shape (2,), residual_rms the RMS point error in dst units.
    """
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    h = (src - src_mean).T @ (dst - dst_mean)
    theta = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    t = dst_mean - rot @ src_mean
    errors = dst - (src @ rot.T + t)
    residual = float(np.sqrt(np.mean(np.sum(errors**2, axis=1))))
    return theta, t, residual


class GpsAnchorEstimator:
    """Accumulates (fix, map position) samples and fits the map frame's GPS anchor.

    The anchor is reported only when there are at least min_points samples, the map-frame track spans at
    least min_spread_m (bounding-box diagonal) and the fit residual is at most max_residual_m.
    """

    def __init__(
        self,
        min_points: int = DEFAULT_MIN_POINTS,
        min_spread_m: float = DEFAULT_MIN_SPREAD_M,
        max_residual_m: float = DEFAULT_MAX_RESIDUAL_M,
        min_sample_spacing_m: float = DEFAULT_MIN_SAMPLE_SPACING_M,
        max_samples: int = DEFAULT_MAX_SAMPLES,
    ) -> None:
        """Initialise the estimator.

        Args:
            min_points (int): Minimum samples before an anchor is reported.
            min_spread_m (float): Minimum map-frame track extent (bounding-box diagonal, metres).
            max_residual_m (float): Maximum RMS fit residual (metres) for an anchor to be reported.
            min_sample_spacing_m (float): Minimum map-frame distance from the last kept sample.
            max_samples (int): Maximum samples kept (oldest dropped first).
        """
        self.min_points = min_points
        self.min_spread_m = min_spread_m
        self.max_residual_m = max_residual_m
        self.min_sample_spacing_m = min_sample_spacing_m
        self.ref: tuple[float, float] | None = None
        self.samples: deque[tuple[float, float, float, float]] = deque(maxlen=max_samples)
        self.lock = threading.Lock()
        self.last_result: dict[str, Any] | None = None

    @property
    def n_points(self) -> int:
        """Number of samples currently held.

        Returns:
            int: Sample count.
        """
        with self.lock:
            return len(self.samples)

    def reset(self) -> None:
        """Drop all samples, the ENU reference and the cached result (call when the SLAM map is reset)."""
        with self.lock:
            self.samples.clear()
            self.ref = None
            self.last_result = None

    def add_sample(self, lat: float, lon: float, map_x: float, map_y: float) -> dict[str, Any] | None:
        """Add a fix paired with the robot's map-frame position and refit.

        A fix rejected by the sample-spacing gate changes nothing and returns the cached last result
        without refitting. Thread-safe.

        Args:
            lat (float): Fix latitude in degrees.
            lon (float): Fix longitude in degrees.
            map_x (float): Robot x in the map frame at the fix time (metres).
            map_y (float): Robot y in the map frame at the fix time (metres).

        Returns:
            dict[str, Any] | None: Anchor {lat, lon, heading_rad, residual_m, n_points, source} when every gate
                passes, otherwise None.
        """
        with self.lock:
            if self.samples:
                last = self.samples[-1]
                if math.hypot(map_x - last[0], map_y - last[1]) < self.min_sample_spacing_m:
                    return self.last_result
            if self.ref is None:
                self.ref = (lat, lon)
            east, north = latlon_to_enu(lat, lon, *self.ref)
            self.samples.append((map_x, map_y, east, north))
            self.last_result = self.compute_fit()
            return self.last_result

    def fit(self) -> dict[str, Any] | None:
        """Fit the anchor from a snapshot of the current samples. Thread-safe.

        Returns:
            dict[str, Any] | None: Anchor dict when all gates pass, otherwise None.
        """
        with self.lock:
            self.last_result = self.compute_fit()
            return self.last_result

    def compute_fit(self) -> dict[str, Any] | None:
        """Fit the anchor from the current samples; the caller holds self.lock.

        Returns:
            dict[str, Any] | None: Anchor dict when all gates pass, otherwise None.
        """
        if self.ref is None or len(self.samples) < self.min_points:
            return None
        data = np.asarray(self.samples, dtype=float)
        src, dst = data[:, :2], data[:, 2:]
        spread = float(np.linalg.norm(src.max(axis=0) - src.min(axis=0)))
        if spread < self.min_spread_m:
            return None
        theta, t, residual = fit_rigid_2d(src, dst)
        if residual > self.max_residual_m:
            return None
        lat, lon = enu_to_latlon(float(t[0]), float(t[1]), *self.ref)
        return {
            "lat": lat,
            "lon": lon,
            "heading_rad": theta,
            "residual_m": residual,
            "n_points": len(self.samples),
            "source": "fit",
        }
