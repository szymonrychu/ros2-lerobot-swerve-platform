"""Arducam ToF bridge: depth, amplitude and camera_info from the ArducamDepthCamera SDK to ROS2.

Topics (one header, one stamp and config.frame_id shared by the three messages of a frame):
  - DEPTH_TOPIC        sensor_msgs/Image 16UC1, millimetres of z-depth (the SDK radial distance divided by the
                       pixel ray norm, config.depth_is_radial); 0 = invalid (confidence below
                       config.confidence_threshold, not finite, <= 0 or beyond 65535 mm).
  - AMPLITUDE_TOPIC    sensor_msgs/Image mono16: the SDK amplitude_data, or confidence_data when the SDK frame
                       carries no amplitude, rounded and clipped to 0..65535. mono16 (not mono8) keeps the SDK's
                       full dynamic range; viewers scale it for display.
  - CAMERA_INFO_TOPIC  sensor_msgs/CameraInfo from config.tof_calibration_file when it is usable, else from the SDK
                       intrinsics (INTRINSIC_FX/FY/CX/CY, reported x100). Not published when neither exists, so no
                       made-up calibration ever goes out.

Range mode: "far" (4 m, the default, suits a head camera looking across a room) or "near" (2 m, finer depth).
The SDK takes the range in millimetres; when it rejects that, metres are tried (the vendor samples use both).

Stamp: the node clock read right after requestFrame returns, minus config.stamp_latency_ms (the SDK frame
timestamp is not used: its clock domain is undocumented).

A timer ticks at config.publish_rate_hz and requests one frame per tick, so publishing never exceeds that rate. A
missing, failed or malformed frame publishes nothing for that tick: it is logged and counted in
tof_frames_dropped_total{reason}. A camera that fails to open or start is logged, counted and retried every
OPEN_RETRY_S; one that yields no valid frame for REOPEN_AFTER_S is closed and reopened.
"""

import time
from collections.abc import Callable
from functools import lru_cache
from pathlib import Path
from typing import Any, NamedTuple

import ArducamDepthCamera as ac
import numpy as np
import rclpy
import yaml
from prometheus_client import Counter
from pydantic import BaseModel, ConfigDict, ValidationError
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from .config import TofConfig, resolve_camera_index

NODE_NAME = "arducam_tof"
DEPTH_TOPIC = "/tof/depth/image_raw"
AMPLITUDE_TOPIC = "/tof/amplitude/image_raw"
CAMERA_INFO_TOPIC = "/tof/camera_info"
DEPTH_ENCODING = "16UC1"
AMPLITUDE_ENCODING = "mono16"
BYTES_PER_PIXEL = 2
PUBLISH_QOS_DEPTH = 5
FRAME_TIMEOUT_MS = 1000
OPEN_RETRY_S = 5.0
REOPEN_AFTER_S = 5.0
LOG_THROTTLE_S = 5.0
SDK_OK = 0
RANGE_MM = {"near": 2000, "far": 4000}
MM_PER_M = 1000
INTRINSIC_SCALE = 100.0
NS_PER_MS = 1_000_000
MAX_UINT16 = 65535
DISTORTION_MODEL = "plumb_bob"
NO_DISTORTION = [0.0] * 5
MATRIX_ELEMENTS = 9
IDENTITY_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

FRAMES_PUBLISHED = Counter("tof_frames_published_total", "ToF frames published (depth, amplitude, camera_info)")
FRAMES_DROPPED = Counter("tof_frames_dropped_total", "ToF frame requests that published nothing, by reason", ["reason"])
NEAR_PIXELS_REMOVED = Counter(
    "tof_near_pixels_removed_total", "ToF pixels zeroed for being nearer than min_range_m (the robot's own arm)"
)
FLYING_PIXELS_REMOVED = Counter(
    "tof_flying_pixels_removed_total", "ToF flying (mixed) pixels zeroed at depth discontinuities"
)
OPEN_FAILURES = Counter("tof_camera_open_failures_total", "Failed attempts to open and start the ToF camera")
REOPENS = Counter("tof_camera_reopens_total", "Times the ToF camera was closed and reopened after yielding no frames")


class CameraMatrix(BaseModel):
    """camera_matrix block of a camera_calibration YAML."""

    model_config = ConfigDict(extra="ignore")

    data: list[float]


class CalibrationFile(BaseModel):
    """The parts of a camera_calibration YAML the bridge reads; distortion coefficients are ignored on purpose."""

    model_config = ConfigDict(extra="ignore")

    image_width: int
    image_height: int
    camera_matrix: CameraMatrix


class Intrinsics(NamedTuple):
    """Pinhole intrinsics in pixels.

    Attributes:
        fx (float): Focal length x.
        fy (float): Focal length y.
        cx (float): Principal point x.
        cy (float): Principal point y.
    """

    fx: float
    fy: float
    cx: float
    cy: float


def depth_to_mm(depth: np.ndarray, confidence: np.ndarray, threshold: float) -> np.ndarray:
    """Convert SDK depth (float millimetres) to 16UC1 millimetres with invalid pixels set to 0.

    Args:
        depth (np.ndarray): Depth per pixel in millimetres (float).
        confidence (np.ndarray): Confidence per pixel, same shape.
        threshold (float): Pixels with confidence below this are invalid.

    Returns:
        np.ndarray: uint16 depth in millimetres; 0 where low confidence, not finite, <= 0 or above 65535 mm.
    """
    mm = np.rint(np.nan_to_num(depth.astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0))
    valid = (confidence >= threshold) & (mm > 0) & (mm <= MAX_UINT16)
    return np.where(valid, mm, 0).astype(np.uint16)


def load_calibration(path: Path) -> tuple[Intrinsics, int, int]:
    """Read the camera matrix of a camera_calibration YAML (distortion coefficients are not used).

    Args:
        path (Path): Calibration YAML.

    Returns:
        tuple[Intrinsics, int, int]: (fx, fy, cx, cy), image_width, image_height.

    Raises:
        ValueError: If the file is missing, is not YAML or lacks a 3x3 camera matrix with positive focal lengths.
    """
    try:
        calibration = CalibrationFile.model_validate(yaml.safe_load(path.read_text()))
    except (OSError, yaml.YAMLError, ValidationError) as e:
        raise ValueError(f"{path}: {e}") from e
    data = calibration.camera_matrix.data
    if len(data) != MATRIX_ELEMENTS or data[0] <= 0 or data[4] <= 0:
        raise ValueError(f"{path}: camera_matrix.data must hold 9 values with positive fx and fy")
    return Intrinsics(data[0], data[4], data[2], data[5]), calibration.image_width, calibration.image_height


def ray_norm(u: np.ndarray, v: np.ndarray, intrinsics: Intrinsics) -> np.ndarray:
    """Return the length of the unit-z ray through each pixel: sqrt(((u-cx)/fx)^2 + ((v-cy)/fy)^2 + 1).

    Args:
        u (np.ndarray): Pixel columns (pixel centres).
        v (np.ndarray): Pixel rows, broadcastable with u.
        intrinsics (Intrinsics): Pinhole intrinsics.

    Returns:
        np.ndarray: Ray norm per pixel (>= 1).
    """
    fx, fy, cx, cy = intrinsics
    return np.sqrt(((u - cx) / fx) ** 2 + ((v - cy) / fy) ** 2 + 1.0)


@lru_cache(maxsize=4)
def ray_norm_image(shape: tuple[int, int], intrinsics: Intrinsics) -> np.ndarray:
    """Return ray_norm for every pixel of an image, cached per (shape, intrinsics).

    Args:
        shape (tuple[int, int]): (height, width).
        intrinsics (Intrinsics): Pinhole intrinsics.

    Returns:
        np.ndarray: float64 (height, width) ray norms; read-only because the cache shares it.
    """
    rows, cols = np.mgrid[0 : shape[0], 0 : shape[1]]
    norms = ray_norm(cols, rows, intrinsics)
    norms.setflags(write=False)
    return norms


def radial_to_z(depth_mm: np.ndarray, intrinsics: Intrinsics) -> np.ndarray:
    """Convert radial distance (along the pixel ray) to z-depth (along the optical axis).

    Args:
        depth_mm (np.ndarray): uint16 radial depth in millimetres; 0 = invalid.
        intrinsics (Intrinsics): Pinhole intrinsics.

    Returns:
        np.ndarray: uint16 z-depth in millimetres, rounded; 0 stays 0.
    """
    z = np.rint(depth_mm / ray_norm_image(depth_mm.shape, intrinsics))
    return np.where(depth_mm > 0, z, 0).astype(np.uint16)


def near_range_filter(
    depth_mm: np.ndarray, min_range_m: float, intrinsics: Intrinsics | None, depth_is_radial: bool
) -> np.ndarray:
    """Zero the pixels whose distance from the camera is below min_range_m.

    Args:
        depth_mm (np.ndarray): uint16 depth in millimetres; 0 = invalid.
        min_range_m (float): Minimum ray distance in metres; 0 or less leaves the depth unchanged.
        intrinsics (Intrinsics | None): Pinhole intrinsics, used to turn z-depth into ray distance; None compares
            the depth as is.
        depth_is_radial (bool): True when depth_mm already is the ray distance.

    Returns:
        np.ndarray: uint16 depth of the same shape with the too-near pixels set to 0.
    """
    if min_range_m <= 0.0:
        return depth_mm
    distance = depth_mm.astype(np.float64)
    if not depth_is_radial and intrinsics is not None:
        distance = distance * ray_norm_image(depth_mm.shape, intrinsics)
    return np.where(distance < min_range_m * 1000.0, 0, depth_mm).astype(np.uint16)


def flying_pixel_filter(depth_mm: np.ndarray, jump_ratio: float, min_jump_mm: int) -> np.ndarray:
    """Zero the mixed ("flying") pixels at depth discontinuities.

    A valid pixel becomes 0 (invalid, REP 118) when a valid pixel of its 3x3 neighbourhood differs from it by more
    than max(min_jump_mm, jump_ratio * its depth). 0 pixels are never compared.

    Args:
        depth_mm (np.ndarray): uint16 z-depth in millimetres; 0 = invalid.
        jump_ratio (float): Depth-relative jump threshold (dimensionless).
        min_jump_mm (int): Absolute floor of the jump threshold in millimetres.

    Returns:
        np.ndarray: uint16 depth of the same shape with the flying pixels set to 0.
    """
    depth = depth_mm.astype(np.int32)
    threshold = np.maximum(float(min_jump_mm), jump_ratio * depth)
    flying = np.zeros(depth.shape, dtype=bool)
    height, width = depth.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            here = (slice(max(dy, 0), height + min(dy, 0)), slice(max(dx, 0), width + min(dx, 0)))
            there = (slice(max(-dy, 0), height + min(-dy, 0)), slice(max(-dx, 0), width + min(-dx, 0)))
            both_valid = (depth[here] > 0) & (depth[there] > 0)
            flying[here] |= both_valid & (np.abs(depth[here] - depth[there]) > threshold[here])
    return np.where(flying, 0, depth_mm).astype(np.uint16)


def amplitude_to_mono16(amplitude: np.ndarray) -> np.ndarray:
    """Convert SDK amplitude (or confidence) to mono16, rounded and clipped to 0..65535 (not finite = 0).

    Args:
        amplitude (np.ndarray): Amplitude per pixel (float).

    Returns:
        np.ndarray: uint16 amplitude.
    """
    values = np.nan_to_num(amplitude.astype(np.float64), nan=0.0, posinf=MAX_UINT16, neginf=0.0)
    return np.clip(np.rint(values), 0, MAX_UINT16).astype(np.uint16)


def image_msg(header: Any, pixels: np.ndarray, encoding: str) -> Any:
    """Build a sensor_msgs/Image of 16-bit little-endian pixels.

    Args:
        header (Any): std_msgs/Header shared by the frame's messages.
        pixels (np.ndarray): 2-D uint16 image.
        encoding (str): DEPTH_ENCODING or AMPLITUDE_ENCODING.

    Returns:
        Any: The sensor_msgs/Image.
    """
    msg = Image()
    msg.header = header
    msg.height, msg.width = pixels.shape
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = msg.width * BYTES_PER_PIXEL
    msg.data = pixels.astype("<u2").tobytes()
    return msg


def camera_info_msg(header: Any, width: int, height: int, intrinsics: Intrinsics) -> Any:
    """Build the sensor_msgs/CameraInfo of the undistorted pinhole model of the published z-depth.

    Args:
        header (Any): std_msgs/Header shared by the frame's messages.
        width (int): Image width in pixels.
        height (int): Image height in pixels.
        intrinsics (Intrinsics): Focal lengths and principal point.

    Returns:
        Any: The sensor_msgs/CameraInfo (plumb_bob, zero distortion, P equal to K: the depth is undistorted).
    """
    fx, fy, cx, cy = intrinsics
    msg = CameraInfo()
    msg.header = header
    msg.width = width
    msg.height = height
    msg.distortion_model = DISTORTION_MODEL
    msg.d = list(NO_DISTORTION)
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = list(IDENTITY_R)
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def frame_arrays(frame: Any) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Extract (depth, confidence, amplitude) from an SDK frame when they form one valid 2-D image.

    Args:
        frame (Any): ac.DepthData frame.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray] | None: Arrays of one non-empty 2-D shape (amplitude is the
        confidence when the frame has no amplitude), or None when the frame is malformed.
    """
    depth = getattr(frame, "depth_data", None)
    confidence = getattr(frame, "confidence_data", None)
    if depth is None or confidence is None:
        return None
    depth = np.asarray(depth)
    confidence = np.asarray(confidence)
    if depth.ndim != 2 or depth.size == 0 or confidence.shape != depth.shape:
        return None
    amplitude = getattr(frame, "amplitude_data", None)
    amplitude = confidence if amplitude is None else np.asarray(amplitude)
    if amplitude.shape != depth.shape:
        return None
    return depth, confidence, amplitude


def sdk_ok(ret: Any) -> bool:
    """Tell whether an SDK call succeeded (TofErrorCode 0; None from calls that return nothing).

    Args:
        ret (Any): The SDK return value.

    Returns:
        bool: True on success.
    """
    return ret is None or int(ret) == SDK_OK


class TofBridge:
    """Publishes one ToF frame per timer tick; owns the SDK camera, its open/retry/reopen cycle and the publishers."""

    def __init__(self, node: Node, config: TofConfig, clock: Callable[[], float] = time.monotonic) -> None:
        """Create the publishers and the publish timer; the camera is opened on the first tick.

        Args:
            node (Node): Node that owns the publishers and the timer.
            config (TofConfig): Validated settings.
            clock (Callable[[], float]): Monotonic seconds, for the retry and reopen delays.
        """
        self.node = node
        self.config = config
        self.clock = clock
        self.logger = node.get_logger()
        self.depth_pub = node.create_publisher(Image, DEPTH_TOPIC, PUBLISH_QOS_DEPTH)
        self.amplitude_pub = node.create_publisher(Image, AMPLITUDE_TOPIC, PUBLISH_QOS_DEPTH)
        self.info_pub = node.create_publisher(CameraInfo, CAMERA_INFO_TOPIC, PUBLISH_QOS_DEPTH)
        self.camera: Any = None
        self.clamp_logged = False
        self.width = 0
        self.height = 0
        self.intrinsics: Intrinsics | None = None
        self.calibration_warned = False
        self.next_open_s = clock()
        self.last_frame_s = clock()
        node.create_timer(1.0 / config.publish_rate_hz, self.step)
        self.logger.info(
            f"arducam_tof: camera_index={config.camera_index} csi_device={config.csi_device} "
            f"range_mode={config.range_mode} "
            f"confidence_threshold={config.confidence_threshold} frame_id={config.frame_id} "
            f"publish_rate_hz={config.publish_rate_hz} stamp_latency_ms={config.stamp_latency_ms} "
            f"depth_is_radial={config.depth_is_radial} tof_calibration_file={config.tof_calibration_file}"
        )

    def step(self) -> None:
        """Timer tick: open the camera when due, else request one frame and publish it when it is valid."""
        now_s = self.clock()
        if self.camera is None:
            if now_s >= self.next_open_s:
                self.open_camera()
            if self.camera is None:
                return
        if self.publish_frame():
            self.last_frame_s = self.clock()
        elif self.clock() - self.last_frame_s >= REOPEN_AFTER_S:
            REOPENS.inc()
            self.logger.warning(f"No valid ToF frame for {REOPEN_AFTER_S} s; reopening the camera.")
            self.close()
            self.next_open_s = self.clock()

    def open_camera(self) -> None:
        """Open and start the SDK camera, set the range and read the intrinsics; on failure log and schedule a retry."""
        camera: Any = None
        opened = started = False
        try:
            index = resolve_camera_index(self.config)
            camera = ac.ArducamCamera()
            ret = camera.open(ac.Connection.CSI, index)
            if not sdk_ok(ret):
                raise RuntimeError(f"open(CSI, {index}) returned {ret}")
            opened = True
            ret = camera.start(ac.FrameType.DEPTH)
            if not sdk_ok(ret):
                raise RuntimeError(f"start(DEPTH) returned {ret}")
            started = True
            self.set_range(camera)
            info = camera.getCameraInfo()
            self.width, self.height = int(info.width), int(info.height)
            self.intrinsics = self.read_intrinsics(camera)
        except Exception as e:  # the SDK raises bare RuntimeErrors from its native layer
            OPEN_FAILURES.inc()
            if opened:  # release the device, else every retry fails with it busy
                self.release(camera, started)
            self.logger.error(
                f"Could not open the ToF camera ({self.config.csi_device or self.config.camera_index}): {e}; "
                f"retrying in {OPEN_RETRY_S} s."
            )
            self.next_open_s = self.clock() + OPEN_RETRY_S
            return
        self.camera = camera
        self.last_frame_s = self.clock()
        self.logger.info(f"ToF camera open: {self.width}x{self.height}, intrinsics {self.intrinsics}")

    def set_range(self, camera: Any) -> None:
        """Set the SDK range for config.range_mode, in millimetres, else in metres when millimetres are rejected.

        Args:
            camera (Any): Opened ac.ArducamCamera.
        """
        range_mm = RANGE_MM[self.config.range_mode]
        if sdk_ok(camera.setControl(ac.Control.RANGE, range_mm)):
            return
        if not sdk_ok(camera.setControl(ac.Control.RANGE, range_mm // MM_PER_M)):
            self.logger.warning(
                f"The SDK rejected range {range_mm} mm and {range_mm // MM_PER_M} m; using its default."
            )

    def read_intrinsics(self, camera: Any) -> Intrinsics | None:
        """Return the calibration file K when usable, else the SDK intrinsics.

        Args:
            camera (Any): Opened ac.ArducamCamera.

        Returns:
            Intrinsics | None: The intrinsics, or None when neither the file nor the SDK provides them.
        """
        calibrated = self.read_calibration()
        return calibrated if calibrated is not None else self.read_sdk_intrinsics(camera)

    def read_calibration(self) -> Intrinsics | None:
        """Load config.tof_calibration_file; a missing, invalid or other-sized file is logged once and gives None.

        Returns:
            Intrinsics | None: The file's K, or None when unset or unusable.
        """
        path = self.config.tof_calibration_file
        if path is None:
            return None
        try:
            intrinsics, width, height = load_calibration(path)
        except ValueError as e:
            self.warn_calibration(self.logger.warning, f"ToF calibration unusable, using the SDK intrinsics: {e}")
            return None
        if (width, height) != (self.width, self.height):
            self.warn_calibration(
                self.logger.error,
                f"ToF calibration {path} is {width}x{height} but the SDK frame is {self.width}x{self.height}; "
                "using the SDK intrinsics.",
            )
            return None
        return intrinsics

    def warn_calibration(self, log: Callable[[str], Any], message: str) -> None:
        """Log a calibration-file problem once per process (the camera may reopen many times).

        Args:
            log (Callable[[str], Any]): Logger method to use.
            message (str): Log text.
        """
        if not self.calibration_warned:
            self.calibration_warned = True
            log(message)

    def read_sdk_intrinsics(self, camera: Any) -> Intrinsics | None:
        """Read the SDK intrinsics; the principal point falls back to the image centre when CX/CY are unavailable.

        Args:
            camera (Any): Opened ac.ArducamCamera.

        Returns:
            Intrinsics | None: The intrinsics, or None when the SDK reports no positive focal length.
        """
        fx = self.read_control(camera, "INTRINSIC_FX")
        fy = self.read_control(camera, "INTRINSIC_FY")
        if fx is None or fy is None:
            self.logger.warning("The SDK reports no focal length; /tof/camera_info will not be published.")
            return None
        cx = self.read_control(camera, "INTRINSIC_CX")
        cy = self.read_control(camera, "INTRINSIC_CY")
        return Intrinsics(fx, fy, self.width / 2 if cx is None else cx, self.height / 2 if cy is None else cy)

    @staticmethod
    def read_control(camera: Any, name: str) -> float | None:
        """Read one x100-scaled intrinsic control.

        Args:
            camera (Any): Opened ac.ArducamCamera.
            name (str): ac.Control member name, e.g. "INTRINSIC_FX".

        Returns:
            float | None: The value in pixels, or None when the control is missing, fails or is not positive.
        """
        control = getattr(ac.Control, name, None)
        if control is None:
            return None
        try:
            value = float(camera.getControl(control)) / INTRINSIC_SCALE
        except Exception:  # an SDK without the control raises from its native layer
            return None
        return value if value > 0 else None

    def publish_frame(self) -> bool:
        """Request one frame and publish depth, amplitude and camera_info under one header when it is valid.

        Returns:
            bool: True when the frame was published, False when nothing was published (logged and counted).
        """
        try:
            frame = self.camera.requestFrame(FRAME_TIMEOUT_MS)
        except Exception as e:  # the SDK raises bare RuntimeErrors from its native layer
            return self.drop("error", f"ToF frame request failed: {e}")
        if frame is None:
            return self.drop("no_frame", f"No ToF frame within {FRAME_TIMEOUT_MS} ms.")
        returned_at = self.node.get_clock().now()  # before any conversion, so their time is not added to the latency
        try:
            if not isinstance(frame, ac.DepthData):
                return self.drop("wrong_type", f"ToF frame is {type(frame).__name__}, not DepthData.")
            arrays = frame_arrays(frame)
            if arrays is None:
                return self.drop("invalid", "Malformed ToF frame (missing or mismatched depth/confidence data).")
            depth, confidence, amplitude = arrays
            depth_mm = depth_to_mm(depth, confidence, self.config.confidence_threshold)
            near_filtered = near_range_filter(
                depth_mm, self.config.min_range_m, self.intrinsics, self.config.depth_is_radial
            )
            NEAR_PIXELS_REMOVED.inc(int(np.count_nonzero(depth_mm)) - int(np.count_nonzero(near_filtered)))
            depth_mm = near_filtered
            if self.config.depth_is_radial and self.intrinsics is not None:
                depth_mm = radial_to_z(depth_mm, self.intrinsics)
            if self.config.flying_pixel_filter:
                filtered = flying_pixel_filter(
                    depth_mm, self.config.flying_pixel_jump_ratio, self.config.flying_pixel_min_jump_mm
                )
                FLYING_PIXELS_REMOVED.inc(int(np.count_nonzero(depth_mm)) - int(np.count_nonzero(filtered)))
                depth_mm = filtered
            amplitude16 = amplitude_to_mono16(amplitude)
        finally:
            self.camera.releaseFrame(frame)

        header = Header()
        header.stamp = self.frame_stamp(returned_at)
        header.frame_id = self.config.frame_id
        self.depth_pub.publish(image_msg(header, depth_mm, DEPTH_ENCODING))
        self.amplitude_pub.publish(image_msg(header, amplitude16, AMPLITUDE_ENCODING))
        if self.intrinsics is not None:
            self.info_pub.publish(camera_info_msg(header, depth_mm.shape[1], depth_mm.shape[0], self.intrinsics))
        FRAMES_PUBLISHED.inc()
        return True

    def frame_stamp(self, returned_at: Any) -> Any:
        """Return the frame's stamp: the frame-return time minus config.stamp_latency_ms, never negative.

        Args:
            returned_at (Any): rclpy Time read right after requestFrame returned the frame.

        Returns:
            Any: builtin_interfaces/Time. A result below zero is clamped to returned_at itself and logged once.
        """
        stamp_ns = returned_at.nanoseconds - round(self.config.stamp_latency_ms * NS_PER_MS)
        if stamp_ns < 0:
            if not self.clamp_logged:
                self.clamp_logged = True
                self.logger.warning(
                    f"stamp_latency_ms={self.config.stamp_latency_ms} exceeds the clock reading; "
                    "stamping frames with the frame-return time."
                )
            return returned_at.to_msg()
        return Time(nanoseconds=stamp_ns, clock_type=returned_at.clock_type).to_msg()

    def drop(self, reason: str, message: str) -> bool:
        """Log and count a frame that publishes nothing.

        Args:
            reason (str): tof_frames_dropped_total reason label.
            message (str): Log text.

        Returns:
            bool: Always False (nothing published).
        """
        FRAMES_DROPPED.labels(reason).inc()
        self.logger.warning(message, throttle_duration_sec=LOG_THROTTLE_S)
        return False

    def close(self) -> None:
        """Stop and close the camera if it is open (idempotent)."""
        camera, self.camera = self.camera, None
        if camera is not None:
            self.release(camera, started=True)

    def release(self, camera: Any, started: bool) -> None:
        """Stop (when started) and close an opened SDK camera, logging instead of raising on SDK errors.

        Args:
            camera (Any): Opened ac.ArducamCamera.
            started (bool): Whether start() succeeded, so stop() is due before close().
        """
        try:
            if started:
                camera.stop()
            camera.close()
        except Exception as e:  # closing a camera that went away must not mask the reason we are closing it
            self.logger.warning(f"Error closing the ToF camera: {e}")


def run(config: TofConfig) -> None:
    """Start metrics, spin the ToF bridge until shutdown, then close the camera.

    Args:
        config (TofConfig): Validated settings.
    """
    start_metrics_server(resolve_metrics_port(config.metrics_port), NODE_NAME)
    rclpy.init()
    node = Node(NODE_NAME)
    tof = TofBridge(node, config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        tof.close()
        node.destroy_node()
        rclpy.try_shutdown()
