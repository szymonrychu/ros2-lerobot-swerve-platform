"""ROS-free ground recording: accumulates head D435 clouds, IMU, head, RGB-D pairs and /ground/state lines.

The npz layout (version 3) keeps the cloud, IMU and head keys of the 2026-10-22 recordings, adds the /head_rgbd pairs
and the camera model of the D435 depth stream that produced the cloud (depth_K, depth_D, depth_size, depth_frame; no
depth frames). It is read by scripts/ground_replay_export.py. Poses are Pose7 = (x y z qx qy qz qw),
"<target> <- <source>".
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

NS_PER_S = 1_000_000_000
LAYOUT_VERSION = 3
POSE_FIELDS = 7
IMU_FIELDS = 11
HEAD_FIELDS = 3
INFO_K_FIELDS = 9
MM_PER_M = 1000.0
INT16_MIN = int(np.iinfo(np.int16).min)
INT16_MAX = int(np.iinfo(np.int16).max)

STREAM_CLOUD = "cloud"
STREAM_IMU = "imu"
STREAM_HEAD = "head"
STREAM_PAIR = "pair"
STREAM_STATE = "state"
STREAMS = (STREAM_CLOUD, STREAM_IMU, STREAM_HEAD, STREAM_PAIR, STREAM_STATE)
REASON_RANGE = "range"
REASON_SIZE = "size"

# 2026-10-22 keys
KEY_STAMPS_NS = "stamps_ns"
KEY_CLOUD_FRAME = "cloud_frame"
KEY_OFFSETS = "offsets"
KEY_POINTS_MM = "points_mm"
KEY_BASE_T_CLOUD = "base_T_cloud"
KEY_ODOM_T_BASE = "odom_T_base"
KEY_IMU = "imu"
KEY_BASE_T_IMU = "base_T_imu"
KEY_HEAD_JS = "head_js"
# new keys
KEY_LAYOUT_VERSION = "layout_version"
KEY_NAME = "name"
KEY_IMU_FRAME = "imu_frame"
KEY_DEPTH_K = "depth_K"
KEY_DEPTH_D = "depth_D"
KEY_DEPTH_SIZE = "depth_size"
KEY_DEPTH_FRAME = "depth_frame"
KEY_PAIR_STAMPS_NS = "pair_stamps_ns"
KEY_PAIR_JPEG = "pair_jpeg"
KEY_PAIR_JPEG_OFFSETS = "pair_jpeg_offsets"
KEY_PAIR_DEPTH_MM = "pair_depth_mm"
KEY_PAIR_BASE_T_RGB = "pair_base_T_rgb"
KEY_PAIR_ODOM_T_BASE = "pair_odom_T_base"
KEY_RGB_K = "rgb_K"
KEY_RGB_D = "rgb_D"
KEY_RGB_SIZE = "rgb_size"
KEY_RGB_FRAME = "rgb_frame"
KEY_STATE_STAMPS_NS = "state_stamps_ns"
KEY_STATE_JSON = "state_json"

Pose7 = tuple[float, ...]


def pose_row(pose: Pose7) -> list[float]:
    """Validate a pose and return it as a list.

    :param pose: x y z qx qy qz qw (Pose7).
    :return: the seven values as floats (list[float]).
    """
    if len(pose) != POSE_FIELDS:
        raise ValueError(f"pose needs {POSE_FIELDS} values, got {len(pose)}")
    return [float(v) for v in pose]


def pose_array(rows: list[list[float]]) -> np.ndarray:
    """Stack pose rows, N x 7 even when empty.

    :param rows: pose rows (list[list[float]]).
    :return: float64 array (N, 7).
    """
    return np.asarray(rows, dtype=np.float64).reshape(-1, POSE_FIELDS)


def stack_images(frames: list[np.ndarray]) -> np.ndarray:
    """Stack equally sized uint16 images, (0, 0, 0) when there are none.

    :param frames: images (list[np.ndarray], each H x W uint16).
    :return: uint16 array (T, H, W).
    """
    if not frames:
        return np.zeros((0, 0, 0), dtype=np.uint16)
    return np.stack(frames).astype(np.uint16, copy=False)


def intrinsics(k: object | None, d: object | None) -> tuple[np.ndarray, np.ndarray]:
    """Normalise camera intrinsics; unset ones are written as zeros / empty.

    :param k: 3x3 camera matrix or 9 values, or None.
    :param d: distortion coefficients, or None.
    :return: (K float64 (9,), D float64 (n,)).
    """
    k_arr = np.zeros(INFO_K_FIELDS) if k is None else np.asarray(k, dtype=np.float64).reshape(INFO_K_FIELDS)
    d_arr = np.zeros(0) if d is None else np.asarray(d, dtype=np.float64).ravel()
    return k_arr, d_arr


class GroundRecording:
    """One in-memory ground recording, written as a compressed npz."""

    def __init__(self, name: str, max_duration_s: float) -> None:
        """Create an empty recording.

        :param name: recording name (str).
        :param max_duration_s: window length from the first sample, seconds (float).
        """
        self.name = name
        self.max_duration_s = float(max_duration_s)
        self.first_ns: int | None = None
        self.cloud_frame = ""
        self.cloud_stamps: list[int] = []
        self.cloud_counts: list[int] = []
        self.cloud_points: list[np.ndarray] = []
        self.base_T_cloud: list[list[float]] = []
        self.odom_T_base: list[list[float]] = []
        self.imu_rows: list[list[float]] = []
        self.imu_frame = ""
        self.base_T_imu: list[float] = [0.0] * POSE_FIELDS
        self.head_rows: list[list[float]] = []
        self.depth_info: tuple[np.ndarray, np.ndarray, tuple[int, int], str] = (*intrinsics(None, None), (0, 0), "")
        self.rgb_info: tuple[np.ndarray, np.ndarray, tuple[int, int], str] = (*intrinsics(None, None), (0, 0), "")
        self.pair_stamps: list[int] = []
        self.pair_jpegs: list[bytes] = []
        self.pair_depth: list[np.ndarray] = []
        self.pair_base_T_rgb: list[list[float]] = []
        self.pair_odom_T_base: list[list[float]] = []
        self.state_stamps: list[int] = []
        self.state_lines: list[str] = []
        self.dropped_counts: dict[str, dict[str, int]] = {s: {} for s in STREAMS}

    def touch(self, stamp_ns: int) -> None:
        """Start the window at the first sample of any stream.

        :param stamp_ns: sample time, nanoseconds (int).
        """
        if self.first_ns is None:
            self.first_ns = int(stamp_ns)

    def drop(self, stream: str, reason: str, n: int = 1) -> None:
        """Count dropped samples.

        :param stream: stream name (str).
        :param reason: drop reason (str).
        :param n: how many samples (int).
        """
        self.dropped_counts[stream][reason] = self.dropped_counts[stream].get(reason, 0) + n

    def add_cloud(
        self, stamp_ns: int, frame_id: str, points_m: np.ndarray, base_T_cloud: Pose7, odom_T_base: Pose7
    ) -> None:
        """Add one point cloud; non-finite or int16-mm out-of-range points are dropped and counted.

        :param stamp_ns: cloud stamp, ns (int).
        :param frame_id: cloud frame (str).
        :param points_m: points in metres (np.ndarray, N x 3 float32).
        :param base_T_cloud: base_link <- cloud frame (Pose7).
        :param odom_T_base: odom <- base_link (Pose7).
        """
        self.touch(stamp_ns)
        pts = np.asarray(points_m, dtype=np.float64).reshape(-1, 3)
        with np.errstate(invalid="ignore"):
            mm = np.rint(pts * MM_PER_M)
            ok = np.isfinite(mm).all(axis=1) & (mm >= INT16_MIN).all(axis=1) & (mm <= INT16_MAX).all(axis=1)
        if not ok.all():
            self.drop(STREAM_CLOUD, REASON_RANGE, int((~ok).sum()))
        self.cloud_frame = frame_id
        self.cloud_stamps.append(int(stamp_ns))
        self.cloud_counts.append(int(ok.sum()))
        self.cloud_points.append(mm[ok].astype(np.int16))
        self.base_T_cloud.append(pose_row(base_T_cloud))
        self.odom_T_base.append(pose_row(odom_T_base))

    def add_imu(
        self,
        stamp_ns: int,
        orientation: tuple[float, ...],
        linear_acceleration: tuple[float, ...],
        angular_velocity: tuple[float, ...],
    ) -> None:
        """Add one IMU sample.

        :param stamp_ns: stamp, ns (int).
        :param orientation: quaternion x y z w (tuple[float, ...]).
        :param linear_acceleration: x y z, m/s^2 (tuple[float, ...]).
        :param angular_velocity: x y z, rad/s (tuple[float, ...]).
        """
        self.touch(stamp_ns)
        self.imu_rows.append([float(stamp_ns), *orientation, *linear_acceleration, *angular_velocity])

    def set_imu_frame(self, frame: str, base_T_imu: Pose7) -> None:
        """Set the IMU frame and its mounting pose.

        :param frame: IMU frame id (str).
        :param base_T_imu: base_link <- IMU frame (Pose7).
        """
        self.imu_frame = frame
        self.base_T_imu = pose_row(base_T_imu)

    def add_head_js(self, stamp_ns: int, pan_rad: float, tilt_rad: float) -> None:
        """Add one head joint sample.

        :param stamp_ns: stamp, ns (int).
        :param pan_rad: pan angle, rad (float).
        :param tilt_rad: tilt angle, rad (float).
        """
        self.touch(stamp_ns)
        self.head_rows.append([float(stamp_ns), float(pan_rad), float(tilt_rad)])

    def set_depth_info(self, k: object, d: object, width: int, height: int, frame_id: str) -> None:
        """Set the camera model of the D435 depth stream that produced the cloud.

        :param k: camera matrix (3x3 or 9 values).
        :param d: distortion coefficients.
        :param width: image width, px (int).
        :param height: image height, px (int).
        :param frame_id: optical frame id (str).
        """
        self.depth_info = (*intrinsics(k, d), (int(width), int(height)), frame_id)

    def add_pair(
        self, stamp_ns: int, rgb_jpeg: bytes, depth_mm: np.ndarray, base_T_rgb: Pose7, odom_T_base: Pose7
    ) -> None:
        """Add one RGB + depth pair; a depth size differing from the first pair is dropped.

        :param stamp_ns: stamp, ns (int).
        :param rgb_jpeg: JPEG-encoded RGB image (bytes).
        :param depth_mm: depth, mm (np.ndarray, h x w uint16).
        :param base_T_rgb: base_link <- RGB frame (Pose7).
        :param odom_T_base: odom <- base_link (Pose7).
        """
        self.touch(stamp_ns)
        depth = np.asarray(depth_mm, dtype=np.uint16)
        if depth.ndim != 2 or (self.pair_depth and depth.shape != self.pair_depth[0].shape):
            self.drop(STREAM_PAIR, REASON_SIZE)
            return
        self.pair_stamps.append(int(stamp_ns))
        self.pair_jpegs.append(bytes(rgb_jpeg))
        self.pair_depth.append(depth)
        self.pair_base_T_rgb.append(pose_row(base_T_rgb))
        self.pair_odom_T_base.append(pose_row(odom_T_base))

    def set_rgb_info(self, k: object, d: object, width: int, height: int, frame_id: str) -> None:
        """Set the RGB camera info.

        :param k: camera matrix (3x3 or 9 values).
        :param d: distortion coefficients.
        :param width: image width, px (int).
        :param height: image height, px (int).
        :param frame_id: optical frame id (str).
        """
        self.rgb_info = (*intrinsics(k, d), (int(width), int(height)), frame_id)

    def add_state_line(self, stamp_ns: int, json_text: str) -> None:
        """Add one /ground/state string.

        :param stamp_ns: stamp, ns (int).
        :param json_text: the state JSON text (str).
        """
        self.touch(stamp_ns)
        self.state_stamps.append(int(stamp_ns))
        self.state_lines.append(json_text)

    def elapsed_s(self, now_ns: int) -> float:
        """Seconds since the first sample.

        :param now_ns: current time, ns (int).
        :return: elapsed seconds, 0.0 before any sample (float).
        """
        return 0.0 if self.first_ns is None else (int(now_ns) - self.first_ns) / NS_PER_S

    def done(self, now_ns: int) -> bool:
        """Whether the window from the first sample has passed.

        :param now_ns: current time, ns (int).
        :return: True once elapsed >= max_duration_s (bool).
        """
        return self.first_ns is not None and self.elapsed_s(now_ns) >= self.max_duration_s

    def counts(self) -> dict[str, int]:
        """Samples held per stream.

        :return: stream name to sample count (dict[str, int]).
        """
        return {
            STREAM_CLOUD: len(self.cloud_stamps),
            STREAM_IMU: len(self.imu_rows),
            STREAM_HEAD: len(self.head_rows),
            STREAM_PAIR: len(self.pair_stamps),
            STREAM_STATE: len(self.state_stamps),
        }

    def dropped(self) -> dict[str, dict[str, int]]:
        """Dropped samples per stream and reason (cloud/range counts points, the others frames).

        :return: stream to reason to count (dict[str, dict[str, int]]).
        """
        return {s: dict(r) for s, r in self.dropped_counts.items()}

    def arrays(self) -> dict[str, np.ndarray]:
        """Build the npz arrays.

        :return: key to array (dict[str, np.ndarray]).
        """
        depth_k, depth_d, depth_size, depth_frame = self.depth_info
        rgb_k, rgb_d, rgb_size, rgb_frame = self.rgb_info
        jpeg_sizes = [len(j) for j in self.pair_jpegs]
        return {
            KEY_LAYOUT_VERSION: np.asarray(LAYOUT_VERSION, dtype=np.int64),
            KEY_NAME: np.asarray(self.name),
            KEY_STAMPS_NS: np.asarray(self.cloud_stamps, dtype=np.int64),
            KEY_CLOUD_FRAME: np.asarray(self.cloud_frame),
            KEY_OFFSETS: np.concatenate([[0], np.cumsum(self.cloud_counts)]).astype(np.int64),
            KEY_POINTS_MM: np.concatenate(self.cloud_points) if self.cloud_points else np.zeros((0, 3), dtype=np.int16),
            KEY_BASE_T_CLOUD: pose_array(self.base_T_cloud),
            KEY_ODOM_T_BASE: pose_array(self.odom_T_base),
            KEY_IMU: np.asarray(self.imu_rows, dtype=np.float64).reshape(-1, IMU_FIELDS),
            KEY_IMU_FRAME: np.asarray(self.imu_frame),
            KEY_BASE_T_IMU: np.asarray(self.base_T_imu, dtype=np.float64),
            KEY_HEAD_JS: np.asarray(self.head_rows, dtype=np.float64).reshape(-1, HEAD_FIELDS),
            KEY_DEPTH_K: depth_k,
            KEY_DEPTH_D: depth_d,
            KEY_DEPTH_SIZE: np.asarray(depth_size, dtype=np.int64),
            KEY_DEPTH_FRAME: np.asarray(depth_frame),
            KEY_PAIR_STAMPS_NS: np.asarray(self.pair_stamps, dtype=np.int64),
            KEY_PAIR_JPEG: np.frombuffer(b"".join(self.pair_jpegs), dtype=np.uint8),
            KEY_PAIR_JPEG_OFFSETS: np.concatenate([[0], np.cumsum(jpeg_sizes)]).astype(np.int64),
            KEY_PAIR_DEPTH_MM: stack_images(self.pair_depth),
            KEY_PAIR_BASE_T_RGB: pose_array(self.pair_base_T_rgb),
            KEY_PAIR_ODOM_T_BASE: pose_array(self.pair_odom_T_base),
            KEY_RGB_K: rgb_k,
            KEY_RGB_D: rgb_d,
            KEY_RGB_SIZE: np.asarray(rgb_size, dtype=np.int64),
            KEY_RGB_FRAME: np.asarray(rgb_frame),
            KEY_STATE_STAMPS_NS: np.asarray(self.state_stamps, dtype=np.int64),
            KEY_STATE_JSON: np.asarray(self.state_lines, dtype=str),
        }

    def write(self, path: Path | str) -> int:
        """Write the recording as a compressed npz.

        :param path: output file (Path | str).
        :return: bytes written (int).
        """
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("wb") as handle:
            np.savez_compressed(handle, **self.arrays())
        return out.stat().st_size
