#!/usr/bin/env python3
"""Trim one live ground recording (npz) into the replay binary the ground_estimator_cpp test_replay gtest reads.

Input keys (numpy npz, as recorded on 2026-10-22): stamps_ns (N, int64), cloud_frame (0-d str), offsets (N+1, int64,
slices points_mm per cloud), points_mm (P x 3, int16 x y z in the cloud frame, mm), base_T_cloud (N x 7) and
odom_T_base (N x 7) as x y z qx qy qz qw at the cloud stamps, imu (M x 11: stamp_ns, orientation x y z w,
linear_acceleration x y z, angular_velocity x y z, frame imu_link), base_T_imu (7), head_js (unused).

Kept: the clouds and IMU samples stamped in [first cloud + START_S, first cloud + START_S + DURATION_S).

Binary layout, little-endian, version 1:
    magic          8 bytes  b"GRNDRPLY"
    version        uint32   1
    frame_len      uint32   then frame_len bytes of the UTF-8 cloud frame id
    cloud_count    uint32
    per cloud      int64 stamp_ns, float64[7] base_T_cloud, float64[7] odom_T_base, uint32 point_count,
                   int16[point_count * 3] x y z mm (cloud frame)
    imu_count      uint32
    per IMU sample int64 stamp_ns, float64[10] orientation x y z w, linear_acceleration x y z, angular_velocity x y z
    base_T_imu     float64[7] x y z qx qy qz qw

Version 2 (input with layout_version 2, the recorder npz of nodes/web_ui/web_ui/ground_recorder.py) writes the same
sections with header version 2, then the guidance section:
    rgb model      float64[9] K, float64[5] D, uint32 width, uint32 height, uint32 frame_len + UTF-8 frame id
    tof model      the same fields
    pair_count     uint32
    per pair       int64 stamp_ns, float64[7] base_T_rgb, float64[7] odom_T_base, uint32 jpeg_len + JPEG bytes,
                   uint32 png_len + PNG bytes (the upsampled depth, uint16 mm, lossless cv2.imencode(".png"))
Pairs inside the same trim window as the clouds are kept; pairs=False writes pair_count 0. A file above
MAX_FIXTURE_BYTES (MAX_PAIR_FIXTURE_BYTES with pairs) is refused and nothing is written.

Usage:
    uv run python scripts/ground_replay_export.py <input.npz> <output.bin>   (one file, constants below)
    uv run python scripts/ground_replay_export.py                            (everything in ground_replay_scenes.yaml)
"""

import argparse
import logging
import struct
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml
from pydantic import BaseModel, ConfigDict

LOG = logging.getLogger("ground_replay_export")

START_S = 5.0  # window start after the first cloud, seconds
DURATION_S = 15.0  # window length, seconds
MAGIC = b"GRNDRPLY"
VERSION = 1  # the layout without guidance pairs
VERSION_PAIRS = 2  # the layout with the guidance section
LAYOUT_VERSION_KEY = "layout_version"
MAX_FIXTURE_BYTES = 3_000_000  # cap of an export without pairs
MAX_PAIR_FIXTURE_BYTES = 6_000_000  # cap of an export with pairs
SCENES_CONFIG = Path(__file__).with_name("ground_replay_scenes.yaml")
CAMERA_K_FIELDS = 9
CAMERA_D_FIELDS = 5
PNG_EXT = ".png"
RANSAC_INLIER_M = 0.01  # floor plane inlier distance
RANSAC_ITERATIONS = 200
RANSAC_MAX_POINTS = 50_000
RANSAC_SEED = 0
MM_PER_M = 1000.0
TRUTH_KEYS = ("board", "ramp", "empty", "box")
NS_PER_S = 1_000_000_000
POSE_FIELDS = 7  # x y z qx qy qz qw
IMU_FIELDS = 11  # stamp_ns plus 10 values
U32 = struct.Struct("<I")
I64 = struct.Struct("<q")
POSE_DTYPE = np.dtype("<f8")
POINT_DTYPE = np.dtype("<i2")
CLOUD_HEADER = np.dtype([("stamp_ns", "<i8"), ("base_T_cloud", "<f8", (7,)), ("odom_T_base", "<f8", (7,))])
IMU_RECORD = np.dtype([("stamp_ns", "<i8"), ("values", "<f8", (IMU_FIELDS - 1,))])
PAIR_HEADER = np.dtype([("stamp_ns", "<i8"), ("base_T_rgb", "<f8", (7,)), ("odom_T_base", "<f8", (7,))])


@dataclass
class CameraModel:
    """A camera model of the guidance section.

    Attributes:
        K: camera matrix, row-major (np.ndarray, 9).
        D: distortion coefficients (np.ndarray, 5).
        width: image width, px (int).
        height: image height, px (int).
        frame_id: optical frame id (str).
    """

    K: np.ndarray
    D: np.ndarray
    width: int
    height: int
    frame_id: str


@dataclass
class ReplayPair:
    """One RGB-D pair.

    Attributes:
        stamp_ns: pair stamp (int).
        base_T_rgb: base_link <- RGB frame, x y z qx qy qz qw (np.ndarray, 7).
        odom_T_base: odom <- base_link at the stamp (np.ndarray, 7).
        jpeg: JPEG-encoded RGB image (bytes).
        depth_mm: upsampled depth, mm (np.ndarray, H x W uint16).
    """

    stamp_ns: int
    base_T_rgb: np.ndarray
    odom_T_base: np.ndarray
    jpeg: bytes
    depth_mm: np.ndarray


class FixtureEntry(BaseModel):
    """One fixture of the scenes config."""

    model_config = ConfigDict(extra="forbid")

    input: Path
    output: Path
    start_s: float = START_S
    duration_s: float = DURATION_S
    pairs: bool = True


class TofFramesEntry(BaseModel):
    """One ToF frame subset of the scenes config."""

    model_config = ConfigDict(extra="forbid")

    input: list[Path]
    output: Path
    every_n: int
    max_frames: int
    truth_yaml: Path


class ScenesConfig(BaseModel):
    """scripts/ground_replay_scenes.yaml."""

    model_config = ConfigDict(extra="forbid")

    fixtures: list[FixtureEntry] = []
    tof_frames: list[TofFramesEntry] = []


@dataclass
class Replay:
    """A decoded replay file.

    Attributes:
        frame_id: frame of the clouds (str).
        stamps_ns: cloud stamps (np.ndarray, N, int64).
        base_T_cloud: base_link <- cloud frame per cloud (np.ndarray, N x 7, x y z qx qy qz qw).
        odom_T_base: odom <- base_link per cloud (np.ndarray, N x 7).
        offsets: slices of points_mm per cloud (np.ndarray, N + 1, int64).
        points_mm: x y z of every cloud, mm (np.ndarray, P x 3, int16).
        imu: IMU samples in the npz layout (np.ndarray, M x 11, float64; column 0 the stamp in ns).
        base_T_imu: base_link <- imu_link (np.ndarray, 7).
        version: file version, 1 or 2 (int).
        rgb_model: RGB camera model, None in version 1 (CameraModel | None).
        tof_model: ToF camera model, None in version 1 (CameraModel | None).
        pairs: RGB-D pairs, empty in version 1 (list[ReplayPair]).
    """

    frame_id: str
    stamps_ns: np.ndarray
    base_T_cloud: np.ndarray
    odom_T_base: np.ndarray
    offsets: np.ndarray
    points_mm: np.ndarray
    imu: np.ndarray
    base_T_imu: np.ndarray
    version: int = VERSION
    rgb_model: CameraModel | None = None
    tof_model: CameraModel | None = None
    pairs: list[ReplayPair] = field(default_factory=list)

    def cloud(self, i: int) -> np.ndarray:
        """Return the points of cloud i.

        :param i: cloud index (int).
        :return: the cloud's x y z in mm (np.ndarray, n x 3, int16).
        """
        return self.points_mm[self.offsets[i] : self.offsets[i + 1]]


def camera_model(arrays: dict[str, np.ndarray], prefix: str, frame_key: str) -> CameraModel:
    """Read one camera model of a layout-2 recording; an unset or shorter D is zero-padded to 5.

    :param arrays: the recording's arrays (dict[str, np.ndarray]).
    :param prefix: "rgb" or "tof" (str).
    :param frame_key: key of the frame id, "rgb_frame" or "tof_frame" (str).
    :return: the model (CameraModel).
    :raises ValueError: when K is not 9 values or D has more than 5.
    """
    k = np.asarray(arrays[f"{prefix}_K"], dtype=np.float64).ravel()
    d = np.asarray(arrays[f"{prefix}_D"], dtype=np.float64).ravel()
    if len(k) != CAMERA_K_FIELDS or len(d) > CAMERA_D_FIELDS:
        raise ValueError(f"{prefix} model needs K of {CAMERA_K_FIELDS} and D of at most {CAMERA_D_FIELDS} values")
    width, height = (int(v) for v in np.asarray(arrays[f"{prefix}_size"]).ravel())
    return CameraModel(
        K=k,
        D=np.concatenate([d, np.zeros(CAMERA_D_FIELDS - len(d))]),
        width=width,
        height=height,
        frame_id=str(arrays[frame_key]),
    )


def trim_pairs(arrays: dict[str, np.ndarray], lo: int, hi: int) -> list[ReplayPair]:
    """Keep the RGB-D pairs stamped in [lo, hi).

    :param arrays: the layout-2 recording's arrays (dict[str, np.ndarray]).
    :param lo: window start, ns (int).
    :param hi: window end, exclusive, ns (int).
    :return: the pairs in the window (list[ReplayPair]).
    """
    stamps = np.asarray(arrays["pair_stamps_ns"], dtype=np.int64)
    jpeg = np.asarray(arrays["pair_jpeg"], dtype=np.uint8)
    offsets = np.asarray(arrays["pair_jpeg_offsets"], dtype=np.int64)
    depth = np.asarray(arrays["pair_depth_mm"], dtype=np.uint16)
    base_T_rgb = np.asarray(arrays["pair_base_T_rgb"], dtype=np.float64).reshape(-1, POSE_FIELDS)
    odom_T_base = np.asarray(arrays["pair_odom_T_base"], dtype=np.float64).reshape(-1, POSE_FIELDS)
    return [
        ReplayPair(
            stamp_ns=int(stamps[i]),
            base_T_rgb=base_T_rgb[i],
            odom_T_base=odom_T_base[i],
            jpeg=jpeg[offsets[i] : offsets[i + 1]].tobytes(),
            depth_mm=depth[i],
        )
        for i in np.flatnonzero((stamps >= lo) & (stamps < hi))
    ]


def trim(
    arrays: dict[str, np.ndarray], start_s: float = START_S, duration_s: float = DURATION_S, pairs: bool = True
) -> Replay:
    """Keep the clouds, IMU samples and pairs stamped inside the window after the first cloud.

    :param arrays: the recording's arrays (dict[str, np.ndarray], keys as in the module docstring; a layout_version
        of 2 adds the guidance keys of ground_recorder.py).
    :param start_s: window start after the first cloud, seconds (float).
    :param duration_s: window length, seconds (float).
    :param pairs: keep the RGB-D pairs of a layout-2 recording (bool); False writes a pair count of 0.
    :return: the trimmed recording, offsets rebuilt (Replay), version 2 for a layout-2 input.
    """
    stamps = np.asarray(arrays["stamps_ns"], dtype=np.int64)
    offsets = np.asarray(arrays["offsets"], dtype=np.int64)
    points = np.asarray(arrays["points_mm"], dtype=np.int16)
    imu = np.asarray(arrays["imu"], dtype=np.float64).reshape(-1, IMU_FIELDS)
    lo = int(stamps[0]) + int(round(start_s * NS_PER_S))
    hi = lo + int(round(duration_s * NS_PER_S))
    kept = np.flatnonzero((stamps >= lo) & (stamps < hi))
    counts = offsets[kept + 1] - offsets[kept]
    chunks = [points[offsets[i] : offsets[i + 1]] for i in kept]
    imu_stamps = imu[:, 0].astype(np.int64)
    replay = Replay(
        frame_id=str(arrays["cloud_frame"]),
        stamps_ns=stamps[kept],
        base_T_cloud=np.asarray(arrays["base_T_cloud"], dtype=np.float64)[kept],
        odom_T_base=np.asarray(arrays["odom_T_base"], dtype=np.float64)[kept],
        offsets=np.concatenate([[0], np.cumsum(counts)]).astype(np.int64),
        points_mm=np.concatenate(chunks) if chunks else np.zeros((0, 3), dtype=np.int16),
        imu=imu[(imu_stamps >= lo) & (imu_stamps < hi)],
        base_T_imu=np.asarray(arrays["base_T_imu"], dtype=np.float64).reshape(POSE_FIELDS),
    )
    if int(arrays.get(LAYOUT_VERSION_KEY, 1)) >= VERSION_PAIRS:
        replay.version = VERSION_PAIRS
        replay.rgb_model = camera_model(arrays, "rgb", "rgb_frame")
        replay.tof_model = camera_model(arrays, "tof", "tof_frame")
        replay.pairs = trim_pairs(arrays, lo, hi) if pairs else []
    return replay


def encode_model(model: CameraModel) -> bytes:
    """Serialise a camera model.

    :param model: the model (CameraModel).
    :return: K, D, width, height, frame id as in the module docstring (bytes).
    """
    frame = model.frame_id.encode("utf-8")
    return b"".join(
        [
            model.K.astype(POSE_DTYPE).tobytes(),
            model.D.astype(POSE_DTYPE).tobytes(),
            U32.pack(model.width),
            U32.pack(model.height),
            U32.pack(len(frame)),
            frame,
        ]
    )


def encode_pair(pair: ReplayPair) -> bytes:
    """Serialise one pair, the depth as a lossless 16-bit PNG.

    :param pair: the pair (ReplayPair).
    :return: the pair record of the module docstring (bytes).
    :raises ValueError: when the depth is not a 2-D uint16 image or PNG encoding fails.
    """
    if pair.depth_mm.dtype != np.uint16 or pair.depth_mm.ndim != 2:
        raise ValueError("pair depth must be a 2-D uint16 image")
    ok, png = cv2.imencode(PNG_EXT, pair.depth_mm)
    if not ok:
        raise ValueError("cv2.imencode failed for the pair depth")
    header = np.zeros(1, dtype=PAIR_HEADER)
    header["stamp_ns"] = pair.stamp_ns
    header["base_T_rgb"] = pair.base_T_rgb
    header["odom_T_base"] = pair.odom_T_base
    return b"".join([header.tobytes(), U32.pack(len(pair.jpeg)), pair.jpeg, U32.pack(png.size), png.tobytes()])


def encode(replay: Replay) -> bytes:
    """Serialise a replay in the binary layout of the module docstring.

    :param replay: the recording to write (Replay); version 2 needs both camera models.
    :return: the file content (bytes).
    """
    frame = replay.frame_id.encode("utf-8")
    parts = [MAGIC, U32.pack(replay.version), U32.pack(len(frame)), frame, U32.pack(len(replay.stamps_ns))]
    for i, stamp in enumerate(replay.stamps_ns):
        header = np.zeros(1, dtype=CLOUD_HEADER)
        header["stamp_ns"] = stamp
        header["base_T_cloud"] = replay.base_T_cloud[i]
        header["odom_T_base"] = replay.odom_T_base[i]
        cloud = replay.cloud(i)
        parts += [header.tobytes(), U32.pack(len(cloud)), cloud.astype(POINT_DTYPE).tobytes()]
    imu = np.zeros(len(replay.imu), dtype=IMU_RECORD)
    imu["stamp_ns"] = replay.imu[:, 0].astype(np.int64)
    imu["values"] = replay.imu[:, 1:]
    parts += [U32.pack(len(imu)), imu.tobytes(), replay.base_T_imu.astype(POSE_DTYPE).tobytes()]
    if replay.version == VERSION_PAIRS:
        if replay.rgb_model is None or replay.tof_model is None:
            raise ValueError("a version 2 replay needs the RGB and ToF models")
        parts += [encode_model(replay.rgb_model), encode_model(replay.tof_model), U32.pack(len(replay.pairs))]
        parts += [encode_pair(pair) for pair in replay.pairs]
    return b"".join(parts)


def decode(data: bytes) -> Replay:
    """Parse the binary layout of the module docstring (version 1 or 2).

    :param data: the file content (bytes).
    :return: the decoded recording (Replay).
    :raises ValueError: on a wrong magic or version, a truncated file, or a PNG that is not 16-bit.
    """
    if data[: len(MAGIC)] != MAGIC:
        raise ValueError(f"not a ground replay file: magic {data[: len(MAGIC)]!r}, expected {MAGIC!r}")
    at = len(MAGIC)

    def take(n: int) -> bytes:
        nonlocal at
        if at + n > len(data):
            raise ValueError(f"truncated ground replay file: need {n} bytes at {at}, have {len(data) - at}")
        chunk = data[at : at + n]
        at += n
        return chunk

    def take_model() -> CameraModel:
        k = np.frombuffer(take(CAMERA_K_FIELDS * POSE_DTYPE.itemsize), dtype=POSE_DTYPE).astype(np.float64)
        d = np.frombuffer(take(CAMERA_D_FIELDS * POSE_DTYPE.itemsize), dtype=POSE_DTYPE).astype(np.float64)
        (width,) = U32.unpack(take(U32.size))
        (height,) = U32.unpack(take(U32.size))
        (length,) = U32.unpack(take(U32.size))
        return CameraModel(K=k, D=d, width=width, height=height, frame_id=take(length).decode("utf-8"))

    (version,) = U32.unpack(take(U32.size))
    if version not in (VERSION, VERSION_PAIRS):
        raise ValueError(f"unsupported ground replay version {version}, expected {VERSION} or {VERSION_PAIRS}")
    (frame_len,) = U32.unpack(take(U32.size))
    frame_id = take(frame_len).decode("utf-8")
    (clouds,) = U32.unpack(take(U32.size))
    stamps = np.zeros(clouds, dtype=np.int64)
    base_T_cloud = np.zeros((clouds, POSE_FIELDS))
    odom_T_base = np.zeros((clouds, POSE_FIELDS))
    offsets = np.zeros(clouds + 1, dtype=np.int64)
    chunks: list[np.ndarray] = []
    for i in range(clouds):
        header = np.frombuffer(take(CLOUD_HEADER.itemsize), dtype=CLOUD_HEADER)[0]
        stamps[i] = header["stamp_ns"]
        base_T_cloud[i] = header["base_T_cloud"]
        odom_T_base[i] = header["odom_T_base"]
        (count,) = U32.unpack(take(U32.size))
        chunks.append(np.frombuffer(take(count * 3 * POINT_DTYPE.itemsize), dtype=POINT_DTYPE).reshape(count, 3))
        offsets[i + 1] = offsets[i] + count
    (imu_count,) = U32.unpack(take(U32.size))
    records = np.frombuffer(take(imu_count * IMU_RECORD.itemsize), dtype=IMU_RECORD)
    imu = np.zeros((imu_count, IMU_FIELDS))
    imu[:, 0] = records["stamp_ns"].astype(np.float64)
    imu[:, 1:] = records["values"]
    base_T_imu = np.frombuffer(take(POSE_FIELDS * POSE_DTYPE.itemsize), dtype=POSE_DTYPE).astype(np.float64)
    rgb_model = tof_model = None
    pairs: list[ReplayPair] = []
    if version == VERSION_PAIRS:
        rgb_model, tof_model = take_model(), take_model()
        (pair_count,) = U32.unpack(take(U32.size))
        for _ in range(pair_count):
            header = np.frombuffer(take(PAIR_HEADER.itemsize), dtype=PAIR_HEADER)[0]
            (jpeg_len,) = U32.unpack(take(U32.size))
            jpeg = take(jpeg_len)
            (png_len,) = U32.unpack(take(U32.size))
            depth = cv2.imdecode(np.frombuffer(take(png_len), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            if depth is None or depth.dtype != np.uint16 or depth.ndim != 2:
                raise ValueError("pair depth PNG does not decode to a 16-bit single channel image")
            pairs.append(
                ReplayPair(
                    stamp_ns=int(header["stamp_ns"]),
                    base_T_rgb=np.array(header["base_T_rgb"]),
                    odom_T_base=np.array(header["odom_T_base"]),
                    jpeg=jpeg,
                    depth_mm=depth,
                )
            )
    if at != len(data):
        raise ValueError(f"ground replay file has {len(data) - at} trailing bytes")
    return Replay(
        frame_id=frame_id,
        stamps_ns=stamps,
        base_T_cloud=base_T_cloud,
        odom_T_base=odom_T_base,
        offsets=offsets,
        points_mm=np.concatenate(chunks).astype(np.int16) if chunks else np.zeros((0, 3), dtype=np.int16),
        imu=imu,
        base_T_imu=base_T_imu,
        version=version,
        rgb_model=rgb_model,
        tof_model=tof_model,
        pairs=pairs,
    )


def read_replay(path: Path) -> Replay:
    """Read a replay binary (version 1 or 2).

    :param path: the .bin file (Path).
    :return: the decoded recording, pairs included (Replay).
    :raises ValueError: on a wrong magic or version, or a truncated file.
    """
    return decode(Path(path).read_bytes())


def export(
    input_path: Path,
    output_path: Path,
    start_s: float = START_S,
    duration_s: float = DURATION_S,
    pairs: bool = True,
) -> Replay:
    """Trim a recording to the window and write it as a replay binary.

    :param input_path: the recording npz (Path).
    :param output_path: the .bin to write (Path).
    :param start_s: window start after the first cloud, seconds (float).
    :param duration_s: window length, seconds (float).
    :param pairs: write the RGB-D pairs of a layout-2 recording (bool).
    :return: what was written (Replay).
    :raises ValueError: when the file would exceed MAX_FIXTURE_BYTES (MAX_PAIR_FIXTURE_BYTES with pairs); nothing is
        written then.
    """
    with np.load(input_path, allow_pickle=False) as npz:
        replay = trim({key: npz[key] for key in npz.files}, start_s, duration_s, pairs)
    data = encode(replay)
    cap = MAX_PAIR_FIXTURE_BYTES if replay.pairs else MAX_FIXTURE_BYTES
    if len(data) > cap:
        raise ValueError(f"{input_path}: replay would be {len(data)} bytes, above the {cap} byte cap; nothing written")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(data)
    LOG.info(
        "%s: %d clouds, %d points, %d IMU samples, %d pairs -> %s (%d bytes)",
        input_path,
        len(replay.stamps_ns),
        len(replay.points_mm),
        len(replay.imu),
        len(replay.pairs),
        output_path,
        len(data),
    )
    return replay


def load_scenes(path: Path = SCENES_CONFIG) -> ScenesConfig:
    """Read and validate the scenes config.

    :param path: the YAML (Path), default scripts/ground_replay_scenes.yaml.
    :return: the validated config (ScenesConfig).
    :raises pydantic.ValidationError: on an unknown or missing key.
    """
    return ScenesConfig.model_validate(yaml.safe_load(Path(path).read_text()) or {})


def export_all(config: Path = SCENES_CONFIG) -> None:
    """Export every fixture and ToF frame subset of a scenes config.

    :param config: the scenes YAML (Path).
    """
    scenes = load_scenes(config)
    for entry in scenes.fixtures:
        export(entry.input, entry.output, entry.start_s, entry.duration_s, entry.pairs)
    for frames in scenes.tof_frames:
        export_tof_frames(frames.input, frames.output, frames.every_n, frames.max_frames, frames.truth_yaml)


def truth_polygon(truth: dict, key: str) -> np.ndarray:
    """Read one truth polygon as 4 xy corners.

    :param truth: the parsed truth YAML (dict); each polygon a list of 4 [x, y] corners.
    :param key: board, ramp, empty or box (str).
    :return: corners in base_link m (np.ndarray, 4 x 2).
    :raises ValueError: when the polygon is missing or is not 4 xy corners.
    """
    if key not in truth:
        raise ValueError(f"truth YAML has no '{key}' polygon")
    corners = np.asarray(truth[key], dtype=np.float64)
    if corners.shape != (4, 2):
        raise ValueError(f"truth '{key}' must be 4 xy corners, got shape {corners.shape}")
    return corners


def inside_convex(points_xy: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Test points against a convex polygon, either winding.

    :param points_xy: points (np.ndarray, N x 2).
    :param polygon: corners in order (np.ndarray, K x 2).
    :return: True where a point lies inside or on an edge (np.ndarray, N bool).
    """
    edges = np.roll(polygon, -1, axis=0) - polygon
    rel = points_xy[:, None, :] - polygon[None, :, :]
    cross = edges[None, :, 0] * rel[..., 1] - edges[None, :, 1] * rel[..., 0]
    return (cross >= 0).all(axis=1) | (cross <= 0).all(axis=1)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation matrix of a quaternion x y z w.

    :param q: quaternion (np.ndarray, 4).
    :return: rotation (np.ndarray, 3 x 3).
    """
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def depth_points_in_base(depth_mm: np.ndarray, k: np.ndarray, d: np.ndarray, base_T: np.ndarray) -> np.ndarray:
    """Back-project a depth image (z along the optical axis) into base_link.

    :param depth_mm: depth, 0 for no return (np.ndarray, H x W uint16).
    :param k: camera matrix (np.ndarray, 9).
    :param d: distortion coefficients (np.ndarray, 5).
    :param base_T: base_link <- optical frame x y z qx qy qz qw (np.ndarray, 7).
    :return: points of the valid pixels, m (np.ndarray, N x 3).
    """
    v, u = np.nonzero(depth_mm)
    z = depth_mm[v, u].astype(np.float64) / MM_PER_M
    pixels = np.stack([u, v], axis=1).astype(np.float64).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pixels, k.reshape(3, 3), d).reshape(-1, 2)
    cam = np.column_stack([norm * z[:, None], z])
    return cam @ quat_to_matrix(base_T[3:]).T + base_T[:3]


def fit_plane(points: np.ndarray) -> np.ndarray:
    """RANSAC plane (inlier distance RANSAC_INLIER_M) refined by a least-squares fit of the inliers.

    :param points: points, m (np.ndarray, N x 3).
    :return: nx ny nz d with n.p + d = 0, unit n with nz >= 0 (np.ndarray, 4).
    :raises ValueError: with fewer than 3 points.
    """
    if len(points) < 3:
        raise ValueError(f"floor plane needs at least 3 points, got {len(points)}")
    rng = np.random.default_rng(RANSAC_SEED)
    if len(points) > RANSAC_MAX_POINTS:
        points = points[rng.choice(len(points), RANSAC_MAX_POINTS, replace=False)]
    best = np.zeros(len(points), dtype=bool)
    for _ in range(RANSAC_ITERATIONS):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        inliers = np.abs((points - a) @ (normal / norm)) <= RANSAC_INLIER_M
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 3:
        raise ValueError("no floor plane found among the points")
    inlier_points = points[best]
    centre = inlier_points.mean(axis=0)
    normal = np.linalg.svd(inlier_points - centre)[2][-1]
    if normal[2] < 0:
        normal = -normal
    return np.append(normal, -normal @ centre)


def export_tof_frames(
    input_npzs: list[Path], output_npz: Path, every_n: int, max_frames: int, truth_yaml: Path
) -> None:
    """Write a compressed npz of ToF depth + amplitude frames with the scene truth, for the amplitude gate and the
    flying-pixel retune tests.

    Every every_n-th frame of each input, at most max_frames // len(input_npzs) per input (the remainder goes to the
    first inputs), so every scene is present. Keys: tof_depth_mm, tof_amplitude (F x H x W uint16), tof_base_T (F x 7),
    tof_stamps_ns (F), tof_K (9), tof_D (5), tof_size (2), scene (F str, the recording name), truth_board,
    truth_ramp, truth_empty, truth_box (4 x 2, base_link m), box_height_m (scalar) and floor_plane (4, n.p + d = 0 in
    base_link: RANSAC at RANSAC_INLIER_M then least squares over the depth points whose xy lies in truth_empty).

    :param input_npzs: layout-2 recordings with tof_* keys (list[Path]).
    :param output_npz: the npz to write (Path).
    :param every_n: keep every n-th frame (int).
    :param max_frames: frame cap over all inputs (int).
    :param truth_yaml: truth YAML with board, ramp, empty, box (4 xy corners each) and box_height_m (Path).
    :raises ValueError: on inputs with differing camera models or no frames.
    """
    truth = yaml.safe_load(Path(truth_yaml).read_text())
    polygons = {key: truth_polygon(truth, key) for key in TRUTH_KEYS}
    quota, extra = divmod(max_frames, len(input_npzs))
    depth, amp, base_t, stamps, scenes = [], [], [], [], []
    model: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
    for n, path in enumerate(input_npzs):
        with np.load(path, allow_pickle=False) as npz:
            this = (npz["tof_K"], npz["tof_D"], npz["tof_size"])
            if model is not None and not all(np.array_equal(a, b) for a, b in zip(model, this, strict=True)):
                raise ValueError(f"{path}: ToF camera model differs from the first input")
            model = model or this
            pick = np.arange(0, len(npz["tof_stamps_ns"]), every_n)[: quota + (1 if n < extra else 0)]
            name = str(npz["name"]) if "name" in npz.files else Path(path).stem
            depth.append(npz["tof_depth_mm"][pick])
            amp.append(npz["tof_amplitude"][pick])
            base_t.append(npz["tof_base_T"][pick])
            stamps.append(npz["tof_stamps_ns"][pick])
            scenes += [name] * len(pick)
    if model is None or not scenes:
        raise ValueError("no ToF frames selected")
    k, d, size = model
    d = np.concatenate([np.ravel(d), np.zeros(CAMERA_D_FIELDS)])[:CAMERA_D_FIELDS]
    depth_all, base_all = np.concatenate(depth), np.concatenate(base_t)
    floor = []
    for frame, pose7 in zip(depth_all, base_all, strict=True):
        pts = depth_points_in_base(frame, np.ravel(k), d, pose7)
        floor.append(pts[inside_convex(pts[:, :2], polygons["empty"])])
    output_npz.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_npz,
        tof_depth_mm=depth_all.astype(np.uint16),
        tof_amplitude=np.concatenate(amp).astype(np.uint16),
        tof_base_T=base_all.astype(np.float64),
        tof_stamps_ns=np.concatenate(stamps).astype(np.int64),
        tof_K=np.ravel(k).astype(np.float64),
        tof_D=d.astype(np.float64),
        tof_size=np.ravel(size).astype(np.int64),
        scene=np.asarray(scenes, dtype=str),
        truth_board=polygons["board"],
        truth_ramp=polygons["ramp"],
        truth_empty=polygons["empty"],
        truth_box=polygons["box"],
        box_height_m=np.asarray(float(truth["box_height_m"])),
        floor_plane=fit_plane(np.concatenate(floor)),
    )
    LOG.info("%d ToF frames of %d recordings -> %s", len(scenes), len(input_npzs), output_npz)


def main() -> None:
    """Export one recording (two positional paths) or, without arguments, every entry of the scenes config."""
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", type=Path, nargs="?", help="recording npz")
    parser.add_argument("output", type=Path, nargs="?", help="replay binary to write")
    args = parser.parse_args()
    if args.input is None and args.output is None:
        export_all()
    elif args.input is None or args.output is None:
        parser.error("give both <input.npz> and <output.bin>, or neither to export the scenes config")
    else:
        export(args.input, args.output)


if __name__ == "__main__":
    main()
