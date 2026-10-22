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

Usage:
    uv run python scripts/ground_replay_export.py <input.npz> <output.bin>
"""

import argparse
import logging
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

LOG = logging.getLogger("ground_replay_export")

START_S = 5.0  # window start after the first cloud, seconds
DURATION_S = 15.0  # window length, seconds
MAGIC = b"GRNDRPLY"
VERSION = 1
NS_PER_S = 1_000_000_000
POSE_FIELDS = 7  # x y z qx qy qz qw
IMU_FIELDS = 11  # stamp_ns plus 10 values
U32 = struct.Struct("<I")
I64 = struct.Struct("<q")
POSE_DTYPE = np.dtype("<f8")
POINT_DTYPE = np.dtype("<i2")
CLOUD_HEADER = np.dtype([("stamp_ns", "<i8"), ("base_T_cloud", "<f8", (7,)), ("odom_T_base", "<f8", (7,))])
IMU_RECORD = np.dtype([("stamp_ns", "<i8"), ("values", "<f8", (IMU_FIELDS - 1,))])


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
    """

    frame_id: str
    stamps_ns: np.ndarray
    base_T_cloud: np.ndarray
    odom_T_base: np.ndarray
    offsets: np.ndarray
    points_mm: np.ndarray
    imu: np.ndarray
    base_T_imu: np.ndarray

    def cloud(self, i: int) -> np.ndarray:
        """Return the points of cloud i.

        :param i: cloud index (int).
        :return: the cloud's x y z in mm (np.ndarray, n x 3, int16).
        """
        return self.points_mm[self.offsets[i] : self.offsets[i + 1]]


def trim(arrays: dict[str, np.ndarray], start_s: float = START_S, duration_s: float = DURATION_S) -> Replay:
    """Keep the clouds and IMU samples stamped inside the window after the first cloud.

    :param arrays: the recording's arrays (dict[str, np.ndarray], keys as in the module docstring).
    :param start_s: window start after the first cloud, seconds (float).
    :param duration_s: window length, seconds (float).
    :return: the trimmed recording, offsets rebuilt (Replay).
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
    return Replay(
        frame_id=str(arrays["cloud_frame"]),
        stamps_ns=stamps[kept],
        base_T_cloud=np.asarray(arrays["base_T_cloud"], dtype=np.float64)[kept],
        odom_T_base=np.asarray(arrays["odom_T_base"], dtype=np.float64)[kept],
        offsets=np.concatenate([[0], np.cumsum(counts)]).astype(np.int64),
        points_mm=np.concatenate(chunks) if chunks else np.zeros((0, 3), dtype=np.int16),
        imu=imu[(imu_stamps >= lo) & (imu_stamps < hi)],
        base_T_imu=np.asarray(arrays["base_T_imu"], dtype=np.float64).reshape(POSE_FIELDS),
    )


def encode(replay: Replay) -> bytes:
    """Serialise a replay in the binary layout of the module docstring.

    :param replay: the recording to write (Replay).
    :return: the file content (bytes).
    """
    frame = replay.frame_id.encode("utf-8")
    parts = [MAGIC, U32.pack(VERSION), U32.pack(len(frame)), frame, U32.pack(len(replay.stamps_ns))]
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
    return b"".join(parts)


def decode(data: bytes) -> Replay:
    """Parse the binary layout of the module docstring.

    :param data: the file content (bytes).
    :return: the decoded recording (Replay).
    :raises ValueError: on a wrong magic or version, or a truncated file.
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

    (version,) = U32.unpack(take(U32.size))
    if version != VERSION:
        raise ValueError(f"unsupported ground replay version {version}, expected {VERSION}")
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
    )


def read_replay(path: Path) -> Replay:
    """Read a replay binary.

    :param path: the .bin file (Path).
    :return: the decoded recording (Replay).
    :raises ValueError: on a wrong magic or version, or a truncated file.
    """
    return decode(Path(path).read_bytes())


def export(input_path: Path, output_path: Path) -> Replay:
    """Trim a recording to the START_S / DURATION_S window and write it as a replay binary.

    :param input_path: the recording npz (Path).
    :param output_path: the .bin to write (Path).
    :return: what was written (Replay).
    """
    with np.load(input_path, allow_pickle=False) as npz:
        replay = trim({key: npz[key] for key in npz.files})
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(encode(replay))
    LOG.info(
        "%s: %d clouds, %d points, %d IMU samples -> %s",
        input_path,
        len(replay.stamps_ns),
        len(replay.points_mm),
        len(replay.imu),
        output_path,
    )
    return replay


def main() -> None:
    """Parse the two positional paths and export."""
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", type=Path, help="recording npz")
    parser.add_argument("output", type=Path, help="replay binary to write")
    args = parser.parse_args()
    export(args.input, args.output)


if __name__ == "__main__":
    main()
