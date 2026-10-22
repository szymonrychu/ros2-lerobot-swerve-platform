"""Unit tests for scripts/ground_replay_export.py: live recording npz to the ground replay binary and back."""

from pathlib import Path

import numpy as np
import pytest

from scripts.ground_replay_export import DURATION_S, START_S, export, read_replay

FRAME = "tof_optical_frame"
T0_NS = 1_792_642_215_220_634_811
CLOUD_PERIOD_NS = 100_000_000
IMU_PERIOD_NS = 40_000_000
NS_PER_S = 1_000_000_000


def pose(i: int, base: float) -> list[float]:
    """A distinct x y z qx qy qz qw row per cloud (unit quaternion).

    :param i: cloud index.
    :param base: offset that tells the pose sets apart.
    :return: the pose row.
    """
    angle = 0.01 * i
    return [base + 0.001 * i, -0.5 * base, 0.3, 0.0, 0.0, float(np.sin(angle / 2)), float(np.cos(angle / 2))]


def write_recording(path: Path, clouds: int, seconds: float) -> dict[str, np.ndarray]:
    """Write a synthetic recording with the real key layout (one cloud per CLOUD_PERIOD_NS, IMU every IMU_PERIOD_NS).

    :param path: the npz to write.
    :param clouds: number of clouds.
    :param seconds: span of the IMU samples, seconds from the first cloud.
    :return: the arrays written.
    """
    rng = np.random.default_rng(3)
    stamps = T0_NS + CLOUD_PERIOD_NS * np.arange(clouds, dtype=np.int64)
    counts = rng.integers(0, 6, size=clouds)
    counts[0] = 4
    offsets = np.concatenate([[0], np.cumsum(counts)]).astype(np.int64)
    points = rng.integers(-3000, 3000, size=(int(offsets[-1]), 3)).astype(np.int16)
    imu_stamps = T0_NS + IMU_PERIOD_NS * np.arange(int(seconds * NS_PER_S) // IMU_PERIOD_NS, dtype=np.int64)
    imu = np.zeros((len(imu_stamps), 11))
    imu[:, 0] = imu_stamps.astype(np.float64)
    imu[:, 1:] = rng.normal(size=(len(imu_stamps), 10))
    arrays = {
        "stamps_ns": stamps,
        "cloud_frame": np.array(FRAME),
        "offsets": offsets,
        "points_mm": points,
        "base_T_cloud": np.array([pose(i, 0.17) for i in range(clouds)]),
        "odom_T_base": np.array([pose(i, -0.004) for i in range(clouds)]),
        "imu": imu,
        "base_T_imu": np.array([0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0]),
        "head_js": np.zeros((5, 3)),
    }
    np.savez(path, **arrays)
    return arrays


def test_export_round_trip(tmp_path: Path) -> None:
    """A recording inside the window decodes back to the same clouds, poses, IMU samples and frame id."""
    npz = tmp_path / "rec.npz"
    out = tmp_path / "rec.bin"
    # Every cloud and IMU sample inside [START_S, START_S + DURATION_S): shift the stamps so the window holds them.
    arrays = write_recording(npz, clouds=12, seconds=1.0)
    shift = int(START_S * NS_PER_S)
    arrays["stamps_ns"] = arrays["stamps_ns"] + shift
    arrays["stamps_ns"][0] -= shift  # the first cloud sets the window start, the rest lie inside it
    arrays["imu"][:, 0] += float(shift)
    np.savez(npz, **arrays)

    export(npz, out)
    replay = read_replay(out)

    assert replay.frame_id == FRAME
    np.testing.assert_array_equal(replay.stamps_ns, arrays["stamps_ns"][1:])
    np.testing.assert_array_equal(replay.base_T_cloud, arrays["base_T_cloud"][1:])
    np.testing.assert_array_equal(replay.odom_T_base, arrays["odom_T_base"][1:])
    first = arrays["offsets"][1]
    np.testing.assert_array_equal(replay.offsets, arrays["offsets"][1:] - first)
    np.testing.assert_array_equal(replay.points_mm, arrays["points_mm"][first:])
    assert replay.points_mm.dtype == np.int16
    np.testing.assert_array_equal(replay.imu, arrays["imu"])
    np.testing.assert_array_equal(replay.base_T_imu, arrays["base_T_imu"])
    for i in range(len(replay.stamps_ns)):
        np.testing.assert_array_equal(
            replay.cloud(i), arrays["points_mm"][arrays["offsets"][i + 1] : arrays["offsets"][i + 2]]
        )


def test_export_trims_window(tmp_path: Path) -> None:
    """Only clouds and IMU samples stamped in [first cloud + START_S, + DURATION_S) are kept, offsets rebuilt."""
    npz = tmp_path / "rec.npz"
    out = tmp_path / "rec.bin"
    span_s = START_S + DURATION_S + 3.0
    arrays = write_recording(npz, clouds=int(span_s * NS_PER_S // CLOUD_PERIOD_NS), seconds=span_s)

    export(npz, out)
    replay = read_replay(out)

    lo = T0_NS + int(START_S * NS_PER_S)
    hi = lo + int(DURATION_S * NS_PER_S)
    keep = (arrays["stamps_ns"] >= lo) & (arrays["stamps_ns"] < hi)
    assert keep.sum() == int(DURATION_S * NS_PER_S // CLOUD_PERIOD_NS)
    np.testing.assert_array_equal(replay.stamps_ns, arrays["stamps_ns"][keep])
    np.testing.assert_array_equal(replay.base_T_cloud, arrays["base_T_cloud"][keep])
    kept = np.flatnonzero(keep)
    counts = np.diff(arrays["offsets"])[kept]
    np.testing.assert_array_equal(replay.offsets, np.concatenate([[0], np.cumsum(counts)]))
    expected_points = np.concatenate(
        [arrays["points_mm"][arrays["offsets"][i] : arrays["offsets"][i + 1]] for i in kept]
    )
    np.testing.assert_array_equal(replay.points_mm, expected_points)
    imu_stamps = arrays["imu"][:, 0].astype(np.int64)
    imu_keep = (imu_stamps >= lo) & (imu_stamps < hi)
    assert 0 < imu_keep.sum() < len(imu_stamps)
    np.testing.assert_array_equal(replay.imu, arrays["imu"][imu_keep])


def test_read_rejects_wrong_magic(tmp_path: Path) -> None:
    """A file that does not start with the replay magic is rejected."""
    bad = tmp_path / "bad.bin"
    bad.write_bytes(b"NOTREPLY" + bytes(16))
    with pytest.raises(ValueError, match="magic"):
        read_replay(bad)
