"""Unit tests for scripts/ground_replay_export.py: live recording npz to the ground replay binary and back."""

import json
import os
import struct
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from ros2_common.camera_geometry import CameraIntrinsics, depth_to_points
from scipy.spatial.transform import Rotation

from scripts.ground_replay_export import (
    DURATION_S,
    MAX_FIXTURE_BYTES,
    MAX_PAIR_FIXTURE_BYTES,
    START_S,
    CameraModel,
    depth_points_in_base,
    export,
    export_all,
    load_scenes,
    read_replay,
)

TINY_V2_PATH = Path(__file__).resolve().parents[1] / "nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin"
UPDATE_TINY_ENV = "UPDATE_TINY_V2"
V1_MAGIC = b"GRNDRPLY"

FRAME = "tof_optical_frame"  # cloud frame of the layout-1 recordings of 2026-10-22
DEPTH_FRAME = "d435_depth_optical_frame"
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


RGB_SIZE = (16, 12)
DEPTH_SIZE = (16, 12)
DEPTH_K = np.array([20.0, 0.0, 8.0, 0.0, 20.0, 6.0, 0.0, 0.0, 1.0])
DEPTH_D = np.zeros(5)


def make_jpeg(i: int, size: tuple[int, int] = RGB_SIZE) -> bytes:
    """A deterministic small JPEG.

    :param i: image index, sets the brightness.
    :param size: (width, height).
    :return: the JPEG bytes.
    """
    image = np.full((size[1], size[0], 3), 40 * i + 20, dtype=np.uint8)
    image[:, : size[0] // 2, 0] = 200
    ok, buf = cv2.imencode(".jpg", image)
    assert ok
    return buf.tobytes()


def write_v3_recording(path: Path, clouds: int = 3, pairs: int = 2, name: str = "synthetic") -> dict[str, np.ndarray]:
    """A synthetic layout-3 recording: write_recording plus the D435 depth model and the guidance keys of
    nodes/web_ui/web_ui/ground_recorder.py, all stamps inside a 1 s window.

    :param path: the npz to write.
    :param clouds: cloud count.
    :param pairs: pair count.
    :param name: recording name.
    :return: the arrays written.
    """
    arrays = write_recording(path, clouds=clouds, seconds=0.5)
    rng = np.random.default_rng(5)
    jpegs = [make_jpeg(i) for i in range(pairs)]
    depth = rng.integers(0, 4000, size=(pairs, RGB_SIZE[1], RGB_SIZE[0])).astype(np.uint16)
    arrays.update(
        {
            "layout_version": np.asarray(3, dtype=np.int64),
            "name": np.asarray(name),
            "cloud_frame": np.asarray(DEPTH_FRAME),
            "rgb_K": np.array([30.0, 0.0, 8.0, 0.0, 31.0, 6.0, 0.0, 0.0, 1.0]),
            "rgb_D": np.array([0.1, -0.2, 0.001, 0.002, 0.03]),
            "rgb_size": np.array(RGB_SIZE, dtype=np.int64),
            "rgb_frame": np.asarray("rgb_optical_frame"),
            "depth_K": DEPTH_K,
            "depth_D": DEPTH_D,
            "depth_size": np.array(DEPTH_SIZE, dtype=np.int64),
            "depth_frame": np.asarray(DEPTH_FRAME),
            "pair_stamps_ns": T0_NS + 150_000_000 * np.arange(pairs, dtype=np.int64),
            "pair_jpeg": np.frombuffer(b"".join(jpegs), dtype=np.uint8),
            "pair_jpeg_offsets": np.concatenate([[0], np.cumsum([len(j) for j in jpegs])]).astype(np.int64),
            "pair_depth_mm": depth,
            "pair_base_T_rgb": np.array([pose(i, 0.05) for i in range(pairs)]),
            "pair_odom_T_base": np.array([pose(i, -0.01) for i in range(pairs)]),
        }
    )
    np.savez(path, **arrays)
    return arrays


def test_layout3_depth_model_round_trips(tmp_path: Path) -> None:
    """A layout-3 recording gives a v2 file whose depth and RGB models, pairs and v1 sections decode back unchanged."""
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    arrays = write_v3_recording(npz)
    export(npz, out, start_s=0.0, duration_s=1.0)

    data = out.read_bytes()
    assert int.from_bytes(data[8:12], "little") == 2
    replay = read_replay(out)
    assert replay.version == 2
    np.testing.assert_array_equal(replay.stamps_ns, arrays["stamps_ns"])
    np.testing.assert_array_equal(replay.base_T_cloud, arrays["base_T_cloud"])
    np.testing.assert_array_equal(replay.points_mm, arrays["points_mm"])
    np.testing.assert_array_equal(replay.imu, arrays["imu"])
    assert replay.frame_id == DEPTH_FRAME
    assert replay.rgb_model is not None and replay.depth_model is not None
    np.testing.assert_array_equal(replay.rgb_model.K, arrays["rgb_K"])
    np.testing.assert_array_equal(replay.rgb_model.D, arrays["rgb_D"])
    assert (replay.rgb_model.width, replay.rgb_model.height) == RGB_SIZE
    assert replay.rgb_model.frame_id == "rgb_optical_frame"
    np.testing.assert_array_equal(replay.depth_model.K, DEPTH_K)
    np.testing.assert_array_equal(replay.depth_model.D, DEPTH_D)
    assert (replay.depth_model.width, replay.depth_model.height) == DEPTH_SIZE
    assert replay.depth_model.frame_id == DEPTH_FRAME
    assert len(replay.pairs) == 2
    jpeg_offsets = arrays["pair_jpeg_offsets"]
    for i, pair in enumerate(replay.pairs):
        assert pair.stamp_ns == arrays["pair_stamps_ns"][i]
        np.testing.assert_array_equal(pair.base_T_rgb, arrays["pair_base_T_rgb"][i])
        np.testing.assert_array_equal(pair.odom_T_base, arrays["pair_odom_T_base"][i])
        assert pair.jpeg == arrays["pair_jpeg"][jpeg_offsets[i] : jpeg_offsets[i + 1]].tobytes()
        assert pair.depth_mm.dtype == np.uint16
        np.testing.assert_array_equal(pair.depth_mm, arrays["pair_depth_mm"][i])


def test_v1_output_unchanged(tmp_path: Path) -> None:
    """A layout-1 input gives the documented version-1 bytes, decoded here field by field with struct.

    This replaces a sha256 of the whole file taken on the author's machine (run 4, commit dc26218c): the recording is
    synthetic numpy random data and sin/cos poses, whose last bits differ by numpy release and libm, so the hash was
    red on CI (linux x86_64) with the writer unchanged. The layout is pinned by the README table instead.
    """
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    arrays = write_recording(npz, clouds=250, seconds=22.0)
    export(npz, out, start_s=START_S, duration_s=DURATION_S)
    data = out.read_bytes()
    stamps, offsets, imu = arrays["stamps_ns"], arrays["offsets"], arrays["imu"]
    lo = int(stamps[0]) + int(round(START_S * NS_PER_S))
    hi = lo + int(round(DURATION_S * NS_PER_S))
    kept = np.flatnonzero((stamps >= lo) & (stamps < hi))
    pos = 0

    def take(fmt: str) -> tuple[float | int, ...]:
        nonlocal pos
        values = struct.unpack_from("<" + fmt, data, pos)
        pos += struct.calcsize("<" + fmt)
        return values

    assert data[: len(V1_MAGIC)] == V1_MAGIC
    pos = len(V1_MAGIC)
    assert take("I") == (1,)
    (frame_len,) = take("I")
    assert data[pos : pos + frame_len].decode() == FRAME
    pos += frame_len
    assert take("I") == (len(kept),)
    for i in kept:
        assert take("q") == (stamps[i],)
        assert take("7d") == tuple(arrays["base_T_cloud"][i])
        assert take("7d") == tuple(arrays["odom_T_base"][i])
        (count,) = take("I")
        expected = arrays["points_mm"][offsets[i] : offsets[i + 1]]
        assert count == len(expected)
        np.testing.assert_array_equal(np.frombuffer(data, "<i2", count * 3, pos).reshape(-1, 3), expected)
        pos += count * 6
    window = imu[(imu[:, 0].astype(np.int64) >= lo) & (imu[:, 0].astype(np.int64) < hi)]
    assert take("I") == (len(window),)
    for row in window:
        assert take("q") == (int(row[0]),)
        assert take("10d") == tuple(row[1:])
    assert take("7d") == tuple(arrays["base_T_imu"])
    assert pos == len(data)
    replay = read_replay(out)
    assert replay.version == 1 and replay.rgb_model is None and replay.depth_model is None and replay.pairs == []


def test_layout2_is_refused(tmp_path: Path) -> None:
    """A layout-2 (Arducam ToF) recording is refused naming layout 3; nothing is written."""
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    arrays = write_v3_recording(npz)
    for key in ("depth_K", "depth_D", "depth_size", "depth_frame"):
        arrays["tof" + key.removeprefix("depth")] = arrays.pop(key)
    arrays["layout_version"] = np.asarray(2, dtype=np.int64)
    np.savez(npz, **arrays)
    with pytest.raises(ValueError, match="layout 3"):
        export(npz, out, start_s=0.0, duration_s=1.0)
    assert not out.exists()


def test_unknown_layout_is_refused(tmp_path: Path) -> None:
    """A layout newer than 3 is refused naming layout 3; nothing is written."""
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    arrays = write_v3_recording(npz)
    arrays["layout_version"] = np.asarray(4, dtype=np.int64)
    np.savez(npz, **arrays)
    with pytest.raises(ValueError, match="layout 3"):
        export(npz, out, start_s=0.0, duration_s=1.0)
    assert not out.exists()


@pytest.mark.parametrize("prefix", ["depth", "rgb"])
def test_recording_without_camera_info_is_refused(tmp_path: Path, prefix: str) -> None:
    """A layout-3 recording whose depth (or RGB) camera_info never arrived (size 0 x 0, K zeros, D empty, no frame,
    as ground_recorder.py writes it) is refused; nothing is written."""
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    arrays = write_v3_recording(npz)
    arrays[f"{prefix}_K"] = np.zeros(9)
    arrays[f"{prefix}_D"] = np.zeros(0)
    arrays[f"{prefix}_size"] = np.zeros(2, dtype=np.int64)
    arrays[f"{prefix}_frame"] = np.asarray("")
    np.savez(npz, **arrays)
    with pytest.raises(ValueError, match=f"{prefix} camera model.*camera_info"):
        export(npz, out, start_s=0.0, duration_s=1.0)
    assert not out.exists()


def test_pairs_false_writes_zero_pairs(tmp_path: Path) -> None:
    """pairs=False writes a v2 file with both camera models and a pair count of 0."""
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    write_v3_recording(npz)
    export(npz, out, start_s=0.0, duration_s=1.0, pairs=False)
    replay = read_replay(out)
    assert replay.version == 2 and replay.pairs == []
    assert replay.rgb_model is not None and replay.depth_model is not None


def test_fixture_size_cap(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """An export above its cap raises with the size and writes nothing."""
    assert MAX_FIXTURE_BYTES == 3_000_000 and MAX_PAIR_FIXTURE_BYTES == 6_000_000
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    write_v3_recording(npz)
    monkeypatch.setattr("scripts.ground_replay_export.MAX_FIXTURE_BYTES", 200)
    with pytest.raises(ValueError, match=r"\d+ bytes"):
        export(npz, out, start_s=0.0, duration_s=1.0, pairs=False)
    assert not out.exists()
    monkeypatch.setattr("scripts.ground_replay_export.MAX_PAIR_FIXTURE_BYTES", 200)
    with pytest.raises(ValueError, match=r"\d+ bytes"):
        export(npz, out, start_s=0.0, duration_s=1.0, pairs=True)
    assert not out.exists()


def test_scenes_config_validates(tmp_path: Path) -> None:
    """The scenes config parses, unknown keys are rejected, and export_all writes every entry."""
    npz = tmp_path / "rec.npz"
    write_v3_recording(npz)
    good = {
        "fixtures": [
            {
                "input": str(npz),
                "output": str(tmp_path / "out/a.bin"),
                "start_s": 0.0,
                "duration_s": 1.0,
                "pairs": True,
            }
        ],
    }
    cfg = tmp_path / "scenes.yaml"
    cfg.write_text(yaml.safe_dump(good))
    scenes = load_scenes(cfg)
    assert len(scenes.fixtures) == 1 and scenes.fixtures[0].pairs is True
    export_all(cfg)
    assert len(read_replay(tmp_path / "out/a.bin").pairs) == 2

    bad = json.loads(json.dumps(good))
    bad["fixtures"][0]["bogus"] = 1
    cfg.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValidationError):
        load_scenes(cfg)
    cfg.write_text(yaml.safe_dump({"fixtures": [], "extra": 1}))
    with pytest.raises(ValidationError):
        load_scenes(cfg)
    cfg.write_text(yaml.safe_dump({"fixtures": [], "tof_frames": []}))  # the Arducam amplitude subsets are retired
    with pytest.raises(ValidationError):
        load_scenes(cfg)


FLOOR_K = np.array([14.0, 0.0, 8.0, 0.0, 14.0, 6.0, 0.0, 0.0, 1.0])
FLOOR_CAMERA_M = np.array([0.0, 0.0, 0.4])  # optical centre in base_link
FLOOR_PITCH_DEG = 35.0  # camera looks this far down


def floor_view() -> tuple[np.ndarray, CameraModel, np.ndarray]:
    """A D435 depth image of the flat floor z = 0 seen from 0.4 m up, looking 35 deg down.

    :return: (depth, mm, H x W uint16, 0 above the horizon or beyond 4 m; the depth model; base_link <- optical frame
        x y z qx qy qz qw).
    """
    w, h = DEPTH_SIZE
    a = np.deg2rad(FLOOR_PITCH_DEG)
    fwd, right = np.array([np.cos(a), 0, -np.sin(a)]), np.array([0.0, -1.0, 0.0])
    rot = np.stack([right, np.cross(fwd, right), fwd], axis=1)
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    rays = np.stack([(u - FLOOR_K[2]) / FLOOR_K[0], (v - FLOOR_K[5]) / FLOOR_K[4], np.ones((h, w))], axis=-1) @ rot.T
    down = rays[..., 2] < -1e-6
    z_mm = np.where(down, -FLOOR_CAMERA_M[2] / np.where(down, rays[..., 2], -1.0) * 1000.0, 0.0)
    depth = np.where(z_mm > 4000, 0, np.round(z_mm)).astype(np.uint16)
    model = CameraModel(K=FLOOR_K, D=np.zeros(5), width=w, height=h, frame_id=DEPTH_FRAME)
    return depth, model, np.array([*FLOOR_CAMERA_M, *Rotation.from_matrix(rot).as_quat()])


def test_depth_points_in_base_matches_depth_to_points(monkeypatch: pytest.MonkeyPatch) -> None:
    """depth_points_in_base is ros2_common.camera_geometry.depth_to_points on the depth model, moved into base_link:
    the floor view lands on z = 0, every valid pixel once, and a shift of the shared helper's points moves it too."""
    depth, model, base_T = floor_view()
    points = depth_points_in_base(depth, model, base_T)

    intrinsics = CameraIntrinsics(width=16, height=12, fx=14.0, fy=14.0, cx=8.0, cy=6.0)
    rot = Rotation.from_quat(base_T[3:]).as_matrix()
    expected = depth_to_points(depth, intrinsics) @ rot.T + base_T[:3]
    np.testing.assert_allclose(points, expected, atol=1e-12)
    assert len(points) == np.count_nonzero(depth) > 0
    np.testing.assert_allclose(points[:, 2], 0.0, atol=0.002)  # mm rounding of the depth

    shared = depth_to_points
    monkeypatch.setattr(
        "scripts.ground_replay_export.depth_to_points", lambda *a, **k: shared(*a, **k) + [0.0, 0.0, 0.1]
    )
    shifted = depth_points_in_base(depth, model, base_T)
    np.testing.assert_allclose(shifted, expected + 0.1 * rot[:, 2], atol=1e-12)


def test_tiny_v2_fixture_is_current(tmp_path: Path) -> None:
    """tiny_v2.bin (3 clouds, 2 pairs, the depth model) equals what the export writes for the layout-3 recording."""
    npz, out = tmp_path / "tiny.npz", tmp_path / "tiny_v2.bin"
    write_v3_recording(npz, clouds=3, pairs=2, name="tiny")
    export(npz, out, start_s=0.0, duration_s=1.0)
    if os.environ.get(UPDATE_TINY_ENV):
        TINY_V2_PATH.write_bytes(out.read_bytes())
    assert TINY_V2_PATH.read_bytes() == out.read_bytes()
    replay = read_replay(TINY_V2_PATH)
    assert len(replay.stamps_ns) == 3 and len(replay.pairs) == 2
    assert replay.depth_model is not None and replay.depth_model.frame_id == DEPTH_FRAME
