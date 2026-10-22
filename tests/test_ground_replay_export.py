"""Unit tests for scripts/ground_replay_export.py: live recording npz to the ground replay binary and back."""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from pydantic import ValidationError

from scripts.ground_replay_export import (
    DURATION_S,
    MAX_FIXTURE_BYTES,
    MAX_PAIR_FIXTURE_BYTES,
    START_S,
    export,
    export_all,
    export_tof_frames,
    load_scenes,
    read_replay,
)

TINY_V2_PATH = Path(__file__).resolve().parents[1] / "nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin"
UPDATE_TINY_ENV = "UPDATE_TINY_V2"

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


RGB_SIZE = (16, 12)
TOF_SIZE = (16, 12)
TOF_K = np.array([20.0, 0.0, 8.0, 0.0, 20.0, 6.0, 0.0, 0.0, 1.0])
TOF_D = np.zeros(5)


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


def write_v2_recording(path: Path, clouds: int = 3, pairs: int = 2, name: str = "synthetic") -> dict[str, np.ndarray]:
    """A synthetic layout-2 recording: write_recording plus the guidance keys, all stamps inside a 1 s window.

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
            "layout_version": np.asarray(2, dtype=np.int64),
            "name": np.asarray(name),
            "rgb_K": np.array([30.0, 0.0, 8.0, 0.0, 31.0, 6.0, 0.0, 0.0, 1.0]),
            "rgb_D": np.array([0.1, -0.2, 0.001, 0.002, 0.03]),
            "rgb_size": np.array(RGB_SIZE, dtype=np.int64),
            "rgb_frame": np.asarray("rgb_optical_frame"),
            "tof_K": TOF_K,
            "tof_D": TOF_D,
            "tof_size": np.array(TOF_SIZE, dtype=np.int64),
            "tof_frame": np.asarray(FRAME),
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


def test_v2_round_trip_pairs(tmp_path: Path) -> None:
    """A layout-2 recording gives a v2 file whose pairs, camera models and v1 sections decode back unchanged."""
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    arrays = write_v2_recording(npz)
    export(npz, out, start_s=0.0, duration_s=1.0)

    data = out.read_bytes()
    assert int.from_bytes(data[8:12], "little") == 2
    replay = read_replay(out)
    assert replay.version == 2
    np.testing.assert_array_equal(replay.stamps_ns, arrays["stamps_ns"])
    np.testing.assert_array_equal(replay.base_T_cloud, arrays["base_T_cloud"])
    np.testing.assert_array_equal(replay.points_mm, arrays["points_mm"])
    np.testing.assert_array_equal(replay.imu, arrays["imu"])
    assert replay.rgb_model is not None and replay.tof_model is not None
    np.testing.assert_array_equal(replay.rgb_model.K, arrays["rgb_K"])
    np.testing.assert_array_equal(replay.rgb_model.D, arrays["rgb_D"])
    assert (replay.rgb_model.width, replay.rgb_model.height) == RGB_SIZE
    assert replay.rgb_model.frame_id == "rgb_optical_frame"
    np.testing.assert_array_equal(replay.tof_model.K, TOF_K)
    assert replay.tof_model.frame_id == FRAME
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
    """A layout-1 input gives the version-1 file, with the header version 1 and no guidance section."""
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    write_recording(npz, clouds=12, seconds=1.0)
    export(npz, out, start_s=0.0, duration_s=1.0)
    data = out.read_bytes()
    assert int.from_bytes(data[8:12], "little") == 1
    replay = read_replay(out)
    assert replay.version == 1 and replay.rgb_model is None and replay.tof_model is None and replay.pairs == []
    # a v2 input with pairs: false keeps the v1 prefix byte for byte
    v2_npz, v2_out = tmp_path / "v2.npz", tmp_path / "v2.bin"
    write_v2_recording(v2_npz)
    export(v2_npz, v2_out, start_s=0.0, duration_s=1.0, pairs=False)
    v1_from_v2 = v2_out.read_bytes()
    assert int.from_bytes(v1_from_v2[8:12], "little") == 2
    assert v1_from_v2[12:].startswith(b"\x11\x00\x00\x00tof_optical_frame")


def test_pairs_false_writes_zero_pairs(tmp_path: Path) -> None:
    """pairs=False writes a v2 file with both camera models and a pair count of 0."""
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    write_v2_recording(npz)
    export(npz, out, start_s=0.0, duration_s=1.0, pairs=False)
    replay = read_replay(out)
    assert replay.version == 2 and replay.pairs == []
    assert replay.rgb_model is not None and replay.tof_model is not None


def test_fixture_size_cap(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """An export above its cap raises with the size and writes nothing."""
    assert MAX_FIXTURE_BYTES == 3_000_000 and MAX_PAIR_FIXTURE_BYTES == 6_000_000
    npz, out = tmp_path / "rec.npz", tmp_path / "rec.bin"
    write_v2_recording(npz)
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
    write_v2_recording(npz)
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
        "tof_frames": [],
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
    cfg.write_text(yaml.safe_dump({"fixtures": [], "tof_frames": [], "extra": 1}))
    with pytest.raises(ValidationError):
        load_scenes(cfg)


def quat_xyzw(rot: np.ndarray) -> list[float]:
    """Quaternion x y z w of a rotation matrix.

    :param rot: 3x3 rotation matrix.
    :return: [qx, qy, qz, qw].
    """
    from scipy.spatial.transform import Rotation

    return [float(v) for v in Rotation.from_matrix(rot).as_quat()]


def write_floor_recording(path: Path, name: str, frames: int, box: bool = False) -> None:
    """A recording of a flat floor z = 0 seen by a camera 0.4 m up looking 35 deg down, optionally with a 5 cm box.

    :param path: npz to write.
    :param name: recording name.
    :param frames: ToF frame count.
    :param box: put a box top (z = 0.05) over x 0.9..1.1, y -0.1..0.1.
    """
    w, h = TOF_SIZE
    k = np.array([[14.0, 0, 8.0], [0, 14.0, 6.0], [0, 0, 1.0]])
    a = np.deg2rad(35.0)
    fwd, right = np.array([np.cos(a), 0, -np.sin(a)]), np.array([0.0, -1.0, 0.0])
    rot = np.stack([right, np.cross(fwd, right), fwd], axis=1)
    cam = np.array([0.0, 0.0, 0.4])
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    rays = np.stack([(u - 8.0) / 14.0, (v - 6.0) / 14.0, np.ones_like(u, dtype=float)], axis=-1) @ rot.T
    ok = rays[..., 2] < -1e-6
    t_floor = np.where(ok, -cam[2] / np.where(ok, rays[..., 2], -1.0), 0.0)
    depth = np.where(ok, t_floor * 1000.0, 0.0)
    depth = np.where(depth > 4000, 0, depth).astype(np.uint16)
    if box:
        hit = cam + rays * ((0.05 - cam[2]) / np.where(ok, rays[..., 2], -1.0))[..., None]
        on = ok & (hit[..., 0] > 0.9) & (hit[..., 0] < 1.1) & (np.abs(hit[..., 1]) < 0.1)
        depth = np.where(on, ((0.05 - cam[2]) / rays[..., 2] * 1000.0), depth).astype(np.uint16)
    pose7 = [*cam, *quat_xyzw(rot)]
    np.savez(
        path,
        layout_version=np.asarray(2),
        name=np.asarray(name),
        tof_stamps_ns=T0_NS + 100_000_000 * np.arange(frames, dtype=np.int64),
        tof_depth_mm=np.stack([depth] * frames),
        tof_amplitude=np.full((frames, h, w), 500, dtype=np.uint16),
        tof_base_T=np.array([pose7] * frames),
        tof_K=k.ravel(),
        tof_D=np.zeros(5),
        tof_size=np.array([w, h], dtype=np.int64),
        tof_frame=np.asarray(FRAME),
    )


def test_export_tof_frames_subset(tmp_path: Path) -> None:
    """Every n-th frame of each recording is kept with its truth polygons and a floor plane within 2 mm."""
    a, b = tmp_path / "a.npz", tmp_path / "b.npz"
    write_floor_recording(a, "home", frames=10)
    write_floor_recording(b, "box", frames=10, box=True)
    square = [[0.3, -0.3], [1.5, -0.3], [1.5, 0.3], [0.3, 0.3]]
    truth = {
        "board": [[0.4, 0.4], [0.5, 0.4], [0.5, 0.5], [0.4, 0.5]],
        "ramp": [[1.0, -0.5], [1.2, -0.5], [1.2, -0.4], [1.0, -0.4]],
        "empty": square,
        "box": [[0.9, -0.1], [1.1, -0.1], [1.1, 0.1], [0.9, 0.1]],
        "box_height_m": 0.05,
    }
    truth_yaml = tmp_path / "truth.yaml"
    truth_yaml.write_text(yaml.safe_dump(truth))
    out = tmp_path / "frames.npz"
    export_tof_frames([a, b], out, every_n=3, max_frames=6, truth_yaml=truth_yaml)

    with np.load(out) as npz:
        assert npz["tof_depth_mm"].shape == (6, TOF_SIZE[1], TOF_SIZE[0])
        assert npz["tof_depth_mm"].dtype == np.uint16 and npz["tof_amplitude"].dtype == np.uint16
        assert npz["tof_base_T"].shape == (6, 7) and npz["tof_stamps_ns"].shape == (6,)
        assert npz["tof_K"].shape == (9,) and npz["tof_D"].shape == (5,) and npz["tof_size"].shape == (2,)
        assert list(npz["scene"]) == ["home"] * 3 + ["box"] * 3
        np.testing.assert_allclose(npz["tof_stamps_ns"][:3], T0_NS + 100_000_000 * np.array([0, 3, 6]))
        np.testing.assert_allclose(npz["truth_board"], truth["board"])
        np.testing.assert_allclose(npz["truth_ramp"], truth["ramp"])
        np.testing.assert_allclose(npz["truth_empty"], square)
        np.testing.assert_allclose(npz["truth_box"], truth["box"])
        assert float(npz["box_height_m"]) == pytest.approx(0.05)
        plane = npz["floor_plane"]
    assert plane.shape == (4,)
    # the true floor is z = 0: normal (0, 0, 1), d = 0
    assert abs(plane[3]) < 0.002
    assert abs(plane[2]) == pytest.approx(1.0, abs=1e-3)
    assert abs(plane[0]) < 0.002 and abs(plane[1]) < 0.002


def test_tiny_v2_fixture_is_current(tmp_path: Path) -> None:
    """tiny_v2.bin (3 clouds, 2 pairs) equals what the export writes for the synthetic recording."""
    import os

    npz, out = tmp_path / "tiny.npz", tmp_path / "tiny_v2.bin"
    write_v2_recording(npz, clouds=3, pairs=2, name="tiny")
    export(npz, out, start_s=0.0, duration_s=1.0)
    if os.environ.get(UPDATE_TINY_ENV):
        TINY_V2_PATH.write_bytes(out.read_bytes())
    assert TINY_V2_PATH.read_bytes() == out.read_bytes()
    replay = read_replay(TINY_V2_PATH)
    assert len(replay.stamps_ns) == 3 and len(replay.pairs) == 2
