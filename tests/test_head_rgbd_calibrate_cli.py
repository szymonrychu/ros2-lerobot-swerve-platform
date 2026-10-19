"""Tests for scripts/head_rgbd_calibrate.py: calibrate, urdf, validate and capture on synthetic bags."""

import os
import shutil
import subprocess
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest
import yaml
from rosbags.rosbag2 import StoragePlugin, Writer
from scipy.spatial.transform import Rotation

from scripts import head_rgbd_calibrate as cli
from scripts.head_rgbd_calibration.charuco import detect, make_board
from scripts.head_rgbd_calibration.config import DEFAULT_CONFIG, BoardConfig, load_calibration_config
from scripts.head_rgbd_calibration.extrinsics import load_extrinsics_yaml
from scripts.head_rgbd_calibration.frames import TopicNames, normalise_amplitude
from scripts.head_rgbd_calibration.pipeline import load_calibration, newest_session, sweep_offset, wall_metrics
from scripts.head_rgbd_calibration.urdf_handoff import read_chain, tof_joint_origin
from tests.rgbd_synthetic import (
    board_poses,
    centred_pose,
    default_extrinsic,
    render_depth,
    render_rgb,
    render_tof_amplitude,
    rgb_camera,
    rotation,
    tof_camera,
)
from tests.test_head_rgbd_calib_frames import TYPESTORE, compressed_msg, image_msg, info_msg

REPO_ROOT = Path(__file__).resolve().parent.parent
BOARD_CFG = BoardConfig()
BOARD = make_board(BOARD_CFG)
TOPICS = TopicNames()
TRUE = default_extrinsic(0.05, 1.5)
POSES = 20
CANDIDATE_POSES = 30
WINDOW_S = 2.0
CYCLE_S = 3.0
START_S = 2.5
T0_NS = 1_700_000_000 * 10**9
TOF_HZ = 10
RGB_EVERY = 5
PERCENTILES = (1.0, 99.0)
EXTRINSIC_TOLERANCE_M = 0.002
EXTRINSIC_TOLERANCE_DEG = 0.2
TOF_SIZE_RC = (180, 240)
RGB_SIZE_RC = (324, 576)
IMAGE = "sensor_msgs/msg/Image"
COMPRESSED = "sensor_msgs/msg/CompressedImage"
INFO = "sensor_msgs/msg/CameraInfo"
URDF_REPO = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"


@lru_cache(maxsize=1)
def scene() -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """(rgb bgr, tof amplitude, tof depth mm) for POSES board poses seen by both cameras."""
    rgb_cam, tof_cam = rgb_camera(), tof_camera()
    poses = board_poses(CANDIDATE_POSES, rgb_cam, BOARD_CFG, seed=11, distance_m=(0.3, 0.45), max_tilt_deg=25.0)
    out = []
    for i, pose in enumerate(poses):
        tof_pose = TRUE.tof_pose(pose)
        rgb = render_rgb(rgb_cam, pose, BOARD_CFG, seed=i)
        amp = render_tof_amplitude(tof_cam, tof_pose, BOARD_CFG, seed=i)
        if detect(rgb, BOARD) is None or detect(normalise_amplitude(amp, PERCENTILES), BOARD) is None:
            continue
        out.append((rgb, amp, render_depth(tof_cam, tof_pose, BOARD_CFG)))
    return out[:POSES]


def add_connections(writer: Writer, with_rgb: bool, with_amp: bool) -> dict[str, object]:
    wanted = [("depth", TOPICS.depth, IMAGE), ("tof_info", TOPICS.tof_info, INFO)]
    if with_amp:
        wanted.append(("amp", TOPICS.amplitude, IMAGE))
    if with_rgb:
        wanted += [("rgb", TOPICS.rgb, COMPRESSED), ("rgb_info", TOPICS.rgb_info, INFO)]
    return {name: writer.add_connection(topic, msg_type, typestore=TYPESTORE) for name, topic, msg_type in wanted}


def write_calibration_bag(path: Path, views: list[tuple[np.ndarray, np.ndarray, np.ndarray]]) -> None:
    """One still window per view, moving noise between windows; ToF 10 Hz, RGB 2 Hz."""
    rng = np.random.default_rng(0)
    noise_depth = views[0][2]
    with Writer(path, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        conns = add_connections(writer, True, True)

        def put(name: str, ns: int, msg: object, msg_type: str) -> None:
            writer.write(conns[name], ns, TYPESTORE.serialize_cdr(msg, msg_type))

        total_s = START_S + len(views) * CYCLE_S
        for i in range(int(total_s * TOF_HZ)):
            t = i / TOF_HZ
            ns = T0_NS + int(t * 1e9)
            index = int((t - START_S) // CYCLE_S) if t >= START_S else -1
            still = index >= 0 and (t - START_S) - index * CYCLE_S < WINDOW_S
            if still:
                rgb, amp, depth = views[index]
            else:
                rgb = rng.integers(0, 255, RGB_SIZE_RC + (3,), dtype=np.uint8)
                amp = rng.integers(500, 4000, TOF_SIZE_RC, dtype=np.uint16)
                depth = noise_depth
            put("amp", ns, image_msg(ns, amp, "mono16"), IMAGE)
            put("depth", ns, image_msg(ns, depth, "16UC1"), IMAGE)
            put("tof_info", ns, info_msg(ns, TOF_SIZE_RC, 191.0), INFO)
            if i % RGB_EVERY == 0:
                rns = ns + 5_000_000
                put("rgb", rns, compressed_msg(rns, rgb), COMPRESSED)
                put("rgb_info", rns, info_msg(rns, RGB_SIZE_RC, 432.0), INFO)


def write_depth_bag(path: Path, depth_mm: int, frames: int = 4) -> None:
    """A flat target at depth_mm seen by the raw ToF."""
    with Writer(path, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        conns = add_connections(writer, False, False)
        depth = np.full(TOF_SIZE_RC, depth_mm, dtype=np.uint16)
        for i in range(frames):
            ns = T0_NS + i * 100_000_000
            writer.write(conns["depth"], ns, TYPESTORE.serialize_cdr(image_msg(ns, depth, "16UC1"), IMAGE))
            writer.write(conns["tof_info"], ns, TYPESTORE.serialize_cdr(info_msg(ns, TOF_SIZE_RC, 191.0), INFO))


def make_config(tmp_path: Path, **overrides: object) -> Path:
    data = yaml.safe_load(DEFAULT_CONFIG.read_text())
    data["capture"]["local_dir"] = str(tmp_path / "bags")
    data["outputs"] = {
        "rgb_calibration": str(tmp_path / "out" / "rgb.yaml"),
        "tof_calibration": str(tmp_path / "out" / "tof.yaml"),
        "extrinsics": str(tmp_path / "out" / "extrinsics.yaml"),
        "urdf": str(tmp_path / "head.urdf"),
        "report_dir": str(tmp_path / "reports"),
    }
    data["head_camera"] = {"config_file": str(tmp_path / "client.yml"), "node": "head_camera_cpp"}
    data["session"] = "s1"
    data.update(overrides)
    (tmp_path / "client.yml").write_text(
        yaml.safe_dump({"ros2_nodes": [{"name": "head_camera_cpp", "config": "lens_position: 1.7\n"}]})
    )
    path = tmp_path / "calibrate.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.fixture(scope="module")
def calibration_bag(tmp_path_factory: pytest.TempPathFactory) -> Path:
    bag = tmp_path_factory.mktemp("good") / "bags" / "s1"
    write_calibration_bag(bag, scene())
    return bag


def stage(tmp_path: Path, bag: Path) -> Path:
    """Config in tmp_path whose local_dir holds a copy of bag under its name."""
    config = make_config(tmp_path)
    shutil.copytree(bag, tmp_path / "bags" / bag.name)
    return config


@pytest.fixture(scope="module")
def calibrated(tmp_path_factory: pytest.TempPathFactory, calibration_bag: Path) -> Path:
    tmp_path = tmp_path_factory.mktemp("calibrated")
    config = stage(tmp_path, calibration_bag)
    assert cli.main(["--config", str(config), "calibrate"]) == 0
    return config


def test_scene_has_enough_views() -> None:
    assert len(scene()) == POSES


def test_calibrate_writes_three_files_matching_truth(calibrated: Path) -> None:
    out = Path(yaml.safe_load(calibrated.read_text())["outputs"]["extrinsics"]).parent
    rgb = yaml.safe_load((out / "rgb.yaml").read_text())
    tof = yaml.safe_load((out / "tof.yaml").read_text())
    ext = yaml.safe_load((out / "extrinsics.yaml").read_text())
    rgb_k, tof_k = rgb["camera_matrix"]["data"], tof["camera_matrix"]["data"]
    assert rgb_k[0] == pytest.approx(432.0, rel=0.01) and rgb_k[4] == pytest.approx(432.0, rel=0.01)
    assert abs(rgb_k[2] - 288) < 3 and abs(rgb_k[5] - 162) < 3
    assert tof_k[0] == pytest.approx(191.0, rel=0.02) and tof_k[4] == pytest.approx(191.0, rel=0.02)
    assert abs(tof_k[2] - 120) < 2 and abs(tof_k[5] - 90) < 2
    assert np.linalg.norm(np.array(ext["translation_m"]) - TRUE.t) < EXTRINSIC_TOLERANCE_M
    r = Rotation.from_quat(ext["rotation_xyzw"]).as_matrix()
    assert np.rad2deg(np.linalg.norm(Rotation.from_matrix(r @ TRUE.r.T).as_rotvec())) < EXTRINSIC_TOLERANCE_DEG
    assert ext["lens_position"] == 1.7
    assert ext["tof_sdk_diff_pct"]["fx"] == pytest.approx(0.0, abs=2.0)


def test_calibrate_writes_report(calibrated: Path) -> None:
    reports = list(Path(yaml.safe_load(calibrated.read_text())["outputs"]["report_dir"]).glob("*.md"))
    assert len(reports) == 1
    text = reports[0].read_text()
    for needle in ("RMS", "views", "SDK", "plane", "lens position", "1.7"):
        assert needle in text


def test_too_few_still_poses_exits_one_and_writes_nothing(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    write_calibration_bag(tmp_path / "bags" / "s1", scene()[:5])
    config = make_config(tmp_path)
    assert cli.main(["--config", str(config), "calibrate"]) == 1
    assert "views" in capsys.readouterr().err
    assert not (tmp_path / "out").exists() and not (tmp_path / "reports").exists()


def test_rms_limit_failure_names_limit_and_writes_nothing(
    tmp_path: Path, calibration_bag: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    config = stage(tmp_path, calibration_bag)
    data = yaml.safe_load(config.read_text())
    data["limits"]["rgb_max_rms_px"] = 1e-6
    config.write_text(yaml.safe_dump(data))
    assert cli.main(["--config", str(config), "calibrate"]) == 1
    assert "rgb_max_rms_px" in capsys.readouterr().err
    assert not (tmp_path / "out").exists()


def test_urdf_rewrites_tof_joint_and_check_is_read_only(
    calibrated: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    data = yaml.safe_load(calibrated.read_text())
    shutil.copy(URDF_REPO, data["outputs"]["urdf"])
    data["outputs"]["urdf"] = str(tmp_path / "head.urdf")
    shutil.copy(URDF_REPO, tmp_path / "head.urdf")
    config = tmp_path / "c.yaml"
    config.write_text(yaml.safe_dump(data))
    original = (tmp_path / "head.urdf").read_bytes()
    assert cli.main(["--config", str(config), "urdf", "--check"]) == 0
    assert "mm" in capsys.readouterr().out
    assert (tmp_path / "head.urdf").read_bytes() == original
    assert cli.main(["--config", str(config), "urdf"]) == 0
    text = (tmp_path / "head.urdf").read_text()
    assert text != original.decode()
    ext = Path(data["outputs"]["extrinsics"])
    expected = tof_joint_origin(read_chain(original.decode()), load_extrinsics_yaml(ext))
    assert read_chain(text).tof.origin.xyz == pytest.approx(expected.xyz, abs=1e-8)
    assert cli.main(["--config", str(config), "urdf", "--check"]) == 0
    assert "0.000 mm" in capsys.readouterr().out


def validation_config(tmp_path: Path, calibrated: Path, biases_mm: dict[float, int]) -> Path:
    data = yaml.safe_load(calibrated.read_text())
    data["capture"]["local_dir"] = str(tmp_path / "bags")
    data["outputs"]["report_dir"] = str(tmp_path / "reports")
    for distance, bias in biases_mm.items():
        write_depth_bag(tmp_path / "bags" / f"s1-{distance:.1f}m", round(distance * 1000) + bias)
    config = tmp_path / "v.yaml"
    config.write_text(yaml.safe_dump(data))
    return config


def test_validate_passes_on_accurate_distance_bags(
    calibrated: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    config = validation_config(tmp_path, calibrated, {0.5: 2, 1.0: -3, 2.0: 5, 3.0: 8})
    assert cli.main(["--config", str(config), "validate"]) == 0
    out = capsys.readouterr().out
    assert "depth accuracy 2 m" in out and "FAIL" not in out and "PASS" in out
    assert list((tmp_path / "reports").glob("s1-validation.md"))


def test_validate_fails_when_bias_exceeds_target(
    calibrated: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    config = validation_config(tmp_path, calibrated, {0.5: 2, 1.0: 40, 2.0: 5, 3.0: 8})
    assert cli.main(["--config", str(config), "validate"]) == 1
    out = capsys.readouterr().out
    assert "FAIL" in out and "depth accuracy 1 m" in out


def test_validate_missing_distance_bag_exits_one(
    calibrated: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    config = validation_config(tmp_path, calibrated, {0.5: 0})
    assert cli.main(["--config", str(config), "validate"]) == 1
    assert "s1-1.0m" in capsys.readouterr().err


class FakeRunner:
    def __init__(self) -> None:
        self.commands: list[list[str]] = []

    def __call__(self, cmd: list[str], **_: object) -> subprocess.CompletedProcess:
        self.commands.append(cmd)
        if cmd[0] == "scp":
            bag = Path(cmd[-1])
            bag.mkdir(parents=True, exist_ok=True)
            (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  message_count: 42\n")
        return subprocess.CompletedProcess(cmd, 0)


class FakeLive:
    def __init__(self) -> None:
        self.started = 0
        self.stopped = 0

    def __call__(self, _cfg: object) -> object:
        self.started += 1
        return self.stop

    def stop(self) -> None:
        self.stopped += 1


@pytest.fixture
def live(monkeypatch: pytest.MonkeyPatch) -> FakeLive:
    fake = FakeLive()
    monkeypatch.setattr(cli, "LIVE", fake)
    return fake


def test_capture_prints_bag_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], live: FakeLive
) -> None:
    runner = FakeRunner()
    monkeypatch.setattr(cli, "RUNNER", runner)
    assert cli.main(["--config", str(make_config(tmp_path)), "capture"]) == 0
    assert capsys.readouterr().out.strip() == str(tmp_path / "bags" / "s1")
    assert "/head_camera/mapping/image_raw/compressed" in runner.commands[0][-1]
    assert len(runner.commands) == 3
    assert (live.started, live.stopped) == (1, 1)


def test_capture_stops_live_when_the_recording_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, live: FakeLive
) -> None:
    monkeypatch.setattr(cli, "RUNNER", lambda cmd, **_: subprocess.CompletedProcess(cmd, 1))
    assert cli.main(["--config", str(make_config(tmp_path)), "capture"]) == 1
    assert (live.started, live.stopped) == (1, 1)


def test_capture_without_live(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, live: FakeLive) -> None:
    monkeypatch.setattr(cli, "RUNNER", FakeRunner())
    config = make_config(tmp_path)
    data = yaml.safe_load(config.read_text())
    data["capture"]["live"] = False
    config.write_text(yaml.safe_dump(data))
    assert cli.main(["--config", str(config), "capture"]) == 0
    assert live.started == 0


def test_capture_validation_names_bag_and_uses_validation_topics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], live: FakeLive
) -> None:
    runner = FakeRunner()
    monkeypatch.setattr(cli, "RUNNER", runner)
    assert cli.main(["--config", str(make_config(tmp_path)), "capture-validation", "0.5m"]) == 0
    assert capsys.readouterr().out.strip() == str(tmp_path / "bags" / "s1-0.5m")
    assert "/head_rgbd/depth/image_raw" in runner.commands[0][-1]
    assert live.started == 0


def test_capture_validation_rejects_unknown_suffix(tmp_path: Path) -> None:
    with pytest.raises(SystemExit):
        cli.main(["--config", str(make_config(tmp_path)), "capture-validation", "9.9m"])


def test_session_defaults_to_newest_calibration_bag(tmp_path: Path) -> None:
    for name in ("old", "new", "new-0.5m"):
        (tmp_path / "bags" / name).mkdir(parents=True)
        (tmp_path / "bags" / name / "metadata.yaml").write_text("x")
    os.utime(tmp_path / "bags" / "old", (1, 1))
    os.utime(tmp_path / "bags" / "new", (100, 100))
    os.utime(tmp_path / "bags" / "new-0.5m", (200, 200))
    assert newest_session(tmp_path / "bags") == "new"


BACKGROUND_MM = 2000
WALL_DISTANCE_M = 0.8
SWEEP_FRAMES = 12
SWEEP_STEP_M = 0.01
SWEEP_DISTANCE_M = 0.4


def sweep_poses(count: int) -> list:
    return [
        centred_pose(BOARD_CFG, rotation(0, 0, 0), np.array([-0.04 + i * SWEEP_STEP_M, 0.0, SWEEP_DISTANCE_M]))
        for i in range(count)
    ]


def test_sweep_offset_is_zero_for_synchronous_streams(calibrated: Path, tmp_path: Path) -> None:
    """RGB and ToF stamped at the same instants of a moving board give a time offset near zero."""
    cfg = load_calibration_config(calibrated)
    rgb_cam, tof_cam = rgb_camera(), tof_camera()
    bag = tmp_path / "sweep"
    with Writer(bag, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        conns = add_connections(writer, True, True)
        for i, pose in enumerate(sweep_poses(SWEEP_FRAMES)):
            ns = T0_NS + i * 100_000_000
            tof_pose = TRUE.tof_pose(pose)
            frames = {
                "rgb": (compressed_msg(ns, render_rgb(rgb_cam, pose, BOARD_CFG, seed=i)), COMPRESSED),
                "amp": (image_msg(ns, render_tof_amplitude(tof_cam, tof_pose, BOARD_CFG, seed=i), "mono16"), IMAGE),
                "depth": (image_msg(ns, render_depth(tof_cam, tof_pose, BOARD_CFG), "16UC1"), IMAGE),
            }
            for name, (msg, msg_type) in frames.items():
                writer.write(conns[name], ns, TYPESTORE.serialize_cdr(msg, msg_type))
    calibration = load_calibration(cfg)
    assert calibration is not None
    assert abs(sweep_offset(bag, cfg, calibration)) < 10.0


def with_background(depth: np.ndarray) -> np.ndarray:
    """Fill the empty pixels with a wall 2 m behind so the board outline is a depth step."""
    return np.where(depth > 0, depth, BACKGROUND_MM).astype(np.uint16)


def test_wall_metrics_on_registered_board(calibrated: Path, tmp_path: Path) -> None:
    cfg = load_calibration_config(calibrated)
    rgb_cam, tof_cam = rgb_camera(), tof_camera()
    pose = centred_pose(BOARD_CFG, rotation(0, 0, 0), np.array([0.0, 0.0, WALL_DISTANCE_M]))
    bag = tmp_path / "wall"
    with Writer(bag, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        conns = {
            name: writer.add_connection(topic, msg_type, typestore=TYPESTORE)
            for name, topic, msg_type in [
                ("raw", TOPICS.depth, IMAGE),
                ("depth", "/head_rgbd/depth/image_raw", IMAGE),
                ("rgb", "/head_rgbd/rgb/image_raw/compressed", COMPRESSED),
            ]
        }
        ns = T0_NS
        frames = {
            "raw": (image_msg(ns, render_depth(tof_cam, TRUE.tof_pose(pose), BOARD_CFG), "16UC1"), IMAGE),
            "depth": (image_msg(ns, with_background(render_depth(rgb_cam, pose, BOARD_CFG)), "16UC1"), IMAGE),
            "rgb": (compressed_msg(ns, render_rgb(rgb_cam, pose, BOARD_CFG)), COMPRESSED),
        }
        for name, (msg, msg_type) in frames.items():
            writer.write(conns[name], ns, TYPESTORE.serialize_cdr(msg, msg_type))
    calibration = load_calibration(cfg)
    assert calibration is not None
    metrics = wall_metrics(bag, cfg, calibration)
    assert abs(metrics["bias"]) < 5.0
    assert 0.0 < metrics["valid"] <= 1.0
    assert 0.0 < metrics["edge"] < 8.0
