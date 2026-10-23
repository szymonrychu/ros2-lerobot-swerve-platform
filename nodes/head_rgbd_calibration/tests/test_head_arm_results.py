"""Tests for the head_arm compute and apply (synthetic gripper-board session, tmp state dir)."""

import importlib.util
import json
import math
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
import yaml
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.config import HeadArmFitConfig, HeadArmFitLimits
from ros2_common.head_rgbd_calibration.head_arm_fit import FLOOR_PARAMS, start_params
from ros2_common.head_rgbd_calibration.urdf_text import Origin, head_chain_json

from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_results import applied_head_chain, base_urdf
from head_rgbd_calibration.head_arm_results import apply_head_arm, compute_head_arm
from head_rgbd_calibration.store import SessionStore
from tests.pose_factory import still_arm_pose

REPO_ROOT = Path(__file__).resolve().parents[3]
URDF_DIR = REPO_ROOT / "nodes" / "web_ui" / "urdf"
SYNTHETIC_SPEC = importlib.util.spec_from_file_location(
    "head_arm_synthetic", REPO_ROOT / "tests" / "head_arm_synthetic.py"
)
assert SYNTHETIC_SPEC is not None and SYNTHETIC_SPEC.loader is not None
synthetic = importlib.util.module_from_spec(SYNTHETIC_SPEC)
SYNTHETIC_SPEC.loader.exec_module(synthetic)

NOW = datetime(2026, 10, 22, 14, 0, 0, tzinfo=UTC)
FLOOR_SID = "floor-20261022-064337"
FLOOR_RID = "r-20261022-065211"
OTHER_FLOOR = {"session": "floor-20261020-101010", "result": "r-20261020-101500"}
MOUNT = "head_mount_joint"
TILT = "head_tilt"
CAMERA = "head_camera_joint"
PAN = "head_pan"
PAN_AXIS_M = (-0.03, -0.01)
TRUTH_DELTAS = {"mount_x_m": 0.006, "mount_yaw_deg": 1.0, "mount_z_m": 0.003, "tilt_zero_deg": 0.4}
NOISE_PX = 0.3
CAMERA_ORIGIN = Origin((0.01, 0.02, 0.03), (0.1, 0.2, 0.3))


def fake_run(args: list[str], **_: Any) -> SimpleNamespace:
    return SimpleNamespace(stdout="ActiveEnterTimestamp=@2000000000\nActiveState=active\n", returncode=0)


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig(
        state_dir=str(tmp_path / "state"),
        head_urdf_file=str(URDF_DIR / "head.urdf"),
        arm_urdf_file=str(URDF_DIR / "so101_arm.urdf"),
        restart_units=["unit-a.service"],
    )


@pytest.fixture
def state(cfg: NodeConfig) -> Path:
    path = Path(cfg.state_dir)
    (path / "applied").mkdir(parents=True)
    return path


@pytest.fixture
def store(cfg: NodeConfig, state: Path) -> SessionStore:
    return SessionStore(state, cfg)


def write_intrinsics(state: Path, size: tuple[int, int] = synthetic.IMAGE_SIZE) -> None:
    k = synthetic.SYNTH_CAMERA_MATRIX
    data = {
        "image_width": size[0],
        "image_height": size[1],
        "camera_name": "head_camera",
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in k.reshape(-1)]},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.0] * 5},
    }
    (state / "applied" / "head_camera_calibration.yaml").write_text(yaml.safe_dump(data, sort_keys=False))


def put_marker(state: Path, **extra: Any) -> None:
    marker = {"session": "calib-20261019-120000", "result": "r-20261019-123045", "files": [], **extra}
    (state / "applied" / "applied.json").write_text(json.dumps(marker))


def truth_params(cfg: NodeConfig) -> dict[str, float]:
    chain = UrdfChain.from_text(Path(cfg.head_urdf_file).read_text())
    start = start_params(chain)
    return {name: start[name] + TRUTH_DELTAS.get(name, 0.0) for name in start}


def write_floor_result(state: Path, cfg: NodeConfig, sid: str, rid: str, offset: float = 0.0) -> None:
    directory = state / "floor_fit" / sid
    (directory / "results" / rid).mkdir(parents=True)
    (directory / "observations.json").write_text(json.dumps({"session": sid, "observations": []}))
    values = truth_params(cfg)
    params = {name: {"value": values[name] + offset, "sigma": 0.001} for name in FLOOR_PARAMS}
    (directory / "results" / rid / "result.json").write_text(json.dumps({"id": rid, "session": sid, "params": params}))


def make_session(
    store: SessionStore, cfg: NodeConfig, poses: int = 24, head_offsets: list | None = None, noise: float = NOISE_PX,
    extra: dict[str, Origin] | None = None, pan_axis_m: tuple[float, float] = (0.0, 0.0),
) -> str:  # fmt: skip
    head_urdf = Path(cfg.head_urdf_file).read_text()
    arm_urdf = Path(cfg.arm_urdf_file).read_text()
    xyz, rpy = synthetic.nominal_board_mount(cfg.fit)
    truth = synthetic.synth_truth(head_urdf, TRUTH_DELTAS, extra, cfg.fit, (xyz, rpy), pan_axis_m)
    views = synthetic.arm_board_scene(
        truth,
        head_urdf,
        arm_urdf,
        synthetic.SYNTH_ARM_POSES[:poses],
        head_offsets or synthetic.DEFAULT_HEAD_OFFSETS_DEG,
        noise,
        5,
        cfg.fit,
    )
    sid = store.create_session(NOW, 0.0, kind="head_arm")
    for index, view in enumerate(views):
        still = still_arm_pose(view.joints, arm_pose=view.arm_pose, tier=view.tier, head_offset=view.head_offset)
        store.add_still_pose(
            sid,
            still.rgb_jpeg,
            still.rgb_image,
            view.detection,
            view.joints,
            still.stamps,
            view.arm_pose,
            0,
            view.head_offset,
        )
    return sid


@pytest.fixture
def ready(cfg: NodeConfig, state: Path, store: SessionStore) -> str:
    write_intrinsics(state)
    put_marker(state, head_chain={"session": FLOOR_SID, "result": FLOOR_RID, "applied_at": "2026-10-22T07:00:00+00:00"})
    write_floor_result(state, cfg, FLOOR_SID, FLOOR_RID)
    return make_session(store, cfg)


def result_dir(store: SessionStore, sid: str, rid: str) -> Path:
    return store.result_dir(sid, rid)


def test_compute_writes_result_report_and_chain(cfg: NodeConfig, store: SessionStore, ready: str) -> None:
    result = compute_head_arm(cfg, store, ready, NOW)
    assert result["passed"] is True, (result["failed_limits"], result["error"])
    assert result["id"] == "r-20261022-140000"
    assert (result["session"], result["kind"], result["error"]) == (ready, "head_arm", None)
    assert result["floor"] == {"session": FLOOR_SID, "result": FLOOR_RID}
    assert set(result) >= {
        "id", "session", "kind", "created", "passed", "failed_limits", "error", "rms_px", "holdout_rms_px",
        "holdout_board_mm", "views", "arm_poses", "pan_span_deg", "tilt_span_deg", "max_correlation", "params",
        "board_mount", "per_view", "offset_rms_px", "limits", "floor",
    }  # fmt: skip
    entry = result["params"]["mount_x_m"]
    assert set(entry) == {"value", "sigma", "start", "floor", "floor_delta"}
    assert entry["value"] == pytest.approx(truth_params(cfg)["mount_x_m"], abs=0.002)
    assert result["params"]["mount_x_m"]["floor"] is None
    assert result["params"]["mount_z_m"]["floor"] is not None
    directory = result_dir(store, ready, result["id"])
    assert json.loads((directory / "result.json").read_text()) == result
    assert "Head-from-arm fit" in (directory / "report.md").read_text()
    chain = json.loads((directory / "head_chain.json").read_text())
    assert "2026-10-22" in chain["note"] and "RMS" in chain["note"] and "held-out" in chain["note"]
    assert (chain["session"], chain["result"]) == (ready, result["id"])
    assert not (directory / "tof_joint.json").exists()
    assert result["limits"]


def test_fitted_joints_merge_over_applied_chain(cfg: NodeConfig, state: Path, store: SessionStore) -> None:
    write_intrinsics(state)
    put_marker(state)
    write_floor_result(state, cfg, FLOOR_SID, FLOOR_RID)
    (state / "applied" / "applied.json").write_text(
        json.dumps({"head_chain": {"session": FLOOR_SID, "result": FLOOR_RID, "applied_at": "x"}})
    )
    (state / "applied" / "head_chain.json").write_text(head_chain_json({CAMERA: CAMERA_ORIGIN}, "n", "s", "r"))
    sid = make_session(store, cfg, extra={CAMERA: CAMERA_ORIGIN})
    result = compute_head_arm(cfg, store, sid, NOW)
    assert result["passed"] is True, (result["failed_limits"], result["error"])
    chain = json.loads((result_dir(store, sid, result["id"]) / "head_chain.json").read_text())
    assert set(chain["joints"]) == {CAMERA, MOUNT, PAN, TILT}
    assert chain["joints"][CAMERA] == {"rpy": list(CAMERA_ORIGIN.rpy), "xyz": list(CAMERA_ORIGIN.xyz)}
    assert chain["joints"][MOUNT]["xyz"][0] == pytest.approx(result["params"]["mount_x_m"]["value"], abs=1e-6)
    assert chain["joints"][MOUNT]["rpy"][2] == pytest.approx(
        math.radians(result["params"]["mount_yaw_deg"]["value"]), abs=1e-6
    )


def test_pan_axis_written_to_head_chain_and_applied(cfg: NodeConfig, state: Path, store: SessionStore) -> None:
    """The fitted pan axis goes into head_chain.json (head_pan origin, head_tilt moved back) and through apply into
    the URDF every consumer builds (apply_overrides)."""
    write_intrinsics(state)
    put_marker(state, head_chain={"session": FLOOR_SID, "result": FLOOR_RID, "applied_at": "2026-10-22T07:00:00+00:00"})
    write_floor_result(state, cfg, FLOOR_SID, FLOOR_RID)
    sid = make_session(store, cfg, pan_axis_m=PAN_AXIS_M)
    result = compute_head_arm(cfg, store, sid, NOW)
    assert result["passed"] is True, (result["failed_limits"], result["error"])
    for name, want in zip(("pan_axis_x_m", "pan_axis_y_m"), PAN_AXIS_M, strict=True):
        entry = result["params"][name]
        assert entry["value"] == pytest.approx(want, abs=0.0015), name
        assert (entry["start"], entry["floor"], entry["floor_delta"]) == (0.0, None, None)
    assert [e["head_offset"] for e in result["offset_rms_px"]] == [0, 1, 2, 3]
    chain = json.loads((result_dir(store, sid, result["id"]) / "head_chain.json").read_text())
    urdf = UrdfChain.from_text(Path(cfg.head_urdf_file).read_text())
    pan_xyz = chain["joints"][PAN]["xyz"]
    assert pan_xyz[:2] == pytest.approx(
        [result["params"]["pan_axis_x_m"]["value"], result["params"]["pan_axis_y_m"]["value"]], abs=1e-6
    )
    tilt_xyz = urdf.origin(TILT)[0]
    assert chain["joints"][TILT]["xyz"] == pytest.approx(
        [tilt_xyz[0] - pan_xyz[0], tilt_xyz[1] - pan_xyz[1], tilt_xyz[2]], abs=1e-6
    )
    apply_head_arm(cfg, store, sid, result["id"], NOW, fake_run)
    built = UrdfChain.from_text(base_urdf(cfg))
    assert built.origin(PAN)[0] == pytest.approx(tuple(pan_xyz), abs=1e-6)


def test_missing_intrinsics_is_an_error_result(cfg: NodeConfig, state: Path, store: SessionStore) -> None:
    sid = make_session(store, cfg, poses=2, head_offsets=[(0.0, 0.0)])
    result = compute_head_arm(cfg, store, sid, NOW)
    assert result["passed"] is False
    assert result["error"] == "no applied head camera calibration"
    directory = result_dir(store, sid, result["id"])
    assert json.loads((directory / "result.json").read_text()) == result
    assert not (directory / "head_chain.json").exists()


def test_intrinsics_size_mismatch_names_both(cfg: NodeConfig, state: Path, store: SessionStore) -> None:
    write_intrinsics(state, (640, 480))
    sid = make_session(store, cfg, poses=2, head_offsets=[(0.0, 0.0)])
    result = compute_head_arm(cfg, store, sid, NOW)
    assert result["passed"] is False
    assert "640" in result["error"] and "576" in result["error"]


def test_failed_limit_writes_only_report_and_result(cfg: NodeConfig, ready: str, store: SessionStore) -> None:
    strict = cfg.model_copy(update={"fit": HeadArmFitConfig(limits=HeadArmFitLimits(rms_px=0.001))})
    result = compute_head_arm(strict, store, ready, NOW)
    assert result["passed"] is False
    assert "rms_px" in result["failed_limits"]
    assert result["error"] is None
    directory = result_dir(store, ready, result["id"])
    assert sorted(p.name for p in directory.iterdir()) == ["report.md", "result.json"]


def test_floor_values_from_applied_record(cfg: NodeConfig, state: Path, store: SessionStore) -> None:
    write_intrinsics(state)
    # The head_chain record names a head_arm apply whose floor entry is the real floor result.
    put_marker(
        state,
        head_chain={
            "session": "calib-20261022-100000",
            "result": "r-20261022-100500",
            "applied_at": "x",
            "source": "head_arm",
            "floor": OTHER_FLOOR,
        },
    )
    write_floor_result(state, cfg, OTHER_FLOOR["session"], OTHER_FLOOR["result"], offset=0.0)
    sid = make_session(store, cfg)
    result = compute_head_arm(cfg, store, sid, NOW)
    assert result["floor"] == OTHER_FLOOR
    assert result["passed"] is True, result["failed_limits"]


def test_legacy_floor_record_is_the_reference(cfg: NodeConfig, ready: str, store: SessionStore) -> None:
    # `ready` holds the legacy {session, result, applied_at} record of the floor apply.
    result = compute_head_arm(cfg, store, ready, NOW)
    assert result["floor"] == {"session": FLOOR_SID, "result": FLOOR_RID}
    assert result["params"]["mount_z_m"]["floor_delta"] == pytest.approx(0.0, abs=0.003)


def test_no_floor_record_fails_cross_check(cfg: NodeConfig, state: Path, store: SessionStore) -> None:
    write_intrinsics(state)
    put_marker(state)
    sid = make_session(store, cfg)
    result = compute_head_arm(cfg, store, sid, NOW)
    assert result["floor"] is None
    assert result["passed"] is False
    assert "floor_cross_check" in result["failed_limits"]
    directory = result_dir(store, sid, result["id"])
    assert not (directory / "head_chain.json").exists()


def test_apply_writes_applied_chain_and_record(cfg: NodeConfig, state: Path, store: SessionStore, ready: str) -> None:
    result = compute_head_arm(cfg, store, ready, NOW)
    body = apply_head_arm(cfg, store, ready, result["id"], NOW, fake_run)
    source = result_dir(store, ready, result["id"]) / "head_chain.json"
    assert (state / "applied" / "head_chain.json").read_bytes() == source.read_bytes()
    record = {
        "session": ready,
        "result": result["id"],
        "applied_at": NOW.isoformat(),
        "source": "head_arm",
        "floor": {"session": FLOOR_SID, "result": FLOOR_RID},
    }
    marker = json.loads((state / "applied" / "applied.json").read_text())
    assert marker["head_chain"] == record
    assert marker["session"] == "calib-20261019-120000"
    assert body == {
        "head_chain": record,
        "units": [{"name": "unit-a.service", "active_state": "active", "restarted": True}],
    }
    found = applied_head_chain(cfg)
    assert found is not None and found["source"] == "head_arm"
    assert found["floor"] == {"session": FLOOR_SID, "result": FLOOR_RID}
    assert not list((state / "applied").glob(".tmp-*"))


def test_failed_result_not_applicable(cfg: NodeConfig, state: Path, store: SessionStore) -> None:
    write_intrinsics(state)
    put_marker(state)
    sid = make_session(store, cfg)
    result = compute_head_arm(cfg, store, sid, NOW)
    with pytest.raises(NotApplicable):
        apply_head_arm(cfg, store, sid, result["id"], NOW, fake_run)
    assert not (state / "applied" / "head_chain.json").exists()
