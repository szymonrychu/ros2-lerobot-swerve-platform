"""Tests for the head floor fit compute, apply and applied record (synthetic observations, tmp state dir)."""

import importlib.util
import json
import math
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.floor_fit import predicted_floor
from ros2_common.head_rgbd_calibration.urdf_text import Origin, head_chain_json

from head_rgbd_calibration import floor_results
from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_results import (
    applied_head_chain,
    apply_floor,
    base_urdf,
    compute_floor,
    read_result,
)
from head_rgbd_calibration.store import InvalidId, NotFound

REPO_ROOT = Path(__file__).resolve().parents[3]
URDF_PATH = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"
SYNTHETIC = importlib.util.spec_from_file_location(
    "head_floor_synthetic", REPO_ROOT / "tests" / "head_floor_synthetic.py"
)
assert SYNTHETIC is not None and SYNTHETIC.loader is not None
synthetic = importlib.util.module_from_spec(SYNTHETIC)
SYNTHETIC.loader.exec_module(synthetic)

SID = "floor-20261021-120000"
NOW = datetime(2026, 10, 21, 12, 30, 0, tzinfo=UTC)
MOUNT = "head_mount_joint"
TILT = "head_tilt"
CAMERA = "d435_joint"
PAN = "head_pan"
TRUTH_DZ_M = 0.03
TRUTH_ROLL_DEG = 1.5
TRUTH_PITCH_DEG = -3.0
TRUTH_TILT_ZERO_DEG = 2.0
GRID_PANS_DEG = (-60, -30, 0, 30, 60)
GRID_TILTS_DEG = (12, 20, 28)
PER_POSE = 3
HEIGHT_NOISE_M = 0.002
CAMERA_XYZ = [0.01, 0.02, 0.03]
CAMERA_RPY = [0.1, 0.2, 0.3]


def truth_origins(chain: UrdfChain) -> dict[str, Origin]:
    (mx, my, mz), (mr, mp, myaw) = chain.origin(MOUNT)
    txyz, (tr, tp, tyaw) = chain.origin(TILT)
    return {
        MOUNT: Origin(
            (mx, my, mz + TRUTH_DZ_M),
            (mr + math.radians(TRUTH_ROLL_DEG), mp + math.radians(TRUTH_PITCH_DEG), myaw),
        ),
        TILT: Origin(txyz, (tr, tp + math.radians(TRUTH_TILT_ZERO_DEG), tyaw)),
    }


def write_observations(state: Path, sid: str, per_pose: int = PER_POSE) -> None:
    """observations.json whose planes are the truth floor seen from the perturbed chain, with small height noise."""
    chain = UrdfChain.from_text(URDF_PATH.read_text())
    truth = truth_origins(chain)
    rng = np.random.default_rng(3)
    observations = []
    pose = 0
    for pan in GRID_PANS_DEG:
        for tilt in GRID_TILTS_DEG:
            normal, height = predicted_floor(chain, math.radians(pan), math.radians(tilt), truth)
            for _ in range(per_pose):
                observations.append(
                    {
                        "pose": pose,
                        "pan_rad": math.radians(pan),
                        "tilt_rad": math.radians(tilt),
                        "normal": [float(v) for v in normal],
                        "height_m": float(height + rng.normal(0.0, HEIGHT_NOISE_M)),
                        "inliers": 1000,
                        "candidates": 1200,
                        "rms_m": 0.002,
                        "stamp_ns": 1,
                    }
                )
            pose += 1
    directory = state / "floor_fit" / sid
    directory.mkdir(parents=True)
    (directory / "observations.json").write_text(
        json.dumps(
            {
                "session": sid,
                "created": "2026-10-21T12:00:00+00:00",
                "grid": [],
                "observations": observations,
                "skipped": [{"pose": 99, "pan_deg": 0.0, "tilt_deg": 0.0, "reason": "no valid plane"}],
                "dropped": {},
            }
        )
    )


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig(
        state_dir=str(tmp_path / "state"), head_urdf_file=str(URDF_PATH), restart_units=["unit-a.service"]
    )


@pytest.fixture
def state(cfg: NodeConfig) -> Path:
    path = Path(cfg.state_dir)
    path.mkdir(parents=True)
    return path


def put_applied(state: Path, **extra: Any) -> None:
    applied = state / "applied"
    applied.mkdir(exist_ok=True)
    marker = {
        "session": "calib-20261019-120000",
        "result": "r-20261019-123045",
        "applied_at": "2026-10-19T12:31:00+00:00",
        "files": ["tof_joint.json"],
        **extra,
    }
    (applied / "applied.json").write_text(json.dumps(marker))


def fake_run(args: list[str], **_: Any) -> SimpleNamespace:
    return SimpleNamespace(stdout="ActiveEnterTimestamp=@2000000000\nActiveState=active\n", returncode=0)


def failing_state(state: Path) -> None:
    write_observations(state, SID, per_pose=0)


def test_compute_keeps_applied_pan_axis(cfg: NodeConfig, state: Path) -> None:
    """A head-from-arm pan axis in the applied chain (head_pan origin) survives a floor compute."""
    write_observations(state, SID)
    applied = state / "applied"
    applied.mkdir()
    pan = Origin((-0.038, -0.011, 0.0), (0.0, 0.0, 0.0))
    (applied / "head_chain.json").write_text(head_chain_json({PAN: pan}, "head-from-arm", "calib-1", "r-1"))
    result = compute_floor(cfg, SID, NOW)
    assert result["passed"] is True, result["failed_limits"]
    chain = json.loads((state / "floor_fit" / SID / "results" / result["id"] / "head_chain.json").read_text())
    assert set(chain["joints"]) == {MOUNT, TILT, PAN}
    assert chain["joints"][PAN] == {"rpy": list(pan.rpy), "xyz": list(pan.xyz)}


def test_compute_writes_result_and_head_chain(cfg: NodeConfig, state: Path) -> None:
    write_observations(state, SID)
    applied = state / "applied"
    applied.mkdir()
    (applied / "head_chain.json").write_text(
        head_chain_json({CAMERA: Origin(tuple(CAMERA_XYZ), tuple(CAMERA_RPY))}, "earlier", "calib-1", "r-1")
    )
    result = compute_floor(cfg, SID, NOW)
    assert result["passed"] is True, result["failed_limits"]
    assert result["id"] == "r-20261021-123000"
    assert result["session"] == SID
    assert set(result) == {
        "id", "session", "created", "passed", "failed_limits", "params", "height_rms_mm", "normal_rms_deg",
        "observation_height_rms_mm", "observation_normal_rms_deg", "poses", "skipped",
    }  # fmt: skip
    assert result["observation_height_rms_mm"] >= result["height_rms_mm"]
    assert len(result["poses"]) == len(GRID_PANS_DEG) * len(GRID_TILTS_DEG)
    assert result["skipped"][0]["pose"] == 99
    params = result["params"]
    base_z = UrdfChain.from_text(URDF_PATH.read_text()).origin(MOUNT)[0][2]
    assert params["mount_z_m"]["value"] == pytest.approx(base_z + TRUTH_DZ_M, abs=0.005)
    assert params["mount_roll_deg"]["value"] == pytest.approx(TRUTH_ROLL_DEG, abs=0.2)
    directory = state / "floor_fit" / SID / "results" / result["id"]
    assert json.loads((directory / "result.json").read_text()) == result
    chain = json.loads((directory / "head_chain.json").read_text())
    assert set(chain["joints"]) == {MOUNT, TILT, CAMERA}
    assert chain["joints"][CAMERA] == {"rpy": CAMERA_RPY, "xyz": CAMERA_XYZ}
    assert chain["joints"][MOUNT]["xyz"][2] == pytest.approx(base_z + TRUTH_DZ_M, abs=0.005)
    assert chain["joints"][TILT]["rpy"][0] == pytest.approx(math.radians(params["tilt_roll_deg"]["value"]), abs=1e-6)
    assert chain["joints"][TILT]["rpy"][1] == pytest.approx(math.radians(params["tilt_zero_deg"]["value"]), abs=1e-6)
    assert "height RMS" in chain["note"] and "normal RMS" in chain["note"] and "2026-10-21" in chain["note"]
    assert (chain["session"], chain["result"]) == (SID, result["id"])
    assert read_result(cfg, SID, result["id"]) == result


def test_failed_compute_writes_no_head_chain(cfg: NodeConfig, state: Path) -> None:
    failing_state(state)
    result = compute_floor(cfg, SID, NOW)
    assert result["passed"] is False
    assert "min_observations" in result["failed_limits"]
    directory = state / "floor_fit" / SID / "results" / result["id"]
    assert (directory / "result.json").is_file()
    assert not (directory / "head_chain.json").exists()


def test_failing_recompute_in_same_second_removes_stale_head_chain(cfg: NodeConfig, state: Path) -> None:
    write_observations(state, SID)
    first = compute_floor(cfg, SID, NOW)
    directory = state / "floor_fit" / SID / "results" / first["id"]
    assert (directory / "head_chain.json").is_file()
    obs = state / "floor_fit" / SID / "observations.json"
    doc = json.loads(obs.read_text())
    doc["observations"] = doc["observations"][:3]
    obs.write_text(json.dumps(doc))
    second = compute_floor(cfg, SID, NOW)
    assert second["passed"] is False
    assert not (directory / "head_chain.json").exists()


def test_compute_rejects_bad_and_unknown_ids(cfg: NodeConfig, state: Path) -> None:
    with pytest.raises(InvalidId):
        compute_floor(cfg, "../etc", NOW)
    with pytest.raises(NotFound):
        compute_floor(cfg, "floor-20200101-000000", NOW)
    write_observations(state, SID)
    with pytest.raises(InvalidId):
        read_result(cfg, SID, "bad")
    with pytest.raises(NotFound):
        read_result(cfg, SID, "r-20200101-000000")


def test_base_urdf_applies_tof_joint_and_head_chain(cfg: NodeConfig, state: Path) -> None:
    applied = state / "applied"
    applied.mkdir()
    (applied / "head_chain.json").write_text(
        head_chain_json({MOUNT: Origin((0.1, 0.2, 0.9), (0.0, 0.0, 0.0))}, "n", "s", "r")
    )
    (applied / "tof_joint.json").write_text(
        json.dumps({"xyz": [0.01, 0.02, 0.03], "rpy": [0.0, 0.0, 0.0], "note": "x"})
    )
    chain = UrdfChain.from_text(base_urdf(cfg))
    assert chain.origin(MOUNT)[0] == pytest.approx((0.1, 0.2, 0.9))
    assert chain.origin("tof_joint")[0] == pytest.approx((0.01, 0.02, 0.03))


def test_apply_keeps_applied_keys_and_adds_record(cfg: NodeConfig, state: Path) -> None:
    write_observations(state, SID)
    put_applied(state)
    result = compute_floor(cfg, SID, NOW)
    body = apply_floor(cfg, SID, result["id"], NOW, fake_run)
    applied = state / "applied"
    source = state / "floor_fit" / SID / "results" / result["id"] / "head_chain.json"
    assert (applied / "head_chain.json").read_bytes() == source.read_bytes()
    marker = json.loads((applied / "applied.json").read_text())
    assert marker["session"] == "calib-20261019-120000"
    assert marker["files"] == ["tof_joint.json"]
    record = {
        "session": SID,
        "result": result["id"],
        "applied_at": NOW.isoformat(),
        "source": "floor",
        "floor": {"session": SID, "result": result["id"]},
    }
    assert marker["head_chain"] == record
    assert body == {
        "head_chain": record,
        "units": [{"name": "unit-a.service", "active_state": "active", "restarted": True}],
    }
    assert not list(applied.glob(".tmp-*"))


def test_floor_apply_records_source(cfg: NodeConfig, state: Path) -> None:
    write_observations(state, SID)
    put_applied(state)
    result = compute_floor(cfg, SID, NOW)
    body = apply_floor(cfg, SID, result["id"], NOW, fake_run)
    record = body["head_chain"]
    assert record["source"] == "floor"
    assert record["floor"] == {"session": SID, "result": result["id"]}


def test_apply_refusals(cfg: NodeConfig, state: Path) -> None:
    failing_state(state)
    put_applied(state)
    failed = compute_floor(cfg, SID, NOW)
    with pytest.raises(NotApplicable):
        apply_floor(cfg, SID, failed["id"], NOW, fake_run)
    assert not (state / "applied" / "head_chain.json").exists()


def test_apply_refused_without_rgbd_calibration(cfg: NodeConfig, state: Path) -> None:
    write_observations(state, SID)
    result = compute_floor(cfg, SID, NOW)
    with pytest.raises(NotApplicable, match="RGB-D"):
        apply_floor(cfg, SID, result["id"], NOW, fake_run)
    assert not (state / "applied").exists()


def test_applied_head_chain_survives_applied_json_rewrite(cfg: NodeConfig, state: Path) -> None:
    assert applied_head_chain(cfg) is None
    write_observations(state, SID)
    put_applied(state)
    result = compute_floor(cfg, SID, NOW)
    apply_floor(cfg, SID, result["id"], NOW, fake_run)
    put_applied(state)  # an RGB-D apply rewrites applied.json without the head_chain key
    assert "head_chain" not in json.loads((state / "applied" / "applied.json").read_text())
    found = applied_head_chain(cfg)
    assert found is not None
    assert (found["session"], found["result"]) == (SID, result["id"])
    assert set(found["joints"]) == {MOUNT, TILT}
    assert len(found["joints"][MOUNT]["xyz"]) == 3
    assert "floor fit" in found["note"]


def test_applied_head_chain_none_when_invalid(cfg: NodeConfig, state: Path) -> None:
    (state / "applied").mkdir()
    (state / "applied" / "head_chain.json").write_text("{not json")
    assert applied_head_chain(cfg) is None
    assert floor_results.applied_chain_origins(state / "applied") == {}
