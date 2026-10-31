"""Tests for the arm offsets compute and apply on a head_arm session (design D9, D17).

The scene is the synthetic arm offsets scene of tests/head_arm_synthetic.py: the gripper-held plate in the D435 colour
camera through a truth head chain and true arm offsets, stored as a head_arm session with the synthetic colour
camera_info, plus a floor session whose observations.json holds the truth floor planes. The head URDF file of the node
config is the synthetic applied chain, so `floor_results.base_urdf` is the chain the scene was rendered from.
"""

import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import pytest
import yaml
from ros2_common.head_rgbd_calibration.arm_offsets_fit import TOOL_NOT_MEASURED_NOTE
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, ArmFitLimits, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.floor_fit import FloorObservation

from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.arm_results import (
    NO_HEAD_CALIBRATION_ERROR,
    SNIPPET_FILE,
    apply_arm,
    compute_arm,
)
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import SessionStore
from tests.pose_factory import still_arm_pose
from tests.test_head_arm_results import fake_run, put_marker, synthetic, synthetic_camera_info

NOW = datetime(2026, 10, 31, 12, 0, 0, tzinfo=UTC)
LATER = datetime(2026, 10, 31, 12, 5, 0, tzinfo=UTC)
FLOOR_SID = "floor-20261031-110000"
FLOOR_RID = "r-20261031-111000"
OTHER_FLOOR_SID = "floor-20261030-090000"
NOISE_PX = 0.3
SEED = 11
FLOOR_SEED = 5
OFFSET_TOLERANCE_RAD = 0.004
PASSED_FILES = {"result.json", "report.md", "head_chain.json", "head.urdf", SNIPPET_FILE}
FAILED_FILES = {"result.json", "report.md"}
RESULT_KEYS = {
    "id", "session", "kind", "floor_session", "floor", "created", "passed", "failed_limits", "error", "rms_px",
    "holdout_rms_px", "holdout_tool_mm", "wrist_roll_fitted", "tool_fitted", "tool_offset_m", "offsets", "head",
    "limits", "snippet",
}  # fmt: skip


def write_observations(state: Path, sid: str, observations: list[FloorObservation]) -> None:
    """A floor session's observations.json, as the sweep writes it."""
    directory = state / "floor_fit" / sid
    directory.mkdir(parents=True, exist_ok=True)
    doc = {
        "session": sid,
        "observations": [
            {
                "pose": o.pose,
                "pan_rad": o.pan_rad,
                "tilt_rad": o.tilt_rad,
                "normal": [float(v) for v in o.plane.normal],
                "height_m": o.plane.height_m,
                "inliers": o.plane.inliers,
                "candidates": o.plane.candidates,
                "rms_m": o.plane.rms_m,
            }
            for o in observations
        ],
    }
    (directory / "observations.json").write_text(json.dumps(doc))


def write_floor_result_dir(state: Path, sid: str, rid: str) -> None:
    """A floor result directory with a result.json (the floor fit's own output is not read by the arm fit)."""
    directory = state / "floor_fit" / sid / "results" / rid
    directory.mkdir(parents=True)
    (directory / "result.json").write_text(json.dumps({"id": rid, "session": sid, "passed": True}))


def node_config(tmp_path: Path, fit: HeadArmFitConfig) -> NodeConfig:
    head = tmp_path / "head.urdf"
    head.write_text(synthetic.synth_head_urdf())
    arm = tmp_path / "so101_arm.urdf"
    arm.write_text(synthetic.synth_arm_urdf())
    return NodeConfig(
        state_dir=str(tmp_path / "state"),
        head_urdf_file=str(head),
        arm_urdf_file=str(arm),
        restart_units=["unit-a.service"],
        fit=fit,
    )


def make_arm_session(store: SessionStore, truth: Any) -> str:
    """A head_arm session holding the synthetic plate views with all seven joints and the colour camera_info."""
    views = synthetic.plate_scene(truth, synthetic.PLATE_ARM_POSES, NOISE_PX, SEED)
    sid = store.create_session(NOW, kind="head_arm")
    info = synthetic_camera_info()
    for view in views:
        still = still_arm_pose(view.joints, arm_pose=view.arm_pose, tier=view.tier, head_offset=view.head_offset)
        store.add_still_pose(
            sid,
            still.rgb_jpeg,
            still.rgb_image,
            view.detection,
            view.joints,
            still.stamps,
            view.arm_pose,
            view.tier,
            view.head_offset,
            info,
        )
    return sid


class Scene:
    """A state dir with an accepted intrinsics check, a floor session and a head_arm session of plate views."""

    def __init__(self, tmp_path: Path, tool: bool = True, **fit: object) -> None:
        self.truth = synthetic.plate_truth()
        tool_in_board = synthetic.tool_in_plate(self.truth) if tool else None
        self.cfg = node_config(tmp_path, synthetic.synth_plate_config(self.truth, tool_in_board_m=tool_in_board, **fit))
        self.state = Path(self.cfg.state_dir)
        (self.state / "applied").mkdir(parents=True)
        put_marker(self.state, kind="intrinsics")
        floor = synthetic.floor_scene(
            self.truth, synthetic.FLOOR_GRID, synthetic.FLOOR_NOISE_DEG, synthetic.FLOOR_NOISE_MM, FLOOR_SEED
        )
        write_observations(self.state, FLOOR_SID, floor)
        write_floor_result_dir(self.state, FLOOR_SID, FLOOR_RID)
        self.store = SessionStore(self.state, self.cfg)
        self.sid = make_arm_session(self.store, self.truth)

    def apply_floor_chain(self) -> None:
        """Record the floor session as the floor reference of the applied head chain."""
        marker = self.state / "applied" / "applied.json"
        data = json.loads(marker.read_text())
        data["head_chain"] = {
            "session": FLOOR_SID,
            "result": FLOOR_RID,
            "applied_at": "2026-10-31T11:15:00+00:00",
            "source": "floor",
            "floor": {"session": FLOOR_SID, "result": FLOOR_RID},
        }
        marker.write_text(json.dumps(data))


@pytest.fixture(scope="module")
def passed(tmp_path_factory: pytest.TempPathFactory) -> tuple[Scene, dict[str, Any]]:
    """One passing arm compute with the tool offset measured and the floor session named in the request."""
    scene = Scene(tmp_path_factory.mktemp("arm"))
    return scene, compute_arm(scene.cfg, scene.store, scene.sid, NOW, FLOOR_SID)


def test_arm_result_files_and_snippet(passed: tuple[Scene, dict[str, Any]]) -> None:
    scene, result = passed
    assert result["passed"] is True, (result["failed_limits"], result["error"])
    assert set(result) == RESULT_KEYS
    assert (result["kind"], result["session"], result["floor_session"]) == ("arm", scene.sid, FLOOR_SID)
    assert result["floor"] == {"session": FLOOR_SID, "result": FLOOR_RID}
    directory = scene.store.result_dir(scene.sid, result["id"])
    assert {p.name for p in directory.iterdir()} == PASSED_FILES
    assert json.loads((directory / "result.json").read_text()) == result
    assert set(result["offsets"]) == set(ARM_JOINT_NAMES)
    for joint in ("shoulder_lift", "elbow_flex", "wrist_flex"):
        entry = result["offsets"][joint]
        assert set(entry) == {"value", "sigma", "current"}
        assert entry["current"] == scene.cfg.fit.joint_offsets_rad[joint]
        assert abs(entry["value"] - scene.truth.joint_offsets_rad[joint]) <= OFFSET_TOLERANCE_RAD
    assert result["offsets"]["shoulder_pan"]["sigma"] is None  # the gauge is fixed
    assert set(result["head"]["mount_x_m"]) == {"value", "sigma", "urdf"}
    assert result["tool_fitted"] is True and len(result["tool_offset_m"]) == 3
    snippet = (directory / SNIPPET_FILE).read_text()
    assert result["snippet"] == snippet
    block = yaml.safe_load(snippet)
    assert set(block["joint_offsets_rad"]) == set(ARM_JOINT_NAMES) and "tool_offset_m" in block
    chain = json.loads((directory / "head_chain.json").read_text())
    assert {"head_mount_joint", "head_tilt", "d435_joint"} <= set(chain["joints"])
    assert "Arm offsets fit" in (directory / "report.md").read_text()
    assert all(set(check) == {"name", "value", "limit", "ok"} for check in result["limits"])


def test_arm_compute_needs_head_calibration(tmp_path: Path) -> None:
    """No floor session in the request and no applied head chain: an error result, no fit, no other file."""
    scene = Scene(tmp_path)
    result = compute_arm(scene.cfg, scene.store, scene.sid, NOW, None)
    assert (result["passed"], result["error"], result["kind"]) == (False, NO_HEAD_CALIBRATION_ERROR, "arm")
    assert NO_HEAD_CALIBRATION_ERROR == "apply a head calibration first"
    assert {p.name for p in scene.store.result_dir(scene.sid, result["id"]).iterdir()} == {"result.json"}


def test_floor_session_arg_wins(tmp_path: Path) -> None:
    """The request's floor session wins over the applied chain's floor reference; without one the reference is used."""
    scene = Scene(tmp_path)
    scene.apply_floor_chain()
    marker = scene.state / "applied" / "applied.json"
    data = json.loads(marker.read_text())
    data["head_chain"]["floor"] = {"session": OTHER_FLOOR_SID, "result": FLOOR_RID}
    marker.write_text(json.dumps(data))
    named = compute_arm(scene.cfg, scene.store, scene.sid, NOW, FLOOR_SID)
    assert named["floor_session"] == FLOOR_SID and named["error"] is None
    fallback = compute_arm(scene.cfg, scene.store, scene.sid, LATER, None)
    assert fallback["floor_session"] == OTHER_FLOOR_SID
    assert fallback["passed"] is False and OTHER_FLOOR_SID in fallback["error"]  # that session was never swept


def test_failed_limits_write_only_report_and_result(tmp_path: Path) -> None:
    scene = Scene(tmp_path, arm_limits=ArmFitLimits(rms_px=0.01))
    scene.apply_floor_chain()
    result = compute_arm(scene.cfg, scene.store, scene.sid, NOW, None)
    assert result["passed"] is False and result["error"] is None
    assert result["floor_session"] == FLOOR_SID
    assert "rms_px" in result["failed_limits"]
    assert {p.name for p in scene.store.result_dir(scene.sid, result["id"]).iterdir()} == FAILED_FILES


def test_tool_not_fitted_without_tool_in_board_m(tmp_path: Path) -> None:
    scene = Scene(tmp_path, tool=False)
    result = compute_arm(scene.cfg, scene.store, scene.sid, NOW, FLOOR_SID)
    assert result["tool_fitted"] is False
    assert result["tool_offset_m"] == list(scene.cfg.fit.tool_offset_m)
    directory = scene.store.result_dir(scene.sid, result["id"])
    assert TOOL_NOT_MEASURED_NOTE in (directory / "report.md").read_text()
    assert result["passed"] is True, (result["failed_limits"], result["error"])
    assert "tool_offset_m" not in yaml.safe_load((directory / SNIPPET_FILE).read_text())


@pytest.mark.parametrize(
    ("change", "key"),
    [
        ({"plate_mount": None}, "plate_mount"),
        ({"wrist_roll_offset_rad": None}, "wrist_roll_offset_rad"),
    ],
)
def test_missing_plate_mount_is_an_error(tmp_path: Path, change: dict[str, Any], key: str) -> None:
    scene = Scene(tmp_path)
    cfg = scene.cfg.model_copy(update={"fit": scene.cfg.fit.model_copy(update=change)})
    result = compute_arm(cfg, scene.store, scene.sid, NOW, FLOOR_SID)
    assert result["passed"] is False and key in result["error"]
    assert {p.name for p in scene.store.result_dir(scene.sid, result["id"]).iterdir()} == {"result.json"}


def test_arm_apply_writes_chain_not_snippet(passed: tuple[Scene, dict[str, Any]]) -> None:
    scene, result = passed
    applied = scene.state / "applied"
    before = json.loads((applied / "applied.json").read_text())
    body = apply_arm(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    record = body["head_chain"]
    assert record == {
        "session": scene.sid,
        "result": result["id"],
        "applied_at": LATER.isoformat(),
        "source": "arm",
        "floor": {"session": FLOOR_SID, "result": FLOOR_RID},
    }
    marker = json.loads((applied / "applied.json").read_text())
    assert marker["kind"] == before["kind"] == "intrinsics"
    assert marker["head_chain"] == record
    assert {p.name for p in applied.iterdir()} == {"applied.json", "head_chain.json"}
    source = scene.store.result_dir(scene.sid, result["id"]) / "head_chain.json"
    assert (applied / "head_chain.json").read_text() == source.read_text()


def test_arm_apply_refuses_a_failed_result(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    result = compute_arm(scene.cfg, scene.store, scene.sid, NOW, None)
    with pytest.raises(NotApplicable):
        apply_arm(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
