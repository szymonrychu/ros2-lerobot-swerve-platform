"""Tests for the arm to camera compute, accept, decline and state (head_rgbd_calibration/arm_camera_results.py).

The scene is the synthetic arm to camera scene of tests/head_arm_synthetic.py (`arm_camera_scene`): the gripper board
seen through the applied head chain (fixed), with arm offsets, an arm base, a board seat skew and a gravity sag that
differ from the configured (applied) values, stored as a head_arm session with the synthetic colour camera_info. The
applied head chain is a floor-fit head_chain.json equal to the chain the scene was rendered with.
"""

import dataclasses
import json
import logging
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import pytest
import yaml
from ros2_common.head_rgbd_calibration.arm_calibration_file import (
    ARM_CALIBRATION_FILE,
    ArmCalibrationFile,
    read_arm_calibration,
    zero_record_path,
)
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, ArmCameraBounds, ArmCameraFitConfig
from ros2_common.head_rgbd_calibration.head_arm_fit import MOUNT_JOINT, TILT_JOINT
from ros2_common.head_rgbd_calibration.urdf_text import Origin, head_chain_json

from head_rgbd_calibration import arm_camera_results, floor_results
from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.arm_camera_results import (
    KIND,
    NO_HEAD_CHAIN_REASON,
    accept_arm_camera,
    arm_camera_state,
    captured_from,
    compute_arm_camera,
    decline_arm_camera,
    read_arm_camera_result,
)
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import SessionStore
from head_rgbd_calibration.telemetry import events_in
from tests.applied_writes import interleave_apply, record_applied_writes
from tests.pose_factory import STAMP_NS, still_arm_pose
from tests.test_compute_arm import INTRINSICS_BOARD
from tests.test_head_arm_results import fake_run, put_marker, synthetic, synthetic_camera_info

NOW = datetime(2026, 11, 9, 12, 0, 0, tzinfo=UTC)
LATER = datetime(2026, 11, 9, 12, 5, 0, tzinfo=UTC)
NOISE_PX = 0.3
SEED = 23
FLOOR_SID = "floor-20261109-100000"
FLOOR_RID = "r-20261109-101000"
# The lift, elbow and wrist_flex offsets are about 1.5 mrad one sigma on this scene (parallel axes, correlated).
OFFSET_TOLERANCE_RAD = 0.005
LOGGER = "head_rgbd_calibration.arm_camera_results"
RESULT_KEYS = {
    "id", "session", "kind", "state", "error", "created_at", "declined", "declined_at", "accepted_at", "head_chain",
    "report", "calibration",
}  # fmt: skip
CALIBRATION_KEYS = {"joint_offsets_rad", "base_in_base_link", "tool_offset_m", "snippet"}
# Capture time of the synthetic stills (pose_factory STAMP_NS, 2023-11-14T22:13:20Z).
CAPTURED_FROM = datetime.fromtimestamp(STAMP_NS / 1e9, tz=UTC)


def node_config(tmp_path: Path, **arm_camera: Any) -> NodeConfig:
    """Node config of the synthetic scene: the synthetic head and arm URDF, the arm to camera fit config."""
    head = tmp_path / "head.urdf"
    head.write_text(synthetic.synth_head_urdf())
    arm = tmp_path / "so101_arm.urdf"
    arm.write_text(synthetic.synth_arm_urdf())
    return NodeConfig(
        state_dir=str(tmp_path / "state"),
        arm_stale_marker=str(tmp_path / "arm" / "calibration_stale.json"),
        head_urdf_file=str(head),
        arm_urdf_file=str(arm),
        restart_units=["unit-a.service"],
        board=INTRINSICS_BOARD,
        fit=synthetic.arm_camera_config(**arm_camera),
    )


def apply_head_chain(state: Path, source: str = "floor") -> None:
    """An applied head_chain.json equal to the synthetic applied chain, recorded with `source`."""
    chain = UrdfChain.from_text(synthetic.synth_head_urdf())
    origins = {name: Origin(*chain.origin(name)) for name in (MOUNT_JOINT, TILT_JOINT)}
    (state / "applied" / "head_chain.json").write_text(head_chain_json(origins, "floor fit", FLOOR_SID, FLOOR_RID))
    marker = json.loads((state / "applied" / "applied.json").read_text())
    marker["head_chain"] = {
        "session": FLOOR_SID,
        "result": FLOOR_RID,
        "applied_at": "2026-11-09T10:15:00+00:00",
        "source": source,
        "floor": {"session": FLOOR_SID, "result": FLOOR_RID},
    }
    (state / "applied" / "applied.json").write_text(json.dumps(marker))


def make_session(store: SessionStore, truth: Any) -> str:
    """A head_arm session holding the synthetic arm to camera views with the colour camera_info."""
    views = synthetic.arm_camera_scene(truth, synthetic.PLATE_ARM_POSES, NOISE_PX, SEED)
    sid = store.create_session(NOW, kind="head_arm")
    info = synthetic_camera_info()
    for view in views:
        still = still_arm_pose(view.joints, arm_pose=view.arm_pose, tier=view.tier, head_offset=view.head_offset)
        store.add_still_pose(
            sid, still.rgb_jpeg, still.rgb_image, view.detection, view.joints, still.stamps, view.arm_pose,
            view.tier, view.head_offset, info,
        )  # fmt: skip
    return sid


class Scene:
    """A state dir with an accepted intrinsics check, an applied floor-fit head chain and a head_arm session."""

    def __init__(self, tmp_path: Path, head_chain: bool = True, **arm_camera: Any) -> None:
        self.truth = synthetic.arm_camera_truth()
        self.cfg = node_config(tmp_path, **arm_camera)
        self.state = Path(self.cfg.state_dir)
        (self.state / "applied").mkdir(parents=True)
        put_marker(self.state, kind="intrinsics")
        if head_chain:
            apply_head_chain(self.state)
        self.store = SessionStore(self.state, self.cfg)
        self.sid = make_session(self.store, self.truth)


@pytest.fixture(scope="module")
def computed(tmp_path_factory: pytest.TempPathFactory) -> tuple[Scene, dict[str, Any]]:
    scene = Scene(tmp_path_factory.mktemp("arm_camera"))
    rid = SessionStore.new_result_id(NOW)
    return scene, compute_arm_camera(scene.cfg, scene.store, scene.sid, rid, NOW)


def test_compute_writes_a_done_plausible_result(computed: tuple[Scene, dict[str, Any]]) -> None:
    scene, result = computed
    assert set(result) == RESULT_KEYS
    assert (result["state"], result["error"], result["kind"]) == ("done", None, KIND)
    assert (result["declined"], result["accepted_at"]) == (False, None)
    assert result["head_chain"] == {"source": "floor_fit", "session": FLOOR_SID, "result": FLOOR_RID}
    report = result["report"]
    assert report["plausible"] is True and report["refusals"] == []
    directory = scene.store.result_dir(scene.sid, result["id"])
    assert json.loads((directory / "result.json").read_text()) == result
    calibration = result["calibration"]
    assert set(calibration) == CALIBRATION_KEYS
    for joint in ARM_JOINT_NAMES:
        assert (
            abs(calibration["joint_offsets_rad"][joint] - scene.truth.joint_offsets_rad[joint]) <= OFFSET_TOLERANCE_RAD
        )
    assert set(calibration["base_in_base_link"]) == {"x", "y", "z", "yaw"}
    assert (directory / "joint_offsets_snippet.yaml").read_text() == calibration["snippet"]
    assert yaml.safe_load(calibration["snippet"])["base_in_base_link"]["x"] == pytest.approx(
        calibration["base_in_base_link"]["x"], abs=1e-4
    )


def test_compute_without_an_applied_head_chain_fails(tmp_path: Path) -> None:
    scene = Scene(tmp_path, head_chain=False)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    assert (result["state"], result["error"], result["report"]) == ("failed", NO_HEAD_CHAIN_REASON, None)
    assert result["calibration"] is None
    assert {p.name for p in scene.store.result_dir(scene.sid, result["id"]).iterdir()} == {"result.json"}


def test_compute_refuses_an_implausible_fit(tmp_path: Path) -> None:
    scene = Scene(tmp_path, bounds=ArmCameraBounds(max_offset_change_deg=1.0))
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    assert result["state"] == "done" and result["report"]["plausible"] is False
    assert "offset_change:wrist_flex" in {r["name"] for r in result["report"]["refusals"]}
    with pytest.raises(NotApplicable, match="not plausible"):
        accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    assert not (scene.state / "applied" / ARM_CALIBRATION_FILE).exists()


def reject_constant(name: str) -> None:
    """json.loads parse_constant hook: NaN and infinities are not JSON."""
    raise ValueError(f"non-JSON constant {name}")


def test_a_non_finite_fit_writes_strict_json(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A fit with a non-finite offset is refused (non_finite) and its result.json stays strict JSON: the
    calibration numbers are null, so the result route can serve it and it can be declined."""
    real_fit = arm_camera_results.fit_arm_camera

    def nan_fit(*args: Any, **kwargs: Any) -> Any:
        fit = real_fit(*args, **kwargs)
        return dataclasses.replace(fit, offsets={**fit.offsets, "shoulder_pan": math.nan})

    monkeypatch.setattr(arm_camera_results, "fit_arm_camera", nan_fit)
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    assert result["state"] == "done" and result["report"]["plausible"] is False
    text = (scene.store.result_dir(scene.sid, result["id"]) / "result.json").read_text()
    stored = json.loads(text, parse_constant=reject_constant)
    assert stored["calibration"]["joint_offsets_rad"]["shoulder_pan"] is None
    declined = decline_arm_camera(scene.store, scene.sid, result["id"], LATER)
    assert declined["declined"] is True


def test_compute_logs_inputs_solver_refusals_and_result(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    scene = Scene(tmp_path, bounds=ArmCameraBounds(max_offset_change_deg=1.0))
    rid = SessionStore.new_result_id(NOW)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        compute_arm_camera(scene.cfg, scene.store, scene.sid, rid, NOW)
    events = events_in(caplog.records)
    assert {(e["flow"], e["run_id"]) for e in events} == {("arm_camera_compute", rid)}
    steps = [e["step"] for e in events]
    assert steps[0] == "inputs" and steps[-1] == "result" and "solver" in steps
    inputs = events[0]["data"]
    assert inputs["head_chain"]["source"] == "floor_fit" and inputs["views"] > 0
    assert set(inputs["applied_offsets_rad"]) == set(ARM_JOINT_NAMES)
    refusals = [e for e in events if e["step"] == "refusal"]
    assert refusals and all(e["outcome"] == "refused" for e in refusals)
    wrist = next(e for e in refusals if e["data"]["name"] == "offset_change:wrist_flex")
    assert wrist["data"]["limit"] == 1.0 and wrist["data"]["value"] > 1.0
    assert events[-1]["outcome"] == "implausible"


def test_accept_writes_the_arm_calibration_and_keeps_the_head_chain(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    applied = scene.state / "applied"
    chain_before = (applied / "head_chain.json").read_text()
    marker_before = json.loads((applied / "applied.json").read_text())
    body = accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    record = body["arm_calibration"]
    assert record == {
        "session": scene.sid,
        "result": result["id"],
        "accepted_at": LATER.isoformat(),
        "score": result["report"]["score"],
    }
    assert body["units"] == [{"name": "unit-a.service", "active_state": "active", "restarted": True}]
    marker = json.loads((applied / "applied.json").read_text())
    assert marker["arm_calibration"] == record
    assert {k: v for k, v in marker.items() if k != "arm_calibration"} == marker_before  # head chain record kept
    assert (applied / "head_chain.json").read_text() == chain_before
    written = json.loads((applied / ARM_CALIBRATION_FILE).read_text())
    assert written["joint_offsets_rad"] == result["calibration"]["joint_offsets_rad"]
    assert written["base_in_base_link"] == result["calibration"]["base_in_base_link"]
    assert written["snippet"] == result["calibration"]["snippet"]
    assert written["accuracy_examples"] == result["report"]["accuracy_examples"]
    assert written["captured_from"] == CAPTURED_FROM.isoformat()
    # What mcp_server applies: the shared reader returns exactly the accepted offsets and base.
    applied_cal = read_arm_calibration(applied)
    assert applied_cal == ArmCalibrationFile.model_validate(written)
    assert applied_cal.joint_offsets_rad == result["calibration"]["joint_offsets_rad"]
    assert applied_cal.base_in_base_link.model_dump() == result["calibration"]["base_in_base_link"]
    assert (applied_cal.session, applied_cal.result, applied_cal.captured_from) == (
        scene.sid,
        result["id"],
        CAPTURED_FROM,
    )
    stored = read_arm_camera_result(scene.store, scene.sid, result["id"], None)
    assert stored["accepted_at"] == LATER.isoformat()
    with pytest.raises(NotApplicable, match="accepted"):
        decline_arm_camera(scene.store, scene.sid, result["id"], LATER)
    state = arm_camera_state(scene.cfg)
    assert state["applied"] == {
        **record,
        "accuracy_examples": result["report"]["accuracy_examples"],
        "head_chain_changed": False,
    }
    assert state["head_chain_source"] == "floor_fit" and state["ready"] is True and state["reason"] is None


def test_next_result_compares_with_the_accepted_one(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    first = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    accept_arm_camera(scene.cfg, scene.store, scene.sid, first["id"], LATER, fake_run)
    second = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(LATER), LATER)
    applied = second["report"]["comparison"]["applied"]
    assert applied["result"] == first["id"]
    assert applied["accuracy_examples"] == first["report"]["accuracy_examples"]


def test_next_result_starts_from_the_accepted_calibration(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    """After an accept the runtime arm calibration is the accepted one, so the next fit starts from and compares with
    it, not with the configured fit.joint_offsets_rad / fit.base_in_base_link."""
    scene = Scene(tmp_path)
    first = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    accept_arm_camera(scene.cfg, scene.store, scene.sid, first["id"], LATER, fake_run)
    rid = SessionStore.new_result_id(LATER)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        second = compute_arm_camera(scene.cfg, scene.store, scene.sid, rid, LATER)
    inputs = next(e for e in events_in(caplog.records) if e["step"] == "inputs")["data"]
    # calib_event rounds floats to 6 decimals
    assert inputs["applied_offsets_rad"] == pytest.approx(first["calibration"]["joint_offsets_rad"], abs=1e-6)
    assert inputs["applied_base"] == pytest.approx(first["calibration"]["base_in_base_link"], abs=1e-6)
    assert inputs["applied_source"] == {"session": scene.sid, "result": first["id"]}
    for joint in ARM_JOINT_NAMES:  # fitted from the accepted start, the same scene gives the same calibration
        assert second["calibration"]["joint_offsets_rad"][joint] == pytest.approx(
            first["calibration"]["joint_offsets_rad"][joint], abs=1e-3
        )
    params = {p["name"]: p for p in second["report"]["params"]}
    assert abs(params["shoulder_pan_offset"]["change"]) < 0.2  # deg against the accepted offset, not the configured one


def test_compute_without_an_accepted_calibration_starts_from_the_config(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    scene = Scene(tmp_path)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    inputs = next(e for e in events_in(caplog.records) if e["step"] == "inputs")["data"]
    assert inputs["applied_offsets_rad"] == scene.cfg.fit.joint_offsets_rad
    assert inputs["applied_source"] is None


def test_compute_with_an_unreadable_accepted_calibration_fails(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    first = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    accept_arm_camera(scene.cfg, scene.store, scene.sid, first["id"], LATER, fake_run)
    (scene.state / "applied" / ARM_CALIBRATION_FILE).write_text("{broken")
    second = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(LATER), LATER)
    assert second["state"] == "failed" and ARM_CALIBRATION_FILE in second["error"]


def write_stale_marker(cfg: NodeConfig, stale: list[str], saved_at: str | None) -> Path:
    """The follower bridge's arm re-zero stale marker (feetech_servos neutral.py format)."""
    path = Path(cfg.arm_stale_marker or "")
    path.parent.mkdir(parents=True, exist_ok=True)
    body = {"version": 1, "group": "follower", "saved_at": saved_at, "joints": {}, "stale": stale}
    path.write_text(json.dumps(body))
    return path


def test_accept_refuses_a_fit_captured_before_an_arm_re_zero(tmp_path: Path) -> None:
    """Offsets fitted from stills of the old arm zero would be wrong in the new one: refused, nothing applied."""
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    after_capture = datetime.fromtimestamp(STAMP_NS / 1e9 + 60, tz=UTC).isoformat()
    marker = write_stale_marker(scene.cfg, ["joint_offsets", "home"], after_capture)
    before = marker.read_text()
    with pytest.raises(NotApplicable, match="re-zero"):
        accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    assert not (scene.state / "applied" / ARM_CALIBRATION_FILE).exists()
    assert marker.read_text() == before


@pytest.mark.parametrize("body", ["{not json", json.dumps({"stale": ["joint_offsets"], "saved_at": None})])
def test_accept_refuses_while_the_re_zero_time_is_unknown(tmp_path: Path, body: str) -> None:
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    marker = write_stale_marker(scene.cfg, [], None)
    marker.write_text(body)
    with pytest.raises(NotApplicable, match="arm zero"):
        accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    assert not (scene.state / "applied" / ARM_CALIBRATION_FILE).exists()


def test_accept_refuses_a_fit_of_a_zero_restored_since_the_capture(tmp_path: Path) -> None:
    """A zero restore after the capture deletes the marker but records the zero change: the offsets were measured in
    the restored-away zero, so the accept is refused."""
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    marker = Path(scene.cfg.arm_stale_marker or "")
    marker.parent.mkdir(parents=True, exist_ok=True)
    after_capture = datetime.fromtimestamp(STAMP_NS / 1e9 + 60, tz=UTC).isoformat()
    zero_record_path(marker).write_text(json.dumps({"version": 1, "group": "follower", "changed_at": after_capture}))
    with pytest.raises(NotApplicable, match="zero changed"):
        accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    assert not (scene.state / "applied" / ARM_CALIBRATION_FILE).exists()


def test_accept_refuses_a_session_without_stills(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    for pid in scene.store.pose_ids(scene.sid):  # a pose without pose.json is not a stored pose
        (scene.store.pose_dir(scene.sid, pid) / "pose.json").unlink()
    with pytest.raises(NotApplicable, match="no still poses"):
        accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)


def test_captured_from_is_the_earliest_stamp_of_any_source(tmp_path: Path) -> None:
    """The arm joint stamps come from the follower bridge (the clock of the zero change times): the earliest stamp of
    any source counts, so a zero change just before a late camera stamp is not missed."""
    scene = Scene(tmp_path)
    pid = scene.store.pose_ids(scene.sid)[3]
    path = scene.store.pose_dir(scene.sid, pid) / "pose.json"
    meta = json.loads(path.read_text())
    meta["stamps"]["arm_joints"] = STAMP_NS - 5_000_000_000
    path.write_text(json.dumps(meta))
    assert captured_from(scene.store, scene.sid) == datetime.fromtimestamp(STAMP_NS / 1e9 - 5, tz=UTC)


@pytest.mark.parametrize(
    ("stale", "seconds"), [(["joint_offsets", "home"], -60), (["home"], 60)], ids=["re-zero-before", "home-only"]
)
def test_accept_with_the_offsets_measured_in_the_current_zero(tmp_path: Path, stale: list[str], seconds: int) -> None:
    """A re-zero before the capture (or a marker without joint_offsets) does not block the accept; the accept leaves
    the marker to mcp_server, which clears joint_offsets when it applies the calibration."""
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    saved_at = datetime.fromtimestamp(STAMP_NS / 1e9 + seconds, tz=UTC).isoformat()
    marker = write_stale_marker(scene.cfg, stale, saved_at)
    before = marker.read_text()
    accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    assert (scene.state / "applied" / ARM_CALIBRATION_FILE).exists()
    assert marker.read_text() == before


def test_decline_records_and_changes_nothing_applied(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    applied = scene.state / "applied"
    before = {p.name: p.read_bytes() for p in applied.iterdir()}
    with caplog.at_level(logging.INFO, logger=LOGGER):
        declined = decline_arm_camera(scene.store, scene.sid, result["id"], LATER)
    assert declined["declined"] is True and declined["declined_at"] == LATER.isoformat()
    assert {p.name: p.read_bytes() for p in applied.iterdir()} == before
    stored = json.loads((scene.store.result_dir(scene.sid, result["id"]) / "result.json").read_text())
    assert stored["declined"] is True
    with pytest.raises(NotApplicable, match="declined"):
        accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    event = next(e for e in events_in(caplog.records) if e["step"] == "decline")
    assert (event["flow"], event["outcome"], event["run_id"]) == ("arm_camera_accept", "declined", result["id"])


def test_accept_and_decline_refuse_a_failed_result(tmp_path: Path) -> None:
    scene = Scene(tmp_path, head_chain=False)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    with pytest.raises(NotApplicable, match="not done"):
        accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    with pytest.raises(NotApplicable, match="not done"):
        decline_arm_camera(scene.store, scene.sid, result["id"], LATER)


def test_read_result_reports_a_running_and_a_crashed_compute(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    rid = SessionStore.new_result_id(NOW)
    running = {"running": True, "session_id": scene.sid, "result_id": rid, "started_at": NOW.isoformat(), "error": None}
    body = read_arm_camera_result(scene.store, scene.sid, rid, running)
    assert (body["state"], body["report"], body["error"], body["created_at"]) == (
        "running",
        None,
        None,
        NOW.isoformat(),
    )
    crashed = {**running, "running": False, "error": "timed out after 600 s"}
    body = read_arm_camera_result(scene.store, scene.sid, rid, crashed)
    assert (body["state"], body["error"]) == ("failed", "timed out after 600 s")


def test_state_without_a_head_chain_is_not_ready(tmp_path: Path) -> None:
    scene = Scene(tmp_path, head_chain=False)
    assert arm_camera_state(scene.cfg) == {
        "applied": None,
        "head_chain_source": "urdf",
        "ready": False,
        "reason": NO_HEAD_CHAIN_REASON,
    }


@pytest.mark.parametrize(("source", "shown"), [("head_arm", "head_arm"), ("arm", "arm_offsets")])
def test_state_names_the_head_chain_source(tmp_path: Path, source: str, shown: str) -> None:
    scene = Scene(tmp_path, head_chain=False)
    apply_head_chain(scene.state, source)
    assert arm_camera_state(scene.cfg)["head_chain_source"] == shown


def test_fit_config_defaults_are_shipped(tmp_path: Path) -> None:
    """The node config carries the arm to camera fit config with its defaults (bounds of decision 324)."""
    cfg = NodeConfig()
    assert cfg.fit.arm_camera == ArmCameraFitConfig()
    bounds = cfg.fit.arm_camera.bounds
    assert (bounds.max_offset_change_deg, bounds.max_base_move_mm, bounds.max_base_turn_deg) == (15.0, 100.0, 10.0)
    assert (bounds.max_skew_mm, bounds.max_skew_deg) == (10.0, 10.0)
    assert math.isclose(cfg.fit.arm_camera.reach_m, 0.30) and math.isclose(cfg.fit.arm_camera.camera_distance_m, 0.5)


def test_accept_refuses_a_fit_of_a_head_chain_no_longer_applied(tmp_path: Path) -> None:
    """The arm calibration holds only against the head chain it was fitted with: after another head chain is applied
    the accept is refused, and an accepted calibration is reported with head_chain_changed."""
    scene = Scene(tmp_path)
    first = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    second = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(LATER), LATER)
    accept_arm_camera(scene.cfg, scene.store, scene.sid, first["id"], LATER, fake_run)
    assert arm_camera_state(scene.cfg)["applied"]["head_chain_changed"] is False
    apply_head_chain(scene.state, "head_arm")  # same origins, another applied record: another head calibration
    marker = json.loads((scene.state / "applied" / "applied.json").read_text())
    marker["head_chain"]["result"] = "r-20261109-130000"
    (scene.state / "applied" / "head_chain.json").write_text(
        (scene.state / "applied" / "head_chain.json").read_text().replace(FLOOR_RID, "r-20261109-130000")
    )
    (scene.state / "applied" / "applied.json").write_text(json.dumps(marker))
    with pytest.raises(NotApplicable, match="head chain"):
        accept_arm_camera(scene.cfg, scene.store, scene.sid, second["id"], LATER, fake_run)
    assert arm_camera_state(scene.cfg)["applied"]["head_chain_changed"] is True


def test_a_head_chain_apply_after_the_accept_check_lands_after_the_accept(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The applied head chain check and the arm calibration write are one step under the applied lock: a head chain
    apply of another flow (e.g. a ground board accept) that starts right after the check lands after the accept,
    which then reports the calibration as fitted against another head chain."""
    scene = Scene(tmp_path)
    result = compute_arm_camera(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    other = {"session": "ground-20261109-120000", "result": "r-20261109-121000"}
    source = tmp_path / "other_head_chain.json"
    source.write_text(
        (scene.state / "applied" / "head_chain.json").read_text().replace(FLOOR_SID, other["session"])
        .replace(FLOOR_RID, other["result"])
    )  # fmt: skip
    record = {**other, "applied_at": LATER.isoformat(), "source": "ground_board", "floor": None}
    landed = record_applied_writes(monkeypatch, scene.state / "applied")
    accepted, applied = interleave_apply(
        monkeypatch,
        arm_camera_results,
        "head_chain_source",
        lambda: accept_arm_camera(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run),
        lambda: floor_results.apply_head_chain_file(
            scene.cfg, source, other["result"], lambda _info: record, LATER, fake_run
        ),
    )
    assert accepted["arm_calibration"]["result"] == result["id"]
    assert applied["head_chain"] == record
    assert landed == [ARM_CALIBRATION_FILE, "applied.json", "head_chain.json", "applied.json"]
    assert arm_camera_state(scene.cfg)["applied"]["head_chain_changed"] is True
