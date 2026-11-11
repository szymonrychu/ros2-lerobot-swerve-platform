"""Accepted arm to camera calibration at runtime (mcp_server/arm_calibration.py): loaded at startup, reloaded when
applied/applied.json changes, swapped into the running arm model at an idle point, client.yml as the fallback."""

import json
import logging
import os
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from arm_collision.ik import ArmKinematics, load_joint_limits
from arm_collision.settings import ArmBaseOffset
from ros2_common.head_rgbd_calibration.applied_arm_calibration import applied_marker_identity
from ros2_common.head_rgbd_calibration.arm_calibration_file import ARM_CALIBRATION_FILE, ARM_CALIBRATION_KEY
from ros2_common.head_rgbd_calibration.urdf_text import APPLIED_MARKER_FILE

from mcp_server import arm_calibration, calibration_marker
from mcp_server.arm import ArmBusyError, ArmController
from mcp_server.arm_calibration import FLOW, ArmCalibrationWatcher, arm_values
from mcp_server.arm_reach import ReachEnvelope
from mcp_server.calib_event import CALIB_EVENT_PREFIX
from mcp_server.config import ArmSettings, BodyCheckSettings, McpServerConfig, SagCompensationSettings
from mcp_server.reach_map import ReachMap, ReachMapStale, checker_from_config, reach_map_key

from .fakes import FakeArmBackend

LOGGER = "mcp_server.arm_calibration"
SESSION = "calib-20261109-120000"
OFFSETS = {"shoulder_pan": 0.03, "shoulder_lift": -0.02, "elbow_flex": -0.09, "wrist_flex": 0.08, "wrist_roll": -0.07}
BASE = {"x": 0.061, "y": -0.048, "z": 0.102, "yaw": 0.0}
CAPTURED_FROM = "2026-11-09T12:01:00+00:00"
BEFORE_CAPTURE = "2026-11-09T11:00:00+00:00"
AFTER_CAPTURE = "2026-11-09T13:00:00+00:00"


def config(tmp_path: Path) -> McpServerConfig:
    """Node config with the applied dir, the stale marker and the home file under tmp_path."""
    cfg = McpServerConfig()
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    cfg.arm.calibration_stale_marker = tmp_path / "arm" / "calibration_stale.json"
    cfg.arm.applied_dir = tmp_path / "applied"
    return cfg


def accept(applied: Path, result: str = "r-1", offsets: dict[str, float] | None = None, **base: float) -> None:
    """Write arm_calibration.json, then the applied.json record, as the head_rgbd_calibration accept does."""
    applied.mkdir(parents=True, exist_ok=True)
    body = {
        "session": SESSION,
        "result": result,
        "accepted_at": "2026-11-09T12:20:00+00:00",
        "captured_from": CAPTURED_FROM,
        "score": {"rms_px": 0.9},
        "joint_offsets_rad": offsets or OFFSETS,
        "base_in_base_link": {**BASE, **base},
        "tool_offset_m": [0.5, 0.5, 0.5],
    }
    (applied / ARM_CALIBRATION_FILE).write_text(json.dumps(body))
    marker_path = applied / APPLIED_MARKER_FILE
    marker = json.loads(marker_path.read_text()) if marker_path.exists() else {}
    marker[ARM_CALIBRATION_KEY] = {"session": SESSION, "result": result, "accepted_at": body["accepted_at"]}
    write_marker(applied, marker)


def write_marker(applied: Path, marker: dict[str, Any]) -> None:
    """applied.json with a distinct mtime (coarse clocks)."""
    path = applied / APPLIED_MARKER_FILE
    old = path.stat().st_mtime_ns if path.exists() else 0
    path.write_text(json.dumps(marker))
    stat = path.stat()
    os.utime(path, ns=(stat.st_atime_ns, max(stat.st_mtime_ns, old + 1_000_000_000)))


def stale_marker(cfg: McpServerConfig, stale: list[str], saved_at: str | None) -> Path:
    """The bridge's arm re-zero stale marker."""
    path = cfg.arm.calibration_stale_marker
    assert path is not None
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"version": 1, "group": "follower", "saved_at": saved_at, "joints": {}, "stale": stale}))
    return path


def controller(cfg: McpServerConfig, sag: bool = False) -> ArmController:
    """Controller built from cfg.arm the way RosRobot builds it."""
    if sag:
        cfg.arm.sag_compensation = SagCompensationSettings(enabled=True, k={"shoulder_lift": 0.05})
    kin = ArmKinematics(
        cfg.arm.urdf_path,
        margin=cfg.limits.arm_limit_margin_rad,
        joint_offsets=cfg.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(cfg.arm.tool_offset_m.model_dump().values()),
        limit_overrides=cfg.arm.joint_limit_overrides_rad,
    )
    return ArmController(FakeArmBackend(), kin, load_joint_limits(cfg.arm.urdf_path), cfg)


def events(caplog: pytest.LogCaptureFixture) -> list[dict[str, Any]]:
    """calib_event payloads of the arm_calibration flow."""
    out = []
    for record in caplog.records:
        message = record.getMessage()
        if message.startswith(CALIB_EVENT_PREFIX):
            payload = json.loads(message[len(CALIB_EVENT_PREFIX) + 1 :])
            if payload["flow"] == FLOW:
                out.append(payload)
    return out


def mount_of(arm: ArmSettings) -> ArmBaseOffset:
    assert arm.base_in_base_link is not None
    return arm.base_in_base_link


# --- startup ------------------------------------------------------------------------------------------------------


def test_startup_without_an_accepted_calibration_keeps_the_config(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    cfg = config(tmp_path)
    before = cfg.arm.model_copy(deep=True)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher = ArmCalibrationWatcher(cfg.arm)
        watcher.startup(cfg.arm)
    assert cfg.arm == before and watcher.loaded is None
    (event,) = events(caplog)
    assert (event["step"], event["outcome"]) == ("load", "ok")
    assert event["data"]["source"] == "config" and event["data"]["new"] == arm_values(before)


def test_startup_applies_exactly_the_fitted_fields(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    cfg = config(tmp_path)
    before = cfg.arm.model_copy(deep=True)
    accept(cfg.arm.applied_dir)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher = ArmCalibrationWatcher(cfg.arm)
        watcher.startup(cfg.arm)
    assert cfg.arm.joint_offsets_rad.model_dump() == OFFSETS
    assert mount_of(cfg.arm).model_dump() == BASE and cfg.arm.arm_base_height_m == BASE["z"]
    untouched = {"joint_offsets_rad", "base_in_base_link", "arm_base_height_m"}
    assert cfg.arm.model_dump(exclude=untouched) == before.model_dump(exclude=untouched)  # tool point stays
    assert watcher.loaded is not None and watcher.loaded.result == "r-1"
    (event,) = events(caplog)
    assert (event["step"], event["outcome"]) == ("load", "ok")
    assert event["data"]["source"] == {"session": SESSION, "result": "r-1", "accepted_at": "2026-11-09T12:20:00Z"}
    assert event["data"]["old"] == arm_values(before)
    assert event["data"]["new"]["joint_offsets_rad"] == OFFSETS


def test_startup_with_an_invalid_file_keeps_the_config_and_logs_an_error(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    cfg = config(tmp_path)
    before = cfg.arm.model_copy(deep=True)
    accept(cfg.arm.applied_dir, offsets={"shoulder_pan": 0.1})
    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher = ArmCalibrationWatcher(cfg.arm)
        watcher.startup(cfg.arm)
    assert cfg.arm == before and watcher.loaded is None
    (event,) = events(caplog)
    assert (event["step"], event["outcome"]) == ("load", "failed")
    assert "joint_offsets_rad" in event["reason"]
    assert any(r.levelno == logging.ERROR for r in caplog.records)


def test_a_calibration_the_arm_model_refuses_keeps_the_config(tmp_path: Path) -> None:
    """The merged arm block is validated as ArmSettings (here: base z above the 1 m mount height limit)."""
    cfg = config(tmp_path)
    before = cfg.arm.model_copy(deep=True)
    accept(cfg.arm.applied_dir, z=1.5)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    assert cfg.arm == before and watcher.loaded is None


def test_no_applied_dir_disables_the_override(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    accept(tmp_path / "applied")
    cfg.arm.applied_dir = None
    before = cfg.arm.model_copy(deep=True)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    assert cfg.arm == before


# --- reload -------------------------------------------------------------------------------------------------------


def test_reload_on_an_applied_json_change_swaps_the_running_arm_model(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    cfg = config(tmp_path)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg, sag=True)
    old = arm_values(cfg.arm)
    accept(cfg.arm.applied_dir)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher.tick(arm.swap_calibration)
    assert arm.kin.offsets == OFFSETS
    assert arm.sag is not None and arm.sag.offsets == OFFSETS
    mount = ArmBaseOffset(**BASE)
    assert arm.floor_guard.mount == mount and arm.body_check.mount == mount
    assert arm.body_check.offset.tolist() == [BASE["x"], BASE["y"], BASE["z"]]
    assert cfg.arm.joint_offsets_rad.model_dump() == OFFSETS and cfg.arm.floor_z_m == -BASE["z"]
    assert arm.calibration is not None and arm.calibration.result == "r-1"
    (event,) = events(caplog)
    assert (event["step"], event["outcome"]) == ("load", "ok")
    assert event["data"]["old"] == old and event["data"]["new"] == arm_values(cfg.arm)


def test_tick_without_a_change_reads_nothing(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg)
    (cfg.arm.applied_dir / ARM_CALIBRATION_FILE).write_text("{broken, but applied.json did not change")
    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher.tick(arm.swap_calibration)
    assert events(caplog) == [] and arm.kin.offsets == OFFSETS


def test_a_new_accept_replaces_the_previous_one(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg)
    newer = {**OFFSETS, "elbow_flex": -0.12}
    accept(cfg.arm.applied_dir, result="r-2", offsets=newer, x=0.0605)
    watcher.tick(arm.swap_calibration)
    assert arm.kin.offsets == newer and arm.body_check.mount.x == 0.0605
    assert watcher.loaded is not None and watcher.loaded.result == "r-2"


def test_an_atomic_accept_within_one_mtime_tick_reloads(tmp_path: Path) -> None:
    """head_rgbd_calibration replaces applied.json atomically; a rewrite with an equal mtime_ns still reloads.

    File timestamps come from the kernel's coarse clock, so two accepts within one tick share mtime_ns: the watcher
    keys on (mtime_ns, size, inode) like the head chain reload.
    """
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg)
    marker = cfg.arm.applied_dir / APPLIED_MARKER_FILE
    old = marker.stat()
    newer = {**OFFSETS, "elbow_flex": -0.12}
    accept(cfg.arm.applied_dir, result="r-2", offsets=newer)
    replacement = cfg.arm.applied_dir / "applied.json.tmp"
    replacement.write_text(marker.read_text())
    os.replace(replacement, marker)
    os.utime(marker, ns=(old.st_atime_ns, old.st_mtime_ns))
    assert marker.stat().st_mtime_ns == old.st_mtime_ns
    watcher.tick(arm.swap_calibration)
    assert arm.kin.offsets == newer
    assert watcher.loaded is not None and watcher.loaded.result == "r-2"


def test_both_reloads_share_one_marker_identity(tmp_path: Path) -> None:
    """The arm calibration watcher and the head chain provider key on the same applied.json identity."""
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    marker = cfg.arm.applied_dir / APPLIED_MARKER_FILE
    st = marker.stat()
    assert applied_marker_identity(cfg.arm.applied_dir) == (st.st_mtime_ns, st.st_size, st.st_ino)
    assert applied_marker_identity(tmp_path / "missing") is None
    assert applied_marker_identity(None) is None
    assert ArmCalibrationWatcher(cfg.arm).marker_stamp() == applied_marker_identity(cfg.arm.applied_dir)


def test_an_invalid_reload_keeps_the_previous_values(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg)
    applied = cfg.arm.applied_dir
    (applied / ARM_CALIBRATION_FILE).write_text("{not json")
    write_marker(applied, json.loads((applied / APPLIED_MARKER_FILE).read_text()) | {"touched": True})
    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher.tick(arm.swap_calibration)
    assert arm.kin.offsets == OFFSETS and watcher.loaded is not None and watcher.loaded.result == "r-1"
    (event,) = events(caplog)
    assert (event["step"], event["outcome"]) == ("load", "failed") and ARM_CALIBRATION_FILE in event["reason"]


def test_a_removed_record_falls_back_to_the_config(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    baseline = cfg.arm.joint_offsets_rad.model_dump()
    accept(cfg.arm.applied_dir)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg)
    write_marker(cfg.arm.applied_dir, {"head_chain": {"session": "s", "result": "r"}})
    watcher.tick(arm.swap_calibration)
    assert arm.kin.offsets == baseline and cfg.arm.joint_offsets_rad.model_dump() == baseline
    assert watcher.loaded is None and arm.calibration is None


def test_a_head_chain_apply_alone_does_not_swap(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    """applied.json changes for a head apply too: the same arm calibration logs `unchanged` and swaps nothing."""
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    calls: list[Any] = []
    marker = json.loads((cfg.arm.applied_dir / APPLIED_MARKER_FILE).read_text())
    write_marker(cfg.arm.applied_dir, {**marker, "head_chain": {"session": "s2", "result": "r2"}})
    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher.tick(lambda arm, cal: calls.append((arm, cal)) or True)
    assert calls == []
    (event,) = events(caplog)
    assert (event["step"], event["outcome"], event["data"]["unchanged"]) == ("load", "ok", True)


def test_a_failing_swap_is_logged_and_never_raises(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    """The tick runs on the monitor timer: a swap that raises is a failed load, the values in use stay."""
    cfg = config(tmp_path)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    accept(cfg.arm.applied_dir)

    def broken(arm: ArmSettings, calibration: Any) -> bool:
        raise RuntimeError("kernel build failed")

    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher.tick(broken)
        watcher.tick(broken)
    assert watcher.loaded is None and watcher.pending is None
    (event,) = events(caplog)
    assert (event["step"], event["outcome"]) == ("apply", "failed") and "kernel build failed" in event["reason"]


# --- idle point ---------------------------------------------------------------------------------------------------


def test_the_swap_waits_for_the_running_motion(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    cfg = config(tmp_path)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg)
    before = dict(arm.kin.offsets)
    accept(cfg.arm.applied_dir)
    with caplog.at_level(logging.INFO, logger=LOGGER), arm.exclusive_motion():
        watcher.tick(arm.swap_calibration)
        watcher.tick(arm.swap_calibration)
        assert arm.kin.offsets == before and cfg.arm.joint_offsets_rad.model_dump() == before
    deferred = [e for e in events(caplog) if e["step"] == "apply"]
    assert len(deferred) == 1 and deferred[0]["outcome"] == "start" and "idle" in deferred[0]["reason"]
    with caplog.at_level(logging.INFO, logger=LOGGER):
        watcher.tick(arm.swap_calibration)  # idle now: applied without another applied.json change
    assert arm.kin.offsets == OFFSETS
    assert [e["step"] for e in events(caplog)][-1] == "load"


def test_a_motion_cannot_start_during_the_swap(tmp_path: Path) -> None:
    """The swap holds the motion lock: a motion asked for meanwhile is refused as busy, never run half swapped."""
    cfg = config(tmp_path)
    arm = controller(cfg)
    seen: list[str] = []
    original = arm.body_check.set_mount

    def set_mount(mount: ArmBaseOffset) -> None:
        try:
            with arm.exclusive_motion():
                seen.append("ran")
        except ArmBusyError:
            seen.append("busy")
        original(mount)

    arm.body_check.set_mount = set_mount  # type: ignore[method-assign]
    assert arm.swap_calibration(cfg.arm.model_copy(update={"joint_offsets_rad": cfg.arm.joint_offsets_rad}), None)
    assert seen == ["busy"]


# --- stale marker -------------------------------------------------------------------------------------------------


def test_applying_clears_joint_offsets_from_a_re_zero_before_the_capture(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    marker = stale_marker(cfg, ["joint_offsets", "home"], BEFORE_CAPTURE)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg)
    accept(cfg.arm.applied_dir)
    watcher.tick(arm.swap_calibration)
    assert json.loads(marker.read_text())["stale"] == ["home"]


def test_accept_clears_the_servo_calibration_reason(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    marker = stale_marker(cfg, ["joint_offsets", "home"], BEFORE_CAPTURE)
    doc = json.loads(marker.read_text())
    doc[calibration_marker.SERVO_CALIBRATION_AT_FIELD] = BEFORE_CAPTURE
    marker.write_text(json.dumps(doc))
    accept(cfg.arm.applied_dir)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    on_disk = json.loads(marker.read_text())
    assert on_disk["stale"] == ["home"]
    assert calibration_marker.SERVO_CALIBRATION_AT_FIELD not in on_disk


def test_the_marker_is_removed_when_joint_offsets_was_the_last_item(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    marker = stale_marker(cfg, ["joint_offsets"], BEFORE_CAPTURE)
    accept(cfg.arm.applied_dir)
    ArmCalibrationWatcher(cfg.arm).startup(cfg.arm)
    assert not marker.exists()


@pytest.mark.parametrize("saved_at", [AFTER_CAPTURE, None], ids=["re-zero-after-capture", "unknown-time"])
def test_a_re_zero_after_the_capture_keeps_joint_offsets_stale(tmp_path: Path, saved_at: str | None) -> None:
    """A restart after a later re-zero loads the calibration again but must not clear the newer marker."""
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    marker = stale_marker(cfg, ["joint_offsets", "home"], saved_at)
    before = marker.read_text()
    ArmCalibrationWatcher(cfg.arm).startup(cfg.arm)
    assert marker.read_text() == before


def test_a_zero_change_recorded_after_the_capture_keeps_joint_offsets_stale(tmp_path: Path) -> None:
    """A zero restore after the capture records the change (a later re-zero relists joint_offsets with an old marker
    time here): the accepted offsets do not hold in the current zero, so the marker stays."""
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    marker = stale_marker(cfg, ["joint_offsets"], BEFORE_CAPTURE)
    record = {"version": 1, "group": "follower", "changed_at": AFTER_CAPTURE}
    marker.with_name("zero_changed.json").write_text(json.dumps(record))
    before = marker.read_text()
    ArmCalibrationWatcher(cfg.arm).startup(cfg.arm)
    assert marker.read_text() == before


def test_a_marker_rewritten_between_check_and_clear_is_kept(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The bridge may write a new re-zero between the time check and the clear: the clear then leaves it."""
    cfg = config(tmp_path)
    accept(cfg.arm.applied_dir)
    marker = stale_marker(cfg, ["joint_offsets", "home"], BEFORE_CAPTURE)
    original = calibration_marker.load_marker
    calls = {"n": 0}

    def load_then_rezero(path: Path) -> Any:
        loaded = original(path)
        calls["n"] += 1
        if calls["n"] == 1:  # the bridge saves a neutral right after the watcher's check read
            stale_marker(cfg, ["joint_offsets", "home"], AFTER_CAPTURE)
        return loaded

    monkeypatch.setattr(calibration_marker, "load_marker", load_then_rezero)
    monkeypatch.setattr(arm_calibration, "load_marker", load_then_rezero)
    ArmCalibrationWatcher(cfg.arm).startup(cfg.arm)
    assert json.loads(marker.read_text())["saved_at"] == AFTER_CAPTURE
    assert json.loads(marker.read_text())["stale"] == ["joint_offsets", "home"]


def test_the_config_fallback_never_touches_the_marker(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    marker = stale_marker(cfg, ["joint_offsets"], BEFORE_CAPTURE)
    ArmCalibrationWatcher(cfg.arm).startup(cfg.arm)
    assert marker.exists()


# --- state --------------------------------------------------------------------------------------------------------


def test_arm_state_names_the_calibration_in_use(tmp_path: Path) -> None:
    cfg = config(tmp_path)
    watcher = ArmCalibrationWatcher(cfg.arm)
    watcher.startup(cfg.arm)
    arm = controller(cfg)
    assert arm.state().arm_calibration is None
    accept(cfg.arm.applied_dir)
    watcher.tick(arm.swap_calibration)
    state = arm.state().arm_calibration
    assert state is not None and (state.session, state.result) == (SESSION, "r-1")


# --- reach envelope and reach map ---------------------------------------------------------------------------------

COARSE = BodyCheckSettings(reach_heading_step_deg=90.0, reach_height_step_m=0.35, reach_wait_s=60.0)
SWAPPED_MOUNT = ArmBaseOffset(x=0.0700, y=-0.040, z=0.100, yaw=0.0)


def test_the_reach_envelope_is_recomputed_after_a_mount_swap() -> None:
    checker = checker_from_config(McpServerConfig())
    envelope = ReachEnvelope(checker, COARSE)
    first = envelope.get(COARSE.reach_wait_s)
    assert first is not None and envelope.compute_count == 1
    checker.set_mount(SWAPPED_MOUNT)
    second = envelope.get(COARSE.reach_wait_s)
    assert second is not None and envelope.compute_count == 2
    shift = [b - a for a, b in zip(first["pan_axis_base_link"], second["pan_axis_base_link"], strict=True)]
    assert shift == pytest.approx([0.0700 - 0.0592, -0.040 + 0.05, 0.0], abs=1e-4)


def test_the_reach_envelope_cell_is_withheld_while_stale() -> None:
    checker = checker_from_config(McpServerConfig())
    envelope = ReachEnvelope(checker, COARSE)
    assert envelope.get(COARSE.reach_wait_s) is not None
    assert envelope.cell(0.0, 0.0) is not None
    checker.kin.set_offsets({**checker.kin.offsets, "shoulder_lift": 0.05})
    assert envelope.cell(0.0, 0.0) is None


def test_a_stale_envelope_disk_cache_is_not_loaded(tmp_path: Path) -> None:
    checker = checker_from_config(McpServerConfig())
    cache = tmp_path / "reach_envelope.json"
    assert ReachEnvelope(checker, COARSE, cache).get(COARSE.reach_wait_s) is not None
    checker.set_mount(SWAPPED_MOUNT)
    envelope = ReachEnvelope(checker, COARSE, cache)
    assert envelope.get(COARSE.reach_wait_s) is not None and envelope.compute_count == 1


def test_the_reach_map_refuses_after_a_calibration_swap() -> None:
    config = McpServerConfig()
    checker = checker_from_config(config)
    rmap = ReachMap(np.zeros((1, 1, 1), dtype=np.uint16), np.zeros((1, 1, 1), dtype=np.float16), {})
    rmap.meta["key"] = reach_map_key(checker, config.reach_map)
    rmap.require_current(checker, config.reach_map)
    checker.set_mount(SWAPPED_MOUNT)
    with pytest.raises(ReachMapStale, match="regenerate"):
        rmap.require_current(checker, config.reach_map)
