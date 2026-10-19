"""Tests for applying a result to the robot and packaging it for download."""

import io
import json
import os
import subprocess
import zipfile
from datetime import UTC, datetime
from pathlib import Path

import pytest

from head_rgbd_calibration import apply as apply_mod
from head_rgbd_calibration.apply import (
    APPLIED_FILES,
    APPLIED_MARKER,
    NotApplicable,
    applied_state,
    apply_result,
    download_zip,
)
from head_rgbd_calibration.compute import REPORT_FILE, URDF_FILE
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import RESULT_FILE, SessionStore

NOW = datetime(2026, 10, 19, 12, 30, 45, tzinfo=UTC)
RID = "r-20261019-123045"
UNITS = ["head_camera_cpp", "head_rgbd_cpp"]


@pytest.fixture
def store(tmp_path: Path) -> SessionStore:
    return SessionStore(tmp_path / "state", NodeConfig())


def make_result(store: SessionStore, passed: bool = True, rid: str = RID, tag: str = "a") -> tuple[str, str]:
    sid = store.create_session(datetime(2026, 10, 19, 12, 0, 0), 1.5)
    path = store.result_dir(sid, rid)
    path.mkdir(parents=True)
    (path / RESULT_FILE).write_text(json.dumps({"passed": passed}))
    if passed:
        for name in (*APPLIED_FILES, URDF_FILE, REPORT_FILE):
            (path / name).write_text(f"{tag}:{name}")
    return sid, rid


def test_apply_writes_marker_last(store: SessionStore, monkeypatch: pytest.MonkeyPatch) -> None:
    sid, rid = make_result(store)
    order: list[str] = []
    real = os.replace
    monkeypatch.setattr(apply_mod.os, "replace", lambda a, b: (order.append(Path(b).name), real(a, b))[1])
    info = apply_result(store, sid, rid, NOW)
    applied = store.state_dir / "applied"
    for name in APPLIED_FILES:
        assert (applied / name).read_text() == f"a:{name}"
    assert order[-1] == APPLIED_MARKER and set(order[:-1]) == set(APPLIED_FILES)
    marker = json.loads((applied / APPLIED_MARKER).read_text())
    assert marker == info
    assert marker["session"] == sid and marker["result"] == rid
    assert marker["applied_at"] == NOW.isoformat() and marker["files"] == list(APPLIED_FILES)


def test_failing_result_refused(store: SessionStore) -> None:
    ok_sid, ok_rid = make_result(store)
    apply_result(store, ok_sid, ok_rid, NOW)
    before = {p.name: p.read_text() for p in (store.state_dir / "applied").iterdir()}
    sid, rid = make_result(store, passed=False, rid="r-20261019-130000")
    with pytest.raises(NotApplicable):
        apply_result(store, sid, rid, NOW)
    assert {p.name: p.read_text() for p in (store.state_dir / "applied").iterdir()} == before


def test_interrupted_apply_keeps_previous_marker(store: SessionStore, monkeypatch: pytest.MonkeyPatch) -> None:
    sid, rid = make_result(store)
    apply_result(store, sid, rid, NOW)
    marker_before = (store.state_dir / "applied" / APPLIED_MARKER).read_text()
    sid2, rid2 = make_result(store, rid="r-20261019-140000", tag="b")
    real = os.replace
    calls = []

    def flaky(a: object, b: object) -> None:
        calls.append(b)
        if len(calls) == 3:
            raise OSError("disk")
        real(a, b)

    monkeypatch.setattr(apply_mod.os, "replace", flaky)
    with pytest.raises(OSError):
        apply_result(store, sid2, rid2, NOW)
    assert (store.state_dir / "applied" / APPLIED_MARKER).read_text() == marker_before


def fake_run(stamp: int, state: str = "active"):
    def run(cmd: list[str], **kwargs: object) -> subprocess.CompletedProcess:
        assert cmd[:3] == ["systemctl", "show", "--timestamp=unix"] and "restart" not in cmd
        return subprocess.CompletedProcess(cmd, 0, f"ActiveState={state}\nActiveEnterTimestamp=@{stamp}\n", "")

    return run


def test_applied_state_none_when_nothing_applied(store: SessionStore) -> None:
    assert applied_state(store, UNITS, fake_run(0)) is None


def test_units_restarted_after_apply(store: SessionStore) -> None:
    sid, rid = make_result(store)
    apply_result(store, sid, rid, NOW)
    at = int(NOW.timestamp())
    newer = applied_state(store, UNITS, fake_run(at + 5))
    assert newer is not None and newer["session"] == sid
    assert newer["units"] == [{"name": u, "active_state": "active", "restarted": True} for u in UNITS]
    older = applied_state(store, UNITS, fake_run(at - 5, "inactive"))
    assert older is not None
    assert all(not u["restarted"] and u["active_state"] == "inactive" for u in older["units"])


def test_download_zip_members(store: SessionStore) -> None:
    sid, rid = make_result(store)
    name, data = download_zip(store, sid, rid)
    assert name == f"{sid}-{rid}.zip"
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        assert sorted(z.namelist()) == sorted([*APPLIED_FILES, URDF_FILE, REPORT_FILE])
        assert z.read("tof_joint.json") == b"a:tof_joint.json"


def test_download_zip_without_yamls(store: SessionStore) -> None:
    sid, rid = make_result(store, passed=False)
    with pytest.raises(NotApplicable):
        download_zip(store, sid, rid)
