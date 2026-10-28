"""Tests for accepting an intrinsics check and packaging a result for download."""

import io
import json
import os
import subprocess
import zipfile
from datetime import UTC, datetime
from pathlib import Path

import pytest

from head_rgbd_calibration import store as store_mod
from head_rgbd_calibration.apply import APPLIED_MARKER, NotApplicable, applied_state, apply_result, download_zip
from head_rgbd_calibration.compute import REPORT_FILE
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import RESULT_FILE, SessionStore

NOW = datetime(2026, 10, 19, 12, 30, 45, tzinfo=UTC)
RID = "r-20261019-123045"
UNITS = ["head_rgbd_cpp", "head_state_publisher"]
HEAD_CHAIN = {"session": "floor-20261021-120000", "result": "r-20261021-123000", "source": "floor"}


@pytest.fixture
def store(tmp_path: Path) -> SessionStore:
    return SessionStore(tmp_path / "state", NodeConfig())


def make_result(store: SessionStore, passed: bool = True, rid: str = RID, kind: str | None = "intrinsics") -> tuple:
    sid = store.create_session(datetime(2026, 10, 19, 12, 0, 0))
    path = store.result_dir(sid, rid)
    path.mkdir(parents=True)
    body = {"id": rid, "session": sid, "passed": passed} | ({} if kind is None else {"kind": kind})
    (path / RESULT_FILE).write_text(json.dumps(body))
    (path / REPORT_FILE).write_text("# D435 intrinsics check\n")
    return sid, rid


def applied_files(store: SessionStore) -> dict[str, str]:
    return {p.name: p.read_text() for p in (store.state_dir / "applied").iterdir()}


def test_intrinsics_apply_records_only(store: SessionStore) -> None:
    """A passed check is recorded in applied.json, nothing is copied, and a failed check is refused."""
    applied = store.state_dir / "applied"
    applied.mkdir(parents=True)
    (applied / APPLIED_MARKER).write_text(json.dumps({"head_chain": HEAD_CHAIN}))
    sid, rid = make_result(store)
    info = apply_result(store, sid, rid, NOW)
    assert info == {"kind": "intrinsics", "session": sid, "result": rid, "applied_at": NOW.isoformat()}
    assert [p.name for p in applied.iterdir()] == [APPLIED_MARKER]
    assert json.loads((applied / APPLIED_MARKER).read_text()) == {**info, "head_chain": HEAD_CHAIN}
    before = applied_files(store)
    failed_sid, failed_rid = make_result(store, passed=False, rid="r-20261019-130000")
    with pytest.raises(NotApplicable, match="did not pass"):
        apply_result(store, failed_sid, failed_rid, NOW)
    assert applied_files(store) == before


def test_apply_writes_the_marker_atomically(store: SessionStore, monkeypatch: pytest.MonkeyPatch) -> None:
    sid, rid = make_result(store)
    order: list[str] = []
    real = os.replace
    monkeypatch.setattr(store_mod.os, "replace", lambda a, b: (order.append(Path(b).name), real(a, b))[1])
    apply_result(store, sid, rid, NOW)
    assert order == [APPLIED_MARKER]
    assert [p.name for p in (store.state_dir / "applied").iterdir()] == [APPLIED_MARKER]


@pytest.mark.parametrize("kind", [None, "head_arm"])
def test_result_of_another_kind_refused(store: SessionStore, kind: str | None) -> None:
    sid, rid = make_result(store, kind=kind)
    with pytest.raises(NotApplicable, match="intrinsics"):
        apply_result(store, sid, rid, NOW)
    assert not (store.state_dir / "applied").exists()


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
        assert sorted(z.namelist()) == [REPORT_FILE, RESULT_FILE]
        assert z.read(REPORT_FILE) == b"# D435 intrinsics check\n"


def test_download_zip_without_a_report(store: SessionStore) -> None:
    sid, rid = make_result(store, passed=False)
    (store.result_dir(sid, rid) / REPORT_FILE).unlink()
    with pytest.raises(NotApplicable):
        download_zip(store, sid, rid)
