"""Tests for the CSI camera (front, back) intrinsics calibration: capture, compute, apply and the /api/csi routes.

Views are synthetic ChArUco renders through a known 1152x648 pinhole (tests/rgbd_synthetic.py of the repo root),
delivered as CompressedImage-like messages to the capture's subscription callback.
"""

import json
import os
import subprocess
import threading
from datetime import UTC, datetime, timedelta
from functools import cache
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np
import pytest
import yaml
from fastapi.testclient import TestClient
from pydantic import ValidationError
from sensor_msgs.msg import CompressedImage

from head_rgbd_calibration.app import build_app
from head_rgbd_calibration.apply import APPLIED_DIR, APPLIED_MARKER, NotApplicable, apply_result
from head_rgbd_calibration.capture import Busy
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.csi_intrinsics import (
    APPLIED_KEY,
    KIND_CSI,
    NO_BOARD,
    SAME_VIEW,
    CsiCalibration,
    applied_file_name,
    csi_router,
    view_differs,
)
from head_rgbd_calibration.floor_results import apply_head_chain_file
from head_rgbd_calibration.store import KIND_INTRINSICS, RESULT_FILE, NotFound, SessionStore, write_json_atomic
from tests.applied_writes import record_applied_writes
from tests.pose_factory import BOARD_CFG, detection_at, load_synthetic

CSI_SIZE = (1152, 648)
RACE_WAIT_S = 0.5  # how long the interleaved writer may run before the first one writes
CSI_FOCAL = 900.0
CSI_D = np.array([0.04, -0.08, 0.0, 0.0, 0.0])
VIEW_COUNT = 8
MIN_VIEWS = 6
T0 = datetime(2026, 10, 31, 12, 0, 0, tzinfo=UTC)
# The head_camera_cpp loader test fixture: the keys a camera_info_file must carry (src/head_camera_node.cpp).
LOADER_FIXTURE = Path(__file__).resolve().parents[2] / "head_camera_cpp" / "test" / "data" / "calibration.yaml"
FRONT_UNIT = "ros2-front_camera.service"
BACK_UNIT = "ros2-back_camera.service"


@cache
def csi_camera() -> Any:
    syn = load_synthetic()
    return syn.make_camera(CSI_SIZE, CSI_FOCAL, CSI_D)


@cache
def csi_view(index: int) -> np.ndarray:
    """BGR render of the board at the index-th of VIEW_COUNT spread poses, 1152x648."""
    syn = load_synthetic()
    poses = syn.board_poses(VIEW_COUNT, csi_camera(), BOARD_CFG, seed=5, distance_m=(0.45, 0.8), max_tilt_deg=35.0)
    return syn.render_rgb(csi_camera(), poses[index], BOARD_CFG, seed=index)


def blank_view() -> np.ndarray:
    return np.full((CSI_SIZE[1], CSI_SIZE[0], 3), 120, dtype=np.uint8)


def message(image: np.ndarray, stamp_s: int) -> Any:
    data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_s, nanosec=0)), data=data)


class FakeLogger:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, text: str, **_: Any) -> None:
        self.lines.append(text)

    def warning(self, text: str, **_: Any) -> None:
        self.lines.append(text)

    def error(self, text: str, **_: Any) -> None:
        self.lines.append(text)


class FakeNode:
    def __init__(self) -> None:
        self.subscriptions: list[Any] = []
        self.destroyed: list[Any] = []
        self.timers: list[Any] = []
        self.logger = FakeLogger()

    def get_logger(self) -> FakeLogger:
        return self.logger

    def create_subscription(self, msg_type: Any, topic: str, callback: Any, qos: Any) -> Any:
        sub = SimpleNamespace(msg_type=msg_type, topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(sub)
        return sub

    def destroy_subscription(self, sub: Any) -> None:
        self.destroyed.append(sub)

    def create_timer(self, period: float, callback: Any) -> Any:
        timer = SimpleNamespace(period=period, callback=callback, cancel=lambda: None)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer: Any) -> None:
        self.destroyed.append(timer)


class Clock:
    def __init__(self) -> None:
        self.now = T0

    def __call__(self) -> datetime:
        self.now += timedelta(seconds=1)
        return self.now


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig.model_validate({"state_dir": str(tmp_path / "state"), "limits": {"min_views": MIN_VIEWS}})


@pytest.fixture
def node() -> FakeNode:
    return FakeNode()


@pytest.fixture
def csi(node: FakeNode, cfg: NodeConfig) -> CsiCalibration:
    return CsiCalibration(node, cfg, Clock())


def feed(node: FakeNode, images: list[np.ndarray], first_stamp: int = 100) -> None:
    callback = node.subscriptions[-1].callback
    for offset, image in enumerate(images):
        callback(message(image, first_stamp + offset))


def captured(csi: CsiCalibration, node: FakeNode, camera: str, count: int) -> str:
    """Start a capture on camera, feed count distinct views, stop it; returns the session id."""
    sid = csi.start(camera, None)["capture"]["session_id"]
    feed(node, [csi_view(i) for i in range(count)])
    csi.stop(camera)
    return sid


def fake_run(cmd: list[str], **_: Any) -> subprocess.CompletedProcess:
    return subprocess.CompletedProcess(cmd, 0, stdout="ActiveState=active\nActiveEnterTimestamp=@9999999999\n")


# config


def test_config_defaults_name_front_and_back_cameras() -> None:
    cameras = NodeConfig().csi_cameras
    assert {name: (c.image_topic, c.unit) for name, c in cameras.items()} == {
        "front": ("/front_camera/image_raw/compressed", FRONT_UNIT),
        "back": ("/back_camera/image_raw/compressed", BACK_UNIT),
    }
    assert NodeConfig().csi_min_view_shift_px > 0


@pytest.mark.parametrize(
    "block",
    [
        {"Front": {"image_topic": "/x", "unit": "u.service"}},
        {"front/../x": {"image_topic": "/x", "unit": "u.service"}},
        {"front": {"image_topic": "/x"}},
        {"front": {"image_topic": "/x", "unit": "u.service", "zoom": True}},
    ],
)
def test_config_rejects_bad_csi_cameras(block: dict[str, Any]) -> None:
    with pytest.raises(ValidationError, match="csi_cameras"):
        NodeConfig.model_validate({"csi_cameras": block})


# capture


def test_view_differs_on_shift_or_too_few_shared_corners() -> None:
    base = detection_at([(100.0 + 10 * i, 100.0) for i in range(8)])
    near = detection_at([(105.0 + 10 * i, 100.0) for i in range(8)])
    far = detection_at([(140.0 + 10 * i, 100.0) for i in range(8)])
    assert view_differs(None, base, 20.0)
    assert not view_differs(base, near, 20.0)
    assert view_differs(base, far, 20.0)
    other_ids = type(base)(ids=base.ids + 20, points=base.points)
    assert view_differs(base, other_ids, 20.0)


def test_capture_subscribes_the_camera_image_topic_until_stopped(csi: CsiCalibration, node: FakeNode) -> None:
    state = csi.start("front", None)
    assert state["capture"]["running"] is True
    assert state["capture"]["camera"] == "front"
    (sub,) = node.subscriptions
    assert (sub.msg_type, sub.topic) == (CompressedImage, "/front_camera/image_raw/compressed")
    assert node.timers[0].period == csi.cfg.max_capture_s
    with pytest.raises(Busy):
        csi.start("back", None)
    csi.stop("front")
    assert sub in node.destroyed
    assert csi.state("front")["capture"]["running"] is False


def test_capture_keeps_only_detected_views_that_differ_from_the_last_kept(
    csi: CsiCalibration, node: FakeNode, cfg: NodeConfig
) -> None:
    sid = csi.start("front", None)["capture"]["session_id"]
    callback = node.subscriptions[0].callback
    callback(message(blank_view(), 100))
    assert csi.state("front")["capture"]["status"] == NO_BOARD
    callback(message(csi_view(0), 101))
    callback(message(csi_view(0), 102))
    assert csi.state("front")["capture"]["status"] == SAME_VIEW
    callback(message(csi_view(1), 103))
    state = csi.state("front")
    assert state["capture"]["view_count"] == 2
    assert state["session"]["id"] == sid
    assert state["session"]["view_count"] == 2
    assert state["session"]["image_size"] == list(CSI_SIZE)
    coverage = state["session"]["coverage"]
    assert (coverage["cols"], coverage["rows"]) == (cfg.coverage.cols, cfg.coverage.rows)
    assert sum(map(sum, coverage["counts"])) > 0
    views = Path(cfg.state_dir) / "csi" / "front" / sid / "views"
    assert sorted(p.name for p in views.iterdir()) == ["001", "002"]
    meta = json.loads((views / "001" / "view.json").read_text())
    assert meta["stamp_ns"] == 101 * 1_000_000_000
    assert len(meta["ids"]) == len(meta["points"]) >= 6
    assert (views / "001" / "image.jpg").is_file()


def test_reopened_session_continues_and_compares_with_its_last_view(csi: CsiCalibration, node: FakeNode) -> None:
    sid = captured(csi, node, "front", 2)
    csi.start("front", sid)
    feed(node, [csi_view(1), csi_view(2)])
    assert csi.state("front")["capture"]["view_count"] == 3
    csi.stop("front")
    with pytest.raises(NotFound):
        csi.start("back", sid)


def test_capture_expires_after_max_capture_s(csi: CsiCalibration, node: FakeNode) -> None:
    csi.start("back", None)
    node.timers[0].callback()
    state = csi.state("back")["capture"]
    assert state["running"] is False
    assert "max_capture_s" in state["status"]


def test_undecodable_frame_is_dropped(csi: CsiCalibration, node: FakeNode) -> None:
    csi.start("front", None)
    bad = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)), data=b"not a jpeg")
    node.subscriptions[0].callback(bad)
    assert csi.state("front")["capture"]["view_count"] == 0
    assert any("dropped" in line for line in node.logger.lines)


def test_unknown_camera_is_not_found(csi: CsiCalibration) -> None:
    with pytest.raises(NotFound):
        csi.start("left", None)
    with pytest.raises(NotFound):
        csi.state("left")


# compute


def test_compute_on_enough_views_returns_k_d_and_rms(csi: CsiCalibration, node: FakeNode) -> None:
    sid = captured(csi, node, "front", VIEW_COUNT)
    result = csi.compute("front", sid)
    k = np.array(result["k"])
    assert k.shape == (3, 3)
    assert k[0, 0] == pytest.approx(CSI_FOCAL, rel=0.02)
    assert k[1, 1] == pytest.approx(CSI_FOCAL, rel=0.02)
    assert k[0, 2] == pytest.approx(CSI_SIZE[0] / 2, abs=10)
    assert k[1, 2] == pytest.approx(CSI_SIZE[1] / 2, abs=10)
    assert len(result["d"]) == 5
    assert 0 < result["rms"] < 1.0
    assert result["view_count"] == VIEW_COUNT
    assert result["image_size"] == list(CSI_SIZE)
    assert (result["kind"], result["camera"], result["session"]) == (KIND_CSI, "front", sid)
    assert csi.results("front", sid) == [result]
    assert csi.result("front", sid, result["result"]) == result


def test_compute_with_too_few_views_names_the_count(csi: CsiCalibration, node: FakeNode) -> None:
    sid = captured(csi, node, "front", 3)
    with pytest.raises(ValueError, match="got 3 views"):
        csi.compute("front", sid)


def test_compute_refused_while_the_session_captures(csi: CsiCalibration, node: FakeNode) -> None:
    sid = csi.start("front", None)["capture"]["session_id"]
    with pytest.raises(Busy):
        csi.compute("front", sid)


# apply


def loader_accepts(path: Path) -> dict[str, Any]:
    """The checks of head_camera_cpp's load_camera_info: sizes, then the K (9), P (12), R (9) and D data."""
    data = yaml.safe_load(path.read_text())
    assert isinstance(data["image_width"], int) and isinstance(data["image_height"], int)
    assert len(data["camera_matrix"]["data"]) == 9
    assert len(data["projection_matrix"]["data"]) == 12
    assert len(data["rectification_matrix"]["data"]) == 9
    assert all(isinstance(v, float) for v in data["distortion_coefficients"]["data"])
    assert set(data) == set(yaml.safe_load(LOADER_FIXTURE.read_text()))
    return data


def test_apply_writes_the_ros_calibration_file_then_the_record(
    csi: CsiCalibration, node: FakeNode, cfg: NodeConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    sid = captured(csi, node, "front", VIEW_COUNT)
    result = csi.compute("front", sid)
    applied = Path(cfg.state_dir) / APPLIED_DIR
    applied.mkdir(parents=True)
    write_json_atomic(applied / APPLIED_MARKER, {"kind": KIND_INTRINSICS, "session": "calib-1", "head_chain": {"a": 1}})
    landed = record_applied_writes(monkeypatch, applied)
    answer = csi.apply("front", sid, result["result"], fake_run)
    assert landed == [applied_file_name("front"), APPLIED_MARKER]
    assert applied_file_name("front") == "front_camera_calibration.yaml"
    data = loader_accepts(applied / "front_camera_calibration.yaml")
    assert (data["image_width"], data["image_height"]) == CSI_SIZE
    assert data["camera_name"] == "front_camera"
    assert data["distortion_model"] == "plumb_bob"
    k = np.array(result["k"])
    assert data["camera_matrix"]["data"] == pytest.approx(k.reshape(-1).tolist())
    assert data["distortion_coefficients"]["data"] == pytest.approx(result["d"])
    assert data["rectification_matrix"]["data"] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert data["projection_matrix"]["data"] == pytest.approx(np.hstack([k, np.zeros((3, 1))]).reshape(-1).tolist())
    marker = json.loads((applied / APPLIED_MARKER).read_text())
    record = marker[APPLIED_KEY]["front"]
    assert set(record) == {"kind", "camera", "session", "result", "applied_at"}
    assert (record["kind"], record["camera"], record["session"], record["result"]) == (
        KIND_CSI,
        "front",
        sid,
        result["result"],
    )
    datetime.fromisoformat(record["applied_at"])
    assert (marker["kind"], marker["head_chain"]) == (KIND_INTRINSICS, {"a": 1}), "other records are kept"
    assert answer["applied"] == record
    assert answer["units"] == [{"name": FRONT_UNIT, "active_state": "active", "restarted": True}]
    assert csi.state("front")["applied"] == record
    assert csi.state("back")["applied"] is None
    assert not any(p.name.startswith(".tmp-") for p in applied.iterdir())


def test_apply_of_an_unknown_result_is_not_found(csi: CsiCalibration, node: FakeNode) -> None:
    sid = captured(csi, node, "back", 1)
    with pytest.raises(NotFound):
        csi.apply("back", sid, "r-20261031-120000", fake_run)


def test_d435_records_ignore_a_csi_only_applied_json(
    csi: CsiCalibration, node: FakeNode, cfg: NodeConfig, tmp_path: Path
) -> None:
    """A CSI apply before any D435 check is accepted leaves the D435 check unaccepted: GET /api/state has no applied
    check, GET /api/applied is 404 and the head chain apply is still refused; a later accept keeps the CSI record."""
    sid = captured(csi, node, "front", VIEW_COUNT)
    csi.apply("front", sid, csi.compute("front", sid)["result"], fake_run)
    client = TestClient(app_with(csi, cfg))
    assert client.get("/api/state").json()["applied"] is None
    assert client.get("/api/applied").status_code == 404
    source = tmp_path / "head_chain.json"
    source.write_text("{}")
    with pytest.raises(NotApplicable, match="no D435 intrinsics check is accepted"):
        apply_head_chain_file(cfg, source, {"session": "s"}, T0, fake_run)
    store = SessionStore(Path(cfg.state_dir), cfg)
    check = store.create_session(T0)
    rid = "r-20261031-120001"
    store.result_dir(check, rid).mkdir(parents=True)
    write_json_atomic(store.result_dir(check, rid) / RESULT_FILE, {"kind": KIND_INTRINSICS, "passed": True})
    apply_result(store, check, rid, T0)
    marker = json.loads((Path(cfg.state_dir) / APPLIED_DIR / APPLIED_MARKER).read_text())
    assert marker["kind"] == KIND_INTRINSICS
    assert marker[APPLIED_KEY]["front"]["session"] == sid


# routes


class IdleCapture:
    def state(self) -> dict[str, Any]:
        return {"running": False, "session_id": None, "status": "idle", "pose_count": 0}


class IdleJob:
    def state(self) -> dict[str, Any]:
        return {"running": False}


def app_with(csi: CsiCalibration, cfg: NodeConfig) -> Any:
    capture: Any = IdleCapture()
    job: Any = IdleJob()
    app = build_app(capture, job, SessionStore(Path(cfg.state_dir), cfg), cfg, fake_run)
    app.include_router(csi_router(csi, fake_run))
    return app


def test_routes_capture_compute_results_and_apply(csi: CsiCalibration, node: FakeNode, cfg: NodeConfig) -> None:
    client = TestClient(app_with(csi, cfg))
    started = client.post("/api/csi/front/start", json={})
    assert started.status_code == 200
    sid = started.json()["capture"]["session_id"]
    feed(node, [csi_view(i) for i in range(VIEW_COUNT)])
    assert client.get("/api/csi/front/state").json()["session"]["view_count"] == VIEW_COUNT
    assert client.post("/api/csi/back/start", json={}).status_code == 409
    assert client.post(f"/api/csi/front/sessions/{sid}/compute").status_code == 409
    assert client.post("/api/csi/front/stop").json()["capture"]["running"] is False
    computed = client.post(f"/api/csi/front/sessions/{sid}/compute")
    assert computed.status_code == 200
    rid = computed.json()["result"]
    assert client.get(f"/api/csi/front/sessions/{sid}/results").json() == [computed.json()]
    assert client.get(f"/api/csi/front/sessions/{sid}/results/{rid}").json() == computed.json()
    applied = client.post(f"/api/csi/front/sessions/{sid}/results/{rid}/apply")
    assert applied.status_code == 200
    assert applied.json()["units"][0]["name"] == FRONT_UNIT
    assert client.get("/api/csi/front/state").json()["applied"]["result"] == rid


def test_routes_unknown_camera_is_404(csi: CsiCalibration, cfg: NodeConfig) -> None:
    client = TestClient(app_with(csi, cfg))
    for method, path in (
        ("get", "/api/csi/left/state"),
        ("post", "/api/csi/left/start"),
        ("post", "/api/csi/left/stop"),
        ("post", "/api/csi/left/sessions/csi-20261031-120000/compute"),
        ("get", "/api/csi/left/sessions/csi-20261031-120000/results"),
        ("post", "/api/csi/left/sessions/csi-20261031-120000/results/r-20261031-120000/apply"),
    ):
        answer = getattr(client, method)(path)
        assert answer.status_code == 404, path
        assert answer.json()["ok"] is False


def test_routes_too_few_views_is_409_naming_the_count(csi: CsiCalibration, node: FakeNode, cfg: NodeConfig) -> None:
    client = TestClient(app_with(csi, cfg))
    sid = captured(csi, node, "back", 2)
    answer = client.post(f"/api/csi/back/sessions/{sid}/compute")
    assert answer.status_code == 409
    assert "got 2 views" in answer.json()["message"]
    assert client.get("/api/csi/back/sessions/csi-bad/results").status_code == 404


def test_concurrent_applies_keep_every_record(
    csi: CsiCalibration, node: FakeNode, cfg: NodeConfig, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A CSI apply that runs while the D435 accept sits between reading and writing applied.json waits for it (one
    applied.json lock), then a head chain apply lands too: no writer drops another's record."""
    sid = captured(csi, node, "front", VIEW_COUNT)
    rid = csi.compute("front", sid)["result"]
    store = SessionStore(Path(cfg.state_dir), cfg)
    check = store.create_session(T0)
    check_rid = "r-20261031-120001"
    store.result_dir(check, check_rid).mkdir(parents=True)
    write_json_atomic(store.result_dir(check, check_rid) / RESULT_FILE, {"kind": KIND_INTRINSICS, "passed": True})
    marker = Path(cfg.state_dir) / APPLIED_DIR / APPLIED_MARKER
    replace = os.replace
    racer: list[threading.Thread] = []

    def interleaving(src: Any, dst: Any) -> None:
        if Path(dst) == marker and not racer:
            # The D435 accept has read applied.json and is about to write it: run the CSI apply now.
            racer.append(threading.Thread(target=csi.apply, args=("front", sid, rid, fake_run)))
            racer[0].start()
            racer[0].join(RACE_WAIT_S)
        replace(src, dst)

    monkeypatch.setattr(os, "replace", interleaving)
    apply_result(store, check, check_rid, T0)
    racer[0].join()
    monkeypatch.setattr(os, "replace", replace)
    source = tmp_path / "head_chain.json"
    source.write_text("{}")
    apply_head_chain_file(cfg, source, {"session": "floor-1"}, T0, fake_run)
    data = json.loads(marker.read_text())
    assert data["kind"] == KIND_INTRINSICS
    assert data[APPLIED_KEY]["front"]["result"] == rid
    assert data["head_chain"] == {"session": "floor-1"}
